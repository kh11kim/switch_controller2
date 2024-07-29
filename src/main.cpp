#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "switch_controller2/switch_controller.hpp"
#include "switch_controller2/panda_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "switch_controller2_interface/msg/ctrl_config.hpp"
#include "switch_controller2_interface/srv/grasp_cmd.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

std::shared_ptr<Waypoint> ConvertToWaypoint(const trajectory_msgs::msg::JointTrajectoryPoint& msg){
  std::shared_ptr<Waypoint> wptr = std::make_shared<Waypoint>();
  for(int i=0;i<7;i++){
    wptr->q[i] = msg.positions[i];
    wptr->dq[i] = msg.velocities[i]; // accelerations are ignored
  }
  wptr->time_from_start = msg.time_from_start.sec;
  wptr->time_from_start += msg.time_from_start.nanosec * 0.000000001;
  return wptr;
}



class SwitchControllerNode : public rclcpp::Node, public SwitchController
{
public:
  SwitchControllerNode(std::shared_ptr<Panda> robot_ptr)
    : Node("switch_controller")
    , SwitchController(robot_ptr)
  {
    //subscription
    ctrl_config_sub = this->create_subscription<switch_controller2_interface::msg::CtrlConfig>(
      "ctrl_config_cmd", 10, std::bind(&SwitchControllerNode::CtrlConfigCallback, this, _1));
    joint_state_d_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_d", 10, std::bind(&SwitchControllerNode::DesiredJointStateCallback, this, _1));
    ee_pose_d_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ee_pose_d", 10, std::bind(&SwitchControllerNode::DesiredEEPoseCallback, this, _1));
    traj_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_traj", 10, std::bind(&SwitchControllerNode::JointTrajectoryCallback, this, _1));
    
    //publisher
    ctrl_config_pub = this->create_publisher<switch_controller2_interface::msg::CtrlConfig>("ctrl_config", 10);
    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    //gripper_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("gripper_states", 10);
    ee_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
    grasp_srv = this->create_service<switch_controller2_interface::srv::GraspCmd>(
      "grasp_cmd", std::bind(&SwitchControllerNode::GraspCmdCallback, this, _1, _2));
    main_loop_timer = this->create_wall_timer(
      10ms, std::bind(&SwitchControllerNode::LoopCallback, this));
    this->SetController(CTRLMODE_IDLE);
  }


private:
  rclcpp::TimerBase::SharedPtr main_loop_timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<switch_controller2_interface::msg::CtrlConfig>::SharedPtr ctrl_config_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub;
  
  rclcpp::Subscription<switch_controller2_interface::msg::CtrlConfig>::SharedPtr ctrl_config_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_d_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_d_sub;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub;

  rclcpp::Service<switch_controller2_interface::srv::GraspCmd>::SharedPtr grasp_srv;
  
  void LoopCallback(){
    // publish joint state
    auto joint_state_msg = sensor_msgs::msg::JointState();
    auto gripper_state_msg = sensor_msgs::msg::JointState();
    auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
    auto ctrl_config_msg = switch_controller2_interface::msg::CtrlConfig();

    joint_state_msg.header.stamp = this->now();
    ee_pose_msg.header.stamp = this->now();
    
    robot->gripper_state = robot->gripper.readOnce();
    for (int i=0;i<9;i++){
      joint_state_msg.name.push_back(robot->joint_names[i]);
      if(i < 7){  
        if (GetCtrlMode() == CTRLMODE_TRAJECTORY_FOLLOWING){
          //for smooth q
          joint_state_msg.position.push_back(robot->state.q_d[i]);
          joint_state_msg.velocity.push_back(robot->state.dq_d[i]);
        }
        else{
          joint_state_msg.position.push_back(robot->state.q[i]);
          joint_state_msg.velocity.push_back(robot->state.dq[i]);
        }
        joint_state_msg.effort.push_back(robot->state.tau_J[i]);
      } 
      else {
        joint_state_msg.position.push_back(robot->gripper_state.width/2+0.04);
        joint_state_msg.velocity.push_back(0.);
        joint_state_msg.effort.push_back(0.);
      }
    }
    {
      Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(robot->state.O_T_EE.data()));
      Eigen::Vector3d EE_pos(EE_pose.translation());
      Eigen::Quaterniond EE_orn(EE_pose.linear());
      ee_pose_msg.pose.position.x = EE_pos[0];
      ee_pose_msg.pose.position.y = EE_pos[1];
      ee_pose_msg.pose.position.z = EE_pos[2];
      ee_pose_msg.pose.orientation.x = EE_orn.x();
      ee_pose_msg.pose.orientation.y = EE_orn.y();
      ee_pose_msg.pose.orientation.z = EE_orn.z();
      ee_pose_msg.pose.orientation.w = EE_orn.w();
    }
    ctrl_config_msg.ctrl_mode = GetCtrlMode();
    ctrl_config_msg.gripper_ctrl_mode = GetGripperCtrlMode();

    joint_state_pub->publish(joint_state_msg);
    ee_pose_pub->publish(ee_pose_msg);
    ctrl_config_pub->publish(ctrl_config_msg);
    
    if(IsStoppedByError()){
      RCLCPP_INFO(this->get_logger(), "Error: set to idle mode");
      SetController(CTRLMODE_IDLE);
      ClearError();
    }
  }

  void CtrlConfigCallback(const switch_controller2_interface::msg::CtrlConfig::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Set Controller Config");
    SetController(msg->ctrl_mode);
  }

  void DesiredJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "joint_state_d subscribed");
    if(GetCtrlMode() != CTRLMODE_JOINT_IMP){
      RCLCPP_INFO(this->get_logger(), "Invalid joint_state_d command: Current ctrl_mode is not Joint Impedance Ctrl mode");
      return;
    }
    SetDesiredQ(msg->position);
  }

  void DesiredEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "ee_pose_d subscribed");
    if(GetCtrlMode() != CTRLMODE_TASK_IMP){
      RCLCPP_INFO(this->get_logger(), "Invalid ee_pose_d command: Current ctrl_mode is not Task Impedance Ctrl mode");
      return;
    }
    Eigen::Vector3d pos_d;
    Eigen::Quaterniond orn_d;
    pos_d[0] = msg->pose.position.x;
    pos_d[1] = msg->pose.position.y;
    pos_d[2] = msg->pose.position.z;
    orn_d.x() = msg->pose.orientation.x;
    orn_d.y() = msg->pose.orientation.y;
    orn_d.z() = msg->pose.orientation.z;
    orn_d.w() = msg->pose.orientation.w;
    SetDesiredEEPose(pos_d, orn_d);
  }

  void JointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "joint_traj subscribed");
    if(GetCtrlMode() != CTRLMODE_TRAJECTORY_FOLLOWING){
      RCLCPP_INFO(this->get_logger(), "Invalid joint_traj command: Current ctrl_mode is not Trajectory Following mode");
      return;
    }
    waypoints.clear();

    int num_waypoints = 0;
    for(const auto &traj_point : msg->points){
      waypoints.push_back(ConvertToWaypoint(traj_point));
      num_waypoints += 1;
    }
    std::cout << "Received waypoints: " << num_waypoints << std::endl;
    ExecuteTrajectoryFollowing();
  }

  void GraspCmdCallback(const std::shared_ptr<switch_controller2_interface::srv::GraspCmd::Request> request,
                        std::shared_ptr<switch_controller2_interface::srv::GraspCmd::Response>      response){
    
    try{
      switch(request->cmd_type){
        case GRASPMODE_IDLE: // grasp
          ; //do nothing
        break;
        case GRASPMODE_GRASP:
          RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Grasp");
          RCLCPP_INFO(this->get_logger(), "width: %f force: %f", request->width, request->force);
          SetGraspController(GRASPMODE_GRASP, request->width, request->force);
        break;
        case GRASPMODE_MOVE: // move
          RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Move");
          RCLCPP_INFO(this->get_logger(), "width: %f", request->width);
          SetGraspController(GRASPMODE_MOVE, request->width, request->force);
        break;
        case GRASPMODE_AUTOGRASP: // auto-grasp
          RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Auto-grasp");
          RCLCPP_INFO(this->get_logger(), "force: %f", request->force);
          SetGraspController(GRASPMODE_AUTOGRASP, request->width, request->force);
        break;
        case GRASPMODE_HOMING: // homing
          RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Homing");
          SetGraspController(GRASPMODE_HOMING, request->width, request->force);
        break;
      }
    }
    catch (const std::bad_function_call& other){
      std::cout << other.what() << std::endl; 
    }
    //RCLCPP_INFO(this->get_logger(), "sending back response: [%s]", response->result ? "true" : "false");
    response->result = grasp_result;
  }      
};


int main(int argc, char *argv[])
{
  const std::string ip = "192.168.151.50";
  std::shared_ptr<Panda> panda = std::make_shared<Panda>(ip);
  
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<SwitchControllerNode>(panda)
  );
  rclcpp::shutdown();
  // std::shared_ptr<Panda> panda = std::make_shared<Panda>(ip); 
  // std::shared_ptr<SwitchController> ctrl = std::make_shared<SwitchController>(panda); 
  

  // ctrl->SetController(CTRLMODE_IDLE);
  // std::this_thread::sleep_for(1s);

  // ctrl->SetController(CTRLMODE_TRAJECTORY_FOLLOWING);
  // std::this_thread::sleep_for(1s);
  
  // ctrl->waypoints.clear();

  // Waypoint w0, w1;
  // w0.q = to_eigen(ctrl->robot->state.q);
  // for(int i=0;i<7;i++){
  //   w1.q[i] = w0.q[i] + 0.1;
  // }
  // w0.time_from_start = 0.;
  // w1.time_from_start = 5.;

  // ctrl->waypoints.push_back(w0);
  // ctrl->waypoints.push_back(w1);
  // ctrl->ExecuteTrajectoryFollowing();

  // while(true){
  //   ;
  // }

  //std::this_thread::sleep_for(10s);
  return 0;
}