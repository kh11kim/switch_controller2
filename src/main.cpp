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
#include "switch_controller2_msg/msg/ctrl_config.hpp"
#include "switch_controller2_msg/srv/grasp_cmd.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class SwitchControllerNode : public rclcpp::Node, public SwitchController
{
public:
  SwitchControllerNode(std::shared_ptr<Panda> robot_ptr)
    : Node("switch_controller")
    , SwitchController(robot_ptr)
  {
    ctrl_config_sub = this->create_subscription<switch_controller2_msg::msg::CtrlConfig>(
      "ctrl_config123", 10, std::bind(&SwitchControllerNode::CtrlConfigCallback, this, _1));
    joint_state_d_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_123", 10, std::bind(&SwitchControllerNode::DesiredJointStateCallback, this, _1));
    ee_pose_d_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ee_pose_d", 10, std::bind(&SwitchControllerNode::DesiredEEPoseCallback, this, _1));

    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    grasp_srv = this->create_service<switch_controller2_msg::srv::GraspCmd>(
      "grasp_cmd", std::bind(&SwitchControllerNode::GraspCmdCallback, this, _1, _2));
    main_loop_timer = this->create_wall_timer(
      10ms, std::bind(&SwitchControllerNode::LoopCallback, this));
    this->SetController(CTRLMODE_IDLE);
  }


private:
  rclcpp::TimerBase::SharedPtr main_loop_timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<switch_controller2_msg::msg::CtrlConfig>::SharedPtr ctrl_config_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_d_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_d_sub;

  rclcpp::Service<switch_controller2_msg::srv::GraspCmd>::SharedPtr grasp_srv;
  
  void LoopCallback(){
    // publish joint state
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    for (int i=0;i<7;i++){
      msg.name.push_back(robot->joint_names[i]);
      msg.position.push_back(robot->state.q[i]);
      msg.velocity.push_back(robot->state.dq[i]);
      msg.effort.push_back(robot->state.tau_J[i]);
    }

    // // fill msg. ee pose(robot_state_publisher)
    joint_state_pub->publish(msg);
    
    if(IsStoppedByError()){
      RCLCPP_INFO(this->get_logger(), "Error: set to idle mode");
      SetController(CTRLMODE_IDLE);
      ClearError();
    }
  }

  void CtrlConfigCallback(const switch_controller2_msg::msg::CtrlConfig::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Set Controller Config");
    SetController(msg->ctrl_mode);
  }

  void DesiredJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    std::cout << "received" << std::endl;
    RCLCPP_INFO(this->get_logger(), "joint_state_d subscribed");
    if(GetCtrlMode() != CTRLMODE_JOINT_IMP){
      RCLCPP_INFO(this->get_logger(), "Invalid joint_state_d command: Current ctrl_mode is not Joint Impedance Ctrl mode");
      return;
    }
    //SetDesiredQ(msg->position);
  }

  void DesiredEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    std::cout << "received" << std::endl;
    RCLCPP_INFO(this->get_logger(), "ee_pose_d subscribed");
    if(GetCtrlMode() != CTRLMODE_TASK_IMP){
      RCLCPP_INFO(this->get_logger(), "Invalid ee_pose_d command: Current ctrl_mode is not Task Impedance Ctrl mode");
      return;
    }
    //SetDesiredQ(msg->position);
  }

  void GraspCmdCallback(const std::shared_ptr<switch_controller2_msg::srv::GraspCmd::Request> request,
                        std::shared_ptr<switch_controller2_msg::srv::GraspCmd::Response>      response){
    switch(request->cmd_type){
      case 0: // grasp
        RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Grasp");
        RCLCPP_INFO(this->get_logger(), "width: %ld force: %ld", request->width, request->force);
        response->result = Grasp(request->width, request->force);
      break;
      case 1: // move
        RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Move");
        RCLCPP_INFO(this->get_logger(), "width: %ld", request->width, request->force);
        response->result = Move(request->width);
      break;
      case 2: // auto-grasp
        RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Auto-grasp");
        RCLCPP_INFO(this->get_logger(), "force: %ld", request->force);
        response->result = AutoGrasp(request->force);
      break;
      case 3: // homing
        RCLCPP_INFO(this->get_logger(), "Incoming grasp request: Homing");
        response->result = Homing();
      break;
    }
    RCLCPP_INFO(this->get_logger(), "sending back response: [%s]", response->result ? "true" : "false");
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