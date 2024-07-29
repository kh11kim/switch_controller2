#pragma once
#include "switch_controller2/core.hpp"
#include "switch_controller2/thread_pool.hpp"
#include "switch_controller2/panda_interface.hpp"

#define Q_D_SET_DIFFERENCE_LIMIT 0.1
#define EE_POS_SET_DIFFERENCE_LIMIT 0.1
#define EE_ORN_SET_DIFFERENCE_LIMIT 0.1

enum CtrlMode{
  CTRLMODE_STOP,
  CTRLMODE_IDLE,
  CTRLMODE_JOINT_IMP,
  CTRLMODE_TASK_IMP,
  CTRLMODE_TRAJECTORY_FOLLOWING,
};

enum GraspMode{
  GRASPMODE_IDLE,
  GRASPMODE_GRASP,
  GRASPMODE_MOVE,
  GRASPMODE_AUTOGRASP,
  GRASPMODE_HOMING,
};

class SwitchController : protected ThreadPool{
public:
  std::shared_ptr<Panda> robot;
  std::vector<std::shared_ptr<Waypoint>> waypoints;
  bool grasp_result;

  SwitchController(std::shared_ptr<Panda> robot_ptr)
    : _ctrl_mode(0)
    , robot(robot_ptr)
    , _is_ctrl_running(false)
    , _is_stopped_by_error(false)
  {};
  ~SwitchController(){ 
    _ctrl_mode = CTRLMODE_STOP;
    _gripper_ctrl_mode = GRASPMODE_IDLE;
    StopCtrl();
  };

  void SetCtrlMode(int mode) {_ctrl_mode = mode;};
  int GetCtrlMode() {return _ctrl_mode;};
  int GetGripperCtrlMode() {return _gripper_ctrl_mode;};
  bool IsStoppedByError() {return _is_stopped_by_error;}
  void ClearError() {_is_stopped_by_error = false;}
  void SetController(int ctrl_mode);
  void SetGraspController(int grasp_mode, double width, double force);
  void SetDesiredQ(const std::vector<double> &q_d_cmd){
    for(int i=0;i<7;i++){
      if(q_d_cmd[i] - robot->state.q[i] > Q_D_SET_DIFFERENCE_LIMIT){
        std::cout << "Error: desired q_d is too far from current q!" << std::endl;
        return;
      }
    }

    for(int i=0;i<7;i++) robot->q_d[i] = q_d_cmd[i];
  }
  void SetDesiredEEPose(const Eigen::Vector3d &EE_pos_d_cmd, const Eigen::Quaterniond &EE_orn_d_cmd){
    Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(robot->state.O_T_EE.data()));
    Eigen::Vector3d EE_pos(EE_pose.translation());
    for(int i=0;i<3;i++){
      if(EE_pos_d_cmd[i] - EE_pos[i] > EE_POS_SET_DIFFERENCE_LIMIT){
        std::cout << "Error: desired ee_pose.position is too far from current pos!" << std::endl;
        return;
      }
    }
    robot->EE_pos_d = EE_pos_d_cmd;
    robot->EE_orn_d = EE_orn_d_cmd;
  }

  void ExecuteTrajectoryFollowing(){
    robot->sp.SetWaypoints(waypoints);
    robot->sp.CalculateParameters();
    robot->InitializeTrajectoryFollowing();
    robot->is_trajectory_following = true;
    std::cout << "Trajectory start!" << std::endl;
  }
  

  // Grasping
  bool Grasp(double width, double force){
    return robot->gripper.grasp(width, 0.1, force, GRIPPER_EPSILON, GRIPPER_EPSILON);
  }
  bool Move(double width){
    return robot->gripper.move(width, 0.1);
  }
  bool AutoGrasp(double force){
    Grasp(0., 0.);
    franka::GripperState gripper_state = robot->gripper.readOnce();
    return Grasp(gripper_state.width, force);
  }
  bool Homing(){
    return robot->gripper.homing();
  }

private:
  int _ctrl_mode;
  bool _is_ctrl_running;
  bool _is_stopped_by_error;
  bool _gripper_ctrl_mode;
  
  void CtrlJob();
  
  void StopCtrl();
};