#pragma once
#include "switch_controller2/core.hpp"
#include "switch_controller2/thread_pool.hpp"
#include "switch_controller2/panda_interface.hpp"

enum CtrlMode{
  CTRLMODE_STOP,
  CTRLMODE_IDLE,
  CTRLMODE_JOINT_IMP,
  CTRLMODE_TASK_IMP,
  CTRLMODE_TRAJECTORY_FOLLOWING,
};

class SwitchController : protected ThreadPool{
public:
  std::shared_ptr<Panda> robot;
  std::vector<Waypoint> waypoints;

  SwitchController(std::shared_ptr<Panda> robot_ptr)
    : _ctrl_mode(0)
    , robot(robot_ptr)
    , _is_ctrl_running(false)
    , _is_stopped_by_error(false)
  {};
  ~SwitchController(){ 
    _ctrl_mode = CTRLMODE_STOP;
    StopCtrl();
  };

  void SetCtrlMode(int mode) {_ctrl_mode = mode;};
  int GetCtrlMode() {return _ctrl_mode;};
  bool IsStoppedByError() {return _is_stopped_by_error;}
  void ClearError() {_is_stopped_by_error = false;}
  void SetController(int ctrl_mode);
  void SetDesiredQ(const std::vector<double> &q_d_cmd){
    for(int i=0;i<7;i++) robot->q_d[i] = q_d_cmd[i];}
  void ExecuteTrajectoryFollowing(){
    robot->sp.SetWaypoints(waypoints);
    robot->sp.CalculateParameters();
    robot->InitializeTrajectoryFollowing();
    robot->is_trajectory_following = true;
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
  void CtrlJob();
  void StopCtrl();
};