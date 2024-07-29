#include "switch_controller2/switch_controller.hpp"

void SwitchController::SetController(int ctrl_mode_cmd){
  if (_is_ctrl_running){
    StopCtrl();
  }
  _is_ctrl_running = true;
  _ctrl_mode = ctrl_mode_cmd;
  EnqueueJob([this](){ this->CtrlJob(); }); 
}

void SwitchController::SetGraspController(int grasp_mode, double width, double force){
  if(_gripper_ctrl_mode){
    std::cout << "Grasping not finished!!" << std::endl;
    return;
  }
  _gripper_ctrl_mode = grasp_mode;

  EnqueueJob([&](){ 
      switch(grasp_mode){
        case GRASPMODE_GRASP: // grasp
          this->grasp_result = Grasp(width, force);
        break;
        case GRASPMODE_MOVE: // move
          this->grasp_result = Move(width);
        break;
        case GRASPMODE_AUTOGRASP: // auto-grasp
          this->grasp_result = AutoGrasp(width);
        break;
        case GRASPMODE_HOMING: // homing
          this->grasp_result = Homing();
        break;
      }
    }
  ); 
  _gripper_ctrl_mode = GRASPMODE_IDLE;

}

void SwitchController::CtrlJob(){
  TorqueCtrlCallback torque_ctrl_cb;
  PositionCtrlCallback pos_ctrl_cb;
  try{
    switch(_ctrl_mode){
      case CTRLMODE_STOP:
        std::cout << "stopped" << std::endl;
        StopCtrl();
      break;
      case CTRLMODE_IDLE:
        std::cout << "idle" << std::endl;
        robot->DoIdleControl();
      break;
      case CTRLMODE_JOINT_IMP:
        std::cout << "joint imp ctrl" << std::endl;
        torque_ctrl_cb = boost::bind(&Panda::JointImpedanceCtrlCallback, this->robot, _1, _2);
        robot->DoTorqueControl(torque_ctrl_cb);
      break;
      case CTRLMODE_TASK_IMP:
        std::cout << "task imp ctrl" << std::endl;
        torque_ctrl_cb = boost::bind(&Panda::TaskImpedanceCtrlCallback, this->robot, _1, _2);
        robot->DoTorqueControl(torque_ctrl_cb);
      break;
      case CTRLMODE_TRAJECTORY_FOLLOWING:
        std::cout << "trajectory following" << std::endl;
        pos_ctrl_cb = boost::bind(&Panda::TrajectoryFollowingCallback, this->robot, _1, _2);
        robot->DoPositionControl(pos_ctrl_cb);
    }
  } catch(int expn) {
    StopCtrl();
    _is_stopped_by_error = true;
    std::cout << "got error" << std::endl;
  }
}

void SwitchController::StopCtrl(){
  _ctrl_mode = CTRLMODE_STOP;
  robot->StopControl();
  std::cout << "Robot control stopped" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

