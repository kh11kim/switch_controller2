#pragma once

//frankalib
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/gripper.h>

#include "switch_controller2/core.hpp"
#include "switch_controller2/spline.hpp"

typedef std::function<franka::Torques(const franka::RobotState&, franka::Duration)> TorqueCtrlCallback;
typedef std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> PositionCtrlCallback;

#define GRIPPER_MAX_WIDTH 0.08
#define GRIPPER_EPSILON 0.005

class Panda{
protected:
  std::string ip;
  
  franka::Robot panda;
  franka::Model model;
  bool _stop_ctrl;
  bool _has_data;
  std::mutex m;
  // joint impedance control
  std::array<double, 7> joint_k_array;
  std::array<double, 7> joint_d_array;
  // task impedance control
  Eigen::MatrixXd task_k_matrix;
  Eigen::MatrixXd task_d_matrix;
  // trajectory following
  double t_traj;

public:
  franka::RobotState state;
  franka::Gripper gripper;
  Spline sp;
  std::array<double, 7> q_d;
  Eigen::Vector3d EE_pos_d;
  Eigen::Quaterniond EE_orn_d;
  std::string joint_names[7] = {
    "panda_joint1", 
    "panda_joint2", 
    "panda_joint3", 
    "panda_joint4", 
    "panda_joint5", 
    "panda_joint6", 
    "panda_joint7"
  };
  bool is_trajectory_following;

  void DoIdleControl(); //Idle loop
  void DoTorqueControl(TorqueCtrlCallback loop_fn);
  void DoPositionControl(PositionCtrlCallback loop_fn);
  void UpdateState(franka::RobotState panda_state){
    if (m.try_lock()){
      _has_data = true;
      state = panda_state;
      m.unlock();
    }
  }
  bool IsDataReceived(){return _has_data;}
  void StopControl(){_stop_ctrl = true;}
  

  franka::Torques JointImpedanceCtrlCallback(const franka::RobotState&, franka::Duration);
  franka::Torques TaskImpedanceCtrlCallback(const franka::RobotState&, franka::Duration);
  franka::JointPositions TrajectoryFollowingCallback(const franka::RobotState&, franka::Duration);
  void InitializeTrajectoryFollowing(){
    t_traj = 0;
    is_trajectory_following = false;
  }

  Panda(const std::string &ip)
    : panda(ip)
    , gripper(ip)
    , model(panda.loadModel())
    , _has_data(false)
    , _stop_ctrl(false)
    , task_k_matrix(6,6)
    , task_d_matrix(6,6)
  {
    panda.setCollisionBehavior({{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
                             {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
                             {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
                             {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});
    panda.setJointImpedance({{2000, 2000, 2000, 1500, 1500, 1000, 1000}});
    panda.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    joint_k_array.fill(200.0);
    joint_d_array.fill(10.0);
    
    double k_trans = 220.;
    double k_rot = 20.;
    task_k_matrix.topLeftCorner(3, 3) << k_trans * Eigen::MatrixXd::Identity(3, 3);
    task_d_matrix.topLeftCorner(3, 3) << 2.0 * sqrt(k_trans) *
                                        Eigen::MatrixXd::Identity(3, 3);
    
    task_k_matrix.bottomRightCorner(3, 3) << k_rot * Eigen::MatrixXd::Identity(3, 3);
    task_d_matrix.bottomRightCorner(3, 3) << 2.0 * sqrt(k_rot) *
                                            Eigen::MatrixXd::Identity(3, 3);
  };
  ~Panda(){_stop_ctrl = true;};
};
