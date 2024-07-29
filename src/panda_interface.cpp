
#include "switch_controller2/panda_interface.hpp"



void Panda::DoIdleControl(){
  _stop_ctrl = false;
  std::function<bool(const franka::RobotState&)> loop_fn_panda; 
  loop_fn_panda = [&](const franka::RobotState& panda_state){
    UpdateState(panda_state);
    return !_stop_ctrl; //if stop_ctrl is true, break
  };
  panda.read(loop_fn_panda);
}

void Panda::DoTorqueControl(TorqueCtrlCallback loop_fn){
  _stop_ctrl = false;
  //initialization
  UpdateState(panda.readOnce());
  q_d = state.q;
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(state.O_T_EE.data()));
  Eigen::Vector3d pos(EE_pose.translation());
  Eigen::Quaterniond orn(EE_pose.linear());
  EE_pos_d = pos;
  EE_orn_d = orn;

  //loop
  panda.control(loop_fn);

}

void Panda::DoPositionControl(PositionCtrlCallback loop_fn){
  _stop_ctrl = false;
  //initialization
  InitializeTrajectoryFollowing();
  UpdateState(panda.readOnce());
  //loop
  panda.control(loop_fn);
  
  //exit
}

franka::Torques Panda::JointImpedanceCtrlCallback(const franka::RobotState& robot_state, franka::Duration time){
  std::array<double, 7> tau_d;
  std::array<double, 7> coriolis = model.coriolis(robot_state);
  UpdateState(robot_state);
  if(m.try_lock()){
    for (size_t i = 0; i < 7; i++) {
      tau_d[i] = joint_k_array[i] * (q_d[i] - robot_state.q[i]) - joint_d_array[i] * robot_state.dq[i];
      tau_d[i] += coriolis[i];
    }
    m.unlock();
  }
  if (_stop_ctrl){
    return franka::MotionFinished(franka::Torques(tau_d));
  }
  return tau_d;
}

franka::Torques Panda::TaskImpedanceCtrlCallback(const franka::RobotState& robot_state, franka::Duration time){
  std::array<double, 7> coriolis_array = model.coriolis(robot_state);
  std::array<double, 42> jacobian_array =
    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
  UpdateState(robot_state);

  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Affine3d EE_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d EE_pos(EE_pose.translation());
  Eigen::Quaterniond EE_orn(EE_pose.linear());
  Eigen::Matrix<double, 6, 1> error;

  //error
  error.head(3) << EE_pos - EE_pos_d;
  if (EE_orn_d.coeffs().dot(EE_orn.coeffs()) < 0.0) {
    EE_orn.coeffs() << -EE_orn.coeffs();
  }
  Eigen::Quaterniond error_qtn(EE_orn.inverse() * EE_orn_d);
  error.tail(3) << error_qtn.x(), error_qtn.y(), error_qtn.z();
  error.tail(3) << -EE_pose.linear() * error.tail(3);

  //Compute control
  Eigen::VectorXd tau_task(7), tau_d(7);
  std::array<double, 7> tau_d_array;
  tau_task << jacobian.transpose() * (-task_k_matrix * error - task_d_matrix * (jacobian * dq));
  tau_d << tau_task + coriolis;

  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
  if (_stop_ctrl){
    return franka::MotionFinished(franka::Torques(tau_d_array));
  }
  return tau_d_array;
}

franka::JointPositions Panda::TrajectoryFollowingCallback(const franka::RobotState& robot_state, franka::Duration period){
  Vector7d q_curr;
  Vector7d dq_curr;
  
  bool is_success = true;
  UpdateState(robot_state);

  //body: set q_curr
  if (is_trajectory_following){
    is_success = sp.GetSplineResult(t_traj, q_curr, dq_curr);
    if(!is_success){
      q_curr = to_eigen(robot_state.q_d);
    }
    
    //update
    t_traj += period.toSec();
    if(t_traj >= sp.last_timestep){
      InitializeTrajectoryFollowing();
    }
  }
  else{
    q_curr = to_eigen(robot_state.q_d);
  }

  // error handling
  franka::JointPositions q_output(to_vec(q_curr));
  if (!is_success){
    std::cout << std::endl << "GetSplineResult Failed" << std::endl;  
    return franka::MotionFinished(q_output);
  } else if(_stop_ctrl){
    std::cout << std::endl << "Finished controller" << std::endl;
    return franka::MotionFinished(q_output);
  } else{
    return q_output;
  }
}