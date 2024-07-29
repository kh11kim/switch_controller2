#pragma once

#include "core.hpp"
#include <vector>

#define BEYOND_HORIZON -1

inline std::array<double, 7> to_vec(const Vector7d &q){
  std::array<double, 7> result;
  for(int i=0;i<7;i++){
    result[i] = q[i];
  }
  return result;
}
inline Vector7d to_eigen(const std::array<double, 7> &qvec){
  Vector7d result;
  for(int i=0;i<7;i++){
    result[i] = qvec[i];
  }
  return result;
}

struct Waypoint
{
  Vector7d q;
  Vector7d dq;
  Vector7d ddq;
  double time_from_start; //sec
  Waypoint(){
    q.setZero();
    dq.setZero();
    ddq.setZero();
    time_from_start = 0.;
  }
};


class Spline
{
private:
  int dof = 7;
  std::vector<std::vector< Vector6d >> coeffs; //axis0:time, axis1:joint, axis3:dof - quintic spline
  std::vector<std::shared_ptr<Waypoint>> waypoints;
  std::vector<double> timestamps;

public:
  double last_timestep;
  int last_timestep_index = 0;

  Spline(){}
  void SetWaypoints(const std::vector<std::shared_ptr<Waypoint>>& waypoints){
    this->waypoints = waypoints;
  }

  Vector6d GetParameter(
    const double &x_start, const double &x_end, 
    const double &dx_start, const double &dx_end, 
    const double &ddx_start, const double &ddx_end, 
    const double &t_start, const double &t_end)
  {   
    Matrix6d B_matrix;
    Vector6d temp_b_vector;

    B_matrix << 1,  t_start,  std::pow(t_start,2),  std::pow(t_start,3),  std::pow(t_start,4),  std::pow(t_start,5),
                0,  1,        2*t_start,3*std::pow(t_start,2),4*std::pow(t_start,3),5*std::pow(t_start,4),
                0,         0,                         2,            6*t_start,12*std::pow(t_start,2),20*std::pow(t_start,3),
                1, t_end,     std::pow(t_end,2),  std::pow(t_end,3),  std::pow(t_end,4),  std::pow(t_end,5),
                0,         1,               2*t_end,3*std::pow(t_end,2),4*std::pow(t_end,3),5*std::pow(t_end,4),
                0,         0,                         2,6*std::pow(t_end,1),12*std::pow(t_end,2),20*std::pow(t_end,3);

    temp_b_vector << x_start, dx_start, ddx_start, x_end, dx_end, ddx_end;
    return B_matrix.inverse()*temp_b_vector;
  }

  bool CalculateParameters()
  {
    double first_timestep = waypoints[0]->time_from_start;
    //Update C_vector
    int num_waypoints = waypoints.size();
    last_timestep_index = num_waypoints-1;
    last_timestep = waypoints[last_timestep_index]->time_from_start;
    if(first_timestep >= last_timestep){
      std::cout << "The last timestep is earlier than the start" << std::endl;
      std::cout << "Start time " << first_timestep << std::endl;
      std::cout << "Last time " << last_timestep << std::endl;
      return false;
    }
    coeffs.clear();
    timestamps.clear();

    //number of spline is (# of way points - 1)
    //std::cout << "num_way_points : " << num_waypoints << std::endl;
    coeffs.resize(num_waypoints - 1);
    
    // k:waypoint index, i:joint index
    for(int k=0; k<(num_waypoints-1); k++)
    {
      double time_start = waypoints[k]->time_from_start;
      double time_end = waypoints[k+1]->time_from_start;
      double duration = time_end - time_start;
      for(int i=0; i<dof; i++)
      {
        
        Vector6d coeff = GetParameter(waypoints[k]->q[i],   waypoints[k+1]->q[i], 
                          waypoints[k]->dq[i],  waypoints[k+1]->dq[i], 
                          waypoints[k]->ddq[i], waypoints[k+1]->ddq[i], 
                          0, duration);
        coeffs.at(k).push_back(coeff);
      }
      
      timestamps.push_back(time_start);
    }
    timestamps.push_back(waypoints[num_waypoints-1]->time_from_start);
    
    
    std::cout<<"number of way points : "<< num_waypoints <<std::endl;
    return true;
  }

  int GetTimestepIndex(const double &time)
  {  
    //If time is beyond the planned horizon,
    if(time >= timestamps[timestamps.size()-1]) return BEYOND_HORIZON; 
    
    // Binary search
    int start = 0;
    int end = timestamps.size()-1;
    int mid = 0;
    while( (start + 1) < end)
    {
      mid = (start + end)/2;
      if(timestamps[mid] <= time) start = mid;
      else end = mid;
    }
    return start;
  }

  bool GetSplineResult(const double &time, Vector7d& q_d, Vector7d& dq_d)
  {
    if(timestamps.empty()){
      std::cout << "No precalculated spline" << std::endl;
      return false;
    }
    
    int timestep_index = GetTimestepIndex(time);
    if(timestep_index == BEYOND_HORIZON)
    {   
      q_d = waypoints[last_timestep_index]->q;
      dq_d = waypoints[last_timestep_index]->dq;
    }
    else
    {   
      double del_time = time - timestamps[timestep_index];
      Eigen::Matrix<double, 3,6> temp;
      temp << 1, del_time, std::pow(del_time,2),   std::pow(del_time,3),  std::pow(del_time,4), std::pow(del_time,5),
              0,        1,           2*del_time, 3*std::pow(del_time,2), 4*std::pow(del_time,3), 5*std::pow(del_time,4),
              0,        0,                    2,             6*del_time, 12*std::pow(del_time,2), 20*std::pow(del_time,3);

      for(int j=0; j<dof; j++)
      {
        Eigen::Vector3d temp_ans = temp * coeffs[timestep_index][j];
        q_d[j] = temp_ans[0];
        dq_d[j] = temp_ans[1];
      }
    }
    return true;
  }
};