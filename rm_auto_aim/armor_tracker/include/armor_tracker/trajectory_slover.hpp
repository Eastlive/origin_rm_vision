#ifndef SCURM23_AUTOAIM_TRAJECTORYSLOVER_HPP
#define SCURM23_AUTOAIM_TRAJECTORYSLOVER_HPP

//c++
#include <iostream>
#include <vector>

//third part lib


#include <Eigen/Dense>
#define PI 3.1415926535897932384626433832795
namespace rm_auto_aim
{

class TrajectorySlover
{
public:
  //function
  TrajectorySlover(
    int max_iter, float stop_error, int R_K_iter, double bullet_speed, bool is_bigger);
  ~TrajectorySlover() = default;
  double calcPitchCompensate(Eigen::Vector3d & point_world);

  //set bullet speed
  inline void setBulletSpeed(const double & speed) {bullet_speed_ = speed;}
  inline double getBulletSpeed() {return bullet_speed_;}

  inline std::vector<Eigen::Vector3d> getTrajectoryWorld() {return trajectory_world_;}

private:
  std::vector<std::pair<double, double>> trajectory_;
  std::vector<Eigen::Vector3d> trajectory_world_;

private:
  //membership
  //TrajectorySlover param
  int max_iter_{0};
  float stop_error_{0};
  int R_K_iter_{0};
  double bullet_speed_{0};

private:
  const double g = 9.784297539700257;  //defalut
  const double K_SMALL = 0.001903;     //origin //small bullet k = CρS/(2m)
  double k;
  //const double K_SMALL =  0.002403;//test infantry4
  const double K_BIG = 0.005300;  //big bullet
};

}  // namespace rm_auto_aim

#endif  //SCURM23_AUTOAIM_TRAJECTORYSLOVER_HPP
