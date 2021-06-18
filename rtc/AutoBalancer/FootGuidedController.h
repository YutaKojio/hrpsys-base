/* -*- coding:utf-8-unix mode:c++ -*- */
#ifndef FOOT_H_
#define FOOT_H_
#include <iostream>
#include <cmath>
#include <hrpUtil/Eigen3d.h>
#include "hrpsys/util/Hrpsys.h"
#include <boost/shared_ptr.hpp>
#include "../TorqueFilter/IIRFilter.h"

static const double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

class foot_guided_control_base
{
private:
  void calc_u(const std::size_t N, const double ref_dcm, const double ref_zmp, const bool is_double, const double start_ref_zmp, const double goal_ref_zmp, const size_t double_N, const size_t double_whole_N, const double ad_ref_zmp);
  void calc_wheel_u(const double T, const double ref_dcm, const double ref_zmp, const bool is_move, const double start_ref_zmp, const double goal_ref_zmp, const double move_T, const double whole_T); // TODO: only support 1 linear
  void truncate_u();
  void calc_x_k();
protected:
  Eigen::Matrix<double, 2, 2> A; // state matrix
  Eigen::Matrix<double, 2, 1> b; // input matrix
  Eigen::Matrix<double, 2, 2> Phi; // convert matrix : pos, vel => dcm, ccm
  Eigen::Matrix<double, 2, 2> Phi_inv; // convert matrix : dcm, ccm => pos, vel
  Eigen::Matrix<double, 2, 1> x_k; // pos, vel
  Eigen::Matrix<double, 2, 1> act_x_k; // pos, vel
  Eigen::Matrix<double, 2, 1> w_k; // dcm, ccm
  Eigen::Matrix<double, 2, 1> act_w_k; // dcm, ccm
  double u_k; // zmp
  double act_u_k; // zmp
  double w_k_offset; // dcm offset
  double mu; // friction coefficient
  double dt, g;
  double dz, xi, h, h_;
  double act_vel_ratio; // how much act_vel is used (0.0 ~ 1.0)
  double dc_off; // offset for disturbance compensation
  double mass;
  boost::shared_ptr<FirstOrderLowPassFilter<double> > zmp_filter;
public:
  // constructor
  foot_guided_control_base() {}
  foot_guided_control_base(const double _dt,  const double _dz, const double _mass, const double _freq,
                           const double _g = DEFAULT_GRAVITATIONAL_ACCELERATION)
    : dt(_dt), dz(_dz), g(_g), mass(_mass), act_vel_ratio(1.0),
      x_k(Eigen::Matrix<double, 2, 1>::Zero()), act_x_k(Eigen::Matrix<double, 2, 1>::Zero()), u_k(0.0), act_u_k(0.0), w_k_offset(0.0), mu(0.5)
  {
    set_mat();
    zmp_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(_freq, _dt, 0.0));
  }
  foot_guided_control_base(const double _dt,  const double _dz, const double init_xk, const double _mass, const double _freq,
                           const double _g = DEFAULT_GRAVITATIONAL_ACCELERATION)
    : dt(_dt), dz(_dz), g(_g), mass(_mass), act_vel_ratio(1.0),
      x_k(Eigen::Matrix<double, 2, 1>::Zero()), act_x_k(Eigen::Matrix<double, 2, 1>::Zero()), u_k(0.0), act_u_k(0.0), w_k_offset(0.0), mu(0.5)
  {
    set_mat();
    act_x_k(0) = x_k(0) = init_xk;
    zmp_filter = boost::shared_ptr<FirstOrderLowPassFilter<double> >(new FirstOrderLowPassFilter<double>(_freq, _dt, 0.0));
  }
  // destructor
  ~foot_guided_control_base() {};
  // update function
  void update_control(double& zmp, double& feedforward_zmp, const std::size_t N, const double ref_dcm, const double ref_zmp, const bool is_double, const double start_ref_zmp, const double goal_ref_zmp, const size_t double_N, const size_t double_whole_N, const double ad_ref_zmp);
  void update_wheel_control(double& zmp, double& feedforward_zmp, const double T, const double ref_dcm, const double ref_zmp, const bool is_move, const double start_ref_zmp, const double goal_ref_zmp, const double move_T, const double whole_T); // TODO: only support 1 linear
  void update_state(double& pos, const double fx);
  void update(double& zmp, double& pos, const std::size_t N, const double ref_dcm, const double ref_zmp);
  // set function
  void set_mat();
  void set_act_x_k(const double pos, const double vel, const bool is_start_or_end_phase)
  {
    act_x_k(0) = pos;
    act_x_k(1) = vel;
  }
  void set_x_k(const double pos, const double vel)
  {
    x_k(0) = pos;
    x_k(1) = vel;
  }
  void set_pos (const double x) { x_k(0) = x; }
  void set_dc_off (const double off) { dc_off = off; }
  void set_zmp (const double act_u, const double u) {
    act_u_k = act_u;
    u_k = u;
  }
  void set_offset (const double offset) { w_k_offset = offset; }
  void set_dz(const double _dz) {
    dz = _dz;
    set_mat();
  };
  void set_act_vel_ratio (const double ratio) { act_vel_ratio = ratio; }
  // get_function
  void get_pos (double& ret) { ret = x_k(0); }
  void get_dc_off (double& ret) { ret = dc_off; }
  void get_vel (double& ret) { ret = x_k(1); }
  void get_acc (double& ret) { ret = xi * xi * ( x_k(0) - u_k ); }
  void get_zmp (double& ret) { ret = u_k; }
  double get_xi() { return xi; }
  double get_dz() { return dz; }
};

template <std::size_t dim>
class foot_guided_controller
{
private:
  foot_guided_control_base *controllers;
protected:
public:
  // constructor
  foot_guided_controller(const double _dt,  const double _dz, const hrp::Vector3& init_xk, const double _mass, const double _freq,
                         const double _g = DEFAULT_GRAVITATIONAL_ACCELERATION)
  {
    controllers = new foot_guided_control_base[dim];
    for (size_t i = 0; i < dim; i++) controllers[i] = foot_guided_control_base(_dt, _dz, init_xk[i], _mass, _freq, _g);
  }
  // destructor
  ~foot_guided_controller()
  {
    delete[] controllers;
  };
  // update function
  void update_control(hrp::Vector3& p_ret, hrp::Vector3& p_ret2, const std::size_t N, const hrp::Vector3& ref_dcm, const hrp::Vector3& ref_zmp, const bool is_double, const hrp::Vector3& start_ref_zmp, const hrp::Vector3& goal_ref_zmp, const std::size_t double_N, const size_t double_whole_N, const hrp::Vector3& ad_ref_zmp)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].update_control(p_ret[i], p_ret2[i], N, ref_dcm[i], ref_zmp[i], is_double, start_ref_zmp[i], goal_ref_zmp[i], double_N, double_whole_N, ad_ref_zmp[i]);
  }
  void update_wheel_control(hrp::Vector3& zmp, hrp::Vector3& feedforward_zmp, const double T, const hrp::Vector3 ref_dcm, const hrp::Vector3 ref_zmp, const bool is_move, const hrp::Vector3 start_ref_zmp, const hrp::Vector3 goal_ref_zmp, const double move_T, const double whole_T) // TODO: only support 1 linear
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].update_wheel_control(zmp(i), feedforward_zmp(i), T, ref_dcm(i), ref_zmp(i), is_move, start_ref_zmp(i), goal_ref_zmp(i), move_T, whole_T);
  }

  void update_state(hrp::Vector3& x_ret, const hrp::Vector3 fx)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].update_state(x_ret[i], fx[i]);
  }
  void update(hrp::Vector3& p_ret, hrp::Vector3& x_ret, const std::size_t N, const hrp::Vector3& ref_dcm, const hrp::Vector3& ref_zmp)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].update(p_ret[i], x_ret[i], N, ref_dcm[i], ref_zmp[i]);
  }
  // set function
  void set_dc_off(const hrp::Vector3& off)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].set_dc_off(off[i]);
  }
  void set_offset(const hrp::Vector3& offset)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].set_offset(offset[i]);
  }
  void set_act_x_k (const hrp::Vector3& pos, const hrp::Vector3& vel, const bool is_start_or_end_phase)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].set_act_x_k(pos(i), vel(i), is_start_or_end_phase);
  }
  void set_x_k (const hrp::Vector3& pos, const hrp::Vector3& vel)
  {
    for (size_t i = 0; i < dim; i++)
      controllers[i].set_x_k(pos(i), vel(i));
  }
  void set_act_vel_ratio (const double ratio) {
    for (size_t i = 0; i < dim; i++) {
      controllers[i].set_act_vel_ratio(ratio);
    }
  }
  void set_zmp (const hrp::Vector3& zmp, const hrp::Vector3& fzmp) {
    for (size_t i = 0; i < dim - 1; i++) {
      controllers[i].set_zmp(zmp(i), fzmp(i));
    }
  }
  // get function
  double get_dz() { return controllers[0].get_dz(); }
  void get_pos(hrp::Vector3& ret) {
    for (size_t i = 0; i < dim; i++)
      controllers[i].get_pos(ret[i]);
  }
  void get_dc_off(hrp::Vector3& ret) {
    for (size_t i = 0; i < dim; i++)
      controllers[i].get_dc_off(ret[i]);
  }
  void get_vel(hrp::Vector3& ret) {
    for (size_t i = 0; i < dim; i++)
      controllers[i].get_vel(ret[i]);
  }
  void get_acc(hrp::Vector3& ret) {
    for (size_t i = 0; i < dim; i++)
      controllers[i].get_acc(ret[i]);
  }
  void set_dz(const double _dz) {
    for (size_t i = 0; i < dim; i++) {
      controllers[i].set_dz(_dz);
    }
  }
};
#endif /*FOOT_H_*/
