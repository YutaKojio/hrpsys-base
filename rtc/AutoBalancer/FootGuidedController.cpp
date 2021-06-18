/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "FootGuidedController.h"


void foot_guided_control_base::set_mat()
{
  xi = std::sqrt(g / dz);
  h = 1 + xi * dt;
  h_ = 1 - xi * dt;
  A <<
    1.0, dt,
    xi * xi * dt, 1.0;
  b <<
    0.0,
    -xi * xi * dt;
  Phi <<
    1.0, 1.0 / xi,
    1.0, -1.0 / xi;
  Phi_inv <<
    1.0, 1.0,
    xi, -xi;
  Phi_inv = 2.0 * Phi_inv;
}

// Discrete ver.
// void foot_guided_control_base::calc_u(const std::size_t N, const double ref_dcm, const double ref_zmp)
// {
//   if ( N > 0 ) {
//     double hn = std::pow(h, N);
//     double hgain =  (hn / h) * (1 + h) / ( 1 - hn * hn );
//     w_k = Phi * x_k;
//     double dxsp = ref_dcm - ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset;
//     u_k = ref_zmp - hgain * (hn * xcp - dxsp);
//   } else {
//     u_k = ref_zmp;
//   }
//   truncate_u();
// }

void foot_guided_control_base::calc_u(const std::size_t N, const double ref_dcm, const double ref_zmp, const bool is_double, const double start_ref_zmp, const double goal_ref_zmp, const size_t double_N, const size_t double_whole_N, const double ad_ref_zmp)
{
  bool is_second_ver = true;
  w_k = Phi * x_k;
  act_w_k = Phi * act_x_k;
  double T(N * dt), double_T(double_N * dt);
  T = std::max(2e-3, T); // lower limit, refer ; Bipedal walking control based on capture point dynamics
  // double_T = std::max(50e-3, double_T); // lower limit, refer ; Bipedal walking control based on capture point dynamics
  if (is_second_ver) {
    if (double_whole_N == 0) {
      double dxsp = ref_dcm - ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset;
      u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp) / (1 - std::exp(-2 * xi * T));
      act_u_k = ad_ref_zmp + zmp_filter->passFilter(2 * (act_xcp - std::exp(- xi * T) * dxsp) / (1 - std::exp(-2 * xi * T)));
    } else if (is_double) {
      double dxsp = ref_dcm - goal_ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset, a = (goal_ref_zmp - start_ref_zmp) / (double_whole_N * dt);
      u_k = ref_zmp+ 2 * (xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - 1)) / (1 - std::exp(-2 * xi * T));
      act_u_k = ad_ref_zmp + zmp_filter->passFilter(2 * (act_xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - 1)) / (1 - std::exp(-2 * xi * T)));
    } else {
      double dxsp = ref_dcm - goal_ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset, a = (goal_ref_zmp - start_ref_zmp) / (double_whole_N * dt);
      u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - std::exp(- xi * double_T))) / (1 - std::exp(-2 * xi * T));
      act_u_k = ad_ref_zmp + zmp_filter->passFilter(2 * (act_xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - std::exp(- xi * double_T))) / (1 - std::exp(-2 * xi * T)));
    }
  } else {
    if (is_double) {
      double dxsp = ref_dcm - goal_ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset, a = (goal_ref_zmp - start_ref_zmp) / (double_whole_N * dt);
      u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * double_T) - 1)) / (1 - std::exp(-2 * xi * T));
      act_u_k = ref_zmp + 2 * (act_xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * double_T) - 1)) / (1 - std::exp(-2 * xi * T));
    } else {
      double dxsp = ref_dcm - ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset;
      u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp) / (1 - std::exp(-2 * xi * T));
      act_u_k = ref_zmp + 2 * (act_xcp - std::exp(- xi * T) * dxsp) / (1 - std::exp(-2 * xi * T));
    }
  }
}

void foot_guided_control_base::calc_wheel_u(const double T, const double ref_dcm, const double ref_zmp, const bool is_move, const double start_ref_zmp, const double goal_ref_zmp, const double move_T, const double whole_T) // TODO: only support 1 linear
{
  w_k = Phi * x_k;
  act_w_k = Phi * act_x_k;
  if (is_move) {
    double dxsp = ref_dcm - goal_ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset, a = (goal_ref_zmp - start_ref_zmp) / whole_T;
    u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - 1)) / (1 - std::exp(-2 * xi * T));
    act_u_k = ref_zmp + zmp_filter->passFilter(2 * (act_xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - 1)) / (1 - std::exp(-2 * xi * T)));
  } else {
    double dxsp = ref_dcm - goal_ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, act_xcp = act_w_k(0) - ref_zmp + w_k_offset, a = (goal_ref_zmp - start_ref_zmp) / whole_T;
    u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - std::exp(- xi * move_T))) / (1 - std::exp(-2 * xi * T));
    act_u_k = ref_zmp + zmp_filter->passFilter(2 * (act_xcp - std::exp(- xi * T) * dxsp + a/xi * (std::exp(- xi * T) - std::exp(- xi * move_T))) / (1 - std::exp(-2 * xi * T)));
  }
}

void foot_guided_control_base::truncate_u()
{
  double acc;
  get_acc(acc);
  if (std::abs(acc) > mu * g) {
    acc = mu * g * sgn(acc);
    u_k = x_k(0) - acc / (xi * xi);
  }
}

// assumed after calc_u
void foot_guided_control_base::calc_x_k()
{
  x_k = A * x_k + b * u_k;
}

void foot_guided_control_base::update_control(double& zmp, double& feedforward_zmp, const std::size_t N, const double ref_dcm, const double ref_zmp, const bool is_double, const double start_ref_zmp, const double goal_ref_zmp, const size_t double_N, const size_t double_whole_N, const double ad_ref_zmp)
{
  calc_u(N, ref_dcm, ref_zmp, is_double, start_ref_zmp, goal_ref_zmp, double_N, double_whole_N, ad_ref_zmp);
  zmp = act_u_k;
  feedforward_zmp = u_k;
}

void foot_guided_control_base::update_wheel_control(double& zmp, double& feedforward_zmp, const double T, const double ref_dcm, const double ref_zmp, const bool is_move, const double start_ref_zmp, const double goal_ref_zmp, const double move_T, const double whole_T) // TODO: only support 1 linear
{
  calc_wheel_u(T, ref_dcm, ref_zmp, is_move, start_ref_zmp, goal_ref_zmp, move_T, whole_T);
  zmp = act_u_k;
  feedforward_zmp = u_k;
}

void foot_guided_control_base::update_state(double& pos, const double fx)
{
  calc_x_k();
  dc_off = - fx / (xi * xi * mass); // 反力
  pos = x_k(0) + dc_off;
}

// void foot_guided_control_base::update(double& zmp, double& pos, const std::size_t N, const double ref_dcm, const double ref_zmp)
// {
//   update_control(zmp, N, ref_dcm, ref_zmp);
//   update_state(pos);
// }
