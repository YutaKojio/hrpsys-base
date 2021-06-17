/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
#include <numeric>

namespace rats
{
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif
  void cycloid_midpoint (hrp::Vector3& ret,
                         const double ratio, const hrp::Vector3& start,
                         const hrp::Vector3& goal, const double height,
                         const double default_top_ratio)
  {
    hrp::Vector3 u ( goal - start );
    hrp::Vector3 uz (0,0, ratio * u(2));
    u(2) = 0.0;
    double pth = 2 * M_PI * ratio, norm_u = u.norm();
    if ( !eps_eq(norm_u, 0.0,1e-3*0.01) )
      u =  u.normalized();
    /* check ratio vs 0.5 for default_top_ratio blending */
    hrp::Vector3 cycloid_point( ((0.5 > ratio) ? ( 2 * default_top_ratio * norm_u ) : ( 2 * (1 - default_top_ratio) * norm_u )) * ( pth - sin(pth) ) / (2 * M_PI) -
			   ((0.5 > ratio) ? 0.0 : (norm_u * (1 - 2 * default_top_ratio)) ), // local x
			   0, // local y
			   ( 0.5 * height * ( 1 - cos(pth) )) ); // local z
    hrp::Vector3 v(hrp::Vector3(0,0,1).cross(u));
    hrp::Matrix33 dvm;
    dvm << u(0), v(0), 0,
      u(1), v(1), 0,
      u(2), v(2), 1;
    ret = dvm * cycloid_point + start + uz;
  };
  void multi_mid_coords (coordinates& ret, const std::vector<coordinates>& cs, const double eps)
  {
      if (cs.size() == 1) {
          ret = cs.front();
      } else {
          std::vector<coordinates> tmp_mid_coords;
          double ratio = (1.0 - 1.0 / cs.size());
          for (size_t i = 1; i < cs.size(); i++) {
              coordinates tmp;
              mid_coords(tmp, ratio, cs.front(), cs.at(i), eps);
              tmp_mid_coords.push_back(tmp);
          }
          multi_mid_coords(ret, tmp_mid_coords, eps);
      }
      return;
  };

  std::string leg_type_to_leg_type_string (const leg_type l_r)
  {
      return ((l_r==LLEG)?std::string("lleg"):
              (l_r==RARM)?std::string("rarm"):
              (l_r==LARM)?std::string("larm"):
              std::string("rleg"));
  };

  double set_value_according_to_toe_heel_type (const toe_heel_type tht, const double toe_value, const double heel_value, const double default_value)
  {
      if (tht == TOE) {
          return toe_value;
      } else if (tht == HEEL) {
          return heel_value;
      } else {
          return default_value;
      }
  };

  /* member function implementation for refzmp_generator */
  void refzmp_generator::push_refzmp_from_footstep_nodes_for_dual (const std::vector<step_node>& fns,
                                                                   const std::vector<step_node>& _support_leg_steps,
                                                                   const std::vector<step_node>& _swing_leg_steps)
  {
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> dzl;
    hrp::Vector3 ret_zmp;
    hrp::Vector3 tmp_zero = hrp::Vector3::Zero();
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;
    for (std::vector<step_node>::const_iterator it = _support_leg_steps.begin(); it != _support_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
    }
    for (std::vector<step_node>::const_iterator it = _swing_leg_steps.begin(); it != _swing_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
        foot_x_axises.push_back( hrp::Vector3(it->worldcoords.rot * hrp::Vector3::UnitX()) );
    }
    foot_x_axises_list.push_back(foot_x_axises);
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back( rzmp );
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i < fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back( swing_leg_types );
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time/dt));
    toe_heel_types_list.push_back(toe_heel_types(SOLE, SOLE));
    //std::cerr << "double " << (fns[fs_index].l_r==RLEG?LLEG:RLEG) << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

  void refzmp_generator::push_refzmp_from_footstep_nodes_for_single (const std::vector<step_node>& fns, const std::vector<step_node>& _support_leg_steps, const toe_heel_types& tht)
  {
    // support leg = prev fns l_r
    // swing leg = fns l_r
    hrp::Vector3 rzmp, tmp_zero=hrp::Vector3::Zero();
    std::vector<hrp::Vector3> dzl;
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;

    for (std::vector<step_node>::const_iterator it = _support_leg_steps.begin(); it != _support_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
        foot_x_axises.push_back( hrp::Vector3(it->worldcoords.rot * hrp::Vector3::UnitX()) );
    }
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back( rzmp );
    foot_x_axises_list.push_back(foot_x_axises);
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i< fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back( swing_leg_types );
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time/dt));
    toe_heel_types_list.push_back(tht);
    //std::cerr << "single " << fns[fs_index-1].l_r << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

  void refzmp_generator::calc_current_refzmp (hrp::Vector3& ret, std::vector<hrp::Vector3>& swing_foot_zmp_offsets, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const double default_double_support_static_ratio_before, const double default_double_support_static_ratio_after)
  {
    size_t cnt = one_step_count - refzmp_count; // current counter (0 -> one_step_count)
    size_t double_support_count_half_before = default_double_support_ratio_before * one_step_count;
    size_t double_support_count_half_after = default_double_support_ratio_after * one_step_count;
    size_t double_support_static_count_half_before = default_double_support_static_ratio_before * one_step_count;
    size_t double_support_static_count_half_after = default_double_support_static_ratio_after * one_step_count;
    for (size_t i = 0; i < swing_leg_types_list[refzmp_index].size(); i++) {
        swing_foot_zmp_offsets.push_back(default_zmp_offsets[swing_leg_types_list[refzmp_index].at(i)]);
    }
    double zmp_diff = 0.0; // difference between total swing_foot_zmp_offset and default_zmp_offset
    //if (cnt==0) std::cerr << "z " << refzmp_index << " " << refzmp_cur_list.size() << " " << fs_index << " " << (refzmp_index == refzmp_cur_list.size()-2) << " " << is_final_double_support_set << std::endl;

    // Calculate swing foot zmp offset for toe heel zmp transition
    if (use_toe_heel_transition &&
        !(is_start_double_support_phase() || is_end_double_support_phase())) { // Do not use toe heel zmp transition during start and end double support period because there is no swing foot
        double first_zmp_offset_x, second_zmp_offset_x;
        if (use_toe_heel_auto_set) {
            first_zmp_offset_x = set_value_according_to_toe_heel_type(toe_heel_types_list[refzmp_index].src_type, toe_zmp_offset_x, heel_zmp_offset_x, swing_foot_zmp_offsets.front()(0));
            second_zmp_offset_x = set_value_according_to_toe_heel_type(toe_heel_types_list[refzmp_index].dst_type, toe_zmp_offset_x, heel_zmp_offset_x, swing_foot_zmp_offsets.front()(0));
        } else {
            first_zmp_offset_x = toe_zmp_offset_x;
            second_zmp_offset_x = heel_zmp_offset_x;
        }
        if (thp.is_between_phases(cnt, SOLE0)) {
            double ratio = thp.calc_phase_ratio(cnt+1, SOLE0);
            swing_foot_zmp_offsets.front()(0) = (1-ratio)*swing_foot_zmp_offsets.front()(0) + ratio*first_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, HEEL2SOLE, SOLE2)) {
            double ratio = thp.calc_phase_ratio(cnt, HEEL2SOLE, SOLE2);
            swing_foot_zmp_offsets.front()(0) = ratio*swing_foot_zmp_offsets.front()(0) + (1-ratio)*second_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE0, SOLE2TOE)) {
            swing_foot_zmp_offsets.front()(0) = first_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE2HEEL, HEEL2SOLE)) {
            swing_foot_zmp_offsets.front()(0) = second_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE2TOE, SOLE2HEEL)) {
            double ratio = thp.calc_phase_ratio(cnt, SOLE2TOE, SOLE2HEEL);
            swing_foot_zmp_offsets.front()(0) = ratio * second_zmp_offset_x + (1-ratio) * first_zmp_offset_x;
        }
        zmp_diff = swing_foot_zmp_offsets.front()(0)-default_zmp_offsets[swing_leg_types_list[refzmp_index].front()](0);
        if ((is_second_phase() && ( cnt < double_support_count_half_before )) ||
            (is_second_last_phase() && ( cnt > one_step_count - double_support_count_half_after ))) {
            // "* 0.5" is for double supprot period
            zmp_diff *= 0.5;
        }
    }

    // Calculate total reference ZMP
    if (is_start_double_support_phase() || is_end_double_support_phase()) {
      ret = refzmp_cur_list[refzmp_index];
    } else if ( cnt < double_support_static_count_half_before ) { // Start double support static period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
      double ratio = (is_second_phase()?1.0:0.5);
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_static_count_half_after ) { // End double support static period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = (is_second_last_phase()?1.0:0.5);
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else if ( cnt < double_support_count_half_before ) { // Start double support period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
      double ratio = ((is_second_phase()?1.0:0.5) / (double_support_count_half_before-double_support_static_count_half_before)) * (double_support_count_half_before-cnt);
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_count_half_after ) { // End double support period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = ((is_second_last_phase()?1.0:0.5) / (double_support_count_half_after-double_support_static_count_half_after)) * (cnt - 1 - (one_step_count - double_support_count_half_after));
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else {
      ret = refzmp_cur_list[refzmp_index];
    }
  };

  void refzmp_generator::update_refzmp ()
  {
    if ( 1 <= refzmp_count ) {
      refzmp_count--;
    } else {
      refzmp_index++;
      // Check length of step_count_list and refzmp_index
      //   The case if !(refzmp_index <= step_count_list.size()-1) is finalizing of gait_generator.
      //   If finalizing, this can be neglected.
      if (refzmp_index <= step_count_list.size()-1) {
          refzmp_count = one_step_count = step_count_list[refzmp_index];
          thp.set_one_step_count(one_step_count);
      }
      //std::cerr << "fs " << fs_index << "/" << fnl.size() << " rf " << refzmp_index << "/" << refzmp_cur_list.size() << " flg " << std::endl;
    }
  };

  void leg_coords_generator::calc_current_swing_foot_rot (std::map<leg_type, hrp::Vector3>& tmp_swing_foot_rot, const double _default_double_support_ratio_before, const double _default_double_support_ratio_after)
  {
    // interpolation
    int support_len_before = one_step_count * _default_double_support_ratio_before;
    int support_len_after = one_step_count * (_default_double_support_ratio_after + swing_rot_count_ratio);
    int current_swing_count = (one_step_count - lcg_count); // 0->one_step_count
    // swing foot rot interpolator interpolates difference from src to dst.
    if (current_swing_count == support_len_before) {
        for (std::vector<step_node>::iterator it = swing_leg_src_steps.begin(); it != swing_leg_src_steps.end(); it++) {
            swing_foot_rot_interpolator[it->l_r]->clear();
            double tmp[3] = {};
            swing_foot_rot_interpolator[it->l_r]->set(tmp);
        }
        int swing_len = one_step_count - support_len_before - support_len_after;
        for (size_t ii = 0; ii < swing_leg_dst_steps.size(); ii++) {
            leg_type lt = swing_leg_dst_steps[ii].l_r;
            swing_foot_rot_interpolator[lt]->setGoal(hrp::rpyFromRot(swing_leg_src_steps[ii].worldcoords.rot.transpose() * swing_leg_dst_steps[ii].worldcoords.rot).data(),
                                                     dt * swing_len);
            swing_foot_rot_interpolator[lt]->sync();
        }
    } else if ( (current_swing_count > support_len_before) && (current_swing_count < (one_step_count-support_len_after) ) ) {
        int tmp_len = (lcg_count - support_len_after);
        for (size_t ii = 0; ii < swing_leg_dst_steps.size(); ii++) {
            leg_type lt = swing_leg_dst_steps[ii].l_r;
            swing_foot_rot_interpolator[lt]->setGoal(hrp::rpyFromRot(swing_leg_src_steps[ii].worldcoords.rot.transpose() * swing_leg_dst_steps[ii].worldcoords.rot).data(),
                                                     dt * tmp_len);
            swing_foot_rot_interpolator[lt]->sync();
        }
    }
    for (size_t ii = 0; ii < swing_leg_dst_steps.size(); ii++) {
        hrp::Vector3 tmpv;
        if ( !swing_foot_rot_interpolator[swing_leg_dst_steps[ii].l_r]->isEmpty() ) {
            swing_foot_rot_interpolator[swing_leg_dst_steps[ii].l_r]->get(tmpv.data(), true);
        } else {
            if ( (current_swing_count < support_len_before) ) {
                tmpv  = hrp::Vector3::Zero();
            } else if (current_swing_count >= (one_step_count-support_len_after)) {
                tmpv  = hrp::rpyFromRot(swing_leg_src_steps[ii].worldcoords.rot.transpose() * swing_leg_dst_steps[ii].worldcoords.rot);
            }
        }
        tmp_swing_foot_rot.insert(std::pair<leg_type, hrp::Vector3>(swing_leg_dst_steps[ii].l_r, tmpv));
    }
  };

  /* member function implementation for leg_coords_generator */
  void leg_coords_generator::calc_current_swing_leg_steps (std::vector<step_node>& rets, const double step_height, const double _current_toe_angle, const double _current_heel_angle, const double _default_double_support_ratio_before, const double _default_double_support_ratio_after)
  {
    /* match the src step order and the dst step order */
    std::sort(swing_leg_src_steps.begin(), swing_leg_src_steps.end(),
              ((&boost::lambda::_1->* &step_node::l_r) < (&boost::lambda::_2->* &step_node::l_r)));
    std::sort(swing_leg_dst_steps.begin(), swing_leg_dst_steps.end(),
              ((&boost::lambda::_1->* &step_node::l_r) < (&boost::lambda::_2->* &step_node::l_r)));
    std::map<leg_type, hrp::Vector3> tmp_swing_foot_rot;
    calc_current_swing_foot_rot(tmp_swing_foot_rot, _default_double_support_ratio_before, _default_double_support_ratio_after);
    size_t swing_trajectory_generator_idx = 0;
    for (std::vector<step_node>::iterator it1 = swing_leg_src_steps.begin(), it2 = swing_leg_dst_steps.begin(), it3 = current_swing_leg_steps.begin();
         it1 != swing_leg_src_steps.end() && it2 != swing_leg_dst_steps.end() && it3 != current_swing_leg_steps.end();
         it1++, it2++, it3++) {
      coordinates ret;
      ret.rot = it1->worldcoords.rot * hrp::rotFromRpy(tmp_swing_foot_rot[it2->l_r]);
      if (it1->l_r == RLEG && act_contact_states[1] ||
          it1->l_r == LLEG && act_contact_states[0]) {
        touch_ground_count++;
      } else {
        is_touch_ground = false;
        touch_ground_count = 0;
      }
      if (touch_ground_count > static_cast<int>((_default_double_support_ratio_before+_default_double_support_ratio_after)/dt) || !use_act_states) is_touch_ground = true;
      switch (default_orbit_type) {
      case SHUFFLING:
        ret.pos = swing_ratio*it1->worldcoords.pos + (1-swing_ratio)*it2->worldcoords.pos;
        break;
      case CYCLOID:
        cycloid_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case RECTANGLE:
        rectangle_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height, swing_trajectory_generator_idx, it3->worldcoords, it1->l_r);
        break;
      case STAIR:
        stair_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case CYCLOIDDELAY:
        cycloid_delay_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height, swing_trajectory_generator_idx);
        break;
      case CYCLOIDDELAYKICK:
        cycloid_delay_kick_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height);
        break;
      case CROSS:
        cross_delay_midcoords(ret, it1->worldcoords, it2->worldcoords, step_height, it1->l_r);
        break;
      default: break;
      }
      swing_trajectory_generator_idx++;
      // if (std::fabs(step_height) > 1e-3*10) { // TMP : unsupport toe heel
      //     if (swing_leg_src_steps.size() == 1) /* only biped or crawl because there is only one toe_heel_interpolator */
      //         modif_foot_coords_for_toe_heel_phase(ret, _current_toe_angle, _current_heel_angle);
      // }
      rets.push_back(step_node(it1->l_r, ret, 0, 0, 0, 0));
    }
  };

  void leg_coords_generator::calc_ratio_from_double_support_ratio (const double default_double_support_ratio_before, const double default_double_support_ratio_after)
  {
    int support_len_before = one_step_count * default_double_support_ratio_before;
    int support_len_after = next_one_step_count * default_double_support_ratio_after;
    // int support_len = 2*static_cast<int>(one_step_count * default_double_support_ratio * 0.5);
    int swing_len = one_step_count - support_len_before - support_len_after;
    int current_swing_len = lcg_count - support_len_before;
    double tmp_current_swing_time;
    int current_swing_count = (one_step_count - lcg_count); // 0->one_step_count
    if ( current_swing_count < support_len_before ) { // First double support period
      swing_ratio = 0.0;
      tmp_current_swing_time = current_swing_len * dt - swing_len * dt;
      is_swing_phase = false;
    } else if ( current_swing_count >= support_len_before+swing_len ) { // Last double support period
      swing_ratio = 1.0;
      tmp_current_swing_time = current_swing_len * dt + (support_len_before + support_len_after + next_one_step_count) * dt;
      is_swing_phase = false;
    } else {
      tmp_current_swing_time = current_swing_len * dt;
      swing_ratio = static_cast<double>(current_swing_count-support_len_before)/swing_len;
      //std::cerr << "gp " << swing_ratio << " " << swing_rot_ratio << std::endl;
      if (current_step_height > 0.0) is_swing_phase = true;
      else is_swing_phase = false;
    }
    for (std::vector<leg_type>::const_iterator it = support_leg_types.begin(); it != support_leg_types.end(); it++) {
        current_swing_time.at(*it) = (lcg_count + default_double_support_ratio_before * next_one_step_count) * dt;
    }
    for (std::vector<leg_type>::const_iterator it = swing_leg_types.begin(); it != swing_leg_types.end(); it++) {
        if (current_step_height > 0.0) {
            current_swing_time.at(*it) = tmp_current_swing_time;
        } else {
            current_swing_time.at(*it) = (lcg_count + default_double_support_ratio_before * next_one_step_count) * dt;
        }
    }
    //std::cerr << "sl " << support_leg << " " << current_swing_time[support_leg==RLEG?0:1] << " " << current_swing_time[support_leg==RLEG?1:0] << " " << tmp_current_swing_time << " " << lcg_count << std::endl;
  };

  double leg_coords_generator::calc_interpolated_toe_heel_angle (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double start, const double goal)
  {
      double tmp_ip_ratio;
      size_t current_count = one_step_count - lcg_count;
      if (thp.is_phase_starting(current_count, start_phase)) {
          toe_heel_interpolator->clear();
          toe_heel_interpolator->set(&start);
          //toe_heel_interpolator->go(&goal, thp.calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->setGoal(&goal, thp.calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->sync();
      }
      if (!toe_heel_interpolator->isEmpty()) {
          toe_heel_interpolator->get(&tmp_ip_ratio, true);
      } else {
          toe_heel_interpolator->get(&tmp_ip_ratio, false);
      }
      return tmp_ip_ratio;
  };

  void leg_coords_generator::modif_foot_coords_for_toe_heel_phase (coordinates& org_coords, const double _current_toe_angle, const double _current_heel_angle)
  {
      coordinates new_coords;
      size_t current_count = one_step_count - lcg_count;
      double dif_angle = 0.0;
      hrp::Vector3 ee_local_pivot_pos(hrp::Vector3(0,0,0));
      double first_goal_angle, second_goal_angle, first_pos_offset_x, second_pos_offset_x;
      if (use_toe_heel_auto_set) {
          first_goal_angle = set_value_according_to_toe_heel_type(current_src_toe_heel_type, _current_toe_angle, -1 * _current_heel_angle, 0);
          second_goal_angle = set_value_according_to_toe_heel_type(current_dst_toe_heel_type, _current_toe_angle, -1 * _current_heel_angle, 0);
          first_pos_offset_x = set_value_according_to_toe_heel_type(current_src_toe_heel_type, toe_pos_offset_x, heel_pos_offset_x, 0);
          second_pos_offset_x = set_value_according_to_toe_heel_type(current_dst_toe_heel_type, toe_pos_offset_x, heel_pos_offset_x, 0);
      } else {
          first_goal_angle = _current_toe_angle;
          second_goal_angle = -1 * _current_heel_angle;
          first_pos_offset_x = toe_pos_offset_x;
          second_pos_offset_x = heel_pos_offset_x;
      }
      if ( thp.is_between_phases(current_count, SOLE0, SOLE2TOE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE0, SOLE2TOE, 0.0, first_goal_angle);
          ee_local_pivot_pos(0) = first_pos_offset_x;
      } else if ( thp.is_between_phases(current_count, SOLE2HEEL, HEEL2SOLE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE2HEEL, HEEL2SOLE, second_goal_angle, 0.0);
          ee_local_pivot_pos(0) = second_pos_offset_x;
      } else if ( thp.is_between_phases(current_count, SOLE2TOE, SOLE2HEEL) ) {
          // If SOLE1 phase does not exist, interpolate toe => heel smoothly, without 0 velocity phase.
          if ( thp.is_no_SOLE1_phase() ) {
              dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, SOLE2HEEL, first_goal_angle, second_goal_angle);
              double tmpd = (second_goal_angle-first_goal_angle);
              if (std::fabs(tmpd) > 1e-5) {
                  ee_local_pivot_pos(0) = (second_pos_offset_x - first_pos_offset_x) * (dif_angle - first_goal_angle) / tmpd + first_pos_offset_x;
              } else {
                  ee_local_pivot_pos(0) = first_pos_offset_x;
              }
          } else {
              if ( thp.is_between_phases(current_count, SOLE2TOE, TOE2SOLE) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, TOE2SOLE, first_goal_angle, 0.0);
                  ee_local_pivot_pos(0) = first_pos_offset_x;
              } else if ( thp.is_between_phases(current_count, SOLE1, SOLE2HEEL) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE1, SOLE2HEEL, 0.0, second_goal_angle);
                  ee_local_pivot_pos(0) = second_pos_offset_x;
              }
          }
      }
      foot_dif_rot_angle = (dif_angle > 0.0 ? deg2rad(dif_angle) : 0.0);
      if (use_toe_joint && dif_angle > 0.0) dif_angle = 0.0;
      toe_heel_dif_angle = dif_angle;
      Eigen::AngleAxis<double> tmpr(deg2rad(dif_angle), hrp::Vector3::UnitY());
      rotm3times(new_coords.rot, org_coords.rot, tmpr.toRotationMatrix());
      new_coords.pos = org_coords.pos + org_coords.rot * ee_local_pivot_pos - new_coords.rot * ee_local_pivot_pos;
      org_coords = new_coords;
  };

  void leg_coords_generator::cycloid_midcoords (coordinates& ret, const coordinates& start,
                                                                const coordinates& goal, const double height) const
  {
    cycloid_midpoint (ret.pos, swing_ratio, start.pos, goal.pos, height, default_top_ratio);
  };

  void leg_coords_generator::rectangle_midcoords (coordinates& ret, const coordinates& start,
                                                  const coordinates& goal, const double height, const size_t swing_trajectory_generator_idx, const coordinates& current_coords, leg_type lr)
  {
    rdtg[swing_trajectory_generator_idx].set_swing_leg(lr);
    rdtg[swing_trajectory_generator_idx].set_rectangle_trajectory_way_point_offset(rectangle_way_point_offset);
    rdtg[swing_trajectory_generator_idx].is_touch_ground = is_touch_ground;
    rdtg[swing_trajectory_generator_idx].get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height, hrp::Vector3(current_coords.pos));
  };

  void leg_coords_generator::stair_midcoords (coordinates& ret, const coordinates& start,
                                                              const coordinates& goal, const double height)
  {
    sdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_midcoords (coordinates& ret, const coordinates& start,
                                                      const coordinates& goal, const double height, const size_t swing_trajectory_generator_idx)
  {
    cdtg[swing_trajectory_generator_idx].get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_kick_midcoords (coordinates& ret, const coordinates& start,
                                                                      const coordinates& goal, const double height)
  {
    cdktg.set_start_rot(hrp::Matrix33(start.rot));
    cdktg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cross_delay_midcoords (coordinates& ret, const coordinates& start,
                                                    const coordinates& goal, const double height, leg_type lr)
  {
    crdtg.set_swing_leg(lr);
    crdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  bool leg_coords_generator::is_same_footstep_nodes(const std::vector<step_node>& fns_1, const std::vector<step_node>& fns_2) const
  {
      bool matching_flag = true;
      if (fns_1.size() == fns_2.size()) {
          for (std::vector<step_node>::const_iterator it1 = fns_1.begin(); it1 != fns_1.end(); it1++) {
              std::vector<step_node>::const_iterator it2 = std::find_if(fns_2.begin(), fns_2.end(), (&boost::lambda::_1->* &step_node::l_r == it1->l_r));
              if (it2 == fns_2.end()) {
                  matching_flag = false;
                  break;
              }
          }
      } else {
          matching_flag = false;
      }
      return matching_flag;
  };

  void leg_coords_generator::calc_swing_support_mid_coords ()
  {
      std::vector<coordinates> swg_src_coords, swg_dst_coords,sup_coords;
      for (std::vector<step_node>::const_iterator it = swing_leg_src_steps.begin(); it != swing_leg_src_steps.end(); it++) {
          if (it->l_r == RLEG or it->l_r == LLEG) swg_src_coords.push_back(it->worldcoords);
      }
      for (std::vector<step_node>::const_iterator it = swing_leg_dst_steps.begin(); it != swing_leg_dst_steps.end(); it++) {
          if (it->l_r == RLEG or it->l_r == LLEG) swg_dst_coords.push_back(it->worldcoords);
      }
      for (std::vector<step_node>::const_iterator it = support_leg_steps.begin(); it != support_leg_steps.end(); it++) {
          if (it->l_r == RLEG or it->l_r == LLEG) sup_coords.push_back(it->worldcoords);
      }
      coordinates tmp_swg_src_mid, tmp_swg_dst_mid, tmp_swg_mid, tmp_sup_mid;
      const double rot_eps = 1e-5; // eps for mid_rot calculation
      if (swg_src_coords.size() > 0) multi_mid_coords(tmp_swg_src_mid, swg_src_coords, rot_eps);
      if (swg_dst_coords.size() > 0) multi_mid_coords(tmp_swg_dst_mid, swg_dst_coords, rot_eps);
      if (sup_coords.size() > 0) multi_mid_coords(tmp_sup_mid, sup_coords, rot_eps);
      if (lcg_count == one_step_count) {
          foot_midcoords_interpolator->clear();
          double tmp[foot_midcoords_interpolator->dimension()];
          for (size_t ii = 0; ii < 3; ii++) {
              tmp[ii] = tmp_swg_src_mid.pos(ii);
              tmp[ii+3] = 0;
          }
          foot_midcoords_interpolator->set(tmp);
          // set dst
          hrp::Matrix33 difrot(tmp_swg_src_mid.rot.transpose() * tmp_swg_dst_mid.rot);
          hrp::Vector3 tmpr = hrp::rpyFromRot(difrot);
          for (size_t ii = 0; ii < 3; ii++) {
              tmp[ii] = tmp_swg_dst_mid.pos(ii);
              tmp[ii+3] = tmpr(ii);
          }
          foot_midcoords_interpolator->setGoal(tmp, dt*one_step_count, true);
          foot_midcoords_interpolator->sync();
      } else {
          double tmp[foot_midcoords_interpolator->dimension()];
          hrp::Matrix33 difrot(tmp_swg_src_mid.rot.transpose() * tmp_swg_dst_mid.rot);
          hrp::Vector3 tmpr = hrp::rpyFromRot(difrot);
          for (size_t ii = 0; ii < 3; ii++) {
              tmp[ii] = tmp_swg_dst_mid.pos(ii);
              tmp[ii+3] = tmpr(ii);
          }
          foot_midcoords_interpolator->setGoal(tmp, dt*lcg_count, true);
          foot_midcoords_interpolator->sync();
      }
      if (!foot_midcoords_interpolator->isEmpty()) {
          double tmp[foot_midcoords_interpolator->dimension()];
          foot_midcoords_interpolator->get(tmp, true);
          hrp::Vector3 tmpr;
          for (size_t ii = 0; ii < 3; ii++) {
              tmp_swg_mid.pos(ii) = tmp[ii];
              tmpr(ii) = tmp[ii+3];
          }
          tmp_swg_mid.rot = tmp_swg_src_mid.rot * hrp::rotFromRpy(tmpr);
      } else {
          tmp_swg_mid = tmp_swg_dst_mid;
      }
      mid_coords(swing_support_midcoords, static_cast<double>(sup_coords.size()) / (swg_src_coords.size() + sup_coords.size()), tmp_swg_mid, tmp_sup_mid, rot_eps);
  };

  void leg_coords_generator::update_leg_steps (const std::vector< std::vector<step_node> >& fnsl, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const toe_heel_type_checker& thtc)
  {
    // Get current swing coords, support coords, and support leg parameters
    calc_swing_support_params_from_footstep_nodes_list(fnsl);
    current_src_toe_heel_type = thtc.check_toe_heel_type_from_swing_support_coords(swing_leg_src_steps.front().worldcoords, support_leg_steps.front().worldcoords, toe_pos_offset_x, heel_pos_offset_x);
    current_dst_toe_heel_type = thtc.check_toe_heel_type_from_swing_support_coords(swing_leg_dst_steps.front().worldcoords, support_leg_steps.front().worldcoords, toe_pos_offset_x, heel_pos_offset_x);
    calc_swing_support_mid_coords ();

    calc_ratio_from_double_support_ratio(default_double_support_ratio_before, default_double_support_ratio_after);
    swing_leg_steps.clear();
    calc_current_swing_leg_steps(swing_leg_steps, current_step_height, current_toe_angle, current_heel_angle, default_double_support_ratio_before, default_double_support_ratio_after);
    if ( 1 <= lcg_count ) {
      lcg_count--;
      current_swing_leg_steps = swing_leg_steps;
    } else {
      //std::cerr << "gp " << footstep_index << std::endl;
      if (footstep_index < fnsl.size() - 1) {
        footstep_index++;
      }
      if (footstep_index < fnsl.size() - 1) {
        current_step_height = fnsl[footstep_index].front().step_height;
        current_toe_angle = fnsl[footstep_index].front().toe_angle;
        current_heel_angle = fnsl[footstep_index].front().heel_angle;
      } else {
        current_step_height = current_toe_angle = current_heel_angle = 0.0;
      }
      if (footstep_index < fnsl.size()) {
        one_step_count = static_cast<size_t>(fnsl[footstep_index].front().step_time/dt);
        thp.set_one_step_count(one_step_count);
      }
      if (footstep_index + 1 < fnsl.size()) {
        next_one_step_count = static_cast<size_t>(fnsl[footstep_index+1].front().step_time/dt);
      }
      lcg_count = one_step_count;
      switch (default_orbit_type) {
      case RECTANGLE:
          for (size_t i = 0; i < rdtg.size(); i++)
              rdtg[i].reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
          break;
      case STAIR:
          sdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
          break;
      case CYCLOIDDELAY:
          for (size_t i = 0; i < cdtg.size(); i++)
              cdtg[i].reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
          break;
      case CYCLOIDDELAYKICK:
          cdktg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
          break;
      case CROSS:
          crdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
          break;
      default:
          break;
      }
      current_swing_leg_steps =  swing_leg_src_steps;
    }
  };

  /* member function implementation for gait_generator */
  void gait_generator::initialize_gait_parameter (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_refcog,
                                                  const std::vector<step_node>& initial_support_leg_steps,
                                                  const std::vector<step_node>& initial_swing_leg_dst_steps,
                                                  const double delay)
  {
    /* clear all gait_parameter */
    size_t one_step_len = footstep_nodes_list.front().front().step_time / dt;
    finalize_count = 0;
    for (std::vector<step_node>::iterator it_fns = footstep_nodes_list.front().begin(); it_fns != footstep_nodes_list.front().end(); it_fns++) {
        for (std::vector<step_node>::const_iterator it_init = initial_swing_leg_dst_steps.begin(); it_init != initial_swing_leg_dst_steps.end(); it_init++) {
            if (it_fns->l_r == it_init->l_r) {
                /* initial_swing_leg_dst_steps has dummy step_height, step_time, toe_angle and heel_angle. */
                it_fns->worldcoords = it_init->worldcoords;
                break;
            }
        }
    }
    // get initial_foot_mid_coords
    std::vector<coordinates> cv;
    for (size_t i = 0; i < initial_support_leg_steps.size(); i++) {
        cv.push_back(initial_support_leg_steps[i].worldcoords);
    }
    for (size_t i = 0; i < initial_swing_leg_dst_steps.size(); i++) {
        cv.push_back(initial_swing_leg_dst_steps[i].worldcoords);
    }
    multi_mid_coords(initial_foot_mid_coords, cv);
    // rg+lcg initialization
    rg.reset(one_step_len);
    rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list.front(), initial_support_leg_steps, initial_swing_leg_dst_steps);
    if ( preview_controller_ptr != NULL ) {
      delete preview_controller_ptr;
      preview_controller_ptr = NULL;
    }
    //preview_controller_ptr = new preview_dynamics_filter<preview_control>(dt, cur_cog(2) - refzmp_cur_list[0](2), refzmp_cur_list[0]);
    preview_controller_ptr = new preview_dynamics_filter<extended_preview_control>(dt, cur_cog(2) - rg.get_refzmp_cur()(2), rg.get_refzmp_cur(), gravitational_acceleration);
    if ( foot_guided_controller_ptr != NULL ) {
      delete foot_guided_controller_ptr;
      foot_guided_controller_ptr = NULL;
    }
    foot_guided_controller_ptr = new foot_guided_controller<3>(dt, cur_cog(2) - rg.get_refzmp_cur()(2), cur_refcog, total_mass, fg_zmp_cutoff_freq, gravitational_acceleration);
    // foot_guided_controller_ptr->set_act_vel_ratio(act_vel_ratio);
    foot_guided_controller_ptr->set_dc_off(hrp::Vector3::Zero());
    is_first_count = false;
    prev_short_of_zmp = hrp::Vector3::Zero();
    prev_use_roll_flywheel = false;
    prev_use_pitch_flywheel = false;
    prev_act_cp = cur_cog; // assume that robot is stopping when starting walking
    fxy = hrp::Vector3::Zero();
    sum_fx = sum_fy = hrp::Vector3::Zero();
    fx_count = fy_count = 0;
    prev_fg_ref_zmp = rg.get_refzmp_cur();
    // fx_filter->reset(hrp::Vector3::Zero()); // not used for now
    zmp_filter->reset(rg.get_refzmp_cur());
    is_prev_double_remain_one = false;
    // double omega = std::sqrt(gravitational_acceleration / (cur_cog - rg.get_refzmp_cur())(2));
    cp_filter->reset(cur_cog); // assume that robot is stopping when starting walking
    lr_region[0] = lr_region[1] = false;
    is_emergency_touch_wall = false;
    is_stuck = false;
    lcg.reset(one_step_len, footstep_nodes_list.at(1).front().step_time/dt, initial_swing_leg_dst_steps, initial_swing_leg_dst_steps, initial_support_leg_steps, default_double_support_ratio_swing_before, default_double_support_ratio_swing_after);
    if (!double_support_zmp_interpolator->isEmpty()) double_support_zmp_interpolator->clear();
    /* make another */
    lcg.set_swing_support_steps_list(footstep_nodes_list);
    for (size_t i = 1; i < footstep_nodes_list.size()-1; i++) {
        std::vector<step_node> tmp_swing_leg_src_steps;
        lcg.calc_swing_leg_src_steps(tmp_swing_leg_src_steps, footstep_nodes_list, i);
        toe_heel_types tht(thtc.check_toe_heel_type_from_swing_support_coords(tmp_swing_leg_src_steps.front().worldcoords, lcg.get_support_leg_steps_idx(i).front().worldcoords, lcg.get_toe_pos_offset_x(), lcg.get_heel_pos_offset_x()),
                           thtc.check_toe_heel_type_from_swing_support_coords(lcg.get_swing_leg_dst_steps_idx(i).front().worldcoords, lcg.get_support_leg_steps_idx(i).front().worldcoords, lcg.get_toe_pos_offset_x(), lcg.get_heel_pos_offset_x()));
        rg.push_refzmp_from_footstep_nodes_for_single(footstep_nodes_list.at(i), lcg.get_support_leg_steps_idx(i), tht);
    }
    rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list.back(),
                                                lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1),
                                                lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1));
    emergency_flg = IDLING;
  };

  bool gait_generator::proc_one_tick (hrp::Vector3 cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_cmp)
  {
    solved = false;
    if (lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt)) { // for go-velocity
      modified_d_footstep = hrp::Vector3::Zero();
      modified_d_step_time = 0.0;
      updated_vel_footsteps = false;
      is_after_double_support_phase = false;
      was_enlarged_time = false;
    }
    hrp::Vector3 cur_refcog, cur_refcogvel, dc_off;
    foot_guided_controller_ptr->get_pos(cur_refcog);
    foot_guided_controller_ptr->get_vel(cur_refcogvel);
    foot_guided_controller_ptr->get_dc_off(dc_off);
    cur_cog -= dc_off;
    /* update refzmp */
    if (emergency_flg == EMERGENCY_STOP && lcg.get_footstep_index() > 0) {
        leg_type cur_leg = footstep_nodes_list[lcg.get_footstep_index()].front().l_r;
        leg_type first_step = overwritable_footstep_index_offset % 2 == 0 ? cur_leg : (cur_leg == RLEG ? LLEG : RLEG);

        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(first_step, footstep_nodes_list[get_overwritable_index()].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(first_step==RLEG?LLEG:RLEG, footstep_nodes_list[get_overwritable_index() - 1].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_nodes_list.push_back(boost::assign::list_of(step_node(first_step, footstep_nodes_list[get_overwritable_index()].front().worldcoords, 0, default_step_time, 0, 0)));

        overwrite_refzmp_queue(overwrite_footstep_nodes_list, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
        overwrite_footstep_nodes_list.clear();
        emergency_flg = STOPPING;
    } else if ( lcg.get_lcg_count() <= get_overwrite_check_timing() && !updated_vel_footsteps ) {
      if (velocity_mode_flg != VEL_IDLING && lcg.get_footstep_index() > 0) {
        std::vector< std::vector<step_node> > cv;
        calc_next_coords_velocity_mode(cv, get_overwritable_index(),
                                       (overwritable_footstep_index_offset == 0 ? 4 : 3) // Why?
                                       );
        if (velocity_mode_flg == VEL_ENDING) {
          velocity_mode_flg = VEL_IDLING;
          if (is_stable_go_stop_mode) is_emergency_step = true;
        }
        if (overwritable_footstep_index_offset != 0) overwrite_footstep_nodes_list.push_back(footstep_nodes_list[lcg.get_footstep_index()]);
        for (size_t i = 0; i < cv.size(); i++) {
            std::vector<step_node> tmp_fsn;
            for (size_t j = 0; j < cv.at(i).size(); j++) {
                if (overwritable_footstep_index_offset == 0) cv.at(i).at(j).worldcoords.pos += modified_d_footstep;
                tmp_fsn.push_back(step_node(cv.at(i).at(j).l_r, cv.at(i).at(j).worldcoords,
                                            lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle()));
                if (overwritable_footstep_index_offset == 0 && i == 0) tmp_fsn.back().step_time += modified_d_step_time;
            }
            overwrite_footstep_nodes_list.push_back(tmp_fsn);
        }
        overwrite_refzmp_queue(overwrite_footstep_nodes_list, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
        overwrite_footstep_nodes_list.clear();
      } else if ( !overwrite_footstep_nodes_list.empty() && // If overwrite_footstep_node_list exists
                  (lcg.get_footstep_index() < footstep_nodes_list.size()-1) &&  // If overwrite_footstep_node_list is specified and current footstep is not last footstep.
                  get_overwritable_index() == overwrite_footstep_index ) {
        overwrite_refzmp_queue(overwrite_footstep_nodes_list, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
        overwrite_footstep_nodes_list.clear();
      }
      updated_vel_footsteps = true;
    }
    // limit stride
    if (use_stride_limitation && lcg.get_footstep_index() > 0 && lcg.get_footstep_index() < footstep_nodes_list.size()-overwritable_footstep_index_offset-2 &&
        (overwritable_footstep_index_offset == 0 || lcg.get_lcg_count() == get_overwrite_check_timing())) {
      if (lcg.get_footstep_index() == footstep_nodes_list.size()-overwritable_footstep_index_offset-3) {
        hrp::Vector3 orig_footstep_pos = footstep_nodes_list[get_overwritable_index()].front().worldcoords.pos;
        limit_stride(footstep_nodes_list[get_overwritable_index()].front(), footstep_nodes_list[get_overwritable_index()-1].front(), overwritable_stride_limitation);
        for (size_t i = get_overwritable_index() + 1; i < footstep_nodes_list.size(); i++) {
          footstep_nodes_list[i].front().worldcoords.pos -= orig_footstep_pos - footstep_nodes_list[get_overwritable_index()].front().worldcoords.pos;
        }
      } else {
        limit_stride(footstep_nodes_list[get_overwritable_index()].front(), footstep_nodes_list[get_overwritable_index()-1].front(), overwritable_stride_limitation);
      }
      overwrite_footstep_nodes_list.insert(overwrite_footstep_nodes_list.end(), footstep_nodes_list.begin()+get_overwritable_index(), footstep_nodes_list.end());
      overwrite_refzmp_queue(overwrite_footstep_nodes_list);
      overwrite_footstep_nodes_list.clear();
    }

    if (is_preview) {
      if(modify_footsteps) modify_footsteps_for_recovery(); // modify footsteps based on diff_cp
      update_preview_controller(solved);
    } else { // foot guided
      if (lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 1.0) - 1 && lcg.get_footstep_index() > 0) {
        leg_type cur_leg = footstep_nodes_list[lcg.get_footstep_index()].front().l_r;
        lr_region[cur_leg] = false;
        if (lr_region[cur_leg == RLEG ? LLEG : RLEG]) {
          hrp::Matrix33 prev_fs_rot, preprev_fs_rot;
          step_node preprev_fs = (lcg.get_footstep_index()==1 ? lcg.get_swing_leg_src_steps().front() : footstep_nodes_list[lcg.get_footstep_index()-2].front());
          hrp::Vector3 prev_fs_pos = footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos, preprev_fs_pos = preprev_fs.worldcoords.pos;
          hrp::Vector3 ez = hrp::Vector3::UnitZ();
          calc_foot_origin_rot(prev_fs_rot, footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.rot, ez);
          calc_foot_origin_rot(preprev_fs_rot, preprev_fs.worldcoords.rot, ez);
          if (cur_leg == RLEG) {
            stride_limitation_polygon[0] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(-overwritable_stride_limitation[3], -overwritable_stride_limitation[4]-leg_margin[3], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[1] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(-overwritable_stride_limitation[3], -overwritable_stride_limitation[1], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[2] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(overwritable_stride_limitation[0], -overwritable_stride_limitation[1], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[3] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(overwritable_stride_limitation[0], -overwritable_stride_limitation[4]-leg_margin[3], 0.0)) - preprev_fs_pos)).head(2);
          } else {
            stride_limitation_polygon[0] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(-overwritable_stride_limitation[3], overwritable_stride_limitation[1], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[1] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(-overwritable_stride_limitation[3], overwritable_stride_limitation[4]+leg_margin[3], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[2] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(overwritable_stride_limitation[0], overwritable_stride_limitation[4]+leg_margin[3], 0.0)) - preprev_fs_pos)).head(2);
            stride_limitation_polygon[3] = (preprev_fs_rot.transpose() * ((prev_fs_pos + prev_fs_rot * hrp::Vector3(overwritable_stride_limitation[0], overwritable_stride_limitation[1], 0.0)) - preprev_fs_pos)).head(2);
          }
          for (size_t i = 0; i < steppable_region[cur_leg == RLEG ? LLEG : RLEG].size(); i++) {
            bool is_nan = false;
            hrp::Vector2 prev_p = hrp::Vector2::Zero();
            hrp::Vector2 pos_off = hrp::Vector2(0.0, 0.0);
            for (size_t j = 0; j < steppable_region[cur_leg == RLEG ? LLEG : RLEG][i].size(); j++) {
              if (std::isnan(steppable_region[cur_leg == RLEG ? LLEG : RLEG][i][j](0))) is_nan = true;
              if ((prev_p - steppable_region[cur_leg == RLEG ? LLEG : RLEG][i][j]).norm() < 1e-4) is_nan = true;
              prev_p = steppable_region[cur_leg == RLEG ? LLEG : RLEG][i][j];
              steppable_region[cur_leg == RLEG ? LLEG : RLEG][i][j] += pos_off;
            }
            if (is_nan || steppable_region[cur_leg == RLEG ? LLEG : RLEG][i].size() == 0) {
              steppable_region[cur_leg == RLEG ? LLEG : RLEG][i].resize(1);
            } else {
              steppable_region[cur_leg == RLEG ? LLEG : RLEG][i] = calc_intersect_convex(steppable_region[cur_leg == RLEG ? LLEG : RLEG][i], stride_limitation_polygon);
            }
          }
        }

        changed_step_time_stair = false;
      }
      // dc fxy
      hrp::Vector3 prev_fxy = fxy;
      hrp::Vector3 tmp_fxy = hrp::Vector3::Zero();
      double cur_omega = std::sqrt(gravitational_acceleration / (cur_cog - rg.get_refzmp_cur())(2));
      double ref_omega = std::sqrt(gravitational_acceleration / (cur_refcog - rg.get_refzmp_cur())(2));
      act_cp = cur_cog + cur_cogvel / cur_omega + dc_off;
      ref_cp = cur_refcog + cur_refcogvel / ref_omega;
      hrp::Vector3 act_cpvel = (act_cp - prev_act_cp) / dt;
      if (use_act_states && (lcg.get_footstep_index() > 0 && lcg.get_footstep_index() < footstep_nodes_list.size()-2)) {
        tmp_fxy.head(2) = (total_mass * cur_omega * (act_cpvel - cur_omega * (act_cp - refzmp))).head(2);
        if (isfinite(tmp_fxy(0)) && isfinite(tmp_fxy(1))) {
          sum_fx += tmp_fxy;
          sum_fy += tmp_fxy;
          fx_count++;
          fy_count++;
        }
        if (lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 1.0) - 1) { // TODO: cannot completely support turn walking
          hrp::Matrix33 prev_fs_rot;
          hrp::Vector3 ez = hrp::Vector3::UnitZ();
          calc_foot_origin_rot(prev_fs_rot, footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.rot, ez);
          sum_fx = prev_fs_rot.transpose() * sum_fx;
          if (lcg.get_footstep_index() % 2 == 0) {
            sum_fy = prev_fs_rot.transpose() * sum_fy;
            des_fxy = hrp::Vector3(sum_fx(0) / fx_count, sum_fy(1) / fy_count, 0.0);
            fy_count = 0;
            sum_fy = hrp::Vector3::Zero();
          } else {
            des_fxy = prev_fs_rot.transpose() * des_fxy;
            des_fxy = hrp::Vector3(sum_fx(0) / fx_count, des_fxy(1), 0.0);
          }
          fx_count = 0;
          sum_fx = hrp::Vector3::Zero();
          // TODO: tmp_limit
          hrp::Vector3 fx_limit = hrp::Vector3::Zero();
          fx_limit(0) = safe_leg_margin[0] * total_mass * ref_omega * ref_omega;
          fx_limit(1) = safe_leg_margin[2] * total_mass * ref_omega * ref_omega;
          for (size_t i = 0; i < 2; i++) {
            des_fxy(i) = std::min(std::max(des_fxy(i), -fx_limit(i)), fx_limit(i));
          }
          des_fxy = prev_fs_rot * des_fxy;
        }
      } else {
	des_fxy = hrp::Vector3::Zero();
      }
      tmp[15] = tmp_fxy(0);
      tmp[16] = tmp_fxy(1);
      // fx = fx_filter->passFilter(fx);
      // fxy = prev_fxy + dc_gain * (des_fxy - prev_fxy);
      for (size_t i = 0; i < 2; i++) {
        double tmp_gain = dc_gain;
        if (fabs(des_fxy(i)) < fabs(prev_fxy(i))) tmp_gain *= 10;
        fxy(i) = prev_fxy(i) + tmp_gain * (des_fxy(i) - prev_fxy(i));
      }
      if (debug_set_landing_height && lcg.get_footstep_index() == 0) debug_orig_height = footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2);
      if (!solved) update_foot_guided_controller(solved, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
      if (use_act_states && modify_footsteps && (lcg.get_footstep_index() > 0 && lcg.get_footstep_index() < footstep_nodes_list.size()-2)) modify_footsteps_for_foot_guided(cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
      else if (is_emergency_step && lcg.get_footstep_index() == footstep_nodes_list.size()-1) {
        is_emergency_step = false;
        default_step_time = orig_default_step_time;
        default_double_support_ratio_swing_before = orig_default_double_support_static_ratio_before;
        default_double_support_ratio_swing_after = orig_default_double_support_static_ratio_after;
        min_time = orig_min_time;
      }
      {
        // calc landing pos relative to supporting foot for vision
        cur_supporting_foot = (footstep_nodes_list[lcg.get_footstep_index()].front().l_r == LLEG ? 0 : 1);
        hrp::Matrix33 cur_rot;
        hrp::Vector3 ez = hrp::Vector3::UnitZ();
        step_node cur_sup_fs(lcg.get_footstep_index() > 0 ? footstep_nodes_list[lcg.get_footstep_index()-1].front() : lcg.get_support_leg_steps().front());
        calc_foot_origin_rot(cur_rot, cur_sup_fs.worldcoords.rot, ez);
        rel_landing_pos = cur_rot.transpose() * (footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos - cur_sup_fs.worldcoords.pos);
        // calc end cog and cogvel relative to supporting foot for touch wall
        hrp::Vector3 tmp_cog = cur_rot.transpose() * (cur_cog + dc_off - cur_sup_fs.worldcoords.pos);
        hrp::Vector3 tmp_vel = cur_rot.transpose() * cur_cogvel;
        hrp::Vector3 tmp_zmp = cur_rot.transpose() * (refzmp - cur_sup_fs.worldcoords.pos);
        double remain_time = remain_count * dt;
        end_cog = (tmp_cog - tmp_zmp) * std::cosh(cur_omega * remain_time) + tmp_vel / cur_omega * std::sinh(cur_omega * remain_time) + tmp_zmp;
        end_cogvel = (tmp_cog - tmp_zmp) * cur_omega * std::sinh(cur_omega * remain_time) + tmp_vel * std::cosh(cur_omega * remain_time);
        end_cog(2) = (cur_cog - refzmp)(2);
        end_cogvel(2) = 0.0;
      }
      // judge whehter regenerate motion
      if (lcg.get_footstep_index() > 1 &&
          lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 1.0) - 1) { // reset is_stuck
          // minus and x
          for (size_t i = 0; i < 1; i++) {
              if (sum_d_footstep_minus(i) < - sum_d_footstep_thre(i) &&
                  footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(i) < footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i) - footstep_check_delta(i)) {
                  footstep_hist_max(i) = footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i);
                  sum_d_footstep_minus(i) = 0.0;
                  is_stuck = false;
              }
          }
      }
      if (lcg.get_footstep_index() > 5 &&
          lcg.get_lcg_count() >= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * (default_double_support_ratio_after)) &&
          lcg.get_lcg_count() < static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * (1.0 - default_double_support_ratio_before))
          ) {
        for (size_t i = 0; i < 1; i++) {
          // minus
          if (footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i) > footstep_hist_max(i) + footstep_check_delta(i)) {
            footstep_hist_max(i) = footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i);
            sum_d_footstep_minus(i) = 0.0;
            is_stuck = false;
          } else if (footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i) > footstep_hist_max(i)) {
            footstep_hist_max(i) = footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos(i);
          }
          if (sum_d_footstep_minus(i) < - sum_d_footstep_thre(i)
              // || sum_d_footstep_plus(i) > sum_d_footstep_thre(i)
              ) {
            is_stuck = true;
          }
        }
      }

      if (fg_double_remain_count == 1) is_prev_double_remain_one = true;
      else is_prev_double_remain_one = false;
      prev_act_cp = act_cp;
      tmp[17] = fxy(0);
      tmp[18] = fxy(1);
      tmp[19] = dc_off(0);
      tmp[20] = dc_off(1);
    }

    rg.update_refzmp();
    // { // debug
    //   double cart_zmp[3];
    //   preview_controller_ptr->get_cart_zmp(cart_zmp);
    //   std::cerr << "(list " << std::endl;
    //   std::cerr << ":cog "; print_vector(std::cerr, cog);
    //   std::cerr << ":refzmp "; print_vector(std::cerr, refzmp);
    //   std::cerr << ":cart-zmp "; print_vector(std::cerr, cart_zmp, 3);
    //   std::cerr << ")" << std::endl;
    // }

    /* update swing_leg_coords, support_leg_coords */
    if ( solved ) {
      lcg.update_leg_steps(footstep_nodes_list, default_double_support_ratio_swing_before, default_double_support_ratio_swing_after, thtc);
    } else if (finalize_count>0) {
      lcg.clear_interpolators();
    }
    return solved;
  };

  void gait_generator::initialize_wheel_parameter (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_refcog,
                                                 const std::vector<step_node>& initial_support_leg_steps,
                                                 const std::vector<step_node>& initial_swing_leg_dst_steps) {
    wheel_index = 0;
    start_wheel_pos_x = 0.0;
    if (!wheel_interpolator->isEmpty()) wheel_interpolator->clear();
    double tmp = 0.0;
    wheel_interpolator->set(&tmp);
    tmp = 1.0;
    wheel_interpolator->setGoal(&tmp, wheel_nodes_list.at(0).at(1).time, true);

    initial_wheel_midcoords = wheel_nodes_list.at(0).at(0).worldcoords;
    initial_support_leg = initial_support_leg_steps.front();
    initial_swing_leg = initial_swing_leg_dst_steps.front();
    d_wheel_pos = hrp::Vector3::Zero();
    
    foot_guided_controller_ptr = new foot_guided_controller<3>(dt, cur_cog(2) - initial_wheel_midcoords.pos(2), cur_refcog, total_mass, fg_zmp_cutoff_freq, gravitational_acceleration);
    if (!double_support_zmp_interpolator->isEmpty()) double_support_zmp_interpolator->clear();
  }

  bool gait_generator::proc_one_tick_wheel (hrp::Vector3 cur_cog, const hrp::Vector3& cur_cogvel) {
    // update_wheel_index
    {
      if (wheel_interpolator->isEmpty()) {
        if (wheel_index >= wheel_nodes_list.at(0).size() - 2) {
          return false;
        } else {
          wheel_index++;
          double tmp = 0.0;
          wheel_interpolator->set(&tmp);
          tmp = 1.0;
          wheel_interpolator->setGoal(&tmp, wheel_nodes_list.at(0).at(wheel_index).time, true);
          start_wheel_pos_x = cur_wheel_pos_x;
        }
      }
    }

    wheel_interpolator->get(&cur_wheel_ratio);

    // calc_cur_zmp
    {
      wheel_midcoords.pos = (1.0 - cur_wheel_ratio) * wheel_nodes_list.at(0).at(wheel_index).worldcoords.pos + cur_wheel_ratio * wheel_nodes_list.at(0).at(wheel_index + 1).worldcoords.pos;
      wheel_midcoords.rot = wheel_nodes_list.at(0).at(wheel_index).worldcoords.rot;
      d_wheel_pos = wheel_midcoords.pos - initial_wheel_midcoords.pos;
      
    }

    // calc_cur_wheel_angle
    {
      double goal_pos_x = (wheel_nodes_list.at(0).at(wheel_index).worldcoords.rot.transpose() * (wheel_nodes_list.at(0).at(wheel_index + 1).worldcoords.pos - wheel_nodes_list.at(0).at(wheel_index).worldcoords.pos))(0);
      cur_wheel_pos_x = start_wheel_pos_x + cur_wheel_ratio * goal_pos_x;
    }
  }

  void gait_generator::update_preview_controller (bool& solved)
  {
    if ( !solved ) {
      hrp::Vector3 rzmp;
      std::vector<hrp::Vector3> sfzos;
      bool refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
      if (!refzmp_exist_p) {
        finalize_count++;
        rzmp = prev_que_rzmp;
        sfzos = prev_que_sfzos;
      } else {
        prev_que_rzmp = rzmp;
        prev_que_sfzos = sfzos;
      }
      solved = preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offsets, rzmp, sfzos, (refzmp_exist_p || finalize_count < preview_controller_ptr->get_delay()-default_step_time/dt));
    }
  }

  void gait_generator::calc_foot_origin_rot (hrp::Matrix33& foot_rot, const hrp::Matrix33& orig_rot, const hrp::Vector3& n = hrp::Vector3::UnitZ()) const
  {
    hrp::Vector3 en = n / n.norm();
    hrp::Vector3 ex = hrp::Vector3::UnitX();
    hrp::Vector3 xv1(orig_rot * ex);
    xv1 = xv1 - xv1.dot(en) * en;
    xv1.normalize();
    hrp::Vector3 yv1(en.cross(xv1));
    foot_rot(0,0) = xv1(0); foot_rot(1,0) = xv1(1); foot_rot(2,0) = xv1(2);
    foot_rot(0,1) = yv1(0); foot_rot(1,1) = yv1(1); foot_rot(2,1) = yv1(2);
    foot_rot(0,2) = en(0);  foot_rot(1,2) = en(1);  foot_rot(2,2) = en(2);
  }

  void gait_generator::update_foot_guided_controller (bool& solved, const hrp::Vector3& cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_refcog, const hrp::Vector3& cur_refcogvel, const hrp::Vector3& cur_cmp)
  {
    size_t step_num(footstep_nodes_list.size()), step_index(lcg.get_footstep_index());
    // for emergency step
    if (is_emergency_step && step_index < 3) min_time = std::min(orig_min_time, emergency_step_time[step_index]);
    // set param
    solved = true;
    std::vector<step_node> cur_steps(step_index > 0 ? footstep_nodes_list[step_index-1] : lcg.get_support_leg_steps()), dist_steps(footstep_nodes_list[step_index]), next_dist_steps(footstep_nodes_list[step_index+1 >= step_num ? step_index : step_index+1]);
    size_t double_remain_count;
    hrp::Vector3 ref_dcm(hrp::Vector3::Zero()), next_ref_dcm(hrp::Vector3::Zero()), tmp_start_ref_zmp, tmp_goal_ref_zmp;
    bool use_double_support(true), is_second_ver(true);
    bool is_start_or_end_phase(false), is_start_phase(false), is_end2_phase(false), is_end_phase(false);
    double double_support_ratio(default_double_support_ratio_before + default_double_support_ratio_after);
    fg_ref_zmp = hrp::Vector3::Zero();
    remain_count = lcg.get_lcg_count();
    if (lcg.get_lcg_count() == static_cast<size_t>(dist_steps.front().step_time/dt)) fg_step_count = static_cast<size_t>(dist_steps.front().step_time/dt);
    foot_guided_controller_ptr->set_dz(cur_cog(2) - rg.get_refzmp_cur()(2));
    is_set_first_count = false;
    hrp::Vector3 dz = hrp::Vector3(0, 0, foot_guided_controller_ptr->get_dz());
    for (std::vector<step_node>::iterator it = cur_steps.begin(); it != cur_steps.end(); it++) {
      fg_ref_zmp += dz + it->worldcoords.pos + it->worldcoords.rot * rg.get_default_zmp_offset(it->l_r); // ref_zmp has height here
    }
    fg_ref_zmp /= cur_steps.size();
    for (std::vector<step_node>::iterator it = dist_steps.begin(); it != dist_steps.end(); it++) {
      ref_dcm += dz + it->worldcoords.pos + it->worldcoords.rot * rg.get_default_zmp_offset(it->l_r);
    }
    ref_dcm /= dist_steps.size();
    for (std::vector<step_node>::iterator it = next_dist_steps.begin(); it != next_dist_steps.end(); it++) {
      next_ref_dcm += dz + it->worldcoords.pos + it->worldcoords.rot * rg.get_default_zmp_offset(it->l_r);
    }
    next_ref_dcm /= next_dist_steps.size();
    // for start and stop
    if (step_index == 0 ) { // first double support phase
      fg_ref_zmp = (fg_ref_zmp + ref_dcm) / 2.0;
      is_start_phase = is_start_or_end_phase = true;
      is_double_support_phase = false;
      if (is_second_ver) {
        double_support_ratio = default_double_support_ratio_after;
        prev_start_ref_zmp = fg_ref_zmp;
      }
    } else if (step_index == 1) {
      if (!is_second_ver) {
        if (remain_count >= fg_step_count * default_double_support_ratio_after) double_support_ratio = default_double_support_ratio_before;
      } else {
        if (remain_count > fg_step_count * (1 - default_double_support_ratio_before) + 2 - double_remain_count_offset) double_support_ratio = default_double_support_ratio_after;
      }
    } else if (step_index == step_num - 2) { // second to the last double support phase
      ref_dcm = (fg_ref_zmp + ref_dcm) / 2.0;
      is_end2_phase = true;
      if (!is_second_ver) {
        if (remain_count < fg_step_count * default_double_support_ratio_after) double_support_ratio = default_double_support_ratio_after;
      } else {
        if (remain_count < fg_step_count * (1 - default_double_support_ratio_before) + 2 - double_remain_count_offset) double_support_ratio = default_double_support_ratio_after;
      }
    } else if (step_index == step_num - 1) { // last double support phase
      if (is_second_ver) double_support_ratio = 0.0;
      else double_support_ratio = default_double_support_ratio_after;
      fg_ref_zmp = ref_dcm = (fg_ref_zmp + ref_dcm) / 2.0;
      is_start_or_end_phase = true;
      is_end_phase = true;
      remain_count = fg_step_count - finalize_count;
      if (fg_step_count > finalize_count) {
        finalize_count++;
      } else {
        solved = false;
      }
    }
    if (!is_second_ver) fg_goal_ref_zmp = fg_ref_zmp;
    double_remain_count = remain_count;
    if (!is_double_support_phase) prev_start_ref_zmp = fg_ref_zmp;
    if (is_second_ver) fg_start_ref_zmp = prev_start_ref_zmp;

    foot_guided_controller_ptr->set_x_k(cur_refcog, cur_refcogvel);
    if(use_act_states) foot_guided_controller_ptr->set_act_x_k(cur_cog, cur_cogvel, is_start_or_end_phase);
    else foot_guided_controller_ptr->set_act_x_k(cur_refcog, cur_refcogvel, is_start_or_end_phase);

    if (use_double_support) {
      if (!is_start_phase) {
        if (remain_count >= static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_before)) && !is_after_double_support_phase) {
          if (!is_start_or_end_phase) {
            if (is_second_ver) {
              if (std::ceil(static_cast<double>(remain_count) - (fg_step_count * (1 - default_double_support_ratio_before) + 1)) < 0.0) remain_count = (is_end2_phase ? fg_step_count - static_cast<size_t>(fg_step_count*default_double_support_ratio_before) : fg_step_count);
              else remain_count -= static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_before) + 2 - double_remain_count_offset);
              if (remain_count == 0) {
                remain_count = (is_end2_phase ? fg_step_count - static_cast<size_t>(fg_step_count*default_double_support_ratio_before) + 1: fg_step_count + 1);
                is_set_first_count = true;
              }
            } else remain_count -= static_cast<size_t>(fg_step_count * (default_double_support_ratio_before));
            if (static_cast<double>(double_remain_count) - (fg_step_count * (1 - default_double_support_ratio_before) + 2 - double_remain_count_offset) < 0.0) double_remain_count = static_cast<size_t>(fg_step_count - fg_step_count * double_support_ratio) - (is_end2_phase ? static_cast<size_t>(fg_step_count * default_double_support_ratio_before) : double_remain_count_offset - 2);
            else {
              if (!is_start_or_end_phase) double_remain_count -= static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_before) + 2 - double_remain_count_offset);
            }
            if (is_second_ver && double_remain_count == 0) double_remain_count = static_cast<size_t>(fg_step_count - fg_step_count * double_support_ratio) - (is_end2_phase? static_cast<size_t>(fg_step_count * default_double_support_ratio_before) - 2 + double_remain_count_offset: double_remain_count_offset - 4 + double_remain_count_offset);
          } else {
            if (double_remain_count == fg_step_count) {
              if (!is_second_ver) double_remain_count = 0;
            }
          }
        } else if (!is_start_or_end_phase) {
          is_after_double_support_phase = true;
          if (remain_count <= static_cast<size_t>(fg_step_count * default_double_support_ratio_after)) {
            if (remain_count == 0) is_set_first_count = true;
            if (is_second_ver) {
              if (!is_end2_phase) remain_count += static_cast<size_t>(fg_step_count * default_double_support_ratio_after + 1);
              else remain_count += 1;
            } else {
              remain_count += static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_after) + 2);
              if (is_first_double_after) {
                double_remain_count_offset = remain_count - fg_step_count;
                remain_count = 0;
                is_first_double_after = false;
              } else {
                if (is_end2_phase) remain_count += static_cast<size_t>(fg_step_count * default_double_support_ratio_after);
              }
            }
            if (!is_end2_phase) double_remain_count += static_cast<size_t>(fg_step_count * default_double_support_ratio_after + 1);
            else double_remain_count += 1;
            if (is_second_ver && is_first_double_after) {
              is_first_double_after = false;
              double_remain_count = 0;
            } else if (remain_count == 0) double_remain_count = 0;
            if (!is_second_ver) {
              fg_goal_ref_zmp = ref_dcm;
              fg_start_ref_zmp = fg_ref_zmp;
            }
            if (step_index < step_num - 3) ref_dcm = next_ref_dcm;
            else if (step_index == step_num - 3) ref_dcm = (next_ref_dcm + ref_dcm) / 2.0;
          } else {
            double_remain_count = static_cast<size_t>(remain_count - fg_step_count * default_double_support_ratio_after + 2 - double_remain_count_offset);
            if (is_second_ver) {
              if (!is_end2_phase) {
                remain_count += static_cast<size_t>(fg_step_count * default_double_support_ratio_before + 1);
              } else remain_count += 1;
            } else remain_count -= static_cast<size_t>(fg_step_count * default_double_support_ratio_after);
            is_first_double_after = true;
          }
        }
        if (is_start_or_end_phase) remain_count++;
      } else {
        if (remain_count == static_cast<size_t>(fg_step_count * default_double_support_ratio_after)) {
          double_remain_count_offset = remain_count + static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_after) + 2) - fg_step_count;
        }
        if (is_second_ver) remain_count += static_cast<size_t>(fg_step_count * default_double_support_ratio_after + 1);
        else fg_start_ref_zmp = fg_ref_zmp;
      }
      std::vector<hrp::Vector3> sfzos; // unused
      bool refzmp_exist_p = rg.get_current_refzmp(fg_ref_zmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
      fg_ref_zmp += dz;
      if (!refzmp_exist_p) {
        fg_ref_zmp = prev_que_rzmp;
      } else {
        prev_que_rzmp = fg_ref_zmp;
      }

      if (remain_count == 0 && !is_second_ver) ref_dcm = prev_ref_dcm;
      else if ((is_double_support_phase || double_remain_count == 0) && is_second_ver) ref_dcm = prev_ref_dcm;
      if (is_second_ver) {
        fg_goal_ref_zmp = ref_dcm;
        if (!is_double_support_phase) double_remain_count += 1;
      }
    } else {
      remain_count += 1;
    }

    // dcm y offset
    hrp::Vector3 tmp_ref_dcm = ref_dcm;
    if(!(is_start_or_end_phase && !is_start_phase) && !(is_end2_phase && lcg.get_lcg_count() <= static_cast<size_t>(fg_step_count * (1 - default_double_support_ratio_before))))  {
      hrp::Matrix33 cur_step_rot;
      calc_foot_origin_rot(cur_step_rot, cur_steps.front().worldcoords.rot);
      hrp::Vector3 tmp_ref_zmp = cur_step_rot.transpose() * (fg_ref_zmp - cur_steps.front().worldcoords.pos);
      tmp_ref_dcm = cur_step_rot.transpose() * (tmp_ref_dcm - cur_steps.front().worldcoords.pos);
      tmp_ref_dcm(1) += ((tmp_ref_dcm - tmp_ref_zmp)(1) > 0 ? -1 : 1) * dcm_offset;
      tmp_ref_dcm = cur_steps.front().worldcoords.pos + cur_step_rot * tmp_ref_dcm;
    }
    // apply inverse system
    hrp::Vector3 ad_ref_zmp = fg_ref_zmp + zmp_delay_time_const * (fg_ref_zmp - prev_fg_ref_zmp) / dt;
    {
      is_inverse_double_phase = false;
      double double_remain_time = double_remain_count * dt;
      if (double_remain_time <= zmp_delay_time_const && step_index < step_num - 1) {
        if (!is_double_support_phase) {
          if (zmp_delay_time_const+dt >= double_remain_time + (fg_step_count * double_support_ratio * dt)) {
            ad_ref_zmp = fg_goal_ref_zmp;
          } else {
            hrp::Vector3 a = (fg_goal_ref_zmp - fg_start_ref_zmp) / (fg_step_count * double_support_ratio * dt);
            hrp::Vector3 b = fg_start_ref_zmp - a * double_remain_time;
            ad_ref_zmp = a * (zmp_delay_time_const+dt) + b;
            is_inverse_double_phase = true;
          }
        } else {
          ad_ref_zmp = fg_goal_ref_zmp;
        }
      }
      if (is_prev_double_remain_one) {
        if (!is_double_support_phase) ad_ref_zmp = fg_start_ref_zmp;
        else {
          if (zmp_delay_time_const+dt >= (fg_step_count * double_support_ratio * dt)) {
            ad_ref_zmp = fg_goal_ref_zmp;
          } else if (zmp_delay_time_const != 0.0) {
            hrp::Vector3 a = (fg_goal_ref_zmp - fg_start_ref_zmp) / (fg_step_count * double_support_ratio * dt);
            hrp::Vector3 b = fg_start_ref_zmp;
            ad_ref_zmp = a * (zmp_delay_time_const+dt) + b;
            is_inverse_double_phase = true;
          }
        }
      }
    }
    // calc zmp
    hrp::Vector3 feedforward_zmp;
    if (is_end_phase) {
      remain_count = std::max(static_cast<int>(remain_count), static_cast<int>(footguided_balance_time_const/dt));
      double_remain_count = std::max(static_cast<int>(double_remain_count), static_cast<int>(footguided_balance_time_const/dt));
    }
    foot_guided_controller_ptr->update_control(zmp, feedforward_zmp, remain_count, tmp_ref_dcm, fg_ref_zmp, is_double_support_phase, fg_start_ref_zmp, fg_goal_ref_zmp, double_remain_count, fg_step_count * double_support_ratio, ad_ref_zmp);
    // interpolate zmp when double support phase
    if (is_double_support_phase
        // || (is_end_phase && remain_count <= fg_step_count * (default_double_support_ratio_before+default_double_support_ratio_after))
      ) {
      double tmp_ratio = 0.0;
      if (double_support_zmp_interpolator->isEmpty()) {
        double tmp_time = (is_double_support_phase ? fg_step_count * double_support_ratio * dt : remain_count * dt);
        double_support_zmp_interpolator->set(&tmp_ratio);
        tmp_ratio = 1.0;
        double_support_zmp_interpolator->setGoal(&tmp_ratio, tmp_time, true);
      }
      double_support_zmp_interpolator->get(&tmp_ratio, true);
      hrp::Vector3 tmp_zmp = (use_act_states && is_interpolate_zmp_in_double) ? ad_ref_zmp : feedforward_zmp;
      zmp = tmp_ratio * tmp_zmp + (1.0 - tmp_ratio) * zmp;
    } else if (!double_support_zmp_interpolator->isEmpty()) {
      double_support_zmp_interpolator->clear();
    }
    // truncate zmp (assume RLEG, LLEG)
    Eigen::Vector2d tmp_zmp(zmp.head(2)), tmp_fzmp(feedforward_zmp.head(2));
    if (!is_inside_convex_hull(tmp_zmp, hrp::Vector3::Zero(), true)) { // TODO: should consider footstep rot
      zmp.head(2) = tmp_zmp;
    }
    if (!is_inside_convex_hull(tmp_fzmp, hrp::Vector3::Zero(), true)) { // TODO: should consider footstep rot
      feedforward_zmp.head(2) = tmp_fzmp;
    }
    // zmp = zmp_filter->passFilter(zmp);
    foot_guided_controller_ptr->set_zmp(zmp, feedforward_zmp);
    // calc cog
    hrp::Vector3 tmpfxy = hrp::Vector3::Zero();
    if (use_disturbance_compensation) tmpfxy = fxy;
    foot_guided_controller_ptr->update_state(cog, tmpfxy);
    // convert zmp -> refzmp
    refzmp = zmp - dz;
    prev_ref_dcm = ref_dcm;
    prev_fg_ref_zmp = fg_ref_zmp;
    fg_double_remain_count = double_remain_count;
    if (!is_second_ver) {
      if (double_remain_count == 0) is_double_support_phase = !is_double_support_phase;
    } else {
      if ((!is_double_support_phase && double_remain_count == 1) || (is_double_support_phase && double_remain_count == 1)) is_double_support_phase = !is_double_support_phase;
    }

    // for log
    tmp[0] = fg_ref_zmp(0);
    tmp[1] = fg_ref_zmp(1);
    tmp[21] = feedforward_zmp(0);
    tmp[22] = feedforward_zmp(1);
    tmp[2] = tmp_ref_dcm(0);
    tmp[3] = tmp_ref_dcm(1);
    tmp[4] = refzmp(0);
    tmp[5] = refzmp(1);
    tmp[6] = ref_cp(0);
    tmp[7] = ref_cp(1);
    tmp[8] = act_cp(0);
    tmp[9] = act_cp(1);
    tmp[10] = double_remain_count*dt;
    tmp[11] = remain_count*dt;
    tmp[12] = flywheel_tau(0);
    tmp[13] = flywheel_tau(1);
    tmp[14] = falling_direction;
  }

  void gait_generator::set_first_count_flag ()
  {
    if (is_first_count) is_first_count = false;
    if (remain_count == 1 || is_set_first_count) is_first_count = true;
  }

  void gait_generator::limit_stride (step_node& cur_fs, const step_node& prev_fs, const double (&limit)[5]) const
  {
    // limit[5] = {forward, outside, theta, backward, inside}
    leg_type cur_leg = cur_fs.l_r;
    // prev_fs frame
    hrp::Matrix33 prev_fs_rot;
    calc_foot_origin_rot(prev_fs_rot, prev_fs.worldcoords.rot);
    cur_fs.worldcoords.pos = prev_fs_rot.transpose() * (cur_fs.worldcoords.pos - prev_fs.worldcoords.pos);
    double stride_r = std::pow(cur_fs.worldcoords.pos(0), 2.0) + std::pow(cur_fs.worldcoords.pos(1) + footstep_param.leg_default_translate_pos[cur_leg == LLEG ? RLEG : LLEG](1) - footstep_param.leg_default_translate_pos[cur_leg](1), 2.0);
    // front, rear, outside limitation
    double stride_r_limit = std::pow(std::max(limit[cur_fs.worldcoords.pos(0) >= 0 ? 0 : 3], limit[1] - limit[4]), 2.0);
    if (stride_r > stride_r_limit) {
      cur_fs.worldcoords.pos(0) *= sqrt(stride_r_limit / stride_r);
      cur_fs.worldcoords.pos(1) = footstep_param.leg_default_translate_pos[cur_leg](1) - footstep_param.leg_default_translate_pos[cur_leg == LLEG ? RLEG : LLEG](1) +
                                  sqrt(stride_r_limit / stride_r) * (cur_fs.worldcoords.pos(1) + footstep_param.leg_default_translate_pos[cur_leg == LLEG ? RLEG : LLEG](1) - footstep_param.leg_default_translate_pos[cur_leg](1));
    }
    if (cur_fs.worldcoords.pos(0) > limit[0]) cur_fs.worldcoords.pos(0) = limit[0];
    if (cur_fs.worldcoords.pos(0) < -1 * limit[3]) cur_fs.worldcoords.pos(0) = -1 * limit[3];
    if ((cur_leg == LLEG ? 1 : -1) * cur_fs.worldcoords.pos(1) > limit[1]) cur_fs.worldcoords.pos(1) = (cur_leg == LLEG ? 1 : -1) * limit[1];
    // inside limitation
    std::vector<double> cur_leg_vertices_y;
    cur_leg_vertices_y.reserve(4);
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs_rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(leg_margin[0], (cur_leg == LLEG ? 1 : -1) * leg_margin[2], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs_rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(leg_margin[0], (cur_leg == LLEG ? -1 : 1) * leg_margin[3], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs_rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(-1 * leg_margin[1], (cur_leg == LLEG ? 1 : -1) * leg_margin[2], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs_rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(-1 * leg_margin[1], (cur_leg == LLEG ? -1 : 1) * leg_margin[3], 0.0))(1));
    if (cur_leg == LLEG) {
      if (*std::min_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end()) < limit[4]) cur_fs.worldcoords.pos(1) += limit[4] - *std::min_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end());
    } else {
      if (*std::max_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end()) > -1 * limit[4]) cur_fs.worldcoords.pos(1) += -1 * limit[4] - *std::max_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end());
    }
    // world frame
    cur_fs.worldcoords.pos = prev_fs.worldcoords.pos + prev_fs_rot * cur_fs.worldcoords.pos;
  };

  void gait_generator::limit_stride_rectangle (step_node& cur_fs, const step_node& prev_fs, const double (&limit)[5]) const
  {
    // limit[5] = {forward, outside, theta, backward, inside}
    leg_type cur_leg = cur_fs.l_r;
    // prev_fs frame
    hrp::Matrix33 prev_fs_rot;
    calc_foot_origin_rot(prev_fs_rot, prev_fs.worldcoords.rot);
    cur_fs.worldcoords.pos = prev_fs_rot.transpose() * (cur_fs.worldcoords.pos - prev_fs.worldcoords.pos);
    // front, rear, outside limitation
    if (cur_fs.worldcoords.pos(0) > limit[0]) cur_fs.worldcoords.pos(0) = limit[0];
    if (cur_fs.worldcoords.pos(0) < -1 * limit[0]) cur_fs.worldcoords.pos(0) = -1 * limit[3];
    if ((cur_leg == LLEG ? 1 : -1) * cur_fs.worldcoords.pos(1) > limit[1]) cur_fs.worldcoords.pos(1) = (cur_leg == LLEG ? 1 : -1) * limit[1];
    // inside limitation
    std::vector<double> cur_leg_vertices_y;
    cur_leg_vertices_y.reserve(4);
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs.worldcoords.rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(leg_margin[0], (cur_leg == LLEG ? 1 : -1) * leg_margin[2], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs.worldcoords.rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(leg_margin[0], (cur_leg == LLEG ? -1 : 1) * leg_margin[3], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs.worldcoords.rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(-1 * leg_margin[1], (cur_leg == LLEG ? 1 : -1) * leg_margin[2], 0.0))(1));
    cur_leg_vertices_y.push_back((cur_fs.worldcoords.pos + prev_fs.worldcoords.rot.transpose() * cur_fs.worldcoords.rot * hrp::Vector3(-1 * leg_margin[1], (cur_leg == LLEG ? -1 : 1) * leg_margin[3], 0.0))(1));
    if (cur_leg == LLEG) {
      if (*std::min_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end()) < limit[4]) cur_fs.worldcoords.pos(1) += limit[4] - *std::min_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end());
    } else {
      if (*std::max_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end()) > -1 * limit[4]) cur_fs.worldcoords.pos(1) += -1 * limit[4] - *std::max_element(cur_leg_vertices_y.begin(), cur_leg_vertices_y.end());
    }
    // world frame
    cur_fs.worldcoords.pos = prev_fs.worldcoords.pos + prev_fs_rot * cur_fs.worldcoords.pos;
  };

  void gait_generator::limit_stride_vision (step_node& cur_fs, hrp::Vector3& short_of_footstep, const step_node& prev_fs, const step_node& preprev_fs, const double& omega, const hrp::Vector3& cur_cp)
  {
    // preprev foot frame =>
    hrp::Matrix33 preprev_fs_rot, prev_fs_rot;
    hrp::Vector3 prev_fs_pos, preprev_fs_pos = preprev_fs.worldcoords.pos;
    calc_foot_origin_rot(preprev_fs_rot, preprev_fs.worldcoords.rot);
    prev_fs_pos = preprev_fs_rot.transpose() * (prev_fs.worldcoords.pos - preprev_fs_pos);
    // <= preprev foot frame

    // limit stide of second step of stepping up and down on stair
    double stair_margin = 0.02, up_max_stride = 0.01, down_max_stride = 0.05;
    double tmp_forward = overwritable_stride_limitation[0], tmp_rear = overwritable_stride_limitation[3];
    if (prev_fs_pos(2) > lcg.get_default_step_height() - stair_margin) {
      if (prev_fs_pos(0) > 0.0) overwritable_stride_limitation[0] = up_max_stride;
      else overwritable_stride_limitation[3] = up_max_stride;
    } else if (prev_fs_pos(2) < -(lcg.get_default_step_height() - stair_margin)) {
      if (prev_fs_pos(0) > 0.0) overwritable_stride_limitation[0] = down_max_stride;
      else overwritable_stride_limitation[3] = down_max_stride;
    }
    limit_stride_rectangle(cur_fs, prev_fs, overwritable_stride_limitation);
    overwritable_stride_limitation[0] = tmp_forward;
    overwritable_stride_limitation[3] = tmp_rear;

    hrp::Vector3 fspos_for_touch_wall = cur_fs.worldcoords.pos;

    // preprev foot frame
    cur_fs.worldcoords.pos = preprev_fs_rot.transpose() * (cur_fs.worldcoords.pos - preprev_fs_pos);
    calc_foot_origin_rot(prev_fs_rot, prev_fs.worldcoords.rot);
    prev_fs_rot = preprev_fs_rot.transpose() * prev_fs_rot;
    short_of_footstep = hrp::Vector3::Zero();
    hrp::Vector3 rel_cur_cp = preprev_fs_rot.transpose() * (cur_cp - preprev_fs_pos);
    leg_type cur_sup = prev_fs.l_r;

    double limit_r = std::min(safe_leg_margin[2], safe_leg_margin[3]), new_remain_time, tmp_dt = 1e10;
    bool is_out = false, is_read_stepable = false;
    Eigen::Vector2d tmp_pos = Eigen::Vector2d::Zero(), new_pos;
    hrp::Vector3 tmp_short, new_short;
    for (size_t i = 0; i < steppable_region[cur_sup].size(); i++) {
      if (steppable_region[cur_sup][i].size() < 3) continue;
      is_read_stepable = true;
      new_remain_time = remain_count * dt;
      new_pos = cur_fs.worldcoords.pos.head(2);
      new_short = hrp::Vector3::Zero();
      if (!is_inside_convex_hull(new_pos, new_remain_time, new_short, steppable_region[cur_sup][i], hrp::Vector3::Zero(), false, true, limit_r, omega, rel_cur_cp, prev_fs_pos, prev_fs_rot, cur_sup)) {
        if (cur_fs.worldcoords.pos.head(2).dot(new_pos) > cur_fs.worldcoords.pos.head(2).dot(tmp_pos) || !is_out) {
          is_out = true;
          tmp_dt = new_remain_time - remain_count*dt;
          tmp_pos = new_pos;
          tmp_short = new_short;
        }
      } else {
        is_out = false;
        break;
      }
    }
    if (is_out) {
      if (cur_fs.worldcoords.pos(0) > tmp_pos(0)) { // when stride is limited by front edge of steppable region
        cur_fs.worldcoords.pos.head(2) = tmp_pos + front_edge_offset_of_steppable_region;
      } else {
        cur_fs.worldcoords.pos.head(2) = tmp_pos;
      }
      change_step_time(tmp_dt);
      short_of_footstep = tmp_short;
    }

    // world frame
    cur_fs.worldcoords.pos = preprev_fs_pos + preprev_fs_rot * cur_fs.worldcoords.pos;
    short_of_footstep = preprev_fs_rot * short_of_footstep;
    if (!is_read_stepable) cur_fs.worldcoords.pos = preprev_fs_pos;

    // check for touch wall
    // TODO: should consider the case where is_out = true while footstep is inside the limit_stride (like stepping stone)
    if (short_of_footstep.norm() > 5e-2 && (cur_fs.worldcoords.pos - fspos_for_touch_wall).norm() > 1e-2 && lcg.get_footstep_index() > 1) { // 1cm
      is_emergency_touch_wall = true;
    }
  }

  void gait_generator::modify_footsteps_for_recovery ()
  {
    if (isfinite(diff_cp(0)) && isfinite(diff_cp(1))) {
      // calculate diff_cp
      hrp::Vector3 tmp_diff_cp;
      for (size_t i = 0; i < 2; i++) {
        if (std::fabs(diff_cp(i)) > cp_check_margin[i]) {
          is_emergency_walking[i] = true;
          tmp_diff_cp(i) = diff_cp(i) - cp_check_margin[i] * diff_cp(i)/std::fabs(diff_cp(i));
        } else {
          is_emergency_walking[i] = false;
        }
      }
      if (lcg.get_footstep_index() > 0 && lcg.get_footstep_index() < footstep_nodes_list.size()-2) {
        // calculate sum of preview_f
        static double preview_f_sum;
        if (lcg.get_lcg_count() == static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 1.0) - 1) {
          preview_f_sum = preview_controller_ptr->get_preview_f(preview_controller_ptr->get_delay());
          for (size_t i = preview_controller_ptr->get_delay()-1; i >= lcg.get_lcg_count()+1; i--) {
            preview_f_sum += preview_controller_ptr->get_preview_f(i);
          }
          modified_d_footstep = hrp::Vector3::Zero();
        }
        if (lcg.get_lcg_count() <= preview_controller_ptr->get_delay()) {
          preview_f_sum += preview_controller_ptr->get_preview_f(lcg.get_lcg_count());
        }
        // calculate modified footstep position
        double preview_db = 1/6.0 * dt * dt * dt + 1/2.0 * dt * dt * 1/std::sqrt(gravitational_acceleration / (cog(2) - refzmp(2)));
        hrp::Vector3 d_footstep = -1/preview_f_sum * 1/preview_db * footstep_modification_gain * tmp_diff_cp;
        d_footstep(2) = 0.0;
        // overwrite footsteps
        if (lcg.get_lcg_count() <= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 1.0) - 1 &&
            lcg.get_lcg_count() >= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * (default_double_support_ratio_after + margin_time_ratio)) - 1 &&
            !(lcg.get_lcg_count() <= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1 && act_contact_states[0] && act_contact_states[1])) {
          // stride limitation check
          hrp::Vector3 orig_footstep_pos = footstep_nodes_list[get_overwritable_index()].front().worldcoords.pos;
          for (size_t i = 0; i < 2; i++) {
            if (is_emergency_walking[i]) footstep_nodes_list[get_overwritable_index()].front().worldcoords.pos(i) += d_footstep(i);
          }
          limit_stride(footstep_nodes_list[get_overwritable_index()].front(), footstep_nodes_list[get_overwritable_index()-1].front(), overwritable_stride_limitation);
          d_footstep = footstep_nodes_list[get_overwritable_index()].front().worldcoords.pos - orig_footstep_pos;
          for (size_t i = lcg.get_footstep_index()+1; i < footstep_nodes_list.size(); i++) {
            footstep_nodes_list[i].front().worldcoords.pos += d_footstep;
          }
          if (is_emergency_walking[0] || is_emergency_walking[1]) {
            overwrite_footstep_nodes_list.insert(overwrite_footstep_nodes_list.end(), footstep_nodes_list.begin()+lcg.get_footstep_index(), footstep_nodes_list.end());
            // overwrite zmp
            overwrite_refzmp_queue(overwrite_footstep_nodes_list);
            overwrite_footstep_nodes_list.clear();
            modified_d_footstep += d_footstep;
          }
        }
      } else {
        modified_d_footstep = hrp::Vector3::Zero();
      }
    }
  }

  void gait_generator::modify_footsteps_for_foot_guided (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_refcog, const hrp::Vector3& cur_refcogvel, const hrp::Vector3& cur_cmp)
  {
    double omega = std::sqrt(gravitational_acceleration / (cur_cog - rg.get_refzmp_cur())(2));
    bool is_modify = false;
    hrp::Vector3 orig_footstep_pos = footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos;
    hrp::Matrix33 orig_footstep_rot = footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot;
    hrp::Vector3 d_footstep = hrp::Vector3::Zero(), short_of_footstep = hrp::Vector3::Zero();
    hrp::Vector3 cur_footstep_pos = footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.pos;
    hrp::Matrix33 cur_footstep_rot;
    calc_foot_origin_rot(cur_footstep_rot, footstep_nodes_list[lcg.get_footstep_index()-1].front().worldcoords.rot);
    hrp::Matrix33 next_footstep_rot;
    calc_foot_origin_rot(next_footstep_rot, footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot);
    leg_type cur_sup = footstep_nodes_list[lcg.get_footstep_index()-1].front().l_r;
    hrp::Vector3 cur_cp = cp_filter->passFilter(cur_cog + cur_cogvel / omega);
    cur_cp = cur_footstep_rot.transpose() * (cur_cp - cur_footstep_pos);
    hrp::Vector3 next_step_pos =  cur_footstep_rot.transpose() * (orig_footstep_pos - cur_footstep_pos);
    bool is_modify_pahse = false;
    double stride_limitation_with_off[5];
    // assume that zmp_offsts is set to be symmetry
    next_step_pos(0) += rg.get_default_zmp_offset(RLEG)(0);
    next_step_pos(1) += (rg.get_default_zmp_offset(RLEG)(1) + dcm_offset) * (cur_sup == RLEG ? -1 : 1);
    stride_limitation_with_off[0] = overwritable_stride_limitation[0] + rg.get_default_zmp_offset(RLEG)(0);
    stride_limitation_with_off[1] = overwritable_stride_limitation[1] - rg.get_default_zmp_offset(RLEG)(1) - dcm_offset;
    stride_limitation_with_off[3] = overwritable_stride_limitation[3] - rg.get_default_zmp_offset(RLEG)(0);
    stride_limitation_with_off[4] = overwritable_stride_limitation[4] - rg.get_default_zmp_offset(RLEG)(1) - dcm_offset;
    // stride_limitation_with_off[0] = overwritable_stride_limitation[0];
    // stride_limitation_with_off[1] = overwritable_stride_limitation[1];
    // stride_limitation_with_off[3] = overwritable_stride_limitation[3];
    // stride_limitation_with_off[4] = overwritable_stride_limitation[4];

    if (lcg.get_lcg_count() >= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * (default_double_support_ratio_after + margin_time_ratio)) &&
        lcg.get_lcg_count() < static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * (1.0 - default_double_support_ratio_before)) &&
        // !(lcg.get_lcg_count() <= static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1 && act_contact_states[0] && act_contact_states[1])
        !(act_contact_states[0] && act_contact_states[1])
        ) {
      is_modify_pahse = true;
      // step timing extenstion for stair
      if (is_slow_stair_mode && lcg.get_footstep_index() > 1) { // 1st step is not extended
        double stair_margin = 0.02;
        if (!changed_step_time_stair &&
            std::fabs(footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2) - footstep_nodes_list[lcg.get_footstep_index()-2].front().worldcoords.pos(2)) > std::fabs(lcg.get_default_step_height() - stair_margin) &&
          default_step_time + stair_step_time > footstep_nodes_list[lcg.get_footstep_index()].front().step_time) {
          double tmp_dt = default_step_time + stair_step_time - footstep_nodes_list[lcg.get_footstep_index()].front().step_time;
          change_step_time(tmp_dt);
          changed_step_time_stair = true;
        }
      }
      // step timing modification
      {
        double tmp_dt = 0.0;
        bool is_change_time = false;
        double new_remain_time = remain_count * dt;
        {
          double remain_time = remain_count * dt;
          double end_cp_front = std::exp(omega * remain_time) * cur_cp(0) - (std::exp(omega * remain_time) - 1) * safe_leg_margin[0];
          double end_cp_back = std::exp(omega * remain_time) * cur_cp(0) + (std::exp(omega * remain_time) - 1) * safe_leg_margin[1];
          double end_cp_inside = std::exp(omega * remain_time) * cur_cp(1) - (cur_sup == RLEG ? 1 : -1) * (std::exp(omega * remain_time) - 1) * safe_leg_margin[3];
          double xz_max, l_max;
          bool tmp_is_change_time = false;
          // front, back
          if (end_cp_front > stride_limitation_with_off[0]) {
            tmp_is_change_time = true;
            xz_max = safe_leg_margin[0];
            l_max = stride_limitation_with_off[0];
          } else if (end_cp_back < -1 * stride_limitation_with_off[3]) {
            tmp_is_change_time = true;
            xz_max = -1 * safe_leg_margin[1];
            l_max = -1 * stride_limitation_with_off[3];
          }
          if (tmp_is_change_time) new_remain_time = std::min(new_remain_time, (std::log((l_max - xz_max) / (cur_cp(0) - xz_max)) / omega));
          // inside
          if ((cur_sup == RLEG ? 1 : -1) * end_cp_inside > stride_limitation_with_off[1]) {
            xz_max = (cur_sup == RLEG ? 1 : -1) * safe_leg_margin[3];
            l_max = (cur_sup == RLEG ? 1 : -1) * stride_limitation_with_off[1];
            new_remain_time = std::min(new_remain_time, (std::log((l_max - xz_max) / (cur_cp(1) - xz_max)) / omega));
          }
          if (std::isfinite(new_remain_time)) tmp_dt = new_remain_time - remain_count*dt;
          // min time check
          if (tmp_dt < 0) {
            min_time_check(tmp_dt);
            is_change_time = true;
          } else {
            tmp_dt = 0.0;
          }
        }
        // outside check
        {
          double inside_off = stride_limitation_with_off[4] + leg_margin[3];
          double tmp_remain_time = remain_count * dt + tmp_dt;
          double inside_cp = std::exp(omega * tmp_remain_time) * cur_cp(1) - (cur_sup == RLEG ? -1 : 1) * (std::exp(omega * tmp_remain_time) - 1) * safe_leg_margin[2];
          if ((cur_sup == RLEG ? 1 : -1) * inside_cp < inside_off && safe_leg_margin[2] > (cur_sup == RLEG ? -1 : 1) * cur_cp(1)) {
            new_remain_time = std::log((inside_off + safe_leg_margin[2]) / ((cur_sup == RLEG ? 1 : -1) * cur_cp(1) + safe_leg_margin[2])) / omega;
            if (std::isfinite(new_remain_time)) {
              is_change_time = true;
              was_enlarged_time = true;
              tmp_dt = new_remain_time - remain_count*dt;
              if (footstep_nodes_list[lcg.get_footstep_index()].front().step_time + tmp_dt > overwritable_max_time) {
                tmp_dt = overwritable_max_time - footstep_nodes_list[lcg.get_footstep_index()].front().step_time;
              }
            }
          } else if (safe_leg_margin[2] <= (cur_sup == RLEG ? -1 : 1) * cur_cp(1)) {
            tmp_dt = -0.001; // change to min_time
            min_time_check(tmp_dt);
            is_change_time = true;
          } else if (was_enlarged_time) {
            new_remain_time = std::log((inside_off + safe_leg_margin[2]) / ((cur_sup == RLEG ? 1 : -1) * cur_cp(1) + safe_leg_margin[2])) / omega;
            if (std::isfinite(new_remain_time)) {
              tmp_dt = new_remain_time - remain_count*dt;
              if (footstep_nodes_list[lcg.get_footstep_index()].front().step_time + tmp_dt > overwritable_max_time) {
                tmp_dt = overwritable_max_time - footstep_nodes_list[lcg.get_footstep_index()].front().step_time;
              }
            }
            min_time_check(tmp_dt);
            is_change_time = true;
          }
        }
        if (is_change_time) {
          change_step_time(tmp_dt);
        }
      }
      // footstep modification
      {
        double remain_time = remain_count * dt;
        hrp::Vector3 tmp_zmp = - (next_step_pos - std::exp(omega * remain_time) * cur_cp) / (std::exp(omega * remain_time) - 1);
        if (tmp_zmp(0) > safe_leg_margin[0]) { // front
          d_footstep(0) = std::exp(omega * remain_time) * cur_cp(0) - (std::exp(omega * remain_time) - 1) * safe_leg_margin[0] - next_step_pos(0);
          is_modify = true;
        } else if (tmp_zmp(0) < - safe_leg_margin[1]) { // rear
          d_footstep(0) = std::exp(omega * remain_time) * cur_cp(0) - (std::exp(omega * remain_time) - 1) * -1 * safe_leg_margin[1] - next_step_pos(0);
          is_modify = true;
        } else if ((cur_sup == LLEG ? 1 : -1) * tmp_zmp(1) > safe_leg_margin[2]) { // outside
          d_footstep(1) = std::exp(omega * remain_time) * cur_cp(1) - (std::exp(omega * remain_time) - 1) * (cur_sup == LLEG ? 1 : -1) * safe_leg_margin[2] - next_step_pos(1);
          is_modify = true;
        } else if ((cur_sup == LLEG ? -1 : 1) * tmp_zmp(1) > safe_leg_margin[3]) { // inside
          d_footstep(1) = std::exp(omega * remain_time) * cur_cp(1) - (std::exp(omega * remain_time) - 1) * (cur_sup == LLEG ? -1 : 1) * safe_leg_margin[3] - next_step_pos(1);
          is_modify = true;
        }
        // emergency step when standing
        if (is_emergency_step && footstep_nodes_list.size() - lcg.get_footstep_index() == 3 && is_modify
            && d_footstep.norm() > 1e-2
            ) {
          leg_type cur_leg = footstep_nodes_list[lcg.get_footstep_index()].front().l_r;
          footstep_nodes_list.push_back(boost::assign::list_of(step_node(cur_leg==RLEG?LLEG:RLEG, footstep_nodes_list[lcg.get_footstep_index()+1].front().worldcoords, lcg.get_default_step_height(), default_step_time, 0, 0)));
        }
        d_footstep = cur_footstep_rot * d_footstep; // foot coords -> world coords
        d_footstep(2) = 0.0;
        if (is_modify || is_vision_updated || lr_region[cur_sup] || debug_set_landing_height) {
          step_node preprev_fs = (lcg.get_footstep_index()==1 ? lcg.get_swing_leg_src_steps().front() : footstep_nodes_list[lcg.get_footstep_index()-2].front());
          footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos += d_footstep;
          short_of_footstep = d_footstep;
          if (lr_region[cur_sup]) limit_stride_vision(footstep_nodes_list[lcg.get_footstep_index()].front(), short_of_footstep, footstep_nodes_list[lcg.get_footstep_index()-1].front(), preprev_fs, omega, cur_footstep_pos + cur_footstep_rot * cur_cp);
          else limit_stride_rectangle(footstep_nodes_list[lcg.get_footstep_index()].front(), footstep_nodes_list[lcg.get_footstep_index()-1].front(), overwritable_stride_limitation);
          footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2) = orig_footstep_pos(2);
          if (is_vision_updated || debug_set_landing_height) {
            footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2) = (cur_footstep_pos + rel_landing_height)(2);
            calc_foot_origin_rot(footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot, orig_footstep_rot, cur_footstep_rot * rel_landing_normal);
            if (debug_set_landing_height) {
              if (footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(0) > debug_landing_height_xrange[0] && footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(0) < debug_landing_height_xrange[1]) {
                footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2) = debug_orig_height + debug_landing_height;
              } else {
                footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(2) = debug_orig_height;
              }
            }
            // TODO : why is yaw angle changed
            hrp::Vector3 tmp_rpy = hrp::rpyFromRot(footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot);
            tmp_rpy(2) = hrp::rpyFromRot(orig_footstep_rot)(2);
            footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot = hrp::rotFromRpy(tmp_rpy);
          }
          d_footstep = footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos - orig_footstep_pos;
          if (!(lr_region[cur_sup])) short_of_footstep = d_footstep - short_of_footstep;
          for (size_t i = lcg.get_footstep_index()+1; i < footstep_nodes_list.size(); i++) {
            footstep_nodes_list[i].front().worldcoords.pos += d_footstep;
          }
        }
        overwrite_footstep_nodes_list.insert(overwrite_footstep_nodes_list.end(), footstep_nodes_list.begin()+lcg.get_footstep_index(), footstep_nodes_list.end());
        // overwrite zmp
        overwrite_refzmp_queue(overwrite_footstep_nodes_list, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
        overwrite_footstep_nodes_list.clear();
        modified_d_footstep += d_footstep;
        sum_d_footstep_plus += d_footstep;
        sum_d_footstep_minus += d_footstep;
      }
    }
    // calculate angular momentum
    use_roll_flywheel = use_pitch_flywheel = false;
    {
      hrp::Vector3 short_of_zmp = hrp::Vector3::Zero();
      double remain_time = remain_count * dt;
      short_of_footstep = cur_footstep_rot.transpose() * short_of_footstep;
      if (is_modify_pahse) {
        for (size_t i = 0; i < 2; i++) {
          if(std::fabs(short_of_footstep(i)) > 1e-3) { // > 1mm
            if (use_flywheel_balance) {
              if (i == 0) use_pitch_flywheel = true;
              else use_roll_flywheel = true;
            }
            // short_of_zmp(i) = -short_of_footstep(i) / (std::exp(omega * (remain_time - dt)) * omega * dt);
            short_of_zmp(i) = short_of_footstep(i) / (1 - std::exp(omega * remain_time));
          }
        }
        prev_short_of_zmp = short_of_zmp;
        prev_use_roll_flywheel = use_roll_flywheel;
        prev_use_pitch_flywheel = use_pitch_flywheel;
      } else {
        short_of_zmp = prev_short_of_zmp;
        use_roll_flywheel = prev_use_roll_flywheel;
        use_pitch_flywheel = prev_use_pitch_flywheel;
      }
      flywheel_tau = total_mass * gravitational_acceleration * hrp::Vector3(-short_of_zmp(1), short_of_zmp(0), 0);
      // if (use_roll_flywheel || use_pitch_flywheel) std::cerr << "torque :" << flywheel_tau.transpose()<< std::endl;
    }
    // fall down check // TODO: turn walking is not considered
    {
      // next step pos relative
      double remain_time = remain_count * dt;
      double end_cp_front = std::exp(omega * remain_time) * cur_cp(0) - (std::exp(omega * remain_time) - 1) * leg_margin[0];
      double end_cp_back = std::exp(omega * remain_time) * cur_cp(0) + (std::exp(omega * remain_time) - 1) * leg_margin[1];
      double end_cp_outside = std::exp(omega * remain_time) * cur_cp(1) - (cur_sup == RLEG ? -1 : 1) * (std::exp(omega * remain_time) - 1) * leg_margin[2];
      double end_cp_inside = std::exp(omega * remain_time) * cur_cp(1) - (cur_sup == RLEG ? 1 : -1) * (std::exp(omega * remain_time) - 1) * leg_margin[3];
      hrp::Vector3 new_footstep_pos = footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos;
      hrp::Matrix33 new_footstep_rot;
      calc_foot_origin_rot(new_footstep_rot, footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.rot);
      hrp::Vector3 end_cp = new_footstep_pos;
      size_t tmp_falling_direction = 0;
      falling_direction = 0;
      // x axis
      double xz_max, l_max;
      if (end_cp_front > overwritable_stride_limitation[0] + leg_margin[0]) { // front
        tmp_falling_direction = 1;
        xz_max = leg_margin[0];
        l_max = overwritable_stride_limitation[0];
        end_cp(0) = end_cp_front;
      } else if (end_cp_back < -1 * (overwritable_stride_limitation[3] + leg_margin[1])) { // rear
        tmp_falling_direction = 2;
        xz_max = leg_margin[1];
        l_max = overwritable_stride_limitation[3];
        end_cp(0) = end_cp_back;
      }
      end_cp = new_footstep_rot.transpose() * ((cur_footstep_pos + cur_footstep_rot * end_cp) - new_footstep_pos);
      if (tmp_falling_direction == 1 || tmp_falling_direction == 2) {
        if ((tmp_falling_direction == 1 ? 1 : -1) * end_cp(0) > xz_max - l_max / (1 - std::exp(omega * min_time))) {
          falling_direction = tmp_falling_direction;
        }
      }
      // y axis
      end_cp = new_footstep_pos;
      double yz_1, yz_2, l_1, l_2;
      if ((cur_sup == RLEG ? 1 : -1) * end_cp_inside > overwritable_stride_limitation[1] + leg_margin[2]) { // inside
        tmp_falling_direction = 3;
        yz_1 = leg_margin[2];
        yz_2 = leg_margin[3];
        l_1 = overwritable_stride_limitation[4] + leg_margin[3];
        l_2 = -1 * overwritable_stride_limitation[1];
        end_cp(1) = end_cp_inside;
      } else if ((cur_sup == RLEG ? -1 : 1) * end_cp_outside > leg_margin[2]) { // outside
        tmp_falling_direction = 4;
        yz_1 = leg_margin[3];
        yz_1 = leg_margin[2];
        l_1 = -1 * overwritable_stride_limitation[1];
        l_2 = overwritable_stride_limitation[4] + leg_margin[3];
        end_cp(1) = end_cp_outside;
      }
      end_cp = new_footstep_rot.transpose() * ((cur_footstep_pos + cur_footstep_rot * end_cp) - new_footstep_pos);
      if (tmp_falling_direction == 3 || tmp_falling_direction == 4) {
        if(std::fabs(end_cp(1)) > (yz_1 + yz_2 * std::exp(omega * min_time)) / (1 + std::exp(omega * min_time)) + (l_1 + l_2 * std::exp(omega * min_time)) / (1 - std::exp(2 * omega * min_time))) {
          falling_direction = tmp_falling_direction;
        }
      }
    }
  }

  void gait_generator::min_time_check (double& tmp_dt)
  {
    if (footstep_nodes_list[lcg.get_footstep_index()].front().step_time + tmp_dt < min_time) {
      tmp_dt = min_time - footstep_nodes_list[lcg.get_footstep_index()].front().step_time;
    }
    if (remain_count*dt + tmp_dt < min_time_mgn) {
      if (remain_count*dt < min_time_mgn) {
        tmp_dt = 0.0;
      } else {
        tmp_dt = min_time_mgn - remain_count*dt;
      }
    }
  }

  void gait_generator::change_step_time (const double& tmp_dt)
  {
    remain_count += tmp_dt / dt;
    footstep_nodes_list[lcg.get_footstep_index()].front().step_time += tmp_dt;
    size_t orig_lcg_cnt = lcg.get_lcg_count();
    lcg.set_lcg_count(lcg.get_lcg_count() + static_cast<size_t>(tmp_dt/dt));
    lcg.set_one_step_count(lcg.get_lcg_count() - orig_lcg_cnt);
    rg.set_one_step_count(lcg.get_lcg_count() - orig_lcg_cnt);
    rg.set_refzmp_count(lcg.get_lcg_count());
    lcg.reset_one_step_count(lcg.get_lcg_count() - orig_lcg_cnt, default_double_support_ratio_before, default_double_support_ratio_after);
    modified_d_step_time += tmp_dt;
  }

  bool gait_generator::is_intersect (Eigen::Vector2d& r, const Eigen::Vector2d& a0, const Eigen::Vector2d& a1, const Eigen::Vector2d& b0, const Eigen::Vector2d& b1)
  {
    double D =  calcCrossProduct(a1 - a0, b1 - b0);
    if (D == 0.0) return false;
    double t =  calcCrossProduct(b0 - a0, b1 - b0) / D;
    double s = -calcCrossProduct(a0 - b0, a1 - a0) / D;
    r = a0 + t * (a1 - a0);
    return (t >= 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0);
  }

  std::vector<Eigen::Vector2d> gait_generator::calc_intersect_convex (std::vector<Eigen::Vector2d>& P, std::vector<Eigen::Vector2d>& Q)
  {
    const int n = P.size(), m = Q.size();
    int a = 0, b = 0, aa = 0, ba = 0;
    enum { Pin, Qin, Unknown } in = Unknown;
    std::vector<Eigen::Vector2d> R;
    do {
      int a1 = (a+n-1) % n, b1 = (b+m-1) % m;
      double C = calcCrossProduct(P[a] - P[a1], Q[b] - Q[b1]);
      double A = calcCrossProduct(P[a1] - Q[b], P[a] - Q[b]);
      double B = calcCrossProduct(Q[b1] - P[a], Q[b] - P[a]);
      Eigen::Vector2d r;
      if (is_intersect(r, P[a1], P[a], Q[b1], Q[b])) {
        if (in == Unknown) aa = ba = 0;
        R.push_back(r);
        in = B > 0 ? Pin : A > 0 ? Qin : in;
      }
      if (C == 0 && B == 0 && A == 0) {
        if (in == Pin) { b = (b + 1) % m; ++ba; }
        else           { a = (a + 1) % m; ++aa; }
      } else if (C >= 0) {
        if (A > 0) { if (in == Pin) R.push_back(P[a]); a = (a+1)%n; ++aa; }
        else       { if (in == Qin) R.push_back(Q[b]); b = (b+1)%m; ++ba; }
      } else {
        if (B > 0) { if (in == Qin) R.push_back(Q[b]); b = (b+1)%m; ++ba; }
        else       { if (in == Pin) R.push_back(P[a]); a = (a+1)%n; ++aa; }
      }
    } while ( (aa < n || ba < m) && aa < 2*n && ba < 2*m );
    if (in == Unknown) {
      if (is_inside_convex_hull(P[0], Q)) return P;
      if (is_inside_convex_hull(Q[0], P)) return Q;
    }
    return R;
  }

  /* generate vector of step_node from :go-pos params
   *  x, y and theta are simply divided by using stride params
   *  unit system -> x [mm], y [mm], theta [deg]
   */
  bool gait_generator::go_pos_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                                           const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                                           const std::vector<leg_type>& initial_support_legs,
                                                           std::vector< std::vector<step_node> >& new_footstep_nodes_list,
                                                           const bool is_initialize)
  {
    // Get overwrite footstep index
    size_t overwritable_fs_index = 0;
    if (!is_initialize) {
        if (lcg.get_footstep_index() <= get_overwrite_check_timing()) { // ending half
            overwritable_fs_index = get_overwritable_index()+1;
        } else { // starting half
            overwritable_fs_index = get_overwritable_index();
        }
    }
    // Check overwritable_fs_index
    if (overwritable_fs_index > footstep_nodes_list.size()-1) return false;
    go_pos_param_2_footstep_nodes_list_core (goal_x, goal_y, goal_theta,
                                             initial_support_legs_coords, start_ref_coords, initial_support_legs,
                                             new_footstep_nodes_list, is_initialize, overwritable_fs_index);
    //   For Last double support period
    if (is_initialize) {
        clear_footstep_nodes_list();
        footstep_nodes_list = new_footstep_nodes_list;
    } else {
        set_overwrite_foot_steps_list(new_footstep_nodes_list);
        set_overwrite_foot_step_index(overwritable_fs_index);
    }
    print_footstep_nodes_list();
    return true;
  };

  void gait_generator::go_pos_param_2_footstep_nodes_list_core (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                                                const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                                                const std::vector<leg_type>& initial_support_legs,
                                                                std::vector< std::vector<step_node> >& new_footstep_nodes_list,
                                                                const bool is_initialize, const size_t overwritable_fs_index) const
  {
    // Calc goal ref
    coordinates goal_ref_coords;
    if (is_initialize) {
        goal_ref_coords = start_ref_coords;
    } else {
        goal_ref_coords = initial_foot_mid_coords;
        step_node tmpfs = footstep_nodes_list[overwritable_fs_index-1].front();
        start_ref_coords = tmpfs.worldcoords;
        // start_ref_coords.pos += start_ref_coords.rot * hrp::Vector3(-1*footstep_param.leg_default_translate_pos[tmpfs.l_r]);
        start_ref_coords.pos += start_ref_coords.rot * hrp::Vector3(-1*footstep_param.vel_foot_offset[tmpfs.l_r]);
    }
    goal_ref_coords.pos += goal_ref_coords.rot * hrp::Vector3(goal_x, goal_y, 0.0);
    goal_ref_coords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    std::cerr << "start ref coords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << start_ref_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << start_ref_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    std::cerr << "goal ref midcoords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << goal_ref_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << goal_ref_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

    /* initialize */
    if (is_initialize) {
        // For initial double support period
        std::vector<step_node> initial_footstep_nodes;
        for (size_t i = 0; i < initial_support_legs.size(); i++) {
            initial_footstep_nodes.push_back(step_node(initial_support_legs.at(i), initial_support_legs_coords.at(i), 0, default_step_time, 0, 0));
        }
        new_footstep_nodes_list.push_back(initial_footstep_nodes);
    } else {
        new_footstep_nodes_list.push_back(footstep_nodes_list[overwritable_fs_index]);
    }

    /* footstep generation loop */
    hrp::Vector3 dp, dr;
    start_ref_coords.difference(dp, dr, goal_ref_coords);
    dp = start_ref_coords.rot.transpose() * dp;
    dr = start_ref_coords.rot.transpose() * dr;
    while ( !(eps_eq(std::sqrt(dp(0)*dp(0)+dp(1)*dp(1)), 0.0, 1e-3*0.1) && eps_eq(dr(2), 0.0, deg2rad(0.5))) ) {
      velocity_mode_parameter cur_vel_param;
      cur_vel_param.set(dp(0)/default_step_time, dp(1)/default_step_time, rad2deg(dr(2))/default_step_time);
      append_footstep_list_velocity_mode(new_footstep_nodes_list, cur_vel_param);
      start_ref_coords = new_footstep_nodes_list.back().front().worldcoords;
      // start_ref_coords.pos += start_ref_coords.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[new_footstep_nodes_list.back().front().l_r] * -1.0);
      start_ref_coords.pos += start_ref_coords.rot * hrp::Vector3(footstep_param.vel_foot_offset[new_footstep_nodes_list.back().front().l_r] * -1.0);
      start_ref_coords.difference(dp, dr, goal_ref_coords);
      dp = start_ref_coords.rot.transpose() * dp;
      dr = start_ref_coords.rot.transpose() * dr;
    }
    for (size_t i = 0; i < optional_go_pos_finalize_footstep_num; i++) {
        append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(new_footstep_nodes_list.back(), all_limbs), new_footstep_nodes_list);
    }

    /* finalize */
    //   Align last foot
    append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(new_footstep_nodes_list.back(), all_limbs), new_footstep_nodes_list);
    //   Check align
    coordinates final_step_coords1 = new_footstep_nodes_list[new_footstep_nodes_list.size()-2].front().worldcoords; // Final coords in footstep_node_list
    coordinates final_step_coords2 = start_ref_coords; // Final coords calculated from start_ref_coords + translate pos
    // final_step_coords2.pos += final_step_coords2.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[new_footstep_nodes_list[new_footstep_nodes_list.size()-2].front().l_r]);
    final_step_coords2.pos += final_step_coords2.rot * hrp::Vector3(footstep_param.vel_foot_offset[new_footstep_nodes_list[new_footstep_nodes_list.size()-2].front().l_r]);
    final_step_coords1.difference(dp, dr, final_step_coords2);
    if ( !(eps_eq(dp.norm(), 0.0, 1e-3*0.1) && eps_eq(dr.norm(), 0.0, deg2rad(0.5))) ) { // If final_step_coords1 != final_step_coords2, add steps to match final_step_coords1 and final_step_coords2
        append_go_pos_step_nodes(start_ref_coords, calc_counter_leg_types_from_footstep_nodes(new_footstep_nodes_list.back(), all_limbs), new_footstep_nodes_list);
    }
    //   For Last double support period
    if (is_initialize) {
        append_finalize_footstep(new_footstep_nodes_list);
    }
    return;
  };

  void gait_generator::go_single_step_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_z, const double goal_theta,
                                                             const std::string& tmp_swing_leg,
                                                             const coordinates& _support_leg_coords)
  {
    leg_type _swing_leg = (tmp_swing_leg == "rleg") ? RLEG : LLEG;
    step_node sn0((_swing_leg == RLEG) ? LLEG : RLEG, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    footstep_nodes_list.push_back(boost::assign::list_of(sn0));
    step_node sn1(_swing_leg, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    // hrp::Vector3 trs(2.0 * footstep_param.leg_default_translate_pos[_swing_leg] + hrp::Vector3(goal_x, goal_y, goal_z));
    hrp::Vector3 trs(2.0 * footstep_param.vel_foot_offset[_swing_leg] + hrp::Vector3(goal_x, goal_y, goal_z));
    sn1.worldcoords.pos += sn1.worldcoords.rot * trs;
    sn1.worldcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    footstep_nodes_list.push_back(boost::assign::list_of(sn1));
    footstep_nodes_list.push_back(boost::assign::list_of(sn0));
  };

  bool gait_generator::go_wheel_param_2_wheel_nodes_list (const double goal_x, const double whole_time, const coordinates& start_ref_coords) {
    wheel_nodes_list.clear();
    std::vector<wheel_node> tmp_wheel(2); // 3段階くらいに分けてもいい

    tmp_wheel.at(0) = wheel_node(start_ref_coords, 0); // 0番目は始点

    coordinates abs_goal_coord = start_ref_coords;
    abs_goal_coord.pos += start_ref_coords.rot * hrp::Vector3(goal_x, 0.0, 0.0);
    tmp_wheel.at(1) = wheel_node(abs_goal_coord, whole_time);
    wheel_nodes_list.push_back(tmp_wheel);

    return true;
  }

  void gait_generator::initialize_velocity_mode (const coordinates& _ref_coords,
						 const double vel_x, const double vel_y, const double vel_theta,
                                                 const std::vector<leg_type>& current_legs)
  {
    if (!is_emergency_step) {
      velocity_mode_flg = VEL_DOING;
    }
    orig_default_step_time = default_step_time;
    orig_default_double_support_static_ratio_before = default_double_support_ratio_before;
    orig_default_double_support_static_ratio_after = default_double_support_ratio_after;
    orig_min_time = min_time;
    /* initialize */
    clear_footstep_nodes_list();
    set_velocity_param (vel_x, vel_y, vel_theta);
    if (is_emergency_step) default_step_time = emergency_step_time[0];
    append_go_pos_step_nodes(_ref_coords, current_legs);
    if (is_emergency_step) default_step_time = emergency_step_time[1];
    append_footstep_list_velocity_mode();
    if (is_emergency_step) default_step_time = emergency_step_time[2];
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
  };

  void gait_generator::finalize_velocity_mode ()
  {
    if (velocity_mode_flg == VEL_DOING) velocity_mode_flg = VEL_ENDING;
  };

  void gait_generator::finalize_velocity_mode2 ()
  {
    if (velocity_mode_flg == VEL_DOING) velocity_mode_flg = VEL_IDLING;
  };

  void gait_generator::calc_ref_coords_trans_vector_velocity_mode (coordinates& ref_coords, hrp::Vector3& trans, double& dth, const std::vector<step_node>& sup_fns, const velocity_mode_parameter& cur_vel_param) const
  {
    ref_coords = sup_fns.front().worldcoords;
    // hrp::Vector3 tmpv(footstep_param.leg_default_translate_pos[sup_fns.front().l_r] * -1.0); /* not fair to every support legs */
    hrp::Vector3 tmpv(footstep_param.vel_foot_offset[sup_fns.front().l_r] * -1.0); /* not fair to every support legs */
    ref_coords.pos += ref_coords.rot * tmpv;
    double dx = cur_vel_param.velocity_x + offset_vel_param.velocity_x, dy = cur_vel_param.velocity_y + offset_vel_param.velocity_y;
    dth = cur_vel_param.velocity_theta + offset_vel_param.velocity_theta;
    //std::cerr << "Before limit dx " << dx << " dy " << dy << " dth " << dth << std::endl;
    /* velocity limitation by stride parameters <- this should be based on footstep candidates */
    if (default_stride_limitation_type == SQUARE) {
      dth = std::max(-1 * footstep_param.stride_outside_theta / default_step_time, std::min(footstep_param.stride_outside_theta / default_step_time, dth));
    } else if (default_stride_limitation_type == CIRCLE) {
      dth = std::max(-1 * stride_limitation_for_circle_type[2] / default_step_time, std::min(stride_limitation_for_circle_type[2] / default_step_time, dth));
    }
    if (default_stride_limitation_type == SQUARE) {
      dx  = std::max(-1 * footstep_param.stride_bwd_x / default_step_time, std::min(footstep_param.stride_fwd_x / default_step_time, dx ));
      dy  = std::max(-1 * footstep_param.stride_outside_y     / default_step_time, std::min(footstep_param.stride_outside_y     / default_step_time, dy ));
      /* inside step limitation */
      if (use_inside_step_limitation) {
        if (dy > 0) {
            // If dy>0 (== leftward step) and LLEG/LARM support, do inside limitation
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == LLEG || &boost::lambda::_1->* &step_node::l_r == LARM)) > 0) {
                dy  = std::min(footstep_param.stride_inside_y     / default_step_time, dy);
            }
        } else {
            // If dy<=0 (== rightward step) and RLEG/RARM support, do inside limitation
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == RLEG || &boost::lambda::_1->* &step_node::l_r == RARM)) > 0) {
                dy  = std::max(-1 * footstep_param.stride_inside_y     / default_step_time, dy);
            }
        }
        if (dth > 0) {
            // If dth>0 (== leftward turn step) and LLEG/LARM support, do inside limitation
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == LLEG || &boost::lambda::_1->* &step_node::l_r == LARM)) > 0) {
                dth = std::min(footstep_param.stride_inside_theta / default_step_time, dth);
            }
        } else {
            // If dth<=0 (== rightward turn step) and RLEG/RARM support, do inside limitation
            if (std::count_if(sup_fns.begin(), sup_fns.end(), (&boost::lambda::_1->* &step_node::l_r == RLEG || &boost::lambda::_1->* &step_node::l_r == RARM)) > 0) {
                dth = std::max(-1 * footstep_param.stride_inside_theta / default_step_time, dth);
            }
        }
      }
    }
    //std::cerr << "After Limit dx " << dx << " dy " << dy << " dth " << dth << std::endl;
    trans = hrp::Vector3(dx * default_step_time, dy * default_step_time, 0);
    dth = deg2rad(dth * default_step_time);
  };

  void gait_generator::append_footstep_list_velocity_mode ()
  {
      append_footstep_list_velocity_mode(footstep_nodes_list, vel_param);
  };

  void gait_generator::append_footstep_list_velocity_mode (std::vector< std::vector<step_node> >& _footstep_nodes_list, const velocity_mode_parameter& cur_vel_param) const
  {
    coordinates ref_coords;
    hrp::Vector3 trans;
    double dth;
    calc_ref_coords_trans_vector_velocity_mode(ref_coords, trans, dth, _footstep_nodes_list.back(), cur_vel_param);

    ref_coords.pos += ref_coords.rot * trans;
    ref_coords.rotate(dth, hrp::Vector3(0,0,1));
    append_go_pos_step_nodes(ref_coords, calc_counter_leg_types_from_footstep_nodes(_footstep_nodes_list.back(), all_limbs), _footstep_nodes_list);
    if (default_stride_limitation_type == CIRCLE) limit_stride(_footstep_nodes_list[_footstep_nodes_list.size()-1].front(), _footstep_nodes_list[_footstep_nodes_list.size()-2].front(), stride_limitation_for_circle_type);
  };

  void gait_generator::calc_next_coords_velocity_mode (std::vector< std::vector<step_node> >& ret_list, const size_t idx, const size_t future_step_num)
  {
    coordinates ref_coords;
    hrp::Vector3 trans;
    double dth;
    calc_ref_coords_trans_vector_velocity_mode(ref_coords, trans, dth, footstep_nodes_list[idx-1], vel_param);

    std::vector<leg_type> cur_sup_legs, next_sup_legs;
    for (size_t i = 0; i < footstep_nodes_list[idx-1].size(); i++) cur_sup_legs.push_back(footstep_nodes_list[idx-1].at(i).l_r);
    next_sup_legs = calc_counter_leg_types_from_footstep_nodes(footstep_nodes_list[idx-1], all_limbs);

    for (size_t i = 0; i < future_step_num; i++) {
      std::vector<step_node> ret;
      std::vector<leg_type> forcused_sup_legs;
      switch( i % 2) {
      case 0: forcused_sup_legs = next_sup_legs; break;
      case 1: forcused_sup_legs = cur_sup_legs; break;
      }
      if ( velocity_mode_flg != VEL_ENDING ) {
          ref_coords.pos += ref_coords.rot * trans;
          ref_coords.rotate(dth, hrp::Vector3(0,0,1));
      }
      for (size_t j = 0; j < forcused_sup_legs.size(); j++) {
          ret.push_back(step_node(forcused_sup_legs.at(j), ref_coords, 0, 0, 0, 0));
          // ret[j].worldcoords.pos += ret[j].worldcoords.rot * footstep_param.leg_default_translate_pos[forcused_sup_legs.at(j)];
          ret[j].worldcoords.pos += ret[j].worldcoords.rot * footstep_param.vel_foot_offset[forcused_sup_legs.at(j)];
          if (default_stride_limitation_type == CIRCLE) limit_stride(ret[j], (i == 0 ? footstep_nodes_list[idx-1].at(j) : ret_list[i -1].at(j)), stride_limitation_for_circle_type);
      }
      ret_list.push_back(ret);
    }
  };

  void gait_generator::overwrite_refzmp_queue(const std::vector< std::vector<step_node> >& fnsl, const hrp::Vector3& cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_refcog, const hrp::Vector3& cur_refcogvel, const hrp::Vector3& cur_cmp, const bool& update_vel)
  {
    size_t idx = (update_vel ? get_overwritable_index() : lcg.get_footstep_index());
    footstep_nodes_list.erase(footstep_nodes_list.begin()+idx, footstep_nodes_list.end());

    /* add new next steps ;; the number of next steps is fnsl.size() */
    footstep_nodes_list.insert(footstep_nodes_list.end(), fnsl.begin(), fnsl.end());

    /* Update lcg */
    lcg.set_swing_support_steps_list(footstep_nodes_list);

    /* Update refzmp_generator */
    /*   Remove refzmp after idx for allocation of new refzmp by push_refzmp_from_footstep_nodes */
    rg.remove_refzmp_cur_list_over_length(idx);
    /*   reset index and counter */
    rg.set_indices(idx);
    if (overwritable_footstep_index_offset == 0 || !update_vel) {
        rg.set_refzmp_count(lcg.get_lcg_count()); // Start refzmp_count from current remaining footstep count of swinging.
    } else {
        rg.set_refzmp_count(static_cast<size_t>(fnsl[0][0].step_time/dt)); // Start refzmp_count from step length of first overwrite step
    }
    /*   reset refzmp */
    for (size_t i = 0; i < fnsl.size(); i++) {
        if (emergency_flg == EMERGENCY_STOP)
            rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list[idx+i],
                                                        lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1),
                                                        lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1));
        else {
            if (i==fnsl.size()-1) {
                rg.push_refzmp_from_footstep_nodes_for_dual(footstep_nodes_list[fnsl.size()-1],
                                                            lcg.get_swing_leg_dst_steps_idx(footstep_nodes_list.size()-1),
                                                            lcg.get_support_leg_steps_idx(footstep_nodes_list.size()-1));
            } else {
                std::vector<step_node> tmp_swing_leg_src_steps;
                lcg.calc_swing_leg_src_steps(tmp_swing_leg_src_steps, footstep_nodes_list, idx+i);
                // toe_heel_types tht(thtc.check_toe_heel_type_from_swing_support_coords(tmp_swing_leg_src_steps.front().worldcoords, lcg.get_support_leg_steps_idx(idx+i).front().worldcoords, lcg.get_toe_pos_offset_x(), lcg.get_heel_pos_offset_x()),
                //                    thtc.check_toe_heel_type_from_swing_support_coords(lcg.get_swing_leg_dst_steps_idx(idx+i).front().worldcoords, lcg.get_support_leg_steps_idx(idx+i).front().worldcoords, lcg.get_toe_pos_offset_x(), lcg.get_heel_pos_offset_x()));
                toe_heel_types tht; // TODO : unsupport toe heel phase
                rg.push_refzmp_from_footstep_nodes_for_single(footstep_nodes_list[idx+i], lcg.get_support_leg_steps_idx(idx+i), tht);
            }
        }
    }
    /* Overwrite refzmp index in preview contoroller queue */
    size_t queue_size = preview_controller_ptr->get_preview_queue_size();
    size_t overwrite_idx;
    if (overwritable_footstep_index_offset == 0 || !update_vel) {
      overwrite_idx = 0; // Overwrite all queue
    } else {
      overwrite_idx = lcg.get_lcg_count(); // Overwrite queue except current footstep
    }
    if (is_preview) {
      /* fill preview controller queue by new refzmp */
      hrp::Vector3 rzmp;
      bool refzmp_exist_p;
      std::vector<hrp::Vector3> sfzos;
      for (size_t i = overwrite_idx; i < queue_size - 1; i++) {
        refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
        preview_controller_ptr->set_preview_queue(rzmp, sfzos, i+1);
        if (refzmp_exist_p) {
          prev_que_rzmp = rzmp;
          prev_que_sfzos = sfzos;
        }
        rg.update_refzmp();
        sfzos.clear();
      }
      finalize_count = 0;
      refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzos, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
      if (!refzmp_exist_p) {
        finalize_count++;
        rzmp = prev_que_rzmp;
        sfzos = prev_que_sfzos;
      } else {
        prev_que_rzmp = rzmp;
        prev_que_sfzos = sfzos;
      }
      solved = preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offsets, rzmp, sfzos, (refzmp_exist_p || finalize_count < preview_controller_ptr->get_delay()-default_step_time/dt));
      rg.update_refzmp();
    } else {
      update_foot_guided_controller(solved, cur_cog, cur_cogvel, cur_refcog, cur_refcogvel, cur_cmp);
    }
  };

  const std::vector<leg_type> gait_generator::calc_counter_leg_types_from_footstep_nodes(const std::vector<step_node>& fns, std::vector<std::string> _all_limbs) const {
    std::vector<std::string> fns_names, cntr_legs_names;
    for (std::vector<step_node>::const_iterator it = fns.begin(); it != fns.end(); it++) {
        fns_names.push_back(leg_type_map.find(it->l_r)->second);
    }
    std::sort(_all_limbs.begin(), _all_limbs.end());
    std::sort(fns_names.begin(), fns_names.end());
    std::set_difference(_all_limbs.begin(), _all_limbs.end(), /* all candidates for legs */
                        fns_names.begin(), fns_names.end(),   /* support legs */
                        std::back_inserter(cntr_legs_names)); /* swing   legs */
    std::vector<leg_type> ret;
    for (std::vector<std::string>::const_iterator it = cntr_legs_names.begin(); it != cntr_legs_names.end(); it++) {
        std::map<leg_type, std::string>::const_iterator dst = std::find_if(leg_type_map.begin(), leg_type_map.end(), (&boost::lambda::_1->* &std::map<leg_type, std::string>::value_type::second == *it));
        ret.push_back(dst->first);
    }
    return ret;
  };
}
