/* -*- mode:c++ -*- */
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include "PreviewController.h"
#include "FootGuidedController.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "interpolator.h"
#include <vector>
#include <queue>
#include <boost/assign.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/shared_ptr.hpp>
#include "../TorqueFilter/IIRFilter.h"
#include "AutoBalancerService_impl.h"

#ifdef FOR_TESTGAITGENERATOR
#warning "Compile for testGaitGenerator"
#endif // FOR_TESTGAITGENERATOR

namespace rats
{
    void cycloid_midpoint (hrp::Vector3& ret,
                           const double ratio, const hrp::Vector3& start,
                           const hrp::Vector3& goal, const double height,
                           const double default_top_ratio = 0.5);
    void multi_mid_coords (coordinates& mid_coords, const std::vector<coordinates>& cs, const double eps = 0.001);

    enum orbit_type {SHUFFLING, CYCLOID, RECTANGLE, STAIR, CYCLOIDDELAY, CYCLOIDDELAYKICK, CROSS};
    enum leg_type {RLEG, LLEG, RARM, LARM, BOTH, ALL};
    enum stride_limitation_type {SQUARE, CIRCLE};
    enum swing_phase_type {LIFTOFF, TOUCHDOWN};
    std::string leg_type_to_leg_type_string (const leg_type l_r);

    struct step_node
    {
        leg_type l_r;
        coordinates worldcoords;
        double step_height, step_time, toe_angle, heel_angle;
        step_node () : l_r(RLEG), worldcoords(coordinates()),
                       step_height(), step_time(),
                       toe_angle(), heel_angle(){};
        step_node (const leg_type _l_r, const coordinates& _worldcoords,
                   const double _step_height, const double _step_time,
                   const double _toe_angle, const double _heel_angle)
            : l_r(_l_r), worldcoords(_worldcoords),
              step_height(_step_height), step_time(_step_time),
              toe_angle(_toe_angle), heel_angle(_heel_angle) {};
        step_node (const std::string& _l_r, const coordinates& _worldcoords,
                   const double _step_height, const double _step_time,
                   const double _toe_angle, const double _heel_angle)
            : l_r((_l_r == "rleg") ? RLEG :
                  (_l_r == "rarm") ? RARM :
                  (_l_r == "larm") ? LARM :
                  LLEG), worldcoords(_worldcoords),
              step_height(_step_height), step_time(_step_time),
              toe_angle(_toe_angle), heel_angle(_heel_angle) {};
        friend std::ostream &operator<<(std::ostream &os, const step_node &sn)
        {
            os << "footstep" << std::endl;
            os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):
                                   (sn.l_r==RARM)?std::string("rarm"):
                                   (sn.l_r==LARM)?std::string("larm"):
                                   std::string("rleg")) << "]" << std::endl;
            os << "  pos =";
            os << (sn.worldcoords.pos).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
            os << "  rot =";
            os << (sn.worldcoords.rot).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
            os << "  step_height = " << sn.step_height << "[m], step_time = " << sn.step_time << "[s], "
               << "toe_angle = " << sn.toe_angle << "[deg], heel_angle = " << sn.heel_angle << "[deg]";
            return os;
        };
    };

    struct wheel_node
    {
      coordinates worldcoords;
      double time;
      wheel_node () : worldcoords(coordinates()), time() {};
      wheel_node (const coordinates& _worldcoords, const double _time)
        : worldcoords(_worldcoords), time(_time) {};
    };

    /* footstep parameter */
    struct footstep_parameter
    {
        /* translate pos is translate position of a leg from default foot_midcoords
         *   vector -> (list rleg-pos[m] lleg-pos[m] )
         */
        std::vector<hrp::Vector3> leg_default_translate_pos, vel_foot_offset;
        /* stride params indicate max stride */
        double stride_fwd_x/*[m]*/, stride_outside_y/*[m]*/, stride_outside_theta/*[deg]*/, stride_bwd_x/*[m]*/, stride_inside_y/*[m]*/, stride_inside_theta/*[deg]*/;
        footstep_parameter (const std::vector<hrp::Vector3>& _leg_pos,
                            const double _stride_fwd_x, const double _stride_outside_y, const double _stride_outside_theta,
                            const double _stride_bwd_x, const double _stride_inside_y, const double _stride_inside_theta)
            : leg_default_translate_pos(_leg_pos), vel_foot_offset(_leg_pos),
              stride_fwd_x(_stride_fwd_x), stride_outside_y(_stride_outside_y), stride_outside_theta(_stride_outside_theta),
              stride_bwd_x(_stride_bwd_x), stride_inside_y(_stride_inside_y), stride_inside_theta(_stride_inside_theta) {};
    };

    /* velocity parameter for velocity mode */
    struct velocity_mode_parameter
    {
        /* velocity is [mm/s], [mm/s], [deg/s] */
        double velocity_x, velocity_y, velocity_theta;
        void set (const double _vx, const double _vy, const double _vth)
        {
            velocity_x = _vx;
            velocity_y = _vy;
            velocity_theta = _vth;
        };
        velocity_mode_parameter ()
            :velocity_x(0), velocity_y(0), velocity_theta(0) {};
    };

    /* Phase name of toe heel contact.
     *   SOLE0 : Sole contact (start). Foot angle is 0. ZMP transition is ee -> toe.
     *   SOLE2TOE : Transition of foot angle (0 -> toe_angle). ZMP is on toe.
     *   TOE2SOLE : Transition of foot angle (toe_angle -> 0). ZMP is on toe.
     *   SOLE1 : Foot_angle is 0.
     *   SOLE2HEEL : Transition of foot angle (0 -> -1 * heel_angle). ZMP is on heel.
     *   HEEL2SOLE : Transition of foot angle (-1 * heel_angle -> 0). ZMP is on heel.
     *   SOLE2 : Sole contact (end). Foot angle is 0. ZMP transition is heel -> ee.
     */
    enum toe_heel_phase {SOLE0, SOLE2TOE, TOE2SOLE, SOLE1, SOLE2HEEL, HEEL2SOLE, SOLE2, NUM_TH_PHASES};
    static double no_using_toe_heel_ratio = 1.0; // Ratio not to use toe and heel contact
    static double using_toe_heel_ratio = 0.0; // Ratio to use toe and heel contact

    enum toe_heel_type {SOLE, TOE, HEEL};
    struct toe_heel_types
    {
        toe_heel_type src_type, dst_type;
        toe_heel_types (const toe_heel_type _src_type = SOLE, const toe_heel_type _dst_type = SOLE) : src_type(_src_type), dst_type(_dst_type)
        {
        };
    };

    /* Manager for toe heel phase. */
    class toe_heel_phase_counter
    {
        double toe_heel_phase_ratio[NUM_TH_PHASES];
        size_t toe_heel_phase_count[NUM_TH_PHASES], one_step_count;
        bool calc_toe_heel_phase_count_from_raio ()
        {
            double ratio_sum = 0.0;
            for (size_t i = 0; i < NUM_TH_PHASES; i++) {
                ratio_sum += toe_heel_phase_ratio[i];
                toe_heel_phase_count[i] = static_cast<size_t>(one_step_count * ratio_sum);
            }
            return true;
        };
    public:
        toe_heel_phase_counter () : one_step_count(0)
        {
            toe_heel_phase_ratio[SOLE0] = 0.05;
            toe_heel_phase_ratio[SOLE2TOE] = 0.25;
            toe_heel_phase_ratio[TOE2SOLE] = 0.2;
            toe_heel_phase_ratio[SOLE1] = 0.0;
            toe_heel_phase_ratio[SOLE2HEEL] = 0.2;
            toe_heel_phase_ratio[HEEL2SOLE] = 0.25;
            toe_heel_phase_ratio[SOLE2] = 0.05;
        };
        bool check_toe_heel_phase_ratio_validity (const std::vector<double>& ratio)
        {
            bool ret = true;
            // Check size
            if (ratio.size() != NUM_TH_PHASES) {
                ret = false;
            }
            // Check sum == 1.0
            double sum_ratio = 0.0;
            for (int i = 0; i < NUM_TH_PHASES; i++) sum_ratio += ratio[i];
            if (std::fabs(sum_ratio-1.0) > 1e-3) {
                ret = false;
            }
            if (!ret) {
                std::cerr << "toe_heel_phase_ratio is not set, "
                          << ", required length = " << NUM_TH_PHASES << " != input length " << ratio.size()
                          << ", sum_ratio = " << sum_ratio << " is not 1.0."
                          << std::endl;
            } else {
                std::cerr << "toe_heel_phase_ratio is successfully set." << std::endl;
            }
            return ret;
        };
        // setter
        void set_one_step_count (const size_t _count)
        {
            one_step_count = _count;
            calc_toe_heel_phase_count_from_raio();
        };
        bool set_toe_heel_phase_ratio (const std::vector<double>& ratio)
        {
            if (check_toe_heel_phase_ratio_validity(ratio)) {
                for (size_t i = 0; i < NUM_TH_PHASES; i++) toe_heel_phase_ratio[i] = ratio[i];
                return true;
            } else {
                return false;
            }
        };
        // getter
        void get_toe_heel_phase_ratio (std::vector<double>& ratio) const
        {
            for (size_t i = 0; i < NUM_TH_PHASES; i++) ratio[i] = toe_heel_phase_ratio[i];
        };
        // functions for checking phase and calculating phase time and ratio
        bool is_phase_starting (const size_t current_count, const toe_heel_phase _phase) const
        {
            return (current_count == toe_heel_phase_count[_phase]);
        };
        bool is_between_phases (const size_t current_count, const toe_heel_phase phase0, const toe_heel_phase phase1) const
        {
            return (toe_heel_phase_count[phase0] <= current_count) && (current_count < toe_heel_phase_count[phase1]);
        };
        bool is_between_phases (const size_t current_count, const toe_heel_phase phase1) const
        {
            return (current_count < toe_heel_phase_count[phase1]);
        };
        bool is_no_SOLE1_phase () const { return toe_heel_phase_count[TOE2SOLE] == toe_heel_phase_count[SOLE1]; };
        // Calculate period [s] between start_phase and goal_phase
        double calc_phase_period (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double _dt) const
        {
            return _dt * (toe_heel_phase_count[goal_phase]-toe_heel_phase_count[start_phase]);
        };
        // Calculate ratio between start_phase and goal_phase.
        //   If current_count is 0->goal_phase_count, start_phase : 0 -> goal_phase : 1
        double calc_phase_ratio (const size_t current_count, const toe_heel_phase start_phase, const toe_heel_phase goal_phase) const
        {
            return static_cast<double>(current_count-toe_heel_phase_count[start_phase]) / (toe_heel_phase_count[goal_phase]-toe_heel_phase_count[start_phase]);
        };
        // Calculate ratio to goal_phase.
        //   If current_count is 0->goal_phase_count, start : 0 -> goal_phase : 1
        double calc_phase_ratio (const size_t current_count, const toe_heel_phase goal_phase) const
        {
            return static_cast<double>(current_count) / (toe_heel_phase_count[goal_phase]);
        };
        // Calculate ratio for toe heel transition.
        //   If toe or heel are used, ratio is 0.0. Otherwise, ratio is (0.0, 1.0].
        //   We assume current_count is 0->goal_phase_count.
        double calc_phase_ratio_for_toe_heel_transition (const size_t current_count) const
        {
            if (is_between_phases(current_count, SOLE0)) {
                // ratio : 1 -> 0
                return 1-calc_phase_ratio(current_count, SOLE0);
            } else if (is_between_phases(current_count, HEEL2SOLE, SOLE2) || toe_heel_phase_count[SOLE2] == current_count) {
                // ratio : 0 -> 1
                return calc_phase_ratio(current_count, HEEL2SOLE, SOLE2);
            } else {
                // If using toe or heel, 0
                return using_toe_heel_ratio;
            }
        };
    };

    /* Checker for toe heel type. */
    class toe_heel_type_checker
    {
    private:
        double toe_check_thre, heel_check_thre; // [m]
    public:
        toe_heel_type_checker () : toe_check_thre(0), heel_check_thre(0)
        {
        };
        toe_heel_type_checker (const double _toe_check_thre, const double _heel_check_thre) : toe_check_thre(_toe_check_thre), heel_check_thre(_heel_check_thre)
        {
        };
        toe_heel_type check_toe_heel_type_from_swing_support_coords (const coordinates& swing_coords, const coordinates& support_coords, const double toe_pos_offset_x, const double heel_pos_offset_x) const
        {
            hrp::Vector3 local_toe_pos = support_coords.rot.transpose() * (swing_coords.rot * hrp::Vector3(toe_pos_offset_x,0,0) + swing_coords.pos - support_coords.pos);
            hrp::Vector3 local_heel_pos = support_coords.rot.transpose() * (swing_coords.rot * hrp::Vector3(heel_pos_offset_x,0,0) + swing_coords.pos - support_coords.pos);
            if (local_toe_pos(2) < -50*1e-3 && (local_toe_pos(0) + toe_check_thre < 0 || local_heel_pos(0) - heel_check_thre > 0) ) {
                return TOE;
            } else if (local_toe_pos(0) + toe_check_thre < 0) {
                return TOE;
            } else if (local_heel_pos(0) - heel_check_thre > 0) {
                return HEEL;
            } else {
                return SOLE;
            }
        };
        void print_param (const std::string print_str = "") const
        {
            std::cerr << "[" << print_str << "]   toe_check_thre = " << toe_check_thre << ", heel_check_thre = " << heel_check_thre << std::endl;
        };
        // Setter
        void set_toe_check_thre (const double _toe_check_thre) { toe_check_thre = _toe_check_thre; };
        void set_heel_check_thre (const double _heel_check_thre) { heel_check_thre = _heel_check_thre; };
        // Getter
        double get_toe_check_thre () const { return toe_check_thre; };
        double get_heel_check_thre () const { return heel_check_thre; };
    };

    double set_value_according_to_toe_heel_type (const toe_heel_type tht, const double toe_value, const double heel_value, const double default_value);

    /* refzmp_generator to generate current refzmp from footstep_node_list */
    class refzmp_generator
    {
#ifdef HAVE_MAIN
    public:
#endif
      std::vector<hrp::Vector3> refzmp_cur_list;
      std::map<leg_type, double> zmp_weight_map;
      std::vector< std::vector<hrp::Vector3> > foot_x_axises_list; // Swing foot x axis list according to refzmp_cur_list
      std::vector< std::vector<leg_type> > swing_leg_types_list; // Swing leg list according to refzmp_cur_list
      std::vector<size_t> step_count_list; // Swing leg list according to refzmp_cur_list
      std::vector<toe_heel_types> toe_heel_types_list;
      std::vector<hrp::Vector3> default_zmp_offsets; /* list of RLEG and LLEG */
      size_t refzmp_index, refzmp_count, one_step_count;
      double toe_zmp_offset_x, heel_zmp_offset_x; // [m]
      double dt;
      toe_heel_phase_counter thp;
      bool use_toe_heel_transition, use_toe_heel_auto_set;
      boost::shared_ptr<interpolator> zmp_weight_interpolator;
      void calc_current_refzmp (hrp::Vector3& ret, std::vector<hrp::Vector3>& swing_foot_zmp_offsets, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const double default_double_support_static_ratio_before, const double default_double_support_static_ratio_after);
      const bool is_start_double_support_phase () const { return refzmp_index == 0; };
      const bool is_second_phase () const { return refzmp_index == 1; };
      const bool is_second_last_phase () const { return refzmp_index == refzmp_cur_list.size()-2; };
      const bool is_end_double_support_phase () const { return refzmp_index == refzmp_cur_list.size() - 1; };
#ifndef HAVE_MAIN
    public:
#endif
      refzmp_generator(const double _dt)
        : refzmp_cur_list(), foot_x_axises_list(), swing_leg_types_list(), step_count_list(), toe_heel_types_list(), default_zmp_offsets(),
          refzmp_index(0), refzmp_count(0), one_step_count(0),
          toe_zmp_offset_x(0), heel_zmp_offset_x(0), dt(_dt),
          thp(), use_toe_heel_transition(false), use_toe_heel_auto_set(false)
      {
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
          default_zmp_offsets.push_back(hrp::Vector3::Zero());
          double zmp_weight_initial_value[4] = {1.0, 1.0, 0.1, 0.1};
          zmp_weight_map = boost::assign::map_list_of<leg_type, double>(RLEG, zmp_weight_initial_value[0])(LLEG, zmp_weight_initial_value[1])(RARM, zmp_weight_initial_value[2])(LARM, zmp_weight_initial_value[3]).convert_to_container< std::map<leg_type, double> > ();
          zmp_weight_interpolator = boost::shared_ptr<interpolator>(new interpolator(4, dt));
          zmp_weight_interpolator->set(zmp_weight_initial_value); /* set initial value */
          zmp_weight_interpolator->setName("GaitGenerator zmp_weight_interpolator");
      };
      ~refzmp_generator()
      {
      };
      void remove_refzmp_cur_list_over_length (const size_t len)
      {
        while ( refzmp_cur_list.size() > len) refzmp_cur_list.pop_back();
        while ( foot_x_axises_list.size() > len) foot_x_axises_list.pop_back();
        while ( swing_leg_types_list.size() > len) swing_leg_types_list.pop_back();
        while ( step_count_list.size() > len) step_count_list.pop_back();
        while ( toe_heel_types_list.size() > len) toe_heel_types_list.pop_back();
      };
      void reset (const size_t _refzmp_count)
      {
        set_indices(0);
        one_step_count = _refzmp_count;
        set_refzmp_count(_refzmp_count);
        refzmp_cur_list.clear();
        foot_x_axises_list.clear();
        swing_leg_types_list.clear();
        step_count_list.clear();
        toe_heel_types_list.clear();
        thp.set_one_step_count(one_step_count);
      };
      void push_refzmp_from_footstep_nodes_for_dual (const std::vector<step_node>& fns,
                                                     const std::vector<step_node>& _support_leg_steps,
                                                     const std::vector<step_node>& _swing_leg_steps);
        void push_refzmp_from_footstep_nodes_for_single (const std::vector<step_node>& fns, const std::vector<step_node>& _support_leg_steps, const toe_heel_types& tht);
      void update_refzmp ();
      // setter
      void set_indices (const size_t idx) { refzmp_index = idx; };
      void set_one_step_count (const size_t d_cnt) { one_step_count += d_cnt; };
      void set_refzmp_count(const size_t _refzmp_count) { refzmp_count = _refzmp_count; };
      void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { default_zmp_offsets = tmp; };
      void set_toe_zmp_offset_x (const double _off) { toe_zmp_offset_x = _off; };
      void set_heel_zmp_offset_x (const double _off) { heel_zmp_offset_x = _off; };
      void set_use_toe_heel_transition (const bool _u) { use_toe_heel_transition = _u; };
      void set_use_toe_heel_auto_set (const bool _u) { use_toe_heel_auto_set = _u; };
      void set_zmp_weight_map (const std::map<leg_type, double> _map) {
          double zmp_weight_array[4] = {_map.find(RLEG)->second, _map.find(LLEG)->second, _map.find(RARM)->second, _map.find(LARM)->second};
          if (zmp_weight_interpolator->isEmpty()) {
              zmp_weight_interpolator->clear();
              double zmp_weight_initial_value[4] = {zmp_weight_map[RLEG], zmp_weight_map[LLEG], zmp_weight_map[RARM], zmp_weight_map[LARM]};
              zmp_weight_interpolator->set(zmp_weight_initial_value);
              zmp_weight_interpolator->setGoal(zmp_weight_array, 2.0, true);
          } else {
              std::cerr << "zmp_weight_map cannot be set because interpolating." << std::endl;
          }
      };
      bool set_toe_heel_phase_ratio (const std::vector<double>& ratio) { return thp.set_toe_heel_phase_ratio(ratio); };
      // getter
      bool get_current_refzmp (hrp::Vector3& rzmp, std::vector<hrp::Vector3>& swing_foot_zmp_offsets, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const double default_double_support_static_ratio_before, const double default_double_support_static_ratio_after)
      {
        if (refzmp_cur_list.size() > refzmp_index ) calc_current_refzmp(rzmp, swing_foot_zmp_offsets, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
        return refzmp_cur_list.size() > refzmp_index;
      };
      const hrp::Vector3& get_refzmp_cur () const { return refzmp_cur_list.front(); };
      const hrp::Vector3& get_default_zmp_offset (const leg_type lt) const { return default_zmp_offsets[lt]; };
      double get_toe_zmp_offset_x () const { return toe_zmp_offset_x; };
      double get_heel_zmp_offset_x () const { return heel_zmp_offset_x; };
      bool get_use_toe_heel_transition () const { return use_toe_heel_transition; };
      bool get_use_toe_heel_auto_set () const { return use_toe_heel_auto_set; };
      const std::map<leg_type, double> get_zmp_weight_map () const { return zmp_weight_map; };
      void proc_zmp_weight_map_interpolation () {
          if (!zmp_weight_interpolator->isEmpty()) {
              double zmp_weight_output[4];
              zmp_weight_interpolator->get(zmp_weight_output, true);
              zmp_weight_map = boost::assign::map_list_of<leg_type, double>(RLEG, zmp_weight_output[0])(LLEG, zmp_weight_output[1])(RARM, zmp_weight_output[2])(LARM, zmp_weight_output[3]).convert_to_container < std::map<leg_type, double> > ();
          }
      };
#ifdef FOR_TESTGAITGENERATOR
    std::vector<hrp::Vector3> get_default_zmp_offsets() const { return default_zmp_offsets; };
#endif // FOR_TESTGAITGENERATOR
    };

    class delay_hoffarbib_trajectory_generator
    {
    private:
      hrp::Vector3 pos, vel, acc; // [m], [m/s], [m/s^2]
      double dt; // [s]
      // Parameters for antecedent path generation
      std::vector<hrp::Vector3> point_vec;
      std::vector<double> distance_vec;
      std::vector<double> sum_distance_vec;
      double total_path_length;
      // Implement hoffarbib to configure remain_time;
      void hoffarbib_interpolation (double& _pos, double& _vel, double& _acc, const double tmp_remain_time, const double tmp_goal, const double tmp_goal_vel = 0, const double tmp_goal_acc = 0)
      {
        double jerk = (-9.0/ tmp_remain_time) * (_acc - tmp_goal_acc / 3.0) +
            (-36.0 / (tmp_remain_time * tmp_remain_time)) * (tmp_goal_vel * 2.0 / 3.0 + _vel) +
            (60.0 / (tmp_remain_time * tmp_remain_time * tmp_remain_time)) * (tmp_goal - _pos);
        _acc = _acc + dt * jerk;
        _vel = _vel + dt * _acc;
        _pos = _pos + dt * _vel;
      };
    protected:
      double time_offset; // [s]
      double final_distance_weight;
      double time_offset_xy2z; // [s]
      size_t one_step_count, current_count, double_support_count_before, double_support_count_after; // time/dt
      swing_phase_type spt;
      virtual double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current) = 0;
    public:
      delay_hoffarbib_trajectory_generator () : time_offset(0.35), final_distance_weight(1.0), time_offset_xy2z(0), one_step_count(0), current_count(0), double_support_count_before(0), double_support_count_after(0) {};
      ~delay_hoffarbib_trajectory_generator() { };
      bool is_touch_ground;
      void set_dt (const double _dt) { dt = _dt; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset) { time_offset = _time_offset; };
      void set_swing_trajectory_final_distance_weight (const double _final_distance_weight) { final_distance_weight = _final_distance_weight; };
      void set_swing_trajectory_time_offset_xy2z (const double _tmp) { time_offset_xy2z = _tmp; };
      void reset (const size_t _one_step_len, const double default_double_support_ratio_before, const double default_double_support_ratio_after)
      {
        one_step_count = _one_step_len;
        current_count = 0;
        double_support_count_before = (default_double_support_ratio_before*one_step_count);
        double_support_count_after = (default_double_support_ratio_after*one_step_count);
        spt = LIFTOFF;
      };
      void reset_one_step_count (const size_t d_one_step_len, const double default_double_support_ratio_before, const double default_double_support_ratio_after)
      {
        one_step_count += d_one_step_len;
        // double_support_count_before = (default_double_support_ratio_before*one_step_count);
        // double_support_count_after = (default_double_support_ratio_after*one_step_count);
      };
      void reset_all (const double _dt, const size_t _one_step_len,
                      const double default_double_support_ratio_before, const double default_double_support_ratio_after,
                      const double _time_offset, const double _final_distance_weight, const double _time_offset_xy2z)
      {
          set_dt(_dt);
          reset (_one_step_len, default_double_support_ratio_before, default_double_support_ratio_after);
          set_swing_trajectory_delay_time_offset(_time_offset);
          set_swing_trajectory_final_distance_weight(_final_distance_weight);
          set_swing_trajectory_time_offset_xy2z(_time_offset_xy2z);
      };
      void get_trajectory_point (hrp::Vector3& ret, const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current = hrp::Vector3::Zero()) // TODO : support only rectangle
      {
        if ( current_count <= double_support_count_before ) { // first double support phase
          pos = start;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        } else if ( current_count >= one_step_count - double_support_count_after ) { // last double support phase
          pos = goal;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        }
        if ( double_support_count_before <= current_count && current_count < one_step_count - double_support_count_after ) { // swing phase
          size_t swing_remain_count = one_step_count - current_count - double_support_count_after;
          size_t swing_one_step_count = one_step_count - double_support_count_before - double_support_count_after;
          double final_path_distance_ratio = calc_antecedent_path(start, goal, height, current);
          size_t tmp_time_offset_count = time_offset/dt;
          //size_t final_path_count = 0; // Revert to previous version
          size_t final_path_count = final_path_distance_ratio * swing_one_step_count;
          if (final_path_count>static_cast<size_t>(time_offset_xy2z/dt)) final_path_count = static_cast<size_t>(time_offset_xy2z/dt);
          // XYZ interpolation
          if (static_cast<double>(swing_remain_count) / static_cast<double>(swing_one_step_count) <= 0.5 && spt == LIFTOFF) spt = TOUCHDOWN;
          if (swing_remain_count > tmp_time_offset_count) { // antecedent path is still interpolating
            hrp::Vector3 tmpgoal = interpolate_antecedent_path(static_cast<double>(tmp_time_offset_count) / static_cast<double>(swing_remain_count));
            for (size_t i = 0; i < 3; i++) hoffarbib_interpolation (pos(i), vel(i), acc(i), time_offset, tmpgoal(i));
          } else if (swing_remain_count > 0) { // antecedent path already reached to goal
            for (size_t i = 0; i < 3; i++) hoffarbib_interpolation (pos(i), vel(i), acc(i), swing_remain_count*dt, goal(i));
          } else {
            pos = goal;
          }
        }
        if (!is_touch_ground && start == current) {
          pos = start;
          vel = hrp::Vector3::Zero();
          acc = hrp::Vector3::Zero();
        }
        ret = pos;
        current_count++;
      };
      double get_swing_trajectory_delay_time_offset () const { return time_offset; };
      double get_swing_trajectory_final_distance_weight () const { return final_distance_weight; };
      double get_swing_trajectory_time_offset_xy2z () const { return time_offset_xy2z; };
      // interpolate path vector
      //   tmp_ratio : ratio value [0, 1]
      //   org_point_vec : vector of via points
      //   e.g., move tmp_ratio from 0 to 1 => move point from org_point_vec.front() to org_point_vec.back()
      double calc_antecedent_path_base (const std::vector<hrp::Vector3> org_point_vec)
      {
        total_path_length = 0;
        point_vec.clear();
        distance_vec.clear();
        point_vec.push_back(org_point_vec.front());
        // remove distance-zero points
        for (size_t i = 0; i < org_point_vec.size()-1; i++) {
          double tmp_distance = (org_point_vec[i+1]-org_point_vec[i]).norm();
          if (i==org_point_vec.size()-2) tmp_distance*=final_distance_weight;
          if ( tmp_distance > 1e-5 ) {
            point_vec.push_back(org_point_vec[i+1]);
            distance_vec.push_back(tmp_distance);
            total_path_length += tmp_distance;
          }
        }
        if ( total_path_length < 1e-5 ) { // if total path is zero, return goal point.
          return 0;
        }
        // point_vec        : [p0, p1, ..., pN-1, pN]
        // distance_vec     : [  d0, ...,     dN-1  ]
        // sum_distance_vec : [l0, l1, ..., lN-1, lN] <= lj = \Sum_{i=0}^{j-1} di
        sum_distance_vec.clear();
        sum_distance_vec.push_back(0);
        double tmp_dist = 0;
        for (size_t i = 0; i < distance_vec.size(); i++) {
          sum_distance_vec.push_back(tmp_dist + distance_vec[i]);
          tmp_dist += distance_vec[i];
        }
        return distance_vec.back()/total_path_length;
      };
      hrp::Vector3 interpolate_antecedent_path (const double tmp_ratio)
      {
        if ( total_path_length < 1e-5 ) { // if total path is zero, return goal point.
          return point_vec.back();
        }
        // select current segment in which 'tmp_ratio' is included
        double current_length = tmp_ratio * total_path_length;
        for (size_t i = 0; i < sum_distance_vec.size(); i++) {
          if ( (sum_distance_vec[i] <= current_length) && (current_length <= sum_distance_vec[i+1]) ) {
            double tmpr = ((current_length - sum_distance_vec[i]) / distance_vec[i]);
            return ((1-tmpr) * point_vec[i] + tmpr * point_vec[1+i]);
          }
        }
        // if illegal tmp-ratio
        if (current_length < 0) return point_vec.front();
        else return point_vec.back();
      };
    };

    class rectangle_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
    private:
      leg_type swing_leg;
    public:
      rectangle_delay_hoffarbib_trajectory_generator () : delay_hoffarbib_trajectory_generator(), way_point_offset(0.05, 0.0, 0.0) {};
      ~rectangle_delay_hoffarbib_trajectory_generator () {};
      void set_swing_leg (leg_type _lr) { swing_leg = _lr; };
      void set_rectangle_trajectory_way_point_offset (const hrp::Vector3 _offset) { way_point_offset = _offset; };
      hrp::Vector3 get_rectangle_trajectory_way_point_offset() const { return way_point_offset; };
      hrp::Vector3 way_point_offset;
      double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current)
      {
        std::vector<hrp::Vector3> rectangle_path;
        double max_height = std::max(start(2), goal(2))+height;
        hrp::Vector3 diff_vec = goal - start;
        diff_vec(2) = 0.0;
        rectangle_path.push_back(current);
        switch (spt) {
        case LIFTOFF:
          if (diff_vec(2) > 2e-2) { // 2cm
            rectangle_path.push_back(hrp::Vector3(current(0), current(1), max_height) - way_point_offset(0) * diff_vec.normalized());
          } else {
            rectangle_path.push_back(hrp::Vector3(current(0), current(1), max_height));
          }
          if (diff_vec(0) > 5e-3) rectangle_path.back() += hrp::Vector3(0.0, (swing_leg == LLEG ? 1 : -1) * way_point_offset(1), way_point_offset(2));
          rectangle_path.push_back(hrp::Vector3(goal(0), goal(1), max_height));
          if (diff_vec(0) > 5e-3) rectangle_path.back() += hrp::Vector3(0.0, (swing_leg == LLEG ? 1 : -1) * way_point_offset(1), way_point_offset(2));
          break;
        case TOUCHDOWN:
          rectangle_path.push_back(hrp::Vector3(goal(0), goal(1), current(2)));
          break;
        default:
          break;
        }
        rectangle_path.push_back(goal);
        return calc_antecedent_path_base(rectangle_path);
      };
    };

    class stair_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
      hrp::Vector3 way_point_offset;
      double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current)
      {
        std::vector<hrp::Vector3> path;
        double max_height = std::max(start(2), goal(2))+height;
        hrp::Vector3 diff_vec = goal - start;
        diff_vec(2) = 0.0; // projection on horizontal plane
        path.push_back(start);
        // currently way_point_offset(1) is not used.
        //if ( diff_vec.norm() > 1e-4 && (goal(2) - start(2)) > way_point_offset(2) ) {
        if ( diff_vec.norm() > 1e-4 && (goal(2) - start(2)) > 0.02) {
          path.push_back(hrp::Vector3(start+-1*way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,0,way_point_offset(2)+max_height-start(2))));
        }
        path.push_back(hrp::Vector3(start(0), start(1), max_height));
        path.push_back(hrp::Vector3(goal(0), goal(1), max_height));
        //if ( diff_vec.norm() > 1e-4 && (start(2) - goal(2)) > way_point_offset(2) ) {
        if ( diff_vec.norm() > 1e-4 && (start(2) - goal(2)) > 0.02) {
          path.push_back(hrp::Vector3(goal+way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,0,way_point_offset(2)+max_height-goal(2))));
        }
        // if (height > 20 * 1e-3) {
        //   path.push_back(hrp::Vector3(goal(0), goal(1), 20*1e-3+goal(2)));
        // }
        path.push_back(goal);
        return calc_antecedent_path_base(path);
      };
    public:
      stair_delay_hoffarbib_trajectory_generator () : delay_hoffarbib_trajectory_generator(), way_point_offset(hrp::Vector3(0.03, 0.0, 0.0)) {};
      ~stair_delay_hoffarbib_trajectory_generator () {};
      void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { way_point_offset = _offset; };
      hrp::Vector3 get_stair_trajectory_way_point_offset() const { return way_point_offset; };
    };

    class cycloid_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
      double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current)
      {
        std::vector<hrp::Vector3> cycloid_path;
        hrp::Vector3 tmpv, via_goal(goal);
        double ratio = 0.4;
        via_goal(2) += ratio*height;
        double tmpheight = ((start(2)+goal(2))/2.0+height-(start(2)+via_goal(2))/2.0);
        cycloid_path.push_back(start);
        cycloid_midpoint(tmpv, 0.2, start, via_goal, tmpheight);
        cycloid_path.push_back(tmpv);
        cycloid_midpoint(tmpv, 0.4, start, via_goal, tmpheight);
        cycloid_path.push_back(tmpv);
        cycloid_midpoint(tmpv, 0.6, start, via_goal, tmpheight);
        cycloid_path.push_back(tmpv);
        cycloid_midpoint(tmpv, 0.8, start, via_goal, tmpheight);
        cycloid_path.push_back(tmpv);
        cycloid_path.push_back(via_goal);
        cycloid_path.push_back(goal);
        return calc_antecedent_path_base(cycloid_path);
      };
    };

    class cycloid_delay_kick_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
    private:
      hrp::Matrix33 start_rot;
      hrp::Vector3 kick_point_offset;
    public:
        cycloid_delay_kick_hoffarbib_trajectory_generator() : delay_hoffarbib_trajectory_generator(), kick_point_offset(hrp::Vector3(-0.1, 0.0, 0.0)) {};
      void set_cycloid_delay_kick_point_offset (const hrp::Vector3 _offset) { kick_point_offset = _offset; };
      void set_start_rot (const hrp::Matrix33 _offset) { start_rot = _offset; };
      hrp::Vector3 get_cycloid_delay_kick_point_offset () const { return kick_point_offset; };
      double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current)
      {
        std::vector<hrp::Vector3> cycloid_path;
        hrp::Vector3 tmpv, via_goal(goal);
        double ratio = 0.4;
        via_goal(2) += ratio*height;
        double tmpheight = ((start(2)+goal(2))/2.0+height-(start(2)+via_goal(2))/2.0);
        // kick_point_offset = start_rot * kick_point_offset;
        cycloid_path.push_back(start);
        if(height > 1e-4){
            cycloid_path.push_back(start + start_rot * kick_point_offset);
            cycloid_midpoint(tmpv, 0.2, start + start_rot * kick_point_offset, via_goal, tmpheight);
            cycloid_path.push_back(tmpv);
            cycloid_midpoint(tmpv, 0.4, start + start_rot * kick_point_offset, via_goal, tmpheight);
            cycloid_path.push_back(tmpv);
            cycloid_midpoint(tmpv, 0.6, start + start_rot * kick_point_offset, via_goal, tmpheight);
            cycloid_path.push_back(tmpv);
            cycloid_midpoint(tmpv, 0.8, start + start_rot * kick_point_offset, via_goal, tmpheight);
            cycloid_path.push_back(tmpv);
        }
        cycloid_path.push_back(via_goal);
        cycloid_path.push_back(goal);
        return calc_antecedent_path_base(cycloid_path);
      };
    };
    
    class cross_delay_hoffarbib_trajectory_generator : public delay_hoffarbib_trajectory_generator
    {
    private:
      leg_type swing_leg;
    public:
      cross_delay_hoffarbib_trajectory_generator () : delay_hoffarbib_trajectory_generator(), way_point_offset(hrp::Vector3(0.04, 0.15, 0.0)) {};
      ~cross_delay_hoffarbib_trajectory_generator () {};
      void set_swing_leg (leg_type _lr) { swing_leg = _lr; };
      hrp::Vector3 way_point_offset;
      double calc_antecedent_path (const hrp::Vector3& start, const hrp::Vector3& goal, const double height, const hrp::Vector3& current)
      {
        std::vector<hrp::Vector3> path;
        double max_height = std::max(start(2), goal(2))+height;
        hrp::Vector3 diff_vec = goal - start;
        diff_vec(2) = 0.0; // projection on horizontal plane
        path.push_back(start);
        if ( swing_leg == LLEG ) { // swing_leg is left
            path.push_back(hrp::Vector3(start+-1*way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,way_point_offset(1),way_point_offset(2)+max_height-start(2))));
            path.push_back(hrp::Vector3(goal+way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,way_point_offset(1),way_point_offset(2)+max_height-goal(2))));
        } else { // swing_leg is right
            path.push_back(hrp::Vector3(start+-1*way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,-way_point_offset(1),way_point_offset(2)+max_height-start(2))));
            path.push_back(hrp::Vector3(goal+way_point_offset(0)*diff_vec.normalized()+hrp::Vector3(0,-way_point_offset(1),way_point_offset(2)+max_height-goal(2))));
        }
        if (height > 30 * 1e-3) {
          path.push_back(hrp::Vector3(goal(0), goal(1), 30*1e-3+goal(2)));
        }
        path.push_back(goal);
        return calc_antecedent_path_base(path);
      };
    };

    /* leg_coords_generator to generate current swing_leg_coords and support_leg_coords from footstep_node_list */
    class leg_coords_generator
    {
#ifdef HAVE_MAIN
    public:
#endif
      std::vector< std::vector<step_node> > swing_leg_dst_steps_list, support_leg_steps_list;
      // Support leg coordinates.
      std::vector<step_node> support_leg_steps;
      // Swing leg coordinates is interpolated from swing_leg_src_coords to swing_leg_dst_coords during swing phase.
      std::vector<step_node> swing_leg_steps, swing_leg_src_steps, swing_leg_dst_steps, current_swing_leg_steps;
      double default_step_height, default_top_ratio, current_step_height, swing_ratio, dt, current_toe_angle, current_heel_angle;
      double time_offset, final_distance_weight, time_offset_xy2z;
      std::vector<double> current_swing_time;
      // Index for current footstep. footstep_index should be [0,footstep_node_list.size()]. Current footstep is footstep_node_list[footstep_index].
      size_t footstep_index;
      // one_step_count is total counter num of current steps (= step_time/dt). lcg_count is counter for lcg. During one step, lcg_count decreases from one_step_count to 0.
      size_t lcg_count, one_step_count, next_one_step_count, swing_rot_count_ratio;
      // Current support leg
      std::vector<leg_type> support_leg_types, swing_leg_types;
      orbit_type default_orbit_type;
      bool is_swing_phase;
      // Foot trajectory generators
      std::vector<rectangle_delay_hoffarbib_trajectory_generator> rdtg;
      hrp::Vector3 rectangle_way_point_offset;
      stair_delay_hoffarbib_trajectory_generator sdtg;
      std::vector<cycloid_delay_hoffarbib_trajectory_generator> cdtg;
      cycloid_delay_kick_hoffarbib_trajectory_generator cdktg;
      cross_delay_hoffarbib_trajectory_generator crdtg;
      toe_heel_phase_counter thp;
      interpolator* foot_midcoords_interpolator;
      coordinates swing_support_midcoords;
      // Map for interpolator of each swing foot rot interpolation. In constructor, prepared for all limbs. In control loop, swing foot element is used.
      std::map<leg_type, interpolator*> swing_foot_rot_interpolator;
      // Parameters for toe-heel contact
      interpolator* toe_heel_interpolator;
      double toe_pos_offset_x, heel_pos_offset_x, toe_angle, heel_angle, foot_dif_rot_angle, toe_heel_dif_angle;
      bool use_toe_joint, use_toe_heel_auto_set;
      toe_heel_type current_src_toe_heel_type, current_dst_toe_heel_type;
      std::vector<bool> act_contact_states;
      bool is_touch_ground, use_act_states;
      int touch_ground_count;
      void calc_current_swing_foot_rot (std::map<leg_type, hrp::Vector3>& tmp_swing_foot_rot, const double _default_double_support_ratio_before, const double _default_double_support_ratio_after);
      void calc_current_swing_leg_steps (std::vector<step_node>& rets, const double step_height, const double _current_toe_angle, const double _current_heel_angle, const double _default_double_support_ratio_before, const double _default_double_support_ratio_after);
      double calc_interpolated_toe_heel_angle (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double start, const double goal);
      void modif_foot_coords_for_toe_heel_phase (coordinates& org_coords, const double _current_toe_angle, const double _current_heel_angle);
      void cycloid_midcoords (coordinates& ret, const coordinates& start,
                              const coordinates& goal, const double height) const;
      void rectangle_midcoords (coordinates& ret, const coordinates& start,
                                const coordinates& goal, const double height, const size_t swing_trajectory_generator_idx, const coordinates& current_coords, leg_type lr);
      void stair_midcoords (coordinates& ret, const coordinates& start,
                            const coordinates& goal, const double height);
      void cycloid_delay_midcoords (coordinates& ret, const coordinates& start,
                                    const coordinates& goal, const double height, const size_t swing_trajectory_generator_idx);
      void cycloid_delay_kick_midcoords (coordinates& ret, const coordinates& start,
                                    const coordinates& goal, const double height);
      void cross_delay_midcoords (coordinates& ret, const coordinates& start,
                                  const coordinates& goal, const double height, leg_type lr);
      void calc_ratio_from_double_support_ratio (const double default_double_support_ratio_before, const double default_double_support_ratio_after);
      void calc_swing_support_mid_coords ();
#ifndef HAVE_MAIN
    public:
#endif
      leg_coords_generator(const double _dt)
        : support_leg_steps(), swing_leg_steps(), swing_leg_src_steps(), swing_leg_dst_steps(),
          default_step_height(0.05), default_top_ratio(0.5), current_step_height(0.0), swing_ratio(0), dt(_dt),
          current_toe_angle(0), current_heel_angle(0),
          time_offset(0.35), final_distance_weight(1.0), time_offset_xy2z(0),
          footstep_index(0), lcg_count(0), swing_rot_count_ratio(0.1), default_orbit_type(CYCLOID),
          rdtg(), rectangle_way_point_offset(0.05, 0.0, 0.0), cdtg(),
          thp(), use_act_states(true),
          foot_midcoords_interpolator(NULL), swing_foot_rot_interpolator(), toe_heel_interpolator(NULL),
          toe_pos_offset_x(0.0), heel_pos_offset_x(0.0), toe_angle(0.0), heel_angle(0.0), foot_dif_rot_angle(0.0), toe_heel_dif_angle(0.0), use_toe_joint(false), use_toe_heel_auto_set(false),
          current_src_toe_heel_type(SOLE), current_dst_toe_heel_type(SOLE)
      {
        support_leg_types.assign (1, RLEG);
        swing_leg_types.assign (1, LLEG);
        current_swing_time.assign (4, 0.0);
        sdtg.set_dt(dt);
        cdktg.set_dt(dt);
        crdtg.set_dt(dt);
        if (foot_midcoords_interpolator == NULL) foot_midcoords_interpolator = new interpolator(6, dt); // POS + RPY
        std::vector<leg_type> tmp_leg_types = boost::assign::list_of<leg_type>(RLEG)(LLEG)(RARM)(LARM);
         for (size_t i = 0; i < tmp_leg_types.size(); i++) {
             if ( swing_foot_rot_interpolator.find(tmp_leg_types[i]) == swing_foot_rot_interpolator.end() ) {
                 swing_foot_rot_interpolator.insert(std::pair<leg_type, interpolator*>(tmp_leg_types[i], new interpolator(3, dt))); // RPY
                 swing_foot_rot_interpolator[tmp_leg_types[i]]->setName("GaitGenerator swing_foot_rot_interpolator " + leg_type_to_leg_type_string(tmp_leg_types[i]));
                 std::cerr << "GaitGenerator swing_foot_rot_interpolator " + leg_type_to_leg_type_string(tmp_leg_types[i]) << std::endl;;
             }
         }
        //if (foot_ratio_interpolator == NULL) foot_ratio_interpolator = new interpolator(1, dt, interpolator::LINEAR);
        if (toe_heel_interpolator == NULL) toe_heel_interpolator = new interpolator(1, dt);
        foot_midcoords_interpolator->setName("GaitGenerator foot_midcoords_interpolator");
        toe_heel_interpolator->setName("GaitGenerator toe_heel_interpolator");
      };
      ~leg_coords_generator()
      {
        if (foot_midcoords_interpolator != NULL) {
            delete foot_midcoords_interpolator;
            foot_midcoords_interpolator = NULL;
        }
        for (std::map<leg_type, interpolator*>::iterator it = swing_foot_rot_interpolator.begin(); it != swing_foot_rot_interpolator.end(); it++) {
            if (it->second != NULL) {
                delete it->second;
                it->second = NULL;
            }
        }
        if (toe_heel_interpolator != NULL) {
            delete toe_heel_interpolator;
            toe_heel_interpolator = NULL;
        }
      };
      void set_default_step_height (const double _tmp) { default_step_height = _tmp; };
      void set_default_top_ratio (const double _tmp) { default_top_ratio = _tmp; };
      void set_default_orbit_type (const orbit_type _tmp) { default_orbit_type = _tmp; };
      void set_swing_trajectory_delay_time_offset (const double _time_offset)
      {
        sdtg.set_swing_trajectory_delay_time_offset(_time_offset);
        cdktg.set_swing_trajectory_delay_time_offset(_time_offset);
        crdtg.set_swing_trajectory_delay_time_offset(_time_offset);
        time_offset = _time_offset;
      };
      void set_swing_trajectory_final_distance_weight (const double _final_distance_weight)
      {
        sdtg.set_swing_trajectory_final_distance_weight(_final_distance_weight);
        cdktg.set_swing_trajectory_final_distance_weight(_final_distance_weight);
        crdtg.set_swing_trajectory_final_distance_weight(_final_distance_weight);
        final_distance_weight = _final_distance_weight;
      };
      void set_swing_trajectory_time_offset_xy2z (const double _tmp)
      {
        sdtg.set_swing_trajectory_time_offset_xy2z(_tmp);
        cdktg.set_swing_trajectory_time_offset_xy2z(_tmp);
        crdtg.set_swing_trajectory_time_offset_xy2z(_tmp);
        time_offset_xy2z = _tmp;
      };
      void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { sdtg.set_stair_trajectory_way_point_offset(_offset); };
      void set_rectangle_trajectory_way_point_offset (const hrp::Vector3 _offset) { rectangle_way_point_offset = _offset; };
      void set_cycloid_delay_kick_point_offset (const hrp::Vector3 _offset) { cdktg.set_cycloid_delay_kick_point_offset(_offset); };
      void set_toe_pos_offset_x (const double _offx) { toe_pos_offset_x = _offx; };
      void set_heel_pos_offset_x (const double _offx) { heel_pos_offset_x = _offx; };
      void set_toe_angle (const double _angle) { toe_angle = _angle; };
      void set_heel_angle (const double _angle) { heel_angle = _angle; };
      void set_use_toe_joint (const bool ut) { use_toe_joint = ut; };
      void set_use_toe_heel_auto_set (const bool ut) { use_toe_heel_auto_set = ut; };
      void set_swing_support_steps_list (const std::vector< std::vector<step_node> >& fnsl)
      {
          std::vector<step_node> prev_support_leg_steps = support_leg_steps_list.front();
          support_leg_steps_list.clear();
          swing_leg_dst_steps_list.clear();
          support_leg_steps_list.push_back(prev_support_leg_steps);
          swing_leg_dst_steps_list = fnsl;
          for (size_t i = 0; i < fnsl.size(); i++) {
              if (i > 0) {
                  if (is_same_footstep_nodes(fnsl.at(i), fnsl.at(i-1))) {
                      support_leg_steps_list.push_back(support_leg_steps_list.back());
                  } else {
                      /* current support leg steps = prev swing leg dst steps + (prev support leg steps without current swing leg names) */
                      std::vector<step_node> tmp_support_leg_steps = swing_leg_dst_steps_list.at(i-1);
                      std::copy(support_leg_steps_list.back().begin(),
                                support_leg_steps_list.back().end(),
                                std::back_inserter(tmp_support_leg_steps));
                      for (size_t j = 0; j < swing_leg_dst_steps_list.at(i).size(); j++) {
                          std::vector<step_node>::iterator it = std::remove_if(tmp_support_leg_steps.begin(),
                                                                               tmp_support_leg_steps.end(),
                                                                               (&boost::lambda::_1->* &step_node::l_r == swing_leg_dst_steps_list.at(i).at(j).l_r));
                          tmp_support_leg_steps.erase(it, tmp_support_leg_steps.end());
                      }
                      support_leg_steps_list.push_back(tmp_support_leg_steps);
              }
            }
          }
      };
      bool set_toe_heel_phase_ratio (const std::vector<double>& ratio) { return thp.set_toe_heel_phase_ratio(ratio); };
      void set_act_contact_states (const std::vector<bool>& _act_contact_states) {
        if (act_contact_states.empty()) act_contact_states.resize(_act_contact_states.size());
        for (size_t i = 0; i < act_contact_states.size(); i++) {
          act_contact_states[i] = _act_contact_states[i];
        }
      };
      void set_use_act_states (const bool _use_act_states) { use_act_states = _use_act_states; };
      void reset(const size_t _one_step_count, const size_t _next_one_step_count,
                 const std::vector<step_node>& _swing_leg_dst_steps,
                 const std::vector<step_node>& _swing_leg_src_steps,
                 const std::vector<step_node>& _support_leg_steps,
                 const double default_double_support_ratio_before,
                 const double default_double_support_ratio_after)
      {
        support_leg_steps_list.clear();
        swing_leg_dst_steps_list.clear();
        /* swing_leg_steps.clear(); */
        swing_leg_dst_steps = _swing_leg_dst_steps;
        swing_leg_src_steps = _swing_leg_src_steps;
        current_swing_leg_steps = _swing_leg_src_steps;
        support_leg_steps = _support_leg_steps;
        support_leg_steps_list.push_back(support_leg_steps);
        one_step_count = lcg_count = _one_step_count;
        next_one_step_count = _next_one_step_count;
        thp.set_one_step_count(one_step_count);
        footstep_index = 0;
        current_step_height = 0.0;
        switch (default_orbit_type) {
        case RECTANGLE:
            rdtg.clear();
            for (size_t i = 0; i < swing_leg_dst_steps.size(); i++) {
                rdtg.push_back(rectangle_delay_hoffarbib_trajectory_generator());
                rdtg.back().reset_all(dt, one_step_count,
                                      default_double_support_ratio_before, default_double_support_ratio_after,
                                      time_offset, final_distance_weight, time_offset_xy2z);
            }
            break;
        case STAIR:
            sdtg.reset(one_step_count, default_double_support_ratio_before, default_double_support_ratio_after);
            break;
        case CYCLOIDDELAY:
            cdtg.clear();
            for (size_t i = 0; i < swing_leg_dst_steps.size(); i++) {
                cdtg.push_back(cycloid_delay_hoffarbib_trajectory_generator());
                cdtg.back().reset_all(dt, one_step_count,
                                      default_double_support_ratio_before, default_double_support_ratio_after,
                                      time_offset, final_distance_weight, time_offset_xy2z);
            }
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
        current_src_toe_heel_type = current_dst_toe_heel_type = SOLE;
      };
      void clear_interpolators ( ) {
        double tmpsw[3];
        for (std::map<leg_type, interpolator*>::iterator it = swing_foot_rot_interpolator.begin(); it != swing_foot_rot_interpolator.end(); it++) {
            while (!it->second->isEmpty()) it->second->get(tmpsw, true);
        }
        double tmpfm[foot_midcoords_interpolator->dimension()];
        while (!foot_midcoords_interpolator->isEmpty()) {
            foot_midcoords_interpolator->get(tmpfm, true);
        }
        double tmp;
        while (!toe_heel_interpolator->isEmpty()) {
            toe_heel_interpolator->get(&tmp, true);
        }
      };
      bool is_same_footstep_nodes(const std::vector<step_node>& fns_1, const std::vector<step_node>& fns_2) const;
      void update_leg_steps (const std::vector< std::vector<step_node> >& fnsl, const double default_double_support_ratio_before, const double default_double_support_ratio_after, const toe_heel_type_checker& thtc);
      void calc_swing_leg_src_steps (std::vector<step_node>& ret_swing_leg_src_steps, const std::vector< std::vector<step_node> >& fnsl, const size_t _footstep_index)
      {
          if (_footstep_index > 0) {
              if (is_same_footstep_nodes(fnsl[_footstep_index], fnsl[_footstep_index-1])) {
                  ret_swing_leg_src_steps = swing_leg_dst_steps_list[_footstep_index-1];
              } else {
                  /* current swing leg src coords = (previout support leg coords + previous swing leg dst coords) - current support leg coords */
                  std::vector<step_node> tmp_swing_leg_src_steps = support_leg_steps_list[_footstep_index-1];
                  std::copy(swing_leg_dst_steps_list[_footstep_index-1].begin(),
                            swing_leg_dst_steps_list[_footstep_index-1].end(),
                            std::back_inserter(tmp_swing_leg_src_steps));
                  std::vector<step_node> tmp_support_leg_steps = support_leg_steps_list[_footstep_index];
                  for (size_t i = 0; i < tmp_support_leg_steps.size(); i++) {
                      std::vector<step_node>::iterator it = std::remove_if(tmp_swing_leg_src_steps.begin(), tmp_swing_leg_src_steps.end(), (&boost::lambda::_1->* &step_node::l_r == tmp_support_leg_steps.at(i).l_r));
                      tmp_swing_leg_src_steps.erase(it, tmp_swing_leg_src_steps.end());
                  }
                  ret_swing_leg_src_steps = tmp_swing_leg_src_steps;
              }
          }
      };
      void calc_swing_support_params_from_footstep_nodes_list (const std::vector< std::vector<step_node> >& fnsl)
      {
          // Get current swing coords, support coords, and support leg parameters
          size_t current_footstep_index = (footstep_index < fnsl.size() - 1 ? footstep_index : fnsl.size()-1);
          swing_leg_dst_steps = fnsl[current_footstep_index];
          if (footstep_index != 0) { // If not initial step, support_leg_coords is previous swing_leg_dst_coords // why we need this?
              support_leg_steps = support_leg_steps_list[current_footstep_index];
          }
          support_leg_types.clear();
          for (std::vector<step_node>::iterator it = support_leg_steps.begin(); it != support_leg_steps.end(); it++) {
              support_leg_types.push_back(it->l_r);
          }
          swing_leg_types.clear();
          for (std::vector<step_node>::iterator it = swing_leg_dst_steps.begin(); it != swing_leg_dst_steps.end(); it++) {
              swing_leg_types.push_back(it->l_r);
          }
          calc_swing_leg_src_steps(swing_leg_src_steps, fnsl, current_footstep_index);
      };
      size_t get_footstep_index() const { return footstep_index; };
      size_t get_lcg_count() const { return lcg_count; };
      void set_lcg_count (const size_t cnt) { lcg_count = cnt; };
      void set_one_step_count (const size_t d_cnt) {one_step_count += d_cnt; };
      void reset_one_step_count (const size_t d_cnt, const double before, const double after) {
        for (size_t i = 0; i < rdtg.size(); i++)
          rdtg[i].reset_one_step_count(d_cnt, before, after);
      };
      double get_current_swing_time(const size_t idx) const { return current_swing_time.at(idx); };
      const std::vector<step_node>& get_swing_leg_steps() const { return swing_leg_steps; };
      const std::vector<step_node>& get_support_leg_steps() const { return support_leg_steps; };
      const std::vector<step_node>& get_swing_leg_src_steps() const { return swing_leg_src_steps; };
      const std::vector<step_node>& get_swing_leg_dst_steps() const { return swing_leg_dst_steps; };
      const std::vector<step_node>& get_swing_leg_dst_steps_idx(const size_t idx) const { return swing_leg_dst_steps_list[idx]; };
      const std::vector<step_node>& get_support_leg_steps_idx(const size_t idx) const { return support_leg_steps_list[idx]; };
      const std::vector<leg_type>& get_support_leg_types() const { return support_leg_types;};
      const std::vector<leg_type>& get_swing_leg_types() const { return swing_leg_types;};
      double get_default_step_height () const { return default_step_height;};
      void get_swing_support_mid_coords(coordinates& ret) const
      {
        ret = swing_support_midcoords;
      };
      std::vector<leg_type> get_current_support_states () const
      {
          if ( is_swing_phase ) {
              return get_support_leg_types();
          } else {
              std::vector<leg_type> tmp_sup_types = get_support_leg_types();
              std::vector<leg_type> tmp_swg_types = get_swing_leg_types();
              std::copy(tmp_swg_types.begin(), tmp_swg_types.end(), std::back_inserter(tmp_sup_types));
              return tmp_sup_types;
          }
      };
      orbit_type get_default_orbit_type () const { return default_orbit_type; };
      double get_swing_trajectory_delay_time_offset () const { return time_offset; };
      double get_swing_trajectory_final_distance_weight () const { return final_distance_weight; };
      double get_swing_trajectory_time_offset_xy2z () const { return time_offset_xy2z; };
      hrp::Vector3 get_stair_trajectory_way_point_offset () const { return sdtg.get_stair_trajectory_way_point_offset(); };
      hrp::Vector3 get_rectangle_trajectory_way_point_offset () const { return rectangle_way_point_offset; };
      hrp::Vector3 get_cycloid_delay_kick_point_offset () const { return cdktg.get_cycloid_delay_kick_point_offset() ; };
      double get_toe_pos_offset_x () const { return toe_pos_offset_x; };
      double get_heel_pos_offset_x () const { return heel_pos_offset_x; };
      double get_toe_angle () const { return toe_angle; };
      double get_heel_angle () const { return heel_angle; };
      double get_foot_dif_rot_angle () const { return foot_dif_rot_angle; };
      bool get_use_toe_joint () const { return use_toe_joint; };
      bool get_use_toe_heel_auto_set () const { return use_toe_heel_auto_set; };
      void get_toe_heel_phase_ratio (std::vector<double>& ratio) const { thp.get_toe_heel_phase_ratio(ratio); };
      double get_current_toe_heel_ratio (const bool _use_toe_heel_transition) const
      {
          if (_use_toe_heel_transition && current_step_height > 0.0) { // If swing phase
              return thp.calc_phase_ratio_for_toe_heel_transition(one_step_count - lcg_count);
          } else { // If support phase such as double support phase of starting and ending.
              return no_using_toe_heel_ratio;
          }
      };
#ifdef FOR_TESTGAITGENERATOR
      size_t get_one_step_count() const { return one_step_count; };
      double get_toe_heel_dif_angle() const { return toe_heel_dif_angle; };
#endif // FOR_TESTGAITGENERATOR
    };

  class gait_generator
  {

  public:
    bool use_act_states, is_inverse_double_phase, is_emergency_touch_wall, is_stuck, is_interpolate_zmp_in_double, is_stable_go_stop_mode, use_flywheel_balance;
    double zmp_delay_time_const, overwritable_max_time, fg_zmp_cutoff_freq;
    hrp::Vector3 fxy;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > cp_filter;
    double debug_orig_height, debug_landing_height_xrange[2], debug_landing_height;
    bool debug_set_landing_height;
    Eigen::Vector2d front_edge_offset_of_steppable_region;
    bool is_slow_stair_mode;
    double stair_step_time;
    double footguided_balance_time_const;
#ifndef HAVE_MAIN
  private:
#endif

    enum velocity_mode_flag { VEL_IDLING, VEL_DOING, VEL_ENDING };
    enum emergency_flag { IDLING, EMERGENCY_STOP, STOPPING };
    enum projected_point_region {LEFT, MIDDLE, RIGHT};

    /* member variables for gait_generator */
    // Footstep list to be executed
    //   First and last footstep are used for double support phase.
    std::vector< std::vector<step_node> > footstep_nodes_list;
    // Footstep list for overwriting future footstep queue
    std::vector< std::vector<step_node> > overwrite_footstep_nodes_list;
    refzmp_generator rg;
    leg_coords_generator lcg;
    footstep_parameter footstep_param;
    velocity_mode_parameter vel_param, offset_vel_param;
    toe_heel_type_checker thtc;
    hrp::Vector3 cog, refzmp, prev_que_rzmp, zmp; /* cog by calculating proc_one_tick */
    std::vector<hrp::Vector3> swing_foot_zmp_offsets, prev_que_sfzos;
    double dt; /* control loop [s] */
    std::vector<std::string> all_limbs;
    double default_step_time, orig_default_step_time;
    double default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after, orig_default_double_support_static_ratio_before, orig_default_double_support_static_ratio_after;
    double default_double_support_ratio_swing_before; /*first double support time for leg coords generator */
    double default_double_support_ratio_swing_after; /*last double support time for leg coords generator */
    double gravitational_acceleration;
    size_t finalize_count, optional_go_pos_finalize_footstep_num;
    // overwrite_footstep_index is used for footstep overwriting.
    //   When overwrite_footstep_index == get_overwritable_index(), overwrite footsteps after overwrite_footstep_index.
    size_t overwrite_footstep_index;
    // overwritable_footstep_index_offset is used for emergency stop and velocity mode.
    //   overwritable footstep index is "footstep_index + overwritable_footstep_index_offset", which is obtained by get_overwritable_index().
    size_t overwritable_footstep_index_offset;
    velocity_mode_flag velocity_mode_flg;
    emergency_flag emergency_flg;
    bool use_inside_step_limitation;
    std::map<leg_type, std::string> leg_type_map;
    coordinates initial_foot_mid_coords;
    bool solved, is_first_count, is_preview, is_set_first_count, is_first_double_after, is_double_support_phase, is_after_double_support_phase, was_enlarged_time, is_prev_double_remain_one;
    size_t remain_count;
    double leg_margin[4], safe_leg_margin[4], stride_limitation_for_circle_type[5], overwritable_stride_limitation[5], footstep_modification_gain, cp_check_margin[2], margin_time_ratio;
    bool use_stride_limitation, is_emergency_walking[2], modify_footsteps, is_emergency_step;
    hrp::Vector3 diff_cp, modified_d_footstep, sum_d_footstep_plus, sum_d_footstep_minus, sum_d_footstep_thre, footstep_hist_max, footstep_hist_min, footstep_check_delta;
    double modified_d_step_time, min_time_mgn, min_time, orig_min_time, emergency_step_time[3];
    std::vector<bool> act_contact_states;
    stride_limitation_type default_stride_limitation_type;
    double act_vel_ratio, double_remain_count_offset, fg_double_remain_count;
    hrp::Vector3 fg_ref_zmp, prev_fg_ref_zmp, fg_start_ref_zmp, prev_start_ref_zmp, fg_goal_ref_zmp, prev_ref_dcm, flywheel_tau, prev_short_of_zmp, ref_cp, act_cp, prev_act_cp, sum_fx, sum_fy, des_fxy, ref_footstep_offset;
    bool updated_vel_footsteps, use_roll_flywheel, use_pitch_flywheel, use_disturbance_compensation, prev_use_roll_flywheel, prev_use_pitch_flywheel;
    std::vector<std::vector<Eigen::Vector2d> > foot_vertices;
    std::vector<Eigen::Vector2d> convex_hull;
    size_t fg_step_count, falling_direction;
    double total_mass, dc_gain, dcm_offset;
    double tmp[23];
    hrp::Vector3 rel_landing_pos, end_cog, end_cogvel;
    int cur_supporting_foot, fx_count, fy_count;
    bool is_vision_updated, lr_region[2];
    bool changed_step_time_stair;
    std::vector<Eigen::Vector2d> stride_limitation_polygon;
    hrp::Vector3 dc_foot_rpy, dc_landing_pos, orig_current_foot_rpy, vel_foot_offset, rel_landing_height, rel_landing_normal;
    std::vector<std::vector<std::vector<hrp::Vector2> > > steppable_region;
    std::vector<std::vector<std::vector<hrp::Vector2> > > debug_current_steppable_region;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > fx_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > zmp_filter;
    boost::shared_ptr<interpolator> double_support_zmp_interpolator;
    boost::shared_ptr<interpolator> wheel_interpolator;

    // wheel
    std::vector< std::vector<wheel_node> > wheel_nodes_list;
    int wheel_index;
    double cur_wheel_ratio, cur_wheel_pos_x, start_wheel_pos_x;

    /* preview controller parameters */
    //preview_dynamics_filter<preview_control>* preview_controller_ptr;
    preview_dynamics_filter<extended_preview_control>* preview_controller_ptr;
    foot_guided_controller<3>* foot_guided_controller_ptr;

    void append_go_pos_step_nodes (const coordinates& _ref_coords,
                                   const std::vector<leg_type>& lts)
    {
        append_go_pos_step_nodes(_ref_coords, lts, footstep_nodes_list);
    };

    void append_go_pos_step_nodes (const coordinates& _ref_coords,
                                   const std::vector<leg_type>& lts,
                                   std::vector< std::vector<step_node> >& _footstep_nodes_list) const
    {
      std::vector<step_node> sns;
      for (size_t i = 0; i < lts.size(); i++) {
          sns.push_back(step_node(lts.at(i), _ref_coords,
                                  lcg.get_default_step_height(), default_step_time,
                                  lcg.get_toe_angle(), lcg.get_heel_angle()));
          // sns.at(i).worldcoords.pos += sns.at(i).worldcoords.rot * footstep_param.leg_default_translate_pos[lts.at(i)];
          sns.at(i).worldcoords.pos += sns.at(i).worldcoords.rot * footstep_param.vel_foot_offset[lts.at(i)];
      }
      _footstep_nodes_list.push_back(sns);
    };
    void overwrite_refzmp_queue(const std::vector< std::vector<step_node> >& fnsl, const hrp::Vector3& cur_cog = hrp::Vector3::Zero(), const hrp::Vector3& cur_cogvel = hrp::Vector3::Zero(), const hrp::Vector3& cur_refcog = hrp::Vector3::Zero(), const hrp::Vector3& cur_refcogvel = hrp::Vector3::Zero(), const hrp::Vector3& cur_cmp = hrp::Vector3::Zero(), const bool& update_vel = false);
    void calc_ref_coords_trans_vector_velocity_mode (coordinates& ref_coords, hrp::Vector3& trans, double& dth, const std::vector<step_node>& sup_fns, const velocity_mode_parameter& cur_vel_param) const;
    void calc_next_coords_velocity_mode (std::vector< std::vector<step_node> >& ret_list, const size_t idx, const size_t future_step_num = 3);
    void append_footstep_list_velocity_mode ();
    void append_footstep_list_velocity_mode (std::vector< std::vector<step_node> >& _footstep_nodes_list, const velocity_mode_parameter& cur_vel_param) const;
    inline leg_type get_leg_type_from_ee_name (const std::string& ee_name) const
    {
        return std::find_if(leg_type_map.begin(), leg_type_map.end(), (&boost::lambda::_1->* &std::map<leg_type, std::string>::value_type::second == ee_name))->first;
    };

#ifndef HAVE_MAIN
    /* inhibit copy constructor and copy insertion not by implementing */
    gait_generator (const gait_generator& _p);
    gait_generator &operator=(const gait_generator &_p);
  public:
#endif
    gait_generator (double _dt,
                    /* arguments for footstep_parameter */
                    const std::vector<hrp::Vector3>& _leg_pos, std::vector<std::string> _all_limbs,
                    const double _stride_fwd_x, const double _stride_outside_y, const double _stride_outside_theta,
                    const double _stride_bwd_x, const double _stride_inside_y, const double _stride_inside_theta)
        : footstep_nodes_list(), overwrite_footstep_nodes_list(), rg(_dt), lcg(_dt),
        footstep_param(_leg_pos, _stride_fwd_x, _stride_outside_y, _stride_outside_theta, _stride_bwd_x, _stride_inside_y, _stride_inside_theta),
        vel_param(), offset_vel_param(), thtc(), cog(hrp::Vector3::Zero()), refzmp(hrp::Vector3::Zero()), prev_que_rzmp(hrp::Vector3::Zero()), diff_cp(hrp::Vector3::Zero()), modified_d_footstep(hrp::Vector3::Zero()), zmp(hrp::Vector3::Zero()), modified_d_step_time(0.0),
        dt(_dt), all_limbs(_all_limbs), default_step_time(1.0), default_double_support_ratio_before(0.1), default_double_support_ratio_after(0.1), default_double_support_static_ratio_before(0.0), default_double_support_static_ratio_after(0.0), default_double_support_ratio_swing_before(0.1), default_double_support_ratio_swing_after(0.1), gravitational_acceleration(DEFAULT_GRAVITATIONAL_ACCELERATION),
        finalize_count(0), optional_go_pos_finalize_footstep_num(0), overwrite_footstep_index(0), overwritable_footstep_index_offset(1), is_emergency_step(false),
        velocity_mode_flg(VEL_IDLING), emergency_flg(IDLING), margin_time_ratio(0.01), footstep_modification_gain(5e-6), act_vel_ratio(1.0), use_disturbance_compensation(false), dc_gain(1e-4),
        use_inside_step_limitation(true), use_stride_limitation(false), modify_footsteps(false), default_stride_limitation_type(SQUARE), is_first_count(false), is_first_double_after(true), double_remain_count_offset(0.0), use_roll_flywheel(false), use_pitch_flywheel(false), dcm_offset(0.0), rel_landing_pos(hrp::Vector3::Zero()), cur_supporting_foot(0), is_vision_updated(false), rel_landing_height(hrp::Vector3::Zero()), rel_landing_normal(hrp::Vector3::UnitZ()), zmp_delay_time_const(0.0), is_inverse_double_phase(false), overwritable_max_time(2.0), fg_zmp_cutoff_freq(1e6), is_emergency_touch_wall(false), end_cog(hrp::Vector3::Zero()), end_cogvel(hrp::Vector3::Zero()), sum_d_footstep_plus(hrp::Vector3::Zero()), sum_d_footstep_minus(hrp::Vector3::Zero()), footstep_hist_max(hrp::Vector3::Zero()), footstep_hist_min(hrp::Vector3::Zero()), is_stuck(false), is_interpolate_zmp_in_double(true), is_stable_go_stop_mode(false), use_flywheel_balance(false), stair_step_time(0.5), footguided_balance_time_const(0.3), is_slow_stair_mode(false), changed_step_time_stair(false),
        preview_controller_ptr(NULL), foot_guided_controller_ptr(NULL), is_preview(false), updated_vel_footsteps(false), min_time_mgn(0.2), min_time(0.5), flywheel_tau(hrp::Vector3::Zero()), falling_direction(0), dc_foot_rpy(hrp::Vector3::Zero()), dc_landing_pos(hrp::Vector3::Zero()), sum_fx(hrp::Vector3::Zero()), sum_fy(hrp::Vector3::Zero()), des_fxy(hrp::Vector3::Zero()), fx_count(0), fy_count(0), debug_set_landing_height(false), debug_landing_height(0.0), front_edge_offset_of_steppable_region(Eigen::Vector2d::Zero()) {
        swing_foot_zmp_offsets.assign (1, hrp::Vector3::Zero());
        prev_que_sfzos.assign (1, hrp::Vector3::Zero());
        leg_type_map = boost::assign::map_list_of<leg_type, std::string>(RLEG, "rleg")(LLEG, "lleg")(RARM, "rarm")(LARM, "larm").convert_to_container < std::map<leg_type, std::string> > ();
        stride_limitation_polygon.resize(4);
        steppable_region.resize(2);
        debug_current_steppable_region.resize(2);
        for (size_t i = 0; i < 4; i++) leg_margin[i] = 0.1;
        for (size_t i = 0; i < 4; i++) safe_leg_margin[i] = 0.1;
        for (size_t i = 0; i < 5; i++) stride_limitation_for_circle_type[i] = 0.2;
        for (size_t i = 0; i < 5; i++) overwritable_stride_limitation[i] = 0.2;
        for (size_t i = 0; i < 2; i++) is_emergency_walking[i] = false;
        for (size_t i = 0; i < 2; i++) cp_check_margin[i] = 0.025;
        for (size_t i = 0; i < 3; i++) emergency_step_time[i] = 0.8;
        for (size_t i = 0; i < 2; i++) debug_landing_height_xrange[i] = 0.0;
        sum_d_footstep_thre = hrp::Vector3(0.5, 0.5, 0.0);
        footstep_check_delta = hrp::Vector3(0.03, 0.03, 0.0);
        cp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(1e6, _dt, hrp::Vector3::Zero()));
        fx_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(0.1, _dt, hrp::Vector3::Zero()));
        zmp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, _dt, hrp::Vector3::Zero()));
        double_support_zmp_interpolator = boost::shared_ptr<interpolator>(new interpolator(1, dt, interpolator::HOFFARBIB));
        double_support_zmp_interpolator->setName("GaitGenerator double_support_zmp_interpolator");
        wheel_interpolator = boost::shared_ptr<interpolator>(new interpolator(1, dt, interpolator::LINEAR));
        wheel_interpolator->setName("GaitGenerator wheel_interpolator");
    };
    ~gait_generator () {
      if ( preview_controller_ptr != NULL ) {
        delete preview_controller_ptr;
        preview_controller_ptr = NULL;
      }
      if ( foot_guided_controller_ptr != NULL ) {
        delete foot_guided_controller_ptr;
        foot_guided_controller_ptr = NULL;
      }
    };
    void initialize_gait_parameter (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_refcog,
                                    const std::vector<step_node>& initial_support_leg_steps,
                                    const std::vector<step_node>& initial_swing_leg_dst_steps,
                                    const double delay = 1.6);
    bool proc_one_tick (hrp::Vector3 cur_cog = hrp::Vector3::Zero(), const hrp::Vector3& cur_cogvel = hrp::Vector3::Zero(), const hrp::Vector3& cur_cmp = hrp::Vector3::Zero());
    void initialize_wheel_parameter (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_refcog);
    bool proc_one_tick_wheel (hrp::Vector3 cur_cog = hrp::Vector3::Zero(), const hrp::Vector3& cur_cogvel = hrp::Vector3::Zero());
    void update_preview_controller(bool& solved);
    void update_foot_guided_controller(bool& solved, const hrp::Vector3& cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_refcog, const hrp::Vector3& cur_refcogvel, const hrp::Vector3& cur_cmp);
    void calc_foot_origin_rot (hrp::Matrix33& foot_rot, const hrp::Matrix33& orig_rot, const hrp::Vector3& n) const;
    void set_first_count_flag ();
    void min_time_check (double& tmp_dt);
    void change_step_time (const double& tmp_dt);
    void limit_stride (step_node& cur_fs, const step_node& prev_fs, const double (&limit)[5]) const;
    void limit_stride_rectangle (step_node& cur_fs, const step_node& prev_fs, const double (&limit)[5]) const;
    void limit_stride_vision (step_node& cur_fs, hrp::Vector3& short_of_footstep, const step_node& prev_fs, const step_node& preprev_fs, const double& omega, const hrp::Vector3& cur_cp);
    void modify_footsteps_for_recovery ();
    void modify_footsteps_for_foot_guided (const hrp::Vector3& cur_cog, const hrp::Vector3& cur_cogvel, const hrp::Vector3& cur_refcog, const hrp::Vector3& cur_refcogvel, const hrp::Vector3& cur_cmp);
    void append_footstep_nodes (const std::vector<std::string>& _legs, const std::vector<coordinates>& _fss)
    {
        std::vector<step_node> tmp_sns;
        for (size_t i = 0; i < _legs.size(); i++) {
            tmp_sns.push_back(step_node(_legs[i], _fss[i], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle()));
        }
        footstep_nodes_list.push_back(tmp_sns);
    };
    void append_footstep_nodes (const std::vector<std::string>& _legs, const std::vector<coordinates>& _fss, const double _step_height, const double _step_time, const double _toe_angle, const double _heel_angle)
    {
        std::vector<step_node> tmp_sns;
        for (size_t i = 0; i < _legs.size(); i++) {
            tmp_sns.push_back(step_node(_legs[i], _fss[i], _step_height, _step_time, _toe_angle, _heel_angle));
        }
        footstep_nodes_list.push_back(tmp_sns);
    };
    void clear_footstep_nodes_list () {
        footstep_nodes_list.clear();
        overwrite_footstep_nodes_list.clear();
        overwrite_footstep_index = 0;
    };
    bool go_pos_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                             const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                             const std::vector<leg_type>& initial_support_legs,
                                             const bool is_initialize = true) {
        std::vector< std::vector<step_node> > new_footstep_nodes_list;
        return go_pos_param_2_footstep_nodes_list (goal_x, goal_y, goal_theta,
                                                   initial_support_legs_coords, start_ref_coords,
                                                   initial_support_legs,
                                                   new_footstep_nodes_list,
                                                   is_initialize);
    };
    bool go_pos_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                             const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                             const std::vector<leg_type>& initial_support_legs,
                                             std::vector< std::vector<step_node> >& new_footstep_nodes_list,
                                             const bool is_initialize = true);
    void go_pos_param_2_footstep_nodes_list_core (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                                  const std::vector<coordinates>& initial_support_legs_coords, coordinates start_ref_coords,
                                                  const std::vector<leg_type>& initial_support_legs,
                                                  std::vector< std::vector<step_node> >& new_footstep_nodes_list,
                                                  const bool is_initialize, const size_t overwritable_fs_index) const;
    void go_single_step_param_2_footstep_nodes_list (const double goal_x, const double goal_y, const double goal_z, const double goal_theta, /* [mm] [mm] [mm] [deg] */
                                               const std::string& tmp_swing_leg,
                                               const coordinates& _support_leg_coords);
    bool go_wheel_param_2_wheel_nodes_list (const double goal_x, const double whole_time, const coordinates& start_ref_coords);
    void initialize_velocity_mode (const coordinates& _ref_coords,
				   const double vel_x, const double vel_y, const double vel_theta, /* [mm/s] [mm/s] [deg/s] */
                                   const std::vector<leg_type>& current_legs);
    void finalize_velocity_mode ();
    void finalize_velocity_mode2 ();
    void append_finalize_footstep ()
    {
      append_finalize_footstep(footstep_nodes_list);
    };
    void append_finalize_footstep (std::vector< std::vector<step_node> >& _footstep_nodes_list) const
    {
      std::vector<step_node> sns = _footstep_nodes_list[_footstep_nodes_list.size()-2];
      for (size_t i = 0; i < sns.size(); i++) {
          sns.at(i).step_height = sns.at(i).toe_angle = sns.at(i).heel_angle = 0.0;
      }
      _footstep_nodes_list.push_back(sns);
    };
    void emergency_stop ()
    {
      if (!footstep_nodes_list.empty()) {
        velocity_mode_flg = VEL_IDLING;
        emergency_flg = EMERGENCY_STOP;
      }
    };
    double calcCrossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& o = Eigen::Vector2d::Zero())
    {
      return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
    };
    projected_point_region calcProjectedPoint(Eigen::Vector2d& ret, const Eigen::Vector2d& target, const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
      Eigen::Vector2d v1 = target - b;
      Eigen::Vector2d v2 = a - b;
      double v2_norm = v2.norm();
      if ( v2_norm == 0 ) {
        ret = a;
        return LEFT;
      } else {
        double ratio = v1.dot(v2) / (v2_norm * v2_norm);
        if (ratio < 0){
          ret = b;
          return RIGHT;
        } else if (ratio > 1){
          ret = a;
          return LEFT;
        } else {
          ret = b + ratio * v2;
          return MIDDLE;
        }
      }
    };
    // Assume that the oder of ch is CCW or CW
    bool is_inside_polygon (Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& ch, const hrp::Vector3& offset = hrp::Vector3::Zero(), const bool& truncate_p = false)
    {
      bool is_inside = false;
      for (int i = 0; i < ch.size(); i++) {
        Eigen::Vector2d a = ch[i] - p, b = ch[(i+1)%ch.size()] - p;
        if (a(1) > b(1)) swap(a, b);
        if (a(1) <= 0 && 0 < b(1))
          if (calcCrossProduct(a, b) < 0) is_inside = !is_inside;
        // if (calcCrossProduct(a, b) == 0 && a.dot(b) <= 0) return true; // On edge
      }
      return is_inside;
    }
    bool is_inside_convex_hull (Eigen::Vector2d& p, const hrp::Vector3& offset = hrp::Vector3::Zero(), const bool& truncate_p = false)
    {
      double tmp;
      hrp::Vector3 tmp_pos;
      return is_inside_convex_hull(p, tmp, tmp_pos, convex_hull, offset, truncate_p);
    };
    bool is_inside_convex_hull (Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& ch, const hrp::Vector3& offset = hrp::Vector3::Zero(), const bool& truncate_p = false)
    {
      double tmp;
      hrp::Vector3 tmp_pos;
      return is_inside_convex_hull(p, tmp, tmp_pos, ch, offset, truncate_p);
    };
    bool is_inside_convex_hull (Eigen::Vector2d& p, double& t, hrp::Vector3& short_of_footstep, const std::vector<Eigen::Vector2d>& ch, const hrp::Vector3& offset = hrp::Vector3::Zero(), const bool& truncate_p = false, const bool& change_time = false, const double& r = 0.0, const double& omega = 0.0, const hrp::Vector3& cur_cp = hrp::Vector3::Zero(), const hrp::Vector3& prev_fs_pos = hrp::Vector3::Zero(), const hrp::Matrix33& prev_fs_rot = hrp::Matrix33::Ones(), const leg_type cur_sup = RLEG)
    {
      // set any inner point(ip) and binary search two vertices(ch[v_a], ch[v_b]) between which p is.
      p -= offset.head(2);
      size_t n_ch = ch.size();
      Eigen::Vector2d ip = (ch[0] + ch[n_ch/3] + ch[2*n_ch/3]) / 3.0;
      size_t v_a = 0, v_b = n_ch;
      while (v_a + 1 < v_b) {
        size_t v_c = (v_a + v_b) / 2;
        if (calcCrossProduct(ch[v_a], ch[v_c], ip) > 0) {
          if (calcCrossProduct(ch[v_a], p, ip) > 0 && calcCrossProduct(ch[v_c], p, ip) < 0) v_b = v_c;
          else v_a = v_c;
        } else {
          if (calcCrossProduct(ch[v_a], p, ip) < 0 && calcCrossProduct(ch[v_c], p, ip) > 0) v_a = v_c;
          else v_b = v_c;
        }
      }
      v_b %= n_ch;
      if (calcCrossProduct(ch[v_a], ch[v_b], p) >= 0) {
        p += offset.head(2);
        return true;
      } else {
        if (truncate_p) {
          if (!calc_closest_boundary_point(p, ch, v_a, v_b)) std::cerr << "Cannot calculate closest boundary point on the convex hull" << std::endl;
        } else if (change_time) { // TODO: should consider dcm_off/zmp_offset
          // prev foot frame (do not consider prev foot rot)
          Eigen::Vector2d pa = ch[v_a] - prev_fs_pos.head(2), pb = ch[v_b] - prev_fs_pos.head(2), cp = cur_cp.head(2) - prev_fs_pos.head(2);
          double tmp = fabs(pb(0) - pa(0)) < 1e-3 ? 1e-3 : (pb(0) - pa(0)); // limit 1[mm]
          double a = (pb(1) - pa(1)) / tmp, b = pa(1) - pa(0) * (pb(1) - pa(1)) / tmp;
          p -= prev_fs_pos.head(2);
          double ar = (a*a+1)*r*r;
          double A = ar - std::pow(a*cp(0)-cp(1),2.0);
          double B = -2 * (a*b*cp(0)-b*cp(1)+ar);
          double C = ar - b*b;
          double ewt = std::exp(omega*t);
          double D = A * ewt * ewt + B * ewt + C;
          double tmp_dt = 0.0;
          if (D >= 0) {
            Eigen::Vector2d tmp_pos = ewt * cp;
            // preprev foot frame
            tmp_pos += prev_fs_pos.head(2);
            if (!calc_closest_boundary_point(tmp_pos, ch, v_a, v_b)) std::cerr << "Cannot calculate closest boundary point on the convex hull" << std::endl;
            p = tmp_pos;
          } else {
            D = B*B - 4*A*C;
            if (D >= 0) {
              double tmp_t = std::log((-B + std::sqrt(D)) / (2*A)) / omega;
              if (isnan(tmp_t)) tmp_t = std::log((-B - std::sqrt(D)) / (2*A)) / omega;
              if (isfinite(tmp_t)) {
                p(0) = ((cp(0) + a * cp(1)) * std::exp(omega * tmp_t) - a * b) / (a*a + 1);
                p(1) = a * p(0) + b;
              }
            }
            // preprev foot frame
            p += prev_fs_pos.head(2);
          }
          // project to line segment
          for (size_t i = 0; i < 2; i++) {
            if (p(i) > std::max(ch[v_a](i), ch[v_b](i)))      p(i) = std::max(ch[v_a](i), ch[v_b](i));
            else if (p(i) < std::min(ch[v_a](i), ch[v_b](i))) p(i) = std::min(ch[v_a](i), ch[v_b](i));
          }
          // prev foot frame (consider prev foot rot)
          hrp::Vector3 end_cp = prev_fs_rot.transpose() * (hrp::Vector3(p(0), p(1), 0) - prev_fs_pos);
          hrp::Vector3 rel_cp = prev_fs_rot.transpose() * (cur_cp - prev_fs_pos);
          double end_cp_front = std::exp(omega * t) * rel_cp(0) - (std::exp(omega * t) - 1) * safe_leg_margin[0];
          double end_cp_back = std::exp(omega * t) * rel_cp(0) + (std::exp(omega * t) - 1) * safe_leg_margin[1];
          double end_cp_outside = std::exp(omega * t) * rel_cp(1) - (cur_sup == RLEG ? -1 : 1) * (std::exp(omega * t) - 1) * safe_leg_margin[2];
          double end_cp_inside = std::exp(omega * t) * rel_cp(1) - (cur_sup == RLEG ? 1 : -1) * (std::exp(omega * t) - 1) * safe_leg_margin[3];
          bool is_change_time = false;
          double xz_max, l_max;
          // calc remain_time
          if (end_cp(0) < end_cp_front) {
            xz_max = safe_leg_margin[0];
            l_max = end_cp(0);
            double tmp_t = std::log((l_max - xz_max) / (rel_cp(0) - xz_max)) / omega;
            if (std::fabs(tmp_t - t) > tmp_dt) tmp_dt = tmp_t - t;
            is_change_time = true;
          } else if (end_cp(0) > end_cp_back) {
            xz_max = -1 * safe_leg_margin[1];
            l_max = end_cp(0);
            double tmp_t = std::log((l_max - xz_max) / (rel_cp(0) - xz_max)) / omega;
            if (std::fabs(tmp_t - t) > tmp_dt) tmp_dt = tmp_t - t;
            is_change_time = true;
          }
          if ((cur_sup == RLEG ? -1 : 1) * end_cp(1) < (cur_sup == RLEG ? -1 : 1) * end_cp_outside) {
            xz_max = (cur_sup == RLEG ? -1 : 1) * safe_leg_margin[2];
            l_max = end_cp(1);
            double tmp_t = std::log((l_max - xz_max) / (rel_cp(1) - xz_max)) / omega;
            if (std::fabs(tmp_t - t) > tmp_dt) tmp_dt = tmp_t - t;
            is_change_time = true;
          } else if ((cur_sup == RLEG ? -1 : 1) * end_cp(1) > (cur_sup == RLEG ? -1 : 1) * end_cp_inside) {
            xz_max = (cur_sup == RLEG ? 1 : -1) * safe_leg_margin[3];
            l_max = end_cp(1);
            double tmp_t = std::log((l_max - xz_max) / (rel_cp(1) - xz_max)) / omega;
            if (std::fabs(tmp_t - t) > tmp_dt) tmp_dt = tmp_t - t;
            is_change_time = true;
          }
          if (is_change_time) {
            if (tmp_dt < 0) min_time_check(tmp_dt);
            t = t + tmp_dt;

            // calc target pos according t
            hrp::Vector3 target_p = end_cp;
            end_cp_front = std::exp(omega * t) * rel_cp(0) - (std::exp(omega * t) - 1) * safe_leg_margin[0];
            end_cp_back = std::exp(omega * t) * rel_cp(0) + (std::exp(omega * t) - 1) * safe_leg_margin[1];
            end_cp_outside = std::exp(omega * t) * rel_cp(1) - (cur_sup == RLEG ? -1 : 1) * (std::exp(omega * t) - 1) * safe_leg_margin[2];
            end_cp_inside = std::exp(omega * t) * rel_cp(1) - (cur_sup == RLEG ? 1 : -1) * (std::exp(omega * t) - 1) * safe_leg_margin[3];
            if (end_cp(0) < end_cp_front) {
              xz_max = safe_leg_margin[0];
              target_p(0) = std::exp(omega * t) * rel_cp(0) + (1 - std::exp(omega * t)) * xz_max;
            } else if (end_cp(0) > end_cp_back) {
              xz_max = -1 * safe_leg_margin[1];
              target_p(0) = std::exp(omega * t) * rel_cp(0) + (1 - std::exp(omega * t)) * xz_max;
            }
            if ((cur_sup == RLEG ? -1 : 1) * end_cp(1) < (cur_sup == RLEG ? -1 : 1) * end_cp_outside) {
              xz_max = (cur_sup == RLEG ? -1 : 1) * safe_leg_margin[2];
              target_p(1) = std::exp(omega * t) * rel_cp(1) + (1 - std::exp(omega * t)) * xz_max;
            } else if ((cur_sup == RLEG ? -1 : 1) * end_cp(1) > (cur_sup == RLEG ? -1 : 1) * end_cp_inside) {
              xz_max = (cur_sup == RLEG ? 1 : -1) * safe_leg_margin[3];
              target_p(1) = std::exp(omega * t) * rel_cp(1) + (1 - std::exp(omega * t)) * xz_max;
            }
            // preprev foot frame
            target_p = prev_fs_pos + prev_fs_rot * target_p;
            short_of_footstep.head(2) = p - target_p.head(2);
          }
        }
        p += offset.head(2);
        return false;
      }
    };
    std::vector<Eigen::Vector2d> calc_intersect_convex (std::vector<Eigen::Vector2d>& P, std::vector<Eigen::Vector2d>& Q);
    bool is_intersect (Eigen::Vector2d& r, const Eigen::Vector2d& a0, const Eigen::Vector2d& a1, const Eigen::Vector2d& b0, const Eigen::Vector2d& b1);
    // Compare Vector2d for sorting lexicographically
    static bool compare_eigen2d(const Eigen::Vector2d& lv, const Eigen::Vector2d& rv)
    {
      return lv(0) < rv(0) || (lv(0) == rv(0) && lv(1) < rv(1));
    };
    // Calculate 2D convex hull based on Andrew's algorithm
    // Assume that the order of vs, ee, and cs is the same
    void calc_convex_hull (const std::vector<std::vector<Eigen::Vector2d> >& vs, const std::vector<bool>& cs, const std::vector<hrp::Vector3>& ee_pos, const std::vector <hrp::Matrix33>& ee_rot)
    {
      // transform coordinate
      std::vector<Eigen::Vector2d>  tvs;
      hrp::Vector3 tpos;
      tvs.reserve(cs.size()*vs[0].size());
      for (size_t i = 0; i < cs.size(); i++) {
        if (cs[i]) {
          for (size_t j = 0; j < vs[i].size(); j++) {
            tpos = ee_pos[i] + ee_rot[i] * hrp::Vector3(vs[i][j](0), vs[i][j](1), 0.0);
            tvs.push_back(tpos.head(2));
          }
        }
      }
      // calculate convex hull
      int n_tvs = tvs.size(), n_ch = 0;
      convex_hull.resize(2*n_tvs);
      std::sort(tvs.begin(), tvs.end(), compare_eigen2d);
      for (int i = 0; i < n_tvs; convex_hull[n_ch++] = tvs[i++])
        while (n_ch >= 2 && calcCrossProduct(convex_hull[n_ch-1], tvs[i], convex_hull[n_ch-2]) <= 0) n_ch--;
      for (int i = n_tvs-2, j = n_ch+1; i >= 0; convex_hull[n_ch++] = tvs[i--])
        while (n_ch >= j && calcCrossProduct(convex_hull[n_ch-1], tvs[i], convex_hull[n_ch-2]) <= 0) n_ch--;
      convex_hull.resize(n_ch-1);
    };
    // Calculate closest boundary point on the convex hull
    bool calc_closest_boundary_point (Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& ch, size_t& right_idx, size_t& left_idx) {
      size_t n_ch = ch.size();
      Eigen::Vector2d cur_closest_point;
      for (size_t i = 0; i < n_ch; i++) {
        switch(calcProjectedPoint(cur_closest_point, p, ch[left_idx], ch[right_idx])) {
        case MIDDLE:
          p = cur_closest_point;
          return true;
        case LEFT:
          right_idx = left_idx;
          left_idx = (left_idx + 1) % n_ch;
          if ((p - ch[right_idx]).dot(ch[left_idx] - ch[right_idx]) <= 0) {
            p = cur_closest_point;
            return true;
          }
        case RIGHT:
          left_idx = right_idx;
          right_idx = (right_idx - 1) % n_ch;
          if ((p - ch[left_idx]).dot(ch[right_idx] - ch[left_idx]) <= 0) {
            p = cur_closest_point;
            return true;
          }
        }
      }
      return false;
    };
    // Retrieving only
    hrp::Vector3 calcDampingControl (const hrp::Vector3 prev_d, const double TT)
    {
      return - 1/TT * prev_d * dt + prev_d;
    };
    /* parameter setting */
    void set_default_step_time (const double _default_step_time) { default_step_time = _default_step_time; };
    void set_default_double_support_ratio_before (const double _default_double_support_ratio_before) { default_double_support_ratio_before = _default_double_support_ratio_before; };
    void set_default_double_support_ratio_after (const double _default_double_support_ratio_after) { default_double_support_ratio_after = _default_double_support_ratio_after; };
    void set_default_double_support_static_ratio_before (const double _default_double_support_static_ratio_before) { default_double_support_static_ratio_before = _default_double_support_static_ratio_before; };
    void set_default_double_support_static_ratio_after (const double _default_double_support_static_ratio_after) { default_double_support_static_ratio_after = _default_double_support_static_ratio_after; };
    void set_default_double_support_ratio_swing_before (const double _default_double_support_ratio_swing_before) { default_double_support_ratio_swing_before = _default_double_support_ratio_swing_before; };
    void set_default_double_support_ratio_swing_after (const double _default_double_support_ratio_swing_after) { default_double_support_ratio_swing_after = _default_double_support_ratio_swing_after; };
    void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { rg.set_default_zmp_offsets(tmp); };
    void set_toe_zmp_offset_x (const double _off) { rg.set_toe_zmp_offset_x(_off); };
    void set_heel_zmp_offset_x (const double _off) { rg.set_heel_zmp_offset_x(_off); };
    void set_use_toe_heel_transition (const bool _u) { rg.set_use_toe_heel_transition(_u); };
    void set_use_toe_heel_auto_set (const bool _u) { rg.set_use_toe_heel_auto_set(_u); lcg.set_use_toe_heel_auto_set(_u); };
    void set_zmp_weight_map (const std::map<leg_type, double> _map) { rg.set_zmp_weight_map(_map); };
    void set_default_step_height(const double _tmp) { lcg.set_default_step_height(_tmp); };
    void set_default_top_ratio(const double _tmp) { lcg.set_default_top_ratio(_tmp); };
    void set_velocity_param (const double vel_x, const double vel_y, const double vel_theta) /* [mm/s] [mm/s] [deg/s] */
    {
      vel_param.set(vel_x, vel_y, vel_theta);
    };
    void set_offset_velocity_param (const double vel_x, const double vel_y, const double vel_theta) /* [mm/s] [mm/s] [deg/s] */
    {
      offset_vel_param.set(vel_x, vel_y, vel_theta);
    };
    void set_stride_parameters (const double _stride_fwd_x, const double _stride_outside_y, const double _stride_outside_theta,
                                const double _stride_bwd_x, const double _stride_inside_y, const double _stride_inside_theta)
    {
      footstep_param.stride_fwd_x = _stride_fwd_x;
      footstep_param.stride_outside_y = _stride_outside_y;
      footstep_param.stride_outside_theta = _stride_outside_theta;
      footstep_param.stride_bwd_x = _stride_bwd_x;
      footstep_param.stride_inside_y = _stride_inside_y;
      footstep_param.stride_inside_theta = _stride_inside_theta;
    };
    void set_use_inside_step_limitation(const bool uu) { use_inside_step_limitation = uu; };
    void set_default_orbit_type (const orbit_type type) { lcg.set_default_orbit_type(type); };
    void set_swing_trajectory_delay_time_offset (const double _time_offset) { lcg.set_swing_trajectory_delay_time_offset(_time_offset); };
    void set_swing_trajectory_final_distance_weight (const double _final_distance_weight) { lcg.set_swing_trajectory_final_distance_weight(_final_distance_weight); };
    void set_swing_trajectory_time_offset_xy2z (const double _tmp) { lcg.set_swing_trajectory_time_offset_xy2z(_tmp); };
    void set_stair_trajectory_way_point_offset (const hrp::Vector3 _offset) { lcg.set_stair_trajectory_way_point_offset(_offset); };
    void set_rectangle_trajectory_way_point_offset (const hrp::Vector3 _offset) { lcg.set_rectangle_trajectory_way_point_offset(_offset); };
    void set_cycloid_delay_kick_point_offset (const hrp::Vector3 _offset) { lcg.set_cycloid_delay_kick_point_offset(_offset); };
    void set_gravitational_acceleration (const double ga) { gravitational_acceleration = ga; };
    void set_toe_pos_offset_x (const double _offx) { lcg.set_toe_pos_offset_x(_offx); };
    void set_heel_pos_offset_x (const double _offx) { lcg.set_heel_pos_offset_x(_offx); };
    void set_toe_angle (const double _angle) { lcg.set_toe_angle(_angle); };
    void set_heel_angle (const double _angle) { lcg.set_heel_angle(_angle); };
    bool set_toe_heel_phase_ratio (const std::vector<double>& ratio)
    {
        bool lcgret = lcg.set_toe_heel_phase_ratio(ratio);
        bool rgret = rg.set_toe_heel_phase_ratio(ratio);
        return lcgret && rgret;
    };
    void set_use_toe_joint (const bool ut) { lcg.set_use_toe_joint(ut); };
    void set_leg_default_translate_pos (const std::vector<hrp::Vector3>& off) { footstep_param.leg_default_translate_pos = off;};
    void set_optional_go_pos_finalize_footstep_num (const size_t num) { optional_go_pos_finalize_footstep_num = num; };
    void set_all_limbs (const std::vector<std::string>& _all_limbs) { all_limbs = _all_limbs; };
    void set_overwritable_footstep_index_offset (const size_t _of) { overwritable_footstep_index_offset = _of;};
    void set_foot_steps_list (const std::vector< std::vector<step_node> >& fnsl)
    {
        clear_footstep_nodes_list();
        footstep_nodes_list = fnsl;
        append_finalize_footstep();
        print_footstep_nodes_list();
    };
    void set_overwrite_foot_steps_list (const std::vector< std::vector<step_node> >& fnsl)
    {
        overwrite_footstep_nodes_list.clear();
        overwrite_footstep_nodes_list = fnsl;
        append_finalize_footstep(overwrite_footstep_nodes_list);
        print_footstep_nodes_list(overwrite_footstep_nodes_list);
    };
    void set_leg_margin (const double _leg_margin[4]) {
      for (size_t i = 0; i < 4; i++) {
        leg_margin[i] = _leg_margin[i];
      }
    };
    void set_safe_leg_margin (const double _leg_margin[4]) {
      for (size_t i = 0; i < 4; i++) {
        safe_leg_margin[i] = _leg_margin[i];
      }
    };
    void set_stride_limitation_for_circle_type (const double (&_stride_limitation_for_circle_type)[5]) {
      for (size_t i = 0; i < 5; i++) {
        stride_limitation_for_circle_type[i] = _stride_limitation_for_circle_type[i];
      }
    };
    void set_overwritable_stride_limitation (const double (&_overwritable_stride_limitation)[5]) {
      for (size_t i = 0; i < 5; i++) {
        overwritable_stride_limitation[i] = _overwritable_stride_limitation[i];
      }
    };
    void set_footstep_modification_gain (const double _footstep_modification_gain) { footstep_modification_gain = _footstep_modification_gain; };
    void set_cp_check_margin (const double (&_cp_check_margin)[2]) {
      for (size_t i=0; i < 2; i++) {
        cp_check_margin[i] = _cp_check_margin[i];
      }
    };
    void set_act_contact_states (const std::vector<bool>& _act_contact_states) {
      if (act_contact_states.empty()) act_contact_states.resize(_act_contact_states.size());
      for (size_t i = 0; i < act_contact_states.size(); i++) {
        act_contact_states[i] = _act_contact_states[i];
      }
      lcg.set_act_contact_states(_act_contact_states);
    };
    void set_use_stride_limitation (const bool _use_stride_limitation) { use_stride_limitation = _use_stride_limitation; };
    void set_modify_footsteps (const bool _modify_footsteps) { modify_footsteps = _modify_footsteps; };
    void set_min_time_mgn (const double _t) { min_time_mgn = _t; };
    void set_min_time (const double _t) { min_time = _t; };
    void set_margin_time_ratio (const double _margin_time_ratio) { margin_time_ratio = _margin_time_ratio; };
    void set_diff_cp (const hrp::Vector3& _cp) { diff_cp = _cp; };
    void set_stride_limitation_type (const stride_limitation_type _tmp) { default_stride_limitation_type = _tmp; };
    void set_toe_check_thre (const double _a) { thtc.set_toe_check_thre(_a); };
    void set_heel_check_thre (const double _a) { thtc.set_heel_check_thre(_a); };
    void set_act_vel_ratio (const double ratio) { act_vel_ratio = ratio; };
    void set_vertices (const std::vector<std::vector<Eigen::Vector2d> >& vs) { foot_vertices = vs; };
    void get_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { vs = foot_vertices; };
    void set_total_mass (const double _m) { total_mass = _m; };
    void set_use_disturbance_compensation (const bool _use) { use_disturbance_compensation = _use; };
    void set_dc_gain (const double _gain) { dc_gain = _gain; };
    void set_dcm_offset (const double _off) { dcm_offset = _off; };
    void set_vel_foot_offset (const hrp::Vector3& _off, const leg_type _l_r) { footstep_param.vel_foot_offset[_l_r] = _off; };
    void set_is_vision_updated (const bool _t) { is_vision_updated = _t; };
    void set_rel_landing_height (const hrp::Vector3& _pos) { rel_landing_height = _pos; };
    void set_rel_landing_normal (const hrp::Vector3& _n) { rel_landing_normal = _n; };
    void set_steppable_region (const OpenHRP::TimedSteppableRegion& _region) {
      leg_type next_sup = (_region.data.l_r == 0 ? LLEG : RLEG);
      steppable_region[next_sup].resize(_region.data.region.length());
      for (size_t i = 0; i < steppable_region[next_sup].size(); i++) {
        steppable_region[next_sup][i].resize(_region.data.region[i].length()/2);;
        for (size_t j = 0; j < steppable_region[next_sup][i].size(); j++) {
          steppable_region[next_sup][i][j](0) = _region.data.region[i][2*j];
          steppable_region[next_sup][i][j](1) = _region.data.region[i][2*j+1];
        }
      }
      lr_region[next_sup] = true;
    };
    void set_vertices_from_leg_margin ()
    {
      std::vector<std::vector<Eigen::Vector2d> > vec;
      // RLEG
      {
        std::vector<Eigen::Vector2d> tvec;
        tvec.push_back(Eigen::Vector2d(leg_margin[0], leg_margin[3]));
        tvec.push_back(Eigen::Vector2d(leg_margin[0], -1*leg_margin[2]));
        tvec.push_back(Eigen::Vector2d(-1*leg_margin[1], -1*leg_margin[2]));
        tvec.push_back(Eigen::Vector2d(-1*leg_margin[1], leg_margin[3]));
        vec.push_back(tvec);
      }
      // LLEG
      {
        std::vector<Eigen::Vector2d> tvec;
        tvec.push_back(Eigen::Vector2d(leg_margin[0], leg_margin[2]));
        tvec.push_back(Eigen::Vector2d(leg_margin[0], -1*leg_margin[3]));
        tvec.push_back(Eigen::Vector2d(-1*leg_margin[1], -1*leg_margin[3]));
        tvec.push_back(Eigen::Vector2d(-1*leg_margin[1], leg_margin[2]));
        vec.push_back(tvec);
      }
      set_vertices(vec);
    };
    void set_is_emergency_step (const bool _is_emergency_step) { is_emergency_step = _is_emergency_step; };
    void set_emergency_step_time (const double _emergency_step_time[3]) {
      for (size_t i = 0; i < 3; i++) {
        emergency_step_time[i] = _emergency_step_time[i];
      }
    }
    void set_sum_d_footstep_thre (const hrp::Vector3& _thre) { sum_d_footstep_thre = _thre; };
    void set_footstep_check_delta (const hrp::Vector3& _delta) { footstep_check_delta = _delta; };
    void set_use_act_states() { lcg.set_use_act_states(use_act_states); };
    /* Get overwritable footstep index. For example, if overwritable_footstep_index_offset = 1, overwrite next footstep. If overwritable_footstep_index_offset = 0, overwrite current swinging footstep. */
    size_t get_overwritable_index () const
    {
        return lcg.get_footstep_index()+overwritable_footstep_index_offset;
    };
    bool set_overwrite_foot_step_index (const size_t idx)
    {
        if (idx >= get_overwritable_index()) {
            overwrite_footstep_index = idx;
            return true;
        } else {
            return false;
        }
    };
    bool get_footstep_nodes_by_index (std::vector<step_node>& csl, const size_t idx) const
    {
        if (footstep_nodes_list.size()-1 >= idx) {
            csl = footstep_nodes_list.at(idx);
            return true;
        } else {
            return false;
        }
    };
    void print_footstep_nodes_list (const std::vector< std::vector<step_node> > _footstep_nodes_list) const
    {
        for (size_t i = 0; i < _footstep_nodes_list.size(); i++) {
            for (size_t j = 0; j < _footstep_nodes_list.at(i).size(); j++) {
                std::cerr << "[" << i << "] " << _footstep_nodes_list.at(i).at(j) << std::endl;
            }
        }
    };
    void print_footstep_nodes_list () const
    {
      print_footstep_nodes_list(footstep_nodes_list);
    };
    /* parameter getting */
    const hrp::Vector3& get_cog () const { return cog; };
    hrp::Vector3 get_cog_vel () const {
      double refcog_vel[3];
      preview_controller_ptr->get_refcog_vel(refcog_vel);
      return hrp::Vector3(refcog_vel[0], refcog_vel[1], refcog_vel[2]);
    };
    hrp::Vector3 get_cog_acc () const {
      double refcog_acc[3];
      preview_controller_ptr->get_refcog_acc(refcog_acc);
      return hrp::Vector3(refcog_acc[0], refcog_acc[1], refcog_acc[2]);
    };
    const hrp::Vector3& get_refzmp () const { return refzmp;};
    hrp::Vector3 get_cart_zmp () const
    {
        double czmp[3];
        preview_controller_ptr->get_cart_zmp(czmp);
        return hrp::Vector3(czmp[0], czmp[1], czmp[2]);
    };
    std::vector<std::string> convert_leg_types_to_names (const std::vector<leg_type>& lts) const {
      std::vector<std::string> ret;
      for (std::vector<leg_type>::const_iterator it = lts.begin(); it != lts.end(); it++) {
          ret.push_back(leg_type_map.find(*it)->second);
      }
      return ret;
    };
    const std::vector<hrp::Vector3>& get_swing_foot_zmp_offsets () const { return swing_foot_zmp_offsets;};
    std::vector<hrp::Vector3> get_support_foot_zmp_offsets () const {
      std::vector<hrp::Vector3> ret;
      for (size_t i = 0; i < lcg.get_support_leg_types().size(); i++) {
          ret.push_back(rg.get_default_zmp_offset(lcg.get_support_leg_types().at(i)));
      }
      return ret;
    };
    // Get foot zmp offsets by checking whether given EE name is swing or support
    bool get_swing_support_foot_zmp_offsets_from_ee_name (hrp::Vector3& ret, const std::string& ee_name) const
    {
        leg_type lt = get_leg_type_from_ee_name(ee_name);
        std::vector<leg_type>::const_iterator it = std::find(lcg.get_support_leg_types().begin(), lcg.get_support_leg_types().end(), lt);
        if (it != lcg.get_support_leg_types().end()) { // If support leg
            ret = get_support_foot_zmp_offsets()[std::distance(lcg.get_support_leg_types().begin(), it)];
        } else {
            it = std::find(lcg.get_swing_leg_types().begin(), lcg.get_swing_leg_types().end(), lt);
            if (it != lcg.get_swing_leg_types().end()) { // If swing leg
                ret = get_swing_foot_zmp_offsets()[std::distance(lcg.get_swing_leg_types().begin(), it)];
            } else { // Otherwise
                return false;
            }
        }
        return true;
    };
    double get_toe_zmp_offset_x () const { return rg.get_toe_zmp_offset_x(); };
    double get_heel_zmp_offset_x () const { return rg.get_heel_zmp_offset_x(); };
    bool get_use_toe_heel_transition () const { return rg.get_use_toe_heel_transition(); };
    bool get_use_toe_heel_auto_set () const { return rg.get_use_toe_heel_auto_set(); };
    size_t get_remain_count () const { return remain_count; };
    bool is_before_step_phase () const { return lcg.get_lcg_count() > static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1; };
    leg_type get_cur_leg () const { return footstep_nodes_list[lcg.get_footstep_index()].front().l_r; };
    const std::map<leg_type, double> get_zmp_weight_map () const { return rg.get_zmp_weight_map(); };
    void proc_zmp_weight_map_interpolation () { return rg.proc_zmp_weight_map_interpolation(); };
    std::vector<std::string> get_footstep_front_leg_names () const {
      std::vector<leg_type> lts;
      for (size_t i = 0; i < footstep_nodes_list[0].size(); i++) {
          lts.push_back(footstep_nodes_list[0].at(i).l_r);
      }
      return convert_leg_types_to_names(lts);
    };
    std::vector<std::string> get_footstep_back_leg_names () const {
      std::vector<leg_type> lts;
      for (size_t i = 0; i < footstep_nodes_list.back().size(); i++) {
          lts.push_back(footstep_nodes_list.back().at(i).l_r);
      }
      return convert_leg_types_to_names(lts);
    };
    const std::vector<std::string> get_support_leg_names() const { return convert_leg_types_to_names(lcg.get_support_leg_types());};
    const std::vector<std::string> get_swing_leg_names() const { return convert_leg_types_to_names(lcg.get_swing_leg_types());};
    const std::vector<step_node>& get_swing_leg_steps() const { return lcg.get_swing_leg_steps(); };
    const std::vector<step_node>& get_support_leg_steps() const { return lcg.get_support_leg_steps(); };
    const std::vector<step_node>& get_swing_leg_src_steps() const { return lcg.get_swing_leg_src_steps(); };
    const std::vector<step_node>& get_swing_leg_dst_steps() const { return lcg.get_swing_leg_dst_steps(); };
    const coordinates get_dst_foot_midcoords() const /* get foot_midcoords calculated from swing_leg_dst_coords */
    {
      coordinates tmp(lcg.get_swing_leg_dst_steps().front().worldcoords);
      // tmp.pos += tmp.rot * hrp::Vector3(-1*footstep_param.leg_default_translate_pos[lcg.get_swing_leg_dst_steps().front().l_r]);
      tmp.pos += tmp.rot * hrp::Vector3(-1*footstep_param.vel_foot_offset[lcg.get_swing_leg_dst_steps().front().l_r]);
      return tmp;
    };
    void get_swing_support_mid_coords(coordinates& ret) const { lcg.get_swing_support_mid_coords(ret); };
    void get_stride_parameters (double& _stride_fwd_x, double& _stride_outside_y, double& _stride_outside_theta,
                                double& _stride_bwd_x, double& _stride_inside_y, double& _stride_inside_theta) const
    {
      _stride_fwd_x = footstep_param.stride_fwd_x;
      _stride_outside_y = footstep_param.stride_outside_y;
      _stride_outside_theta = footstep_param.stride_outside_theta;
      _stride_bwd_x = footstep_param.stride_bwd_x;
      _stride_inside_y = footstep_param.stride_inside_y;
      _stride_inside_theta = footstep_param.stride_inside_theta;
    };
    size_t get_footstep_index() const { return lcg.get_footstep_index(); };
    size_t get_lcg_count() const { return lcg.get_lcg_count(); };
    double get_current_swing_time(const size_t idx) const { return lcg.get_current_swing_time(idx); };
    // Get current swing time by checking whether given EE name is swing or support
    double get_current_swing_time_from_ee_name (const std::string ee_name) const
    {
        return get_current_swing_time( get_leg_type_from_ee_name(ee_name) );
    };
    std::vector<leg_type> get_current_support_states() const { return lcg.get_current_support_states();};
    double get_default_step_time () const { return default_step_time; };
    double get_default_step_height () const { return lcg.get_default_step_height(); };
    double get_default_double_support_ratio_before () const { return default_double_support_ratio_before; };
    double get_default_double_support_ratio_after () const { return default_double_support_ratio_after; };
    double get_default_double_support_static_ratio_before () const { return default_double_support_static_ratio_before; };
    double get_default_double_support_static_ratio_after () const { return default_double_support_static_ratio_after; };
    double get_default_double_support_ratio_swing_before () const {return default_double_support_ratio_swing_before; };
    double get_default_double_support_ratio_swing_after () const {return default_double_support_ratio_swing_after; };
    hrp::Vector3 get_flywheel_tau () const { return flywheel_tau; };
    bool get_use_roll_flywheel () const { return use_roll_flywheel; };
    bool get_use_pitch_flywheel () const { return use_pitch_flywheel; };
    std::vector< std::vector<step_node> > get_remaining_footstep_nodes_list () const
    {
        std::vector< std::vector<step_node> > fsnl;
        size_t fsl_size = (footstep_nodes_list.size()>lcg.get_footstep_index() ? footstep_nodes_list.size()-lcg.get_footstep_index() : 0);
        // The rest of fsl are swing dst coords from now.
        for (size_t i = 0; i < fsl_size; i++) {
            fsnl.push_back(footstep_nodes_list[i+lcg.get_footstep_index()]);
        }
        return fsnl;
    };
    orbit_type get_default_orbit_type () const { return lcg.get_default_orbit_type(); };
    double get_swing_trajectory_delay_time_offset () const { return lcg.get_swing_trajectory_delay_time_offset(); };
    double get_swing_trajectory_final_distance_weight () const { return lcg.get_swing_trajectory_final_distance_weight(); };
    double get_swing_trajectory_time_offset_xy2z () const { return lcg.get_swing_trajectory_time_offset_xy2z(); };
    hrp::Vector3 get_stair_trajectory_way_point_offset () const { return lcg.get_stair_trajectory_way_point_offset(); };
    hrp::Vector3 get_rectangle_trajectory_way_point_offset () const { return lcg.get_rectangle_trajectory_way_point_offset(); };
    hrp::Vector3 get_cycloid_delay_kick_point_offset () const { return lcg.get_cycloid_delay_kick_point_offset(); };
    double get_gravitational_acceleration () const { return gravitational_acceleration; } ;
    double get_toe_pos_offset_x () const { return lcg.get_toe_pos_offset_x(); };
    double get_heel_pos_offset_x () const { return lcg.get_heel_pos_offset_x(); };
    double get_toe_angle () const { return lcg.get_toe_angle(); };
    double get_heel_angle () const { return lcg.get_heel_angle(); };
    double get_foot_dif_rot_angle () const { return lcg.get_foot_dif_rot_angle(); };
    void get_toe_heel_phase_ratio (std::vector<double>& ratio) const { lcg.get_toe_heel_phase_ratio(ratio); };
    int get_NUM_TH_PHASES () const { return NUM_TH_PHASES; };
    bool get_use_toe_joint () const { return lcg.get_use_toe_joint(); };
    double get_current_toe_heel_ratio () const { return lcg.get_current_toe_heel_ratio(get_use_toe_heel_transition()); };
    // Get current toe heel ratio by checking whether given EE name is swing or support
    bool get_current_toe_heel_ratio_from_ee_name (double& ret, const std::string& ee_name) const
    {
        leg_type lt = get_leg_type_from_ee_name(ee_name);
        if (std::find(lcg.get_support_leg_types().begin(), lcg.get_support_leg_types().end(), lt)!=lcg.get_support_leg_types().end()) { // If support
            ret = rats::no_using_toe_heel_ratio;
        } else if (std::find(lcg.get_swing_leg_types().begin(), lcg.get_support_leg_types().end(), lt)!=lcg.get_swing_leg_types().end()) { // If swing
            ret = get_current_toe_heel_ratio();
        } else { // Otherwise
            return false;
        }
        return true;
    };
    void get_leg_default_translate_pos (std::vector<hrp::Vector3>& off) const { off = footstep_param.leg_default_translate_pos; };
    size_t get_overwritable_footstep_index_offset () const { return overwritable_footstep_index_offset; };
    const std::vector<leg_type> calc_counter_leg_types_from_footstep_nodes (const std::vector<step_node>& fns, std::vector<std::string> _all_limbs) const;
    const std::map<leg_type, std::string> get_leg_type_map () const { return leg_type_map; };
    size_t get_optional_go_pos_finalize_footstep_num () const { return optional_go_pos_finalize_footstep_num; };
    bool is_finalizing (const double tm) const { return ((preview_controller_ptr->get_delay()*2 - default_step_time/dt)-finalize_count) <= (tm/dt)-1; };
    size_t get_overwrite_check_timing () const { return static_cast<size_t>(footstep_nodes_list[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1;}; // Almost middle of step time
    double get_leg_margin (const size_t idx) const { return leg_margin[idx]; };
    double get_safe_leg_margin (const size_t idx) const { return safe_leg_margin[idx]; };
    double get_stride_limitation_for_circle_type (const size_t idx) const { return stride_limitation_for_circle_type[idx]; };
    double get_overwritable_stride_limitation (const size_t idx) const { return overwritable_stride_limitation[idx]; };
    double get_footstep_modification_gain () const { return footstep_modification_gain; };
    double get_cp_check_margin (const size_t idx) const { return cp_check_margin[idx]; };
    bool get_modify_footsteps () const { return modify_footsteps; };
    double get_min_time_mgn () const { return min_time_mgn; };
    double get_min_time () const { return min_time; };
    double get_margin_time_ratio () const { return margin_time_ratio; };
    bool get_use_stride_limitation () const { return use_stride_limitation; };
    stride_limitation_type get_stride_limitation_type () const { return default_stride_limitation_type; };
    double get_toe_check_thre () const { return thtc.get_toe_check_thre(); };
    double get_heel_check_thre () const { return thtc.get_heel_check_thre(); };
    size_t get_falling_direction () const {return falling_direction;};
    size_t get_step_num () const {return footstep_nodes_list.size(); };
    bool get_is_single_walking () const { return (lcg.get_footstep_index() > 0 && lcg.get_footstep_index() < footstep_nodes_list.size()-2); };
    bool get_is_more_than_1st_step () const { return (lcg.get_footstep_index() > 1); };
    // Get ee coords by checking whether given EE name is swing or support
    bool get_swing_support_ee_coords_from_ee_name (hrp::Vector3& cpos, hrp::Matrix33& crot, const std::string& ee_name) const
    {
        leg_type lt = get_leg_type_from_ee_name(ee_name);
        if (std::find(lcg.get_support_leg_types().begin(), lcg.get_support_leg_types().end(), lt) != lcg.get_support_leg_types().end()) { // If support
            coordinates tmpc = std::find_if(lcg.get_support_leg_steps().begin(), lcg.get_support_leg_steps().end(), (&boost::lambda::_1->* &step_node::l_r == lt))->worldcoords;
            cpos = tmpc.pos;
            crot = tmpc.rot;
        } else if (std::find(lcg.get_swing_leg_types().begin(), lcg.get_swing_leg_types().end(), lt) != lcg.get_swing_leg_types().end()) { // If swing
            coordinates tmpc = std::find_if(lcg.get_swing_leg_steps().begin(), lcg.get_swing_leg_steps().end(), (&boost::lambda::_1->* &step_node::l_r == lt))->worldcoords;
            cpos = tmpc.pos;
            crot = tmpc.rot;
        } else { // Otherwise
            return false;
        }
        return true;
    };
    // Get current support state (true=support, false=swing) by checking whether given EE name is swing or support
    bool get_current_support_state_from_ee_name (const std::string& ee_name) const
    {
        leg_type lt = get_leg_type_from_ee_name(ee_name);
        std::vector<leg_type> tmp = lcg.get_current_support_states();
        return std::find(tmp.begin(), tmp.end(), lt) != tmp.end();
    };
    double get_act_vel_ratio () { return act_vel_ratio; };
    bool get_use_disturbance_compensation () { return use_disturbance_compensation; };
    double get_dc_gain () { return dc_gain; };
    double get_dcm_offset () { return dcm_offset; };
    double get_tmp (const size_t idx) {return tmp[idx];}
    double get_current_landing_pos (const size_t idx) {
      return footstep_nodes_list[lcg.get_footstep_index()].front().worldcoords.pos(idx);
    }
    double get_emergency_step_time (const size_t idx) const { return emergency_step_time[idx]; };
    void get_sum_d_footstep_thre (hrp::Vector3& thre) { thre = sum_d_footstep_thre; };
    void get_footstep_check_delta (hrp::Vector3& delta) { delta = footstep_check_delta; };
    void get_rel_landing_pos (hrp::Vector3& pos, int& l_r) {
      pos = rel_landing_pos;
      l_r = cur_supporting_foot;
    };
    void get_end_cog_state (hrp::Vector3& pos, hrp::Vector3& vel, int& l_r) {
      pos = end_cog;
      vel = end_cogvel;
      l_r = cur_supporting_foot;
    };
    void get_current_steppable_region (OpenHRP::TimedSteppableRegion& _region) {
      leg_type cur_sup = footstep_nodes_list[lcg.get_footstep_index()-1].front().l_r;
      if (lcg.get_footstep_index() > 0 && lr_region[cur_sup]) {
        _region.data.l_r = (cur_sup == RLEG ? 0 : 1);
        hrp::Vector3 preprev_fs_pos = (lcg.get_footstep_index()==1 ? lcg.get_swing_leg_src_steps().front() : footstep_nodes_list[lcg.get_footstep_index()-2].front()).worldcoords.pos;
        _region.data.region.length(steppable_region[cur_sup].size());
        for (size_t i = 0; i < steppable_region[cur_sup].size(); i++) {
          _region.data.region[i].length(steppable_region[cur_sup][i].size()*2);
          for (size_t j = 0; j < steppable_region[cur_sup][i].size(); j++) {
            _region.data.region[i][2*j] = steppable_region[cur_sup][i][j](0) + preprev_fs_pos(0);
            _region.data.region[i][2*j+1] = steppable_region[cur_sup][i][j](1) + preprev_fs_pos(1);
          }
        }
      }
    };
    double get_cur_wheel_pos_x () { return cur_wheel_pos_x; };


#ifdef FOR_TESTGAITGENERATOR
    size_t get_one_step_count() const { return lcg.get_one_step_count(); };
    void get_footstep_nodes_list (std::vector< std::vector<step_node> > & fsl) const
    {
        fsl = footstep_nodes_list;
    };
    double get_toe_heel_dif_angle () const { return lcg.get_toe_heel_dif_angle(); };
    std::vector<hrp::Vector3> get_default_zmp_offsets() const { return rg.get_default_zmp_offsets(); };
#endif // FOR_TESTGAITGENERATOR
    void print_param (const std::string& print_str = "") const
    {
        double stride_fwd_x, stride_outside_y, stride_outside_th, stride_bwd_x, stride_inside_y, stride_inside_th;
        get_stride_parameters(stride_fwd_x, stride_outside_y, stride_outside_th, stride_bwd_x, stride_inside_y, stride_inside_th);
        std::cerr << "[" << print_str << "]   stride_parameter = " << stride_fwd_x << "[m], " << stride_outside_y << "[m], " << stride_outside_th << "[deg], "
                  << stride_bwd_x << "[m], " << stride_inside_y << "[m], " << stride_inside_th << "[deg]" << std::endl;
        std::cerr << "[" << print_str << "]   leg_default_translate_pos = ";
        for (size_t i = 0; i < footstep_param.leg_default_translate_pos.size(); i++) {
            std::cerr << footstep_param.leg_default_translate_pos[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"));
        }
        std::cerr << std::endl;
        std::cerr << "[" << print_str << "]   default_step_time = " << get_default_step_time() << "[s]" << std::endl;
        std::cerr << "[" << print_str << "]   default_step_height = " << get_default_step_height() << "[m]" << std::endl;
        std::cerr << "[" << print_str << "]   default_double_support_ratio_before = " << get_default_double_support_ratio_before() << ", default_double_support_ratio_before = " << get_default_double_support_ratio_after() << ", default_double_support_static_ratio_before = " << get_default_double_support_static_ratio_before() << ", default_double_support_static_ratio_after = " << get_default_double_support_static_ratio_after() << ", default_double_support_static_ratio_swing_before = " << get_default_double_support_ratio_swing_before() << ", default_double_support_static_ratio_swing_after = " << get_default_double_support_ratio_swing_after() << std::endl;
        std::cerr << "[" << print_str << "]   default_orbit_type = ";
        if (get_default_orbit_type() == SHUFFLING) {
            std::cerr << "SHUFFLING" << std::endl;
        } else if (get_default_orbit_type() == CYCLOID) {
            std::cerr << "CYCLOID" << std::endl;
        } else if (get_default_orbit_type() == RECTANGLE) {
            std::cerr << "RECTANGLE" << std::endl;
        } else if (get_default_orbit_type() == STAIR) {
            std::cerr << "STAIR" << std::endl;
        } else if (get_default_orbit_type() == CYCLOIDDELAY) {
            std::cerr << "CYCLOIDDELAY" << std::endl;
        } else if (get_default_orbit_type() == CYCLOIDDELAYKICK) {
            std::cerr << "CYCLOIDDELAYKICK" << std::endl;
        } else if (get_default_orbit_type() == CROSS) {
            std::cerr << "CROSS" << std::endl;
        }
        std::cerr << "[" << print_str << "]   swing_trajectory_delay_time_offset = " << get_swing_trajectory_delay_time_offset() << "[s], swing_trajectory_final_distance_weight = " << get_swing_trajectory_final_distance_weight()
                  << ", swing_trajectory_time_offset_xy2z = " << get_swing_trajectory_time_offset_xy2z() << std::endl;
        hrp::Vector3 tmpv;
        tmpv = get_stair_trajectory_way_point_offset();
        std::cerr << "[" << print_str << "]   stair_trajectory_way_point_offset = " << tmpv.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        tmpv = get_cycloid_delay_kick_point_offset();
        std::cerr << "[" << print_str << "]   cycloid_delay_kick_point_offset = " << tmpv.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << print_str << "]   gravitational_acceleration = " << get_gravitational_acceleration() << "[m/s^2]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_pos_offset_x = " << get_toe_pos_offset_x() << "[mm], heel_pos_offset_x = " << get_heel_pos_offset_x() << "[mm]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_zmp_offset_x = " << get_toe_zmp_offset_x() << "[mm], heel_zmp_offset_x = " << get_heel_zmp_offset_x() << "[mm]" << std::endl;
        std::cerr << "[" << print_str << "]   toe_angle = " << get_toe_angle() << "[deg]" << std::endl;
        std::cerr << "[" << print_str << "]   heel_angle = " << get_heel_angle() << "[deg]" << std::endl;
        std::cerr << "[" << print_str << "]   use_toe_joint = " << (get_use_toe_joint()?"true":"false") << ", use_toe_heel_transition = " << (get_use_toe_heel_transition()?"true":"false") << ", use_toe_heel_auto_set = " << (get_use_toe_heel_auto_set()?"true":"false") << std::endl;
        std::vector<double> tmp_ratio(get_NUM_TH_PHASES(), 0.0);
        get_toe_heel_phase_ratio(tmp_ratio);
        std::cerr << "[" << print_str << "]   toe_heel_phase_ratio = [";
        for (int i = 0; i < get_NUM_TH_PHASES(); i++) std::cerr << tmp_ratio[i] << " ";
        std::cerr << "]" << std::endl;
        std::cerr << "[" << print_str << "]   optional_go_pos_finalize_footstep_num = " << optional_go_pos_finalize_footstep_num << ", overwritable_footstep_index_offset = " << overwritable_footstep_index_offset << std::endl;
        std::cerr << "[" << print_str << "]   default_stride_limitation_type = ";
        if (default_stride_limitation_type == SQUARE) {
          std::cerr << "SQUARE" << std::endl;
        } else if (default_stride_limitation_type == CIRCLE) {
          std::cerr << "CIRCLE" << std::endl;
        }
        thtc.print_param(print_str);
    };
  };
}
#endif /* GAITGENERATOR_H */
