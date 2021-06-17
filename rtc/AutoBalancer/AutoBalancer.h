// -*- C++ -*-
/*!
 * @file  AutoBalancer.h
 * @brief autobalancer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AUTOBALANCER_H
#define AUTOBALANCER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "GaitGenerator.h"
#include "Stabilizer.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalancerService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
//#include "SimpleFullbodyInverseKinematicsSolver.h"
#include "FullbodyInverseKinematicsSolver.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class AutoBalancer
  : public RTC::DataFlowComponentBase
{
 public:
  AutoBalancer(RTC::Manager* manager);
  virtual ~AutoBalancer();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
  bool goPos(const double& x, const double& y, const double& th);
  bool goWheel(const double& x, const double& tm);
  bool goVelocity(const double& vx, const double& vy, const double& vth);
  bool goStop();
  bool emergencyStop ();
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepSequence& fs, const OpenHRP::AutoBalancerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  void waitFootStepsEarly(const double tm);
  bool startAutoBalancer(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param);
  bool adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep);
  bool getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx);
  bool getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep);
  bool releaseEmergencyStop();
  void distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp);

  // Stabilizer
  void getStabilizerParam(OpenHRP::AutoBalancerService::StabilizerParam& i_param);
  void setStabilizerParam(const OpenHRP::AutoBalancerService::StabilizerParam& i_param);
  void startStabilizer(void);
  void stopStabilizer(void);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  TimedDoubleSeq m_qTouchWall;
  InPort<TimedDoubleSeq> m_qTouchWallIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedPoint3D m_zmp;
  InPort<TimedPoint3D> m_zmpIn;
  TimedDoubleSeq m_optionalData;
  InPort<TimedDoubleSeq> m_optionalDataIn;
  std::vector<TimedDoubleSeq> m_ref_force;
  std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
  TimedPoint3D m_diffCP;
  InPort<TimedPoint3D> m_diffCPIn;
  TimedPoint3D m_refFootOriginExtMoment;
  InPort<TimedPoint3D> m_refFootOriginExtMomentIn;
  TimedBoolean m_refFootOriginExtMomentIsHoldValue;
  InPort<TimedBoolean> m_refFootOriginExtMomentIsHoldValueIn;
  TimedOrientation3D m_rpy;
  InPort<TimedOrientation3D> m_rpyIn;
  TimedDoubleSeq m_qRefSeq;
  InPort<TimedDoubleSeq> m_qRefSeqIn;
  TimedBoolean m_touchWallMotionSolved;
  InPort<TimedBoolean> m_touchWallMotionSolvedIn;
  std::vector<TimedDoubleSeq> m_wrenches;
  std::vector<InPort<TimedDoubleSeq> *> m_wrenchesIn;
  OpenHRP::TimedLandingPosition m_landingHeight;
  InPort<OpenHRP::TimedLandingPosition> m_landingHeightIn;
  OpenHRP::TimedSteppableRegion m_steppableRegion;
  InPort<OpenHRP::TimedSteppableRegion> m_steppableRegionIn;

  // for debug
  TimedPoint3D m_cog;
  TimedPoint3D m_originRefZmp, m_originRefCog, m_originRefCogVel, m_originNewZmp;
  TimedPoint3D m_originActZmp, m_originActCog, m_originActCogVel;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  OutPort<TimedPoint3D> m_zmpOut;
  TimedPoint3D m_zmpAct;
  OutPort<TimedPoint3D> m_zmpActOut;
  TimedPoint3D m_refCP;
  OutPort<TimedPoint3D> m_refCPOut;
  TimedPoint3D m_actCP;
  OutPort<TimedPoint3D> m_actCPOut;
  TimedDoubleSeq m_qAbc;
  OutPort<TimedDoubleSeq> m_qAbcOut;
  TimedDoubleSeq m_tmp;
  OutPort<TimedDoubleSeq> m_tmpOut;
  TimedPoint3D m_currentLandingPos;
  OutPort<TimedPoint3D> m_currentLandingPosOut;
  TimedPoint3D m_diffFootOriginExtMoment;
  OutPort<TimedPoint3D> m_diffFootOriginExtMomentOut;
  TimedDoubleSeq m_allEEComp;
  OutPort<TimedDoubleSeq> m_allEECompOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedDoubleSeq m_baseTform;
  OutPort<TimedDoubleSeq> m_baseTformOut;
  TimedPose3D m_basePose;
  OutPort<TimedPose3D> m_basePoseOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;
  TimedBooleanSeq m_actContactStates;
  OutPort<TimedBooleanSeq> m_actContactStatesOut;
  TimedDoubleSeq m_COPInfo;
  OutPort<TimedDoubleSeq> m_COPInfoOut;
  TimedDoubleSeq m_toeheelRatio;
  OutPort<TimedDoubleSeq> m_toeheelRatioOut;
  TimedDoubleSeq m_controlSwingSupportTime;
  OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
  TimedBoolean m_walkingStates;
  OutPort<TimedBoolean> m_walkingStatesOut;
  TimedPoint3D m_sbpCogOffset;
  OutPort<TimedPoint3D> m_sbpCogOffsetOut;
  TimedLong m_emergencySignal;
  OutPort<TimedLong> m_emergencySignalOut;
  TimedBoolean m_emergencyFallMotion;
  OutPort<TimedBoolean> m_emergencyFallMotionOut;
  TimedBoolean m_isStuck;
  OutPort<TimedBoolean> m_isStuckOut;
  TimedBoolean m_useFlywheel;
  OutPort<TimedBoolean> m_useFlywheelOut;
  TimedPoint3D m_estimatedFxy;
  OutPort<TimedPoint3D> m_estimatedFxyOut;
  OpenHRP::TimedLandingPosition m_landingTarget;
  OutPort<OpenHRP::TimedLandingPosition> m_landingTargetOut;
  OpenHRP::TimedCogState m_endCogState;
  OutPort<OpenHRP::TimedCogState> m_endCogStateOut;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<OutPort<TimedDoubleSeq> *> m_ref_forceOut;
  std::vector<TimedPoint3D> m_limbCOPOffset;
  std::vector<OutPort<TimedPoint3D> *> m_limbCOPOffsetOut;
  TimedDoubleSeq m_tau;
  OutPort<TimedDoubleSeq> m_tauOut;
  // for debug
  OutPort<TimedPoint3D> m_cogOut;
  OutPort<TimedPoint3D> m_originRefZmpOut, m_originRefCogOut, m_originRefCogVelOut, m_originNewZmpOut;
  OutPort<TimedPoint3D> m_originActZmpOut, m_originActCogOut, m_originActCogVelOut;
  OpenHRP::TimedSteppableRegion m_currentSteppableRegion;
  OutPort<OpenHRP::TimedSteppableRegion> m_currentSteppableRegionOut;

  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_AutoBalancerServicePort;
  RTC::CorbaPort m_RobotHardwareServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  AutoBalancerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct ABCIKparam {
    hrp::Vector3 target_p0, localPos, adjust_interpolation_target_p0, adjust_interpolation_org_p0, handfix_target_p0;
    hrp::Matrix33 target_r0, localR, adjust_interpolation_target_r0, adjust_interpolation_org_r0;
    hrp::Link* target_link;
    bool is_active, has_toe_joint;
  };
  void getTargetParameters();
  void solveSimpleFullbodyIK ();
  void solveFullbodyIK ();
  void startABCparam(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  void stopABCparam();
  void stopABCparamEmergency();
  void waitABCTransition();
  // Functions to calculate parameters for ABC output.
  // Output parameters are EE, limbCOPOffset, contactStates, controlSwingSupportTime, toeheelPhaseRatio
  void getOutputParametersForWalking ();
  void getOutputParametersForABC ();
  void getOutputParametersForWheeling ();
  void getOutputParametersForIDLE ();
  void interpolateLegNamesAndZMPOffsets();
  void calcFixCoordsForAdjustFootstep (rats::coordinates& tmp_fix_coords);
  void rotateRefForcesForFixCoords (rats::coordinates& tmp_fix_coords);
  void updateTargetCoordsForHandFixMode (rats::coordinates& tmp_fix_coords);
  void calculateOutputRefForces ();
  hrp::Vector3 calcFootMidPosUsingZMPWeightMap ();
  void updateWalkingVelocityFromHandError (rats::coordinates& tmp_fix_coords);
  void calcReferenceJointAnglesForIK ();
  hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
  void fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot);
  void fixLegToCoords2 (rats::coordinates& tmp_fix_coords);
  bool startWalking ();
  void stopWalking ();
  void stopWheeling ();
  void copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs);
  // static balance point offsetting
  void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
  void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height);
  hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
  bool isOptionalDataContact (const std::string& ee_name)
  {
      return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false;
  };
  bool calc_inital_support_legs(const double& y, std::vector<rats::coordinates>& initial_support_legs_coords, std::vector<rats::leg_type>& initial_support_legs, rats::coordinates& start_ref_coords);
  std::string getUseForceModeString ();
  void setActData2ST ();
  void setABCData2ST ();
  void limit_cog (hrp::Vector3& cog);
  bool vlimit(double& ret, const double llimit_value, const double ulimit_value);
  void stopFootForEarlyTouchDown();
  void calcTouchoffRemainTime();
  void limbStretchAvoidanceControl();

  // for gg
  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
  ggPtr gg;
  typedef boost::shared_ptr<Stabilizer> stPtr;
  stPtr st;
  bool gg_is_walking, gg_solved;
  bool gg_is_wheeling;
  // for abc
  typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
  fikPtr fik;
  OpenHRP::AutoBalancerService::IKMode ik_mode;
  hrp::Vector3 ref_cog, ref_zmp, rel_ref_zmp, prev_ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel, hand_fix_initial_offset, orig_cog, prev_orig_cog;
  enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
  enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode;
  std::map<std::string, ABCIKparam> ikp;
  std::map<std::string, size_t> contact_states_index_map;
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::vector<std::string> sensor_names, leg_names, ee_vec;
  hrp::Vector3 target_root_p;
  hrp::Matrix33 target_root_R;
  rats::coordinates fix_leg_coords, fix_leg_coords2;
  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;

  double transition_interpolator_ratio, transition_time, zmp_transition_time, adjust_footstep_transition_time, leg_names_interpolator_ratio;
  interpolator *zmp_offset_interpolator;
  interpolator *transition_interpolator;
  interpolator *adjust_footstep_interpolator;
  interpolator *leg_names_interpolator;
  interpolator *angular_momentum_interpolator;
  interpolator *roll_weight_interpolator;
  interpolator *pitch_weight_interpolator;
  interpolator *go_vel_interpolator;
  interpolator *cog_constraint_interpolator;
  interpolator *limit_cog_interpolator;
  interpolator *hand_fix_interpolator;
  hrp::Vector3 input_zmp, input_basePos, ref_basePos, baseRpy;
  hrp::Matrix33 input_baseRot, ref_baseRot;

  // static balance point offsetting
  hrp::Vector3 sbp_offset, sbp_cog_offset;
  enum {MODE_NO_FORCE, MODE_REF_FORCE, MODE_REF_FORCE_WITH_FOOT, MODE_REF_FORCE_RFU_EXT_MOMENT} use_force;
  std::vector<hrp::Vector3> ref_forces, ref_moments;

  unsigned int m_debugLevel;
  bool is_legged_robot, is_stop_mode, is_hand_fix_mode, is_hand_fix_initial;
  int loop;
  bool graspless_manip_mode;
  std::string graspless_manip_arm;
  hrp::Vector3 graspless_manip_p_gain, orig_dif_p, prev_orig_dif_p;
  rats::coordinates graspless_manip_reference_trans_coords;

  hrp::InvDynStateBuffer idsb;
  std::vector<IIRFilter> invdyn_zmp_filters;

  // Used for ref force balancing.
  hrp::Link* additional_force_applied_link;
  hrp::Vector3 additional_force_applied_point_offset;

  bool is_after_walking;
  hrp::Vector3 dif_ref_act_cog;
  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vertices;
  bool use_act_states;
  std::vector<double> diff_q;
  interpolator *emergency_transition_interpolator;
  interpolator *touch_wall_transition_interpolator;
  hrp::Vector3 touchdown_foot_pos[2];
  bool is_foot_touch[2];
  double touchoff_remain_time[2];
  std::map<std::string, interpolator*> touchdown_transition_interpolator;
  bool prev_roll_state, prev_pitch_state;
  bool is_emergency_step_mode, is_emergency_touch_wall_mode, is_emergency_stopping, is_touch_wall_motion_solved, use_collision_avoidance, is_natural_walk, is_stop_early_foot;
  double cog_z_constraint, touch_wall_retrieve_time, arm_swing_deg;
  bool debug_read_steppable_region;
  double d_wheel_angle, start_d_wheel_angle, wheel_radius;
};


extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
