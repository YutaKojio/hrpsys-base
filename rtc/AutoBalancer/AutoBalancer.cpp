// -*- C++ -*-
/*!
 * @file  AutoBalancer.cpp
 * @brief autobalancer component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "AutoBalancer.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"


typedef coil::Guard<coil::Mutex> Guard;
using namespace rats;

// Module specification
// <rtc-template block="module_spec">
static const char* autobalancer_spec[] =
    {
        "implementation_id", "AutoBalancer",
        "type_name",         "AutoBalancer",
        "description",       "autobalancer component",
        "version",           HRPSYS_PACKAGE_VERSION,
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        // Configuration variables
        "conf.default.debugLevel", "0",
        ""
    };
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

AutoBalancer::AutoBalancer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qTouchWallIn("qTouchWall", m_qTouchWall),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp),
      m_optionalDataIn("optionalData", m_optionalData),
      m_emergencySignalOut("emergencySignal", m_emergencySignal),
      m_emergencyFallMotionOut("emergencyFallMotion", m_emergencyFallMotion),
      m_isStuckOut("isStuck", m_isStuck),
      m_useFlywheelOut("useFlywheel", m_useFlywheel),
      m_estimatedFxyOut("estimatedFxy", m_estimatedFxy),
      m_diffCPIn("diffCapturePoint", m_diffCP),
      m_refFootOriginExtMomentIn("refFootOriginExtMoment", m_refFootOriginExtMoment),
      m_refFootOriginExtMomentIsHoldValueIn("refFootOriginExtMomentIsHoldValue", m_refFootOriginExtMomentIsHoldValue),
      m_touchWallMotionSolvedIn("touchWallMotionSolved", m_touchWallMotionSolved),
      m_rpyIn("rpy", m_rpy),
      m_qRefSeqIn("qRefSeq", m_qRefSeq),
      m_landingHeightIn("landingHeight", m_landingHeight),
      m_steppableRegionIn("steppableRegion", m_steppableRegion),
      m_qOut("q", m_qRef),
      m_qAbcOut("qAbc", m_qAbc),
      m_tauOut("tau", m_tau),
      m_zmpOut("zmpOut", m_zmp),
      m_zmpActOut("zmpAct", m_zmpAct),
      m_refCPOut("refCapturePoint", m_refCP),
      m_actCPOut("actCapturePoint", m_actCP),
      m_actContactStatesOut("actContactStates", m_actContactStates),
      m_COPInfoOut("COPInfo", m_COPInfo),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_baseTformOut("baseTformOut", m_baseTform),
      m_tmpOut("tmp", m_tmp),
      m_currentLandingPosOut("currentLandingPos", m_currentLandingPos),
      m_diffFootOriginExtMomentOut("diffFootOriginExtMoment", m_diffFootOriginExtMoment),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRef", m_accRef),
      m_contactStatesOut("contactStates", m_contactStates),
      m_toeheelRatioOut("toeheelRatio", m_toeheelRatio),
      m_controlSwingSupportTimeOut("controlSwingSupportTime", m_controlSwingSupportTime),
      m_walkingStatesOut("walkingStates", m_walkingStates),
      m_sbpCogOffsetOut("sbpCogOffset", m_sbpCogOffset),
      m_cogOut("cogOut", m_cog),
      m_allEECompOut("allEEComp", m_allEEComp),
      m_landingTargetOut("landingTarget", m_landingTarget),
      m_endCogStateOut("endCogState", m_endCogState),
      m_AutoBalancerServicePort("AutoBalancerService"),
      m_RobotHardwareServicePort("RobotHardwareService"),
      // for debug output
      m_originRefZmpOut("originRefZmp", m_originRefZmp),
      m_originRefCogOut("originRefCog", m_originRefCog),
      m_originRefCogVelOut("originRefCogVel", m_originRefCogVel),
      m_originNewZmpOut("originNewZmp", m_originNewZmp),
      m_originActZmpOut("originActZmp", m_originActZmp),
      m_originActCogOut("originActCog", m_originActCog),
      m_originActCogVelOut("originActCogVel", m_originActCogVel),
      m_currentSteppableRegionOut("currentSteppableRegion", m_currentSteppableRegion),
      // </rtc-template>
      gait_type(BIPED),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0)
{
    m_service0.autobalancer(this);
}

AutoBalancer::~AutoBalancer()
{
}


RTC::ReturnCode_t AutoBalancer::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("qTouchWall", m_qTouchWallIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("zmpIn", m_zmpIn);
    addInPort("optionalData", m_optionalDataIn);
    addInPort("diffCapturePoint", m_diffCPIn);
    addInPort("refFootOriginExtMoment", m_refFootOriginExtMomentIn);
    addInPort("refFootOriginExtMomentIsHoldValue", m_refFootOriginExtMomentIsHoldValueIn);
    addInPort("touchWallMotionSolved", m_touchWallMotionSolvedIn);
    addInPort("rpy", m_rpyIn);
    addInPort("qRefSeq", m_qRefSeqIn);
    addInPort("landingHeight", m_landingHeightIn);
    addInPort("steppableRegion", m_steppableRegionIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("qAbc", m_qAbcOut);
    addOutPort("tau", m_tauOut);
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("zmpAct", m_zmpActOut);
    addOutPort("refCapturePoint", m_refCPOut);
    addOutPort("actCapturePoint", m_actCPOut);
    addOutPort("actContactStates", m_actContactStatesOut);
    addOutPort("COPInfo", m_COPInfoOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("tmpOut", m_tmpOut);
    addOutPort("currentLandingPosOut", m_currentLandingPosOut);
    addOutPort("diffStaticBalancePointOffset", m_diffFootOriginExtMomentOut);
    addOutPort("allEEComp", m_allEECompOut);
    addOutPort("basePoseOut", m_basePoseOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("contactStates", m_contactStatesOut);
    addOutPort("toeheelRatio", m_toeheelRatioOut);
    addOutPort("controlSwingSupportTime", m_controlSwingSupportTimeOut);
    addOutPort("cogOut", m_cogOut);
    addOutPort("walkingStates", m_walkingStatesOut);
    addOutPort("sbpCogOffset", m_sbpCogOffsetOut);
    addOutPort("emergencySignal", m_emergencySignalOut);
    addOutPort("emergencyFallMotion", m_emergencyFallMotionOut);
    addOutPort("isStuck", m_isStuckOut);
    addOutPort("useFlywheel", m_useFlywheelOut);
    addOutPort("estimatedFxy", m_estimatedFxyOut);
    addOutPort("landingTarget", m_landingTargetOut);
    addOutPort("endCogState", m_endCogStateOut);
    // for debug output
    addOutPort("originRefZmp", m_originRefZmpOut);
    addOutPort("originRefCog", m_originRefCogOut);
    addOutPort("originRefCogVel", m_originRefCogVelOut);
    addOutPort("originNewZmp", m_originNewZmpOut);
    addOutPort("originActZmp", m_originActZmpOut);
    addOutPort("originActCog", m_originActCogOut);
    addOutPort("originActCogVel", m_originActCogVelOut);
    addOutPort("currentSteppableRegion", m_currentSteppableRegionOut);

    // Set service provider to Ports
    m_AutoBalancerServicePort.registerProvider("service0", "AutoBalancerService", m_service0);

    // Set CORBA Service Ports
    addPort(m_AutoBalancerServicePort);

    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    // </rtc-template>

    RTC::Properties& prop =  getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    m_qAbc.data.length(m_robot->numJoints());
    m_qCurrent.data.length(m_robot->numJoints());
    m_qRefSeq.data.length(m_robot->numJoints());
    m_tau.data.length(m_robot->numJoints());
    m_qTouchWall.data.length(m_robot->numJoints());
    m_baseTform.data.length(12);
    m_tmp.data.length(27);
    diff_q.resize(m_robot->numJoints());
    // for debug output
    m_originRefZmp.data.x = m_originRefZmp.data.y = m_originRefZmp.data.z = 0.0;
    m_originRefCog.data.x = m_originRefCog.data.y = m_originRefCog.data.z = 0.0;
    m_originRefCogVel.data.x = m_originRefCogVel.data.y = m_originRefCogVel.data.z = 0.0;
    m_originNewZmp.data.x = m_originNewZmp.data.y = m_originNewZmp.data.z = 0.0;
    m_originActZmp.data.x = m_originActZmp.data.y = m_originActZmp.data.z = 0.0;
    m_originActCog.data.x = m_originActCog.data.y = m_originActCog.data.z = 0.0;
    m_originActCogVel.data.x = m_originActCogVel.data.y = m_originActCogVel.data.z = 0.0;
    m_currentSteppableRegion.data.region.length(1);
    m_currentSteppableRegion.data.region[0].length(1);

    control_mode = MODE_IDLE;
    loop = 0;

    // setting from conf file
    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
    // setting leg_pos from conf file
    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
    leg_names.push_back("rleg");
    leg_names.push_back("lleg");

    // Generate FIK

    fik = fikPtr(new FullbodyInverseKinematicsSolver(m_robot, std::string(m_profile.instance_name), m_dt));
    ik_mode = OpenHRP::AutoBalancerService::SIMPLE;
    // Generate ST
    st = stPtr(new Stabilizer(m_robot, std::string(m_profile.instance_name), m_dt));

    // Set service consumers to Ports
    m_RobotHardwareServicePort.registerConsumer("service0", "RobotHardwareService", st->m_robotHardwareService0);
    // Set CORBA Service Ports
    addPort(m_RobotHardwareServicePort);

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    size_t prop_num = 10;
    if (end_effectors_str.size() > 0) {
      size_t num = end_effectors_str.size()/prop_num;
      for (size_t i = 0; i < num; i++) {
        std::string ee_name, ee_target, ee_base;
        coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
        coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
        coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
        ABCIKparam tp;
        hrp::Link* root = m_robot->link(ee_target);
        for (size_t j = 0; j < 3; j++) {
          coil::stringTo(tp.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
        }
        double tmpv[4];
        for (int j = 0; j < 4; j++ ) {
          coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
        }
        tp.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
        // FIK param
        FullbodyInverseKinematicsSolver::IKparam tmp_fikp;
        tmp_fikp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base), m_robot->link(ee_target), m_dt, false, std::string(m_profile.instance_name)));
        tmp_fikp.target_link = m_robot->link(ee_target);
        tmp_fikp.localPos = tp.localPos;
        tmp_fikp.localR = tp.localR;
        tmp_fikp.max_limb_length = tp.localPos.norm();
        while (!root->isRoot()) {
          tmp_fikp.max_limb_length += root->b.norm();
          tmp_fikp.parent_name = root->name;
          tmp_fikp.group_indices.push_back(root->jointId);
          root = root->parent;
        }
        fik->ikp.insert(std::pair<std::string, FullbodyInverseKinematicsSolver::IKparam>(ee_name, tmp_fikp));
        // ST param
        Stabilizer::STIKParam stp;
        for (size_t j = 0; j < 3; j++) {
          stp.localp(j) = tp.localPos(j);
        }
        stp.localR = tp.localR;
        stp.target_name = ee_target;
        stp.ee_name = ee_name;
        stp.avoid_gain = 0.001;
        stp.reference_gain = 0.01;
        stp.ik_loop_count = 3;
        // For swing ee modification
        stp.target_ee_diff_p = hrp::Vector3::Zero();
        stp.d_rpy_swing = hrp::Vector3::Zero();
        stp.d_pos_swing = hrp::Vector3::Zero();
        stp.target_ee_diff_p_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, m_dt, hrp::Vector3::Zero())); // [Hz]
        stp.prev_d_pos_swing = hrp::Vector3::Zero();
        stp.prev_d_rpy_swing = hrp::Vector3::Zero();
        stp.contact_phase = Stabilizer::SUPPORT_PHASE;
        stp.phase_time = 0;
        {
          bool is_ee_exists = false;
          for (size_t j = 0; j < m_robot->numSensors(hrp::Sensor::FORCE); j++) {
            hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, j);
            hrp::Link* alink = m_robot->link(stp.target_name);
            while (alink != NULL && alink->name != ee_base && !is_ee_exists) {
              if ( alink->name == sensor->link->name ) {
                is_ee_exists = true;
                stp.sensor_name = sensor->name;
              }
              alink = alink->parent;
            }
          }
        }
        st->stikp.push_back(stp);
        st->jpe_v.push_back(hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base), m_robot->link(ee_target), m_dt, false, std::string(m_profile.instance_name))));
        // Fix for toe joint
        if (ee_name.find("leg") != std::string::npos && st->jpe_v.back()->numJoints() == 7) { // leg and has 7dof joint (6dof leg +1dof toe)
          std::vector<double> optw;
          for (int j = 0; j < st->jpe_v.back()->numJoints(); j++ ) {
            if ( j == st->jpe_v.back()->numJoints()-1 ) optw.push_back(0.0);
            else optw.push_back(1.0);
          }
          st->jpe_v.back()->setOptionalWeightVector(optw);
        }
        for (int j = 0; j < st->jpe_v.back()->numJoints(); j++ ) {
            st->stikp.back().support_pgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
            st->stikp.back().support_dgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
            st->stikp.back().landing_pgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
            st->stikp.back().landing_dgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
            st->stikp.back().swing_pgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
            st->stikp.back().swing_dgain = hrp::dvector::Constant(st->jpe_v.back()->numJoints(),100);
        }
        st->contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
        st->is_ik_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // Hands ik => disabled, feet ik => enabled, by default
        st->is_feedback_control_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // Hands feedback control => disabled, feet feedback control => enabled, by default
        st->is_zmp_calc_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // To zmp calculation, hands are disabled and feet are enabled, by default
        // Fix for toe joint
        //   Toe joint is defined as end-link joint in the case that end-effector link != force-sensor link
        //   Without toe joints, "end-effector link == force-sensor link" is assumed.
        //   With toe joints, "end-effector link != force-sensor link" is assumed.
        if (m_robot->link(ee_target)->sensors.size() == 0) { // If end-effector link has no force sensor
            std::vector<double> optw(fik->ikp[ee_name].manip->numJoints(), 1.0);
            optw.back() = 0.0; // Set weight = 0 for toe joint by default
            fik->ikp[ee_name].manip->setOptionalWeightVector(optw);
            tp.has_toe_joint = true;
        } else {
            tp.has_toe_joint = false;
        }
        tp.target_link = m_robot->link(ee_target);
        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
        ee_vec.push_back(ee_name);
        std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   target = " << ikp[ee_name].target_link->name << ", base = " << ee_base << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << tp.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   has_toe_joint = " << (tp.has_toe_joint?"true":"false") << std::endl;
        contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
        if (( ee_name == "lleg" ) || ( ee_name == "rleg" )) touchdown_transition_interpolator.insert(std::pair<std::string, interpolator*>(ee_name, new interpolator(1, m_dt, interpolator::CUBICSPLINE)));
        st->swing_modification_interpolator.insert(std::pair<std::string, interpolator*>(ee_name, new interpolator(1, m_dt, interpolator::CUBICSPLINE)));
      }
      m_contactStates.data.length(num);
      m_actContactStates.data.length(num);
      m_toeheelRatio.data.length(num);
      if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
        m_actContactStates.data[contact_states_index_map["rleg"]] = true;
        m_actContactStates.data[contact_states_index_map["lleg"]] = true;
      }
      if (ikp.find("rarm") != ikp.end() && ikp.find("larm") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rarm"]] = false;
        m_contactStates.data[contact_states_index_map["larm"]] = false;
        m_actContactStates.data[contact_states_index_map["rarm"]] = false;
        m_actContactStates.data[contact_states_index_map["larm"]] = false;
      }
      m_controlSwingSupportTime.data.length(num);
      for (size_t i = 0; i < num; i++) m_controlSwingSupportTime.data[i] = 1.0;
      for (size_t i = 0; i < num; i++) m_toeheelRatio.data[i] = rats::no_using_toe_heel_ratio;
      m_COPInfo.data.length(m_contactStates.data.length()*3); // nx, ny, fz for each end-effectors
      for (size_t i = 0; i < m_COPInfo.data.length(); i++) {
        m_COPInfo.data[i] = 0.0;
      }

      // ST param
      st->initStabilizer(prop, num);
    }
    std::vector<hrp::Vector3> leg_pos;
    if (leg_offset_str.size() > 0) {
      hrp::Vector3 leg_offset;
      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
      std::cerr << "[" << m_profile.instance_name << "] abc_leg_offset = " << leg_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
      leg_pos.push_back(hrp::Vector3(leg_offset));
    }
    if (leg_pos.size() < ikp.size()) {
        size_t tmp_leg_pos_size = leg_pos.size();
        for (size_t i = 0; i < ikp.size() - tmp_leg_pos_size; i++) {
            leg_pos.push_back(hrp::Vector3::Zero());
        }
    }

    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    if (interlocking_joints.size() > 0) {
        fik->initializeInterlockingJoints(interlocking_joints);
    }

    zmp_offset_interpolator = new interpolator(ikp.size()*3, m_dt);
    zmp_offset_interpolator->setName(std::string(m_profile.instance_name)+" zmp_offset_interpolator");
    zmp_transition_time = 1.0;
    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
    transition_interpolator_ratio = 0.0;
    emergency_transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    emergency_transition_interpolator->setName(std::string(m_profile.instance_name)+" emergency_transition_interpolator");
    touch_wall_transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    touch_wall_transition_interpolator->setName(std::string(m_profile.instance_name)+" touch_wall_transition_interpolator");
    adjust_footstep_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    adjust_footstep_interpolator->setName(std::string(m_profile.instance_name)+" adjust_footstep_interpolator");
    transition_time = 2.0;
    adjust_footstep_transition_time = 2.0;
    leg_names_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    leg_names_interpolator->setName(std::string(m_profile.instance_name)+" leg_names_interpolator");
    leg_names_interpolator_ratio = 1.0;
    angular_momentum_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB);
    angular_momentum_interpolator->setName(std::string(m_profile.instance_name)+" angular_momentum_interpolator");
    roll_weight_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB);
    roll_weight_interpolator->setName(std::string(m_profile.instance_name)+" roll_weight_interpolator");
    pitch_weight_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB);
    pitch_weight_interpolator->setName(std::string(m_profile.instance_name)+" pitch_weight_interpolator");
    go_vel_interpolator = new interpolator(3, m_dt, interpolator::HOFFARBIB);
    go_vel_interpolator->setName(std::string(m_profile.instance_name)+" go_vel_interpolator");
    cog_constraint_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB);
    cog_constraint_interpolator->setName(std::string(m_profile.instance_name)+" cog_constraint_interpolator");
    limit_cog_interpolator = new interpolator(3, m_dt, interpolator::CUBICSPLINE);
    limit_cog_interpolator->setName(std::string(m_profile.instance_name)+" limit_cog_interpolator");
    hand_fix_interpolator = new interpolator(3, m_dt, interpolator::CUBICSPLINE);
    hand_fix_interpolator->setName(std::string(m_profile.instance_name)+" hand_fix_interpolator");

    // setting stride limitations from conf file
    double stride_fwd_x_limit = 0.15;
    double stride_outside_y_limit = 0.05;
    double stride_outside_th_limit = 10;
    double stride_bwd_x_limit = 0.05;
    double stride_inside_y_limit = stride_outside_y_limit*0.5;
    double stride_inside_th_limit = stride_outside_th_limit*0.5;
    std::cerr << "[" << m_profile.instance_name << "] abc_stride_parameter : " << stride_fwd_x_limit << "[m], " << stride_outside_y_limit << "[m], " << stride_outside_th_limit << "[deg], " << stride_bwd_x_limit << "[m]" << std::endl;
    if (default_zmp_offsets.size() == 0) {
      for (size_t i = 0; i < ikp.size(); i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
    }
    if (leg_offset_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, leg_names, stride_fwd_x_limit/*[m]*/, stride_outside_y_limit/*[m]*/, stride_outside_th_limit/*[deg]*/,
                                          stride_bwd_x_limit/*[m]*/, stride_inside_y_limit/*[m]*/, stride_inside_th_limit/*[m]*/));
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    gg_is_walking = gg_solved = false;
    m_walkingStates.data = false;
    fix_leg_coords = coordinates();

    gg_is_wheeling = false;

    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    // ref force port
    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    unsigned int nvforce = m_vfs.size();
    unsigned int nforce  = npforce + nvforce;
    // check number of force sensors
    if (nforce < m_contactStates.data.length()) {
        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has less force sensors(" << nforce;
        std::cerr << ") than end-effector settings(" << m_contactStates.data.length() << ") !" << std::endl;
    }

    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    m_force.resize(nforce);
    m_ref_forceOut.resize(nforce);
    m_limbCOPOffset.resize(nforce);
    m_limbCOPOffsetOut.resize(nforce);
    m_wrenches.resize(nforce);
    m_wrenchesIn.resize(nforce);
    st->wrenches.resize(nforce);
    st->ref_wrenches.resize(nforce);
    st->limbCOPOffset.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    for (unsigned int i=0; i<nvforce; i++){
        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
            if (it->second.id == (int)i) sensor_names.push_back(it->first);
        }
    }
    // set ref force port
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << nforce << ")" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+sensor_names[i]).c_str(), m_ref_force[i]);
        m_ref_force[i].data.length(6);
        registerInPort(std::string("ref_"+sensor_names[i]).c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string("ref_"+sensor_names[i]) << std::endl;
        ref_forces.push_back(hrp::Vector3(0,0,0));
        ref_moments.push_back(hrp::Vector3(0,0,0));
        // actual inport
        m_wrenchesIn[i] = new InPort<TimedDoubleSeq>(sensor_names[i].c_str(), m_wrenches[i]);
        m_wrenches[i].data.length(6);
        m_allEEComp.data.length(st->stikp.size() * 6); // 6 is pos+rot dim
        st->wrenches[i].resize(6);
        st->ref_wrenches[i].resize(6);
        registerInPort(sensor_names[i].c_str(), *m_wrenchesIn[i]);
    }
    // set force port
    for (unsigned int i=0; i<nforce; i++){
        m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string(sensor_names[i]).c_str(), m_force[i]);
        m_force[i].data.length(6);
        m_force[i].data[0] = m_force[i].data[1] = m_force[i].data[2] = 0.0;
        m_force[i].data[3] = m_force[i].data[4] = m_force[i].data[5] = 0.0;
        registerOutPort(std::string(sensor_names[i]).c_str(), *m_ref_forceOut[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string(sensor_names[i]) << std::endl;
    }
    // set limb cop offset port
    std::cerr << "[" << m_profile.instance_name << "] limbCOPOffset ports (" << nforce << ")" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        std::string nm("limbCOPOffset_"+sensor_names[i]);
        m_limbCOPOffsetOut[i] = new OutPort<TimedPoint3D>(nm.c_str(), m_limbCOPOffset[i]);
        registerOutPort(nm.c_str(), *m_limbCOPOffsetOut[i]);
        m_limbCOPOffset[i].data.x = m_limbCOPOffset[i].data.y = m_limbCOPOffset[i].data.z = 0.0;
        std::cerr << "[" << m_profile.instance_name << "]   name = " << nm << std::endl;
    }
    sbp_offset = hrp::Vector3(0,0,0);
    sbp_cog_offset = hrp::Vector3(0,0,0);
    //use_force = MODE_NO_FORCE;
    use_force = MODE_REF_FORCE;

    if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
      is_legged_robot = true;
    } else {
      is_legged_robot = false;
    }

    m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
    prev_imu_sensor_vel = hrp::Vector3::Zero();

    graspless_manip_mode = false;
    graspless_manip_arm = "arms";
    graspless_manip_p_gain = hrp::Vector3::Zero();

    is_stop_mode = false;
    is_hand_fix_mode = false;

    use_act_states = false;
    gg->use_act_states = st->use_act_states = true;

    is_after_walking = false;
    prev_roll_state = prev_pitch_state = false;
    prev_orig_cog = orig_cog = hrp::Vector3::Zero();
    prev_orig_dif_p = orig_dif_p = hrp::Vector3::Zero();

    is_emergency_step_mode = false;
    is_emergency_touch_wall_mode = false;
    is_emergency_stopping = false;
    is_touch_wall_motion_solved = false;
    touch_wall_retrieve_time = 0.5;
    use_collision_avoidance = false;
    is_natural_walk = false;
    is_stop_early_foot = false;

    cog_z_constraint = 1e-3;
    arm_swing_deg = 30.0;

    debug_read_steppable_region = false;

    wheel_radius = 0.15;
    d_wheel_angle = 0.0;
    start_d_wheel_angle = 0.0;

    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen == NULL) {
        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
    }

    additional_force_applied_link = m_robot->rootLink();
    additional_force_applied_point_offset = hrp::Vector3::Zero();
    return RTC::RTC_OK;
}



RTC::ReturnCode_t AutoBalancer::onFinalize()
{
  delete zmp_offset_interpolator;
  delete transition_interpolator;
  delete emergency_transition_interpolator;
  delete touch_wall_transition_interpolator;
  delete adjust_footstep_interpolator;
  delete leg_names_interpolator;
  delete angular_momentum_interpolator;
  delete roll_weight_interpolator;
  delete pitch_weight_interpolator;
  delete st->transition_interpolator;
  delete go_vel_interpolator;
  delete cog_constraint_interpolator;
  delete limit_cog_interpolator;
  delete hand_fix_interpolator;
  delete st->after_walking_interpolator;
  for ( std::map<std::string, interpolator*>::iterator it = touchdown_transition_interpolator.begin(); it != touchdown_transition_interpolator.end(); it++ ) {
    delete it->second;
  }
  for ( std::map<std::string, interpolator*>::iterator it = st->swing_modification_interpolator.begin(); it != st->swing_modification_interpolator.end(); it++ ) {
    delete it->second;
  }
  if (st->szd == NULL) {
    delete st->szd;
    st->szd = NULL;
  }
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t AutoBalancer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t AutoBalancer::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoBalancer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  Guard guard(m_mutex);
  if (control_mode == MODE_ABC) {
    control_mode = MODE_SYNC_TO_IDLE;
    double tmp_ratio = 0.0;
    transition_interpolator->setGoal(&tmp_ratio, m_dt, true); // sync in one controller loop
  }
  if ( (st->control_mode == Stabilizer::MODE_ST || st->control_mode == Stabilizer::MODE_AIR) ) {
    st->sync_2_idle ();
    st->control_mode = Stabilizer::MODE_IDLE;
    st->transition_count = 1; // sync in one controller loop
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
//#define DEBUGP2 ((loop%200==0))
#define DEBUGP2 (false)
RTC::ReturnCode_t AutoBalancer::onExecute(RTC::UniqueId ec_id)
{
  // std::cerr << "AutoBalancer::onExecute(" << ec_id << ")" << std::endl;
    loop ++;

    if (is_emergency_step_mode && st->is_emergency_step && !gg_is_walking && !is_stop_mode) {
      gg->set_is_emergency_step(true);
      // if (fabs(st->act_cp(0)) > fabs(st->act_cp(1))) {
        if (st->act_cp(0) > 0) goVelocity(0,(st->ref_foot_origin_rot*ikp["lleg"].target_p0)(0) > (st->ref_foot_origin_rot*ikp["rleg"].target_p0)(0) ? -1e-6:1e-6,0);
        else goVelocity(0,(st->ref_foot_origin_rot*ikp["lleg"].target_p0)(0) > (st->ref_foot_origin_rot*ikp["rleg"].target_p0)(0) ? 1e-6:-1e-6,0);
      // } else goVelocity(0,st->act_cp(1)>0?-1e-6:1e-6,0);
    }

    // Read Inport
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if (m_qCurrentIn.isNew()) {
      m_qCurrentIn.read();
    }
    if (m_qTouchWallIn.isNew()) {
        m_qTouchWallIn.read();
    }
    if (m_basePosIn.isNew()) {
      m_basePosIn.read();
      input_basePos(0) = m_basePos.data.x;
      input_basePos(1) = m_basePos.data.y;
      input_basePos(2) = m_basePos.data.z;
    }
    if (m_baseRpyIn.isNew()) {
      m_baseRpyIn.read();
      input_baseRot = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    }
    if (m_zmpIn.isNew()) {
      m_zmpIn.read();
      input_zmp(0) = m_zmp.data.x;
      input_zmp(1) = m_zmp.data.y;
      input_zmp(2) = m_zmp.data.z;
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    if (m_optionalDataIn.isNew()) {
        m_optionalDataIn.read();
    }
    if (st->reset_emergency_flag) {
      m_emergencySignal.data = 0;
      m_emergencySignalOut.write();
      m_emergencyFallMotion.data = false;
      m_emergencyFallMotionOut.write();
      st->reset_emergency_flag = false;
    } else {
      if (st->is_emergency_motion && !is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] emergencyFallMotion is set!" << std::endl;
        is_stop_mode = true;
        m_emergencyFallMotion.data = true;
        m_emergencyFallMotionOut.write();
        gg->finalize_velocity_mode2();
        stopABCparamEmergency();
        st->stopSTEmergency();
      }
      if (gg_is_walking && !is_stop_mode && gg->is_emergency_touch_wall && is_emergency_touch_wall_mode) {
        std::cerr << "[" << m_profile.instance_name << "] emergencyTouchWall is set!" << std::endl;
        m_emergencySignal.data = 2;
        is_stop_mode = true;
        m_emergencySignalOut.write();
        double tmp_ratio = 1.0;
        touch_wall_transition_interpolator->set(&tmp_ratio);
        tmp_ratio = 0.0;
        touch_wall_transition_interpolator->setGoal(&tmp_ratio, touch_wall_retrieve_time, true);
        gg->emergency_stop();
        is_emergency_stopping = true;
      } else if (is_stop_mode && !gg_is_walking && is_emergency_stopping) {
        stopABCparamEmergency();
        st->stopSTEmergency();
        is_emergency_stopping = false;
        touch_wall_transition_interpolator->clear();
      }
    }
    if (m_diffCPIn.isNew()) {
      m_diffCPIn.read();
      gg->set_diff_cp(hrp::Vector3(m_diffCP.data.x, m_diffCP.data.y, m_diffCP.data.z));
    }
    if (m_refFootOriginExtMomentIn.isNew()) {
      m_refFootOriginExtMomentIn.read();
    }
    if (m_refFootOriginExtMomentIsHoldValueIn.isNew()) {
      m_refFootOriginExtMomentIsHoldValueIn.read();
    }
    if (m_touchWallMotionSolvedIn.isNew()) {
      m_touchWallMotionSolvedIn.read();
      is_touch_wall_motion_solved = m_touchWallMotionSolved.data;
    }
    if (m_rpyIn.isNew()) {
      m_rpyIn.read();
    }
    for (size_t i = 0; i < m_wrenchesIn.size(); ++i) {
      if ( m_wrenchesIn[i]->isNew() ) {
        m_wrenchesIn[i]->read();
      }
    }
    gg->set_is_vision_updated(false);
    if (m_landingHeightIn.isNew()) {
      m_landingHeightIn.read();
      if ((gg->get_support_leg_names().front() == "rleg" && m_landingHeight.data.l_r == 0) ||
          (gg->get_support_leg_names().front() == "lleg" && m_landingHeight.data.l_r == 1))
        {
          hrp::Vector3 pos(hrp::Vector3(m_landingHeight.data.x, m_landingHeight.data.y, m_landingHeight.data.z));
          hrp::Vector3 normal(hrp::Vector3(m_landingHeight.data.nx, m_landingHeight.data.ny, m_landingHeight.data.nz));
          if (isfinite(pos(2))) {
            gg->set_is_vision_updated(true);
            gg->set_rel_landing_height(pos);
            gg->set_rel_landing_normal(normal);
          }
        }
    }
    if (m_steppableRegionIn.isNew()) {
      m_steppableRegionIn.read();
      if ((gg->get_support_leg_names().front() == "rleg" && m_steppableRegion.data.l_r == 0) ||
          (gg->get_support_leg_names().front() == "lleg" && m_steppableRegion.data.l_r == 1))
        {
          gg->set_steppable_region(m_steppableRegion);
        }
    }

    // Calculation
    Guard guard(m_mutex);
    if ( is_legged_robot ) {
      // For parameters
      setActData2ST();
      st->getActualParameters();
      gg->set_act_contact_states(st->act_contact_states);
      // if (!go_vel_interpolator->isEmpty()) {
      //   std::vector<double> tmp_v(3);
      //   go_vel_interpolator->get(tmp_v.data(), true);
      //   gg->set_velocity_param(tmp_v[0], tmp_v[1], tmp_v[2]);
      // }
      getTargetParameters();
      // Get transition ratio
      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
      if (!is_transition_interpolator_empty) {
        transition_interpolator->get(&transition_interpolator_ratio, true);
      } else {
        transition_interpolator_ratio = (control_mode == MODE_IDLE) ? 0.0 : 1.0;
      }
      if (control_mode != MODE_IDLE ) {
          switch(ik_mode){
              case OpenHRP::AutoBalancerService::SIMPLE:
                  solveSimpleFullbodyIK();
                  break;
              case OpenHRP::AutoBalancerService::FULLBODY:
                  solveFullbodyIK();
                  break;
              default:
                  break;
          }
//        /////// Inverse Dynamics /////////
//        if(!idsb.is_initialized){
//          idsb.setInitState(m_robot, m_dt);
//          invdyn_zmp_filters.resize(3);
//          for(int i=0;i<3;i++){
//            invdyn_zmp_filters[i].setParameterAsBiquad(25, 1/std::sqrt(2), 1.0/m_dt);
//            invdyn_zmp_filters[i].reset(ref_zmp(i));
//          }
//        }
//        calcAccelerationsForInverseDynamics(m_robot, idsb);
//        if(gg_is_walking){
//          calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp);
//          for(int i=0;i<3;i++) ref_zmp(i) = invdyn_zmp_filters[i].passFilter(ref_zmp(i));
//        }
//        updateInvDynStateBuffer(idsb);

        rel_ref_zmp = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);
        fik->storeCurrentParameters();

        if (is_touch_wall_motion_solved) {
          double tmp_ratio = 1.0;
          if (!touch_wall_transition_interpolator->isEmpty()) {
            touch_wall_transition_interpolator->get(&tmp_ratio, true);
          } else if (is_emergency_stopping) {
            tmp_ratio = 0.0;
          }
          for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = (1-tmp_ratio) * m_qTouchWall.data[i] + tmp_ratio * m_robot->joint(i)->q;
          }
        }

        // overwirte wheel angle
        if (gg_is_wheeling) d_wheel_angle = start_d_wheel_angle + gg->get_cur_wheel_pos_x() / wheel_radius;
        else start_d_wheel_angle = d_wheel_angle;
        m_robot->joint(6)->q = m_qRef.data[6] + d_wheel_angle;
        m_robot->joint(13)->q = m_qRef.data[13] + d_wheel_angle;
      } else {
        if (!emergency_transition_interpolator->isEmpty()) {
          double tmp_ratio;
          emergency_transition_interpolator->get(&tmp_ratio, true);
          for ( int i = 0; i < m_robot->numJoints(); i++ ){
            m_robot->joint(i)->q = m_qRef.data[i] + tmp_ratio * diff_q[i];
          }
        }
        rel_ref_zmp = input_zmp;
        fik->d_root_height = 0.0;
        fik->storeCurrentParameters();
      }
      // Transition
      if (!is_transition_interpolator_empty) {
        // transition_interpolator_ratio 0=>1 : IDLE => ABC
        // transition_interpolator_ratio 1=>0 : ABC => IDLE
        ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_robot->rootLink()->p;
        rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * rel_ref_zmp;
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ) {
          m_robot->joint(i)->q = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_robot->joint(i)->q;
        }
        for (unsigned int i=0; i< m_force.size(); i++) {
            for (unsigned int j=0; j<6; j++) {
                m_force[i].data[j] = transition_interpolator_ratio * m_force[i].data[j] + (1-transition_interpolator_ratio) * m_ref_force[i].data[j];
            }
        }
        for (unsigned int i=0; i< m_limbCOPOffset.size(); i++) {
            // transition (TODO:set stopABCmode value instead of 0)
            m_limbCOPOffset[i].data.x = transition_interpolator_ratio * m_limbCOPOffset[i].data.x;// + (1-transition_interpolator_ratio) * 0;
            m_limbCOPOffset[i].data.y = transition_interpolator_ratio * m_limbCOPOffset[i].data.y;// + (1-transition_interpolator_ratio) * 0;
            m_limbCOPOffset[i].data.z = transition_interpolator_ratio * m_limbCOPOffset[i].data.z;// + (1-transition_interpolator_ratio) * 0;
        }
      } else {
        ref_basePos = m_robot->rootLink()->p;
        ref_baseRot = m_robot->rootLink()->R;
      }
      // mode change for sync
      if (control_mode == MODE_SYNC_TO_ABC) {
        control_mode = MODE_ABC;
      } else if (control_mode == MODE_SYNC_TO_IDLE && transition_interpolator->isEmpty() ) {
        std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                  << "] Finished cleanup" << std::endl;
        control_mode = MODE_IDLE;
      }
      m_robot->calcForwardKinematics();
      hrp::Vector3 tmp_ref_cog(m_robot->calcCM());
      ref_cog(2) = tmp_ref_cog(2);
    }

    // Write Outport
    if ( m_qRef.data.length() != 0 ) { // initialized
      if (is_legged_robot) {
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
          m_qAbc.data[i] = m_robot->joint(i)->q;
        }
      }
      m_qAbcOut.write();
    }
    if (is_legged_robot) {
      // basePos
      m_basePos.data.x = ref_basePos(0);
      m_basePos.data.y = ref_basePos(1);
      m_basePos.data.z = ref_basePos(2);
      m_basePos.tm = m_qRef.tm;
      // baseRpy
      baseRpy = hrp::rpyFromRot(ref_baseRot);
      m_baseRpy.data.r = baseRpy(0);
      m_baseRpy.data.p = baseRpy(1);
      m_baseRpy.data.y = baseRpy(2);
      m_baseRpy.tm = m_qRef.tm;
      // baseTform
      double *tform_arr = m_baseTform.data.get_buffer();
      tform_arr[0] = m_basePos.data.x;
      tform_arr[1] = m_basePos.data.y;
      tform_arr[2] = m_basePos.data.z;
      hrp::setMatrix33ToRowMajorArray(ref_baseRot, tform_arr, 3);
      m_baseTform.tm = m_qRef.tm;
      // basePose
      m_basePose.data.position.x = m_basePos.data.x;
      m_basePose.data.position.y = m_basePos.data.y;
      m_basePose.data.position.z = m_basePos.data.z;
      m_basePose.data.orientation.r = m_baseRpy.data.r;
      m_basePose.data.orientation.p = m_baseRpy.data.p;
      m_basePose.data.orientation.y = m_baseRpy.data.y;
      m_basePose.tm = m_qRef.tm;
      // zmp
      m_zmp.data.x = rel_ref_zmp(0);
      m_zmp.data.y = rel_ref_zmp(1);
      m_zmp.data.z = rel_ref_zmp(2);
      m_zmp.tm = m_qRef.tm;
      // cog
      m_cog.data.x = ref_cog(0);
      m_cog.data.y = ref_cog(1);
      m_cog.data.z = ref_cog(2);
      m_cog.tm = m_qRef.tm;
      // sbpCogOffset
      m_sbpCogOffset.data.x = sbp_cog_offset(0);
      m_sbpCogOffset.data.y = sbp_cog_offset(1);
      m_sbpCogOffset.data.z = sbp_cog_offset(2);
      m_sbpCogOffset.tm = m_qRef.tm;
      // is stuck
      m_isStuck.data = gg->is_stuck;
      m_isStuck.tm = m_qRef.tm;
      // use flywheel
      m_useFlywheel.data = gg->get_use_roll_flywheel() || gg->get_use_pitch_flywheel();
      m_useFlywheel.tm = m_qRef.tm;
      // estimated Fxy
      m_estimatedFxy.data.x = gg->fxy(0);
      m_estimatedFxy.data.y = gg->fxy(1);
      m_estimatedFxy.data.z = gg->fxy(2);
      m_estimatedFxy.tm = m_qRef.tm;
      // diff foot origin ext moment
      m_diffFootOriginExtMoment.data.x = st->diff_foot_origin_ext_moment(0);
      m_diffFootOriginExtMoment.data.y = st->diff_foot_origin_ext_moment(1);
      m_diffFootOriginExtMoment.data.z = st->diff_foot_origin_ext_moment(2);
      m_diffFootOriginExtMoment.tm = m_qRef.tm;
      // write
      m_basePosOut.write();
      m_baseRpyOut.write();
      m_baseTformOut.write();
      m_basePoseOut.write();
      m_zmpOut.write();
      m_cogOut.write();
      m_sbpCogOffsetOut.write();
      m_isStuckOut.write();
      m_useFlywheelOut.write();
      m_estimatedFxyOut.write();
      m_diffFootOriginExtMomentOut.write();
      for (size_t i = 0; i < 21; i++) {
        m_tmp.data[i] = gg->get_tmp(i);
      }
      m_tmp.data[19] = ref_cog(0) - m_tmp.data[19];
      m_tmp.data[20] = ref_cog(1) - m_tmp.data[20];
      hrp::Vector3 tmp_zmp = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_zmp;
      m_tmp.data[21] = tmp_zmp(0);
      m_tmp.data[22] = tmp_zmp(1);
      // tmp_zmp = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cmp;
      // m_tmp.data[23] = tmp_zmp(0); // cmp
      // m_tmp.data[24] = tmp_zmp(1); // cmp
      tmp_zmp = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cmp;
      m_tmp.data[23] = gg->get_tmp(21);
      m_tmp.data[24] = gg->get_tmp(22);
      m_tmp.tm = m_qRef.tm;
      m_tmpOut.write();
      if (gg_is_walking) {
        m_currentLandingPos.data.x = gg->get_current_landing_pos(0);
        m_currentLandingPos.data.y = gg->get_current_landing_pos(1);
        m_currentLandingPos.data.z = gg->get_current_landing_pos(2);
      } else {
        m_currentLandingPos.data.x = 0.0;
        m_currentLandingPos.data.y = 0.0;
        m_currentLandingPos.data.z = 0.0;
      }
      m_currentLandingPos.tm = m_qRef.tm;
      m_currentLandingPosOut.write();
      for (size_t i = 0; i < st->stikp.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
          m_allEEComp.data[6*i+j] = st->stikp[i].d_pos_swing(j);
          m_allEEComp.data[6*i+j+3] = st->stikp[i].d_rpy_swing(j);
        }
      }
      m_allEEComp.tm = m_qRef.tm;
      m_allEECompOut.write();

      // reference acceleration
      hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
      if (sen != NULL) {
          hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
          hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
          // convert to imu sensor local acceleration
          hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
          m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
          m_accRefOut.write();
          prev_imu_sensor_pos = imu_sensor_pos;
          prev_imu_sensor_vel = imu_sensor_vel;
      }

      // control parameters
      m_contactStates.tm = m_qRef.tm;
      m_contactStatesOut.write();
      m_controlSwingSupportTime.tm = m_qRef.tm;
      m_controlSwingSupportTimeOut.write();
      m_toeheelRatio.tm = m_qRef.tm;
      m_toeheelRatioOut.write();
      m_walkingStates.data = gg_is_walking;
      m_walkingStates.tm = m_qRef.tm;
      m_walkingStatesOut.write();

      for (unsigned int i=0; i<m_ref_forceOut.size(); i++){
          m_force[i].tm = m_qRef.tm;
          m_ref_forceOut[i]->write();
      }

      for (unsigned int i=0; i<m_limbCOPOffsetOut.size(); i++){
          m_limbCOPOffset[i].tm = m_qRef.tm;
          m_limbCOPOffsetOut[i]->write();
      }
    }

    if (gg_is_walking) {
      hrp::Vector3 off = hrp::Vector3(0.0, 0.0, 0.0);
      hrp::Vector3 pos, vel;
      int l_r; // rleg: 0, lleg: 1
      gg->get_rel_landing_pos(pos, l_r);
      m_landingTarget.data.x = pos(0) + off(0);
      m_landingTarget.data.y = pos(1) + off(1);
      m_landingTarget.data.z = pos(2) + off(2);
      m_landingTarget.data.l_r = l_r;
      m_landingTargetOut.write();
      gg->get_end_cog_state(pos, vel, l_r);
      m_endCogState.data.x = pos(0);
      m_endCogState.data.y = pos(1);
      m_endCogState.data.z = pos(2);
      m_endCogState.data.vx = vel(0);
      m_endCogState.data.vy = vel(1);
      m_endCogState.data.vz = vel(2);
      m_endCogState.data.l_r = l_r;
      m_endCogStateOut.write();
    }

    setABCData2ST();
    st->execStabilizer();

    if (!st->reset_emergency_flag && st->is_emergency) {
      m_emergencySignal.data = 1;
      m_emergencySignalOut.write();
    }
    if ( m_qRef.data.length() != 0 ) { // initialized
      if (is_legged_robot) {
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
          m_qRef.data[i] = m_robot->joint(i)->q;
          m_tau.data[i] = m_robot->joint(i)->u;
        }
      }
      m_qOut.write();
      m_tau.tm = m_qRef.tm;
      m_tauOut.write();
    }
    // for debug output
    m_originRefZmp.data.x = st->ref_zmp(0); m_originRefZmp.data.y = st->ref_zmp(1); m_originRefZmp.data.z = st->ref_zmp(2);
    m_originRefCog.data.x = st->ref_cog(0); m_originRefCog.data.y = st->ref_cog(1); m_originRefCog.data.z = st->ref_cog(2);
    m_originRefCogVel.data.x = st->ref_cogvel(0); m_originRefCogVel.data.y = st->ref_cogvel(1); m_originRefCogVel.data.z = st->ref_cogvel(2);
    m_originNewZmp.data.x = st->new_refzmp(0); m_originNewZmp.data.y = st->new_refzmp(1); m_originNewZmp.data.z = st->new_refzmp(2);
    m_originActZmp.data.x = st->act_zmp(0); m_originActZmp.data.y = st->act_zmp(1); m_originActZmp.data.z = st->act_zmp(2);
    m_originActCog.data.x = st->act_cog(0); m_originActCog.data.y = st->act_cog(1); m_originActCog.data.z = st->act_cog(2);
    m_originActCogVel.data.x = st->act_cogvel(0); m_originActCogVel.data.y = st->act_cogvel(1); m_originActCogVel.data.z = st->act_cogvel(2);
    m_zmpAct.data.x = st->rel_act_zmp(0); m_zmpAct.data.y = st->rel_act_zmp(1); m_zmpAct.data.z = st->rel_act_zmp(2);
    m_refCP.data.x = st->rel_ref_cp(0); m_refCP.data.y = st->rel_ref_cp(1); m_refCP.data.z = st->rel_ref_cp(2);
    m_actCP.data.x = st->rel_act_cp(0); m_actCP.data.y = st->rel_act_cp(1); m_actCP.data.z = st->rel_act_cp(2);
    for (size_t i = 0; i < m_actContactStates.data.length(); i++) m_actContactStates.data[i] = st->act_contact_states[i];
    for (size_t i = 0; i < m_COPInfo.data.length(); i++) m_COPInfo.data[i] = st->copInfo[i];
    if (debug_read_steppable_region && gg_is_walking) {
      gg->get_current_steppable_region(m_currentSteppableRegion);
      m_currentSteppableRegion.tm = m_qRef.tm;
      m_currentSteppableRegionOut.write();
    }
    m_originRefZmp.tm = m_qRef.tm;
    m_originRefZmpOut.write();
    m_originRefCog.tm = m_qRef.tm;
    m_originRefCogOut.write();
    m_originRefCogVel.tm = m_qRef.tm;
    m_originRefCogVelOut.write();
    m_originNewZmp.tm = m_qRef.tm;
    m_originNewZmpOut.write();
    m_originActZmp.tm = m_qRef.tm;
    m_originActZmpOut.write();
    m_originActCog.tm = m_qRef.tm;
    m_originActCogOut.write();
    m_originActCogVel.tm = m_qRef.tm;
    m_originActCogVelOut.write();
    m_zmpAct.tm = m_qRef.tm;
    m_zmpActOut.write();
    m_refCP.tm = m_qRef.tm;
    m_refCPOut.write();
    m_actCP.tm = m_qRef.tm;
    m_actCPOut.write();
    m_actContactStates.tm = m_qRef.tm;
    m_actContactStatesOut.write();
    m_COPInfo.tm = m_qRef.tm;
    m_COPInfoOut.write();
    return RTC::RTC_OK;
}

void AutoBalancer::setActData2ST()
{
  for ( int i = 0; i < m_robot->numJoints(); i++ ) {
    st->qCurrent[i] =  m_qCurrent.data[i];
  }
  for (size_t j = 0; j < st->wrenches.size(); j++) {
    for (size_t i = 0; i < 6; i++) {
      st->wrenches[j][i] = m_wrenches[j].data[i];
    }
  }
  st->rpy(0) = m_rpy.data.r;
  st->rpy(1) = m_rpy.data.p;
  st->rpy(2) = m_rpy.data.y;
}

void AutoBalancer::setABCData2ST()
{
  for ( int i = 0; i < m_robot->numJoints(); i++ ) {
    st->qRef[i] =  m_robot->joint(i)->q;
    st->qRefSeq[i] =  m_qRefSeq.data[i];
  }
  for (size_t i = 0; i < m_contactStates.data.length(); i++) {
    st->ref_contact_states[i] = m_contactStates.data[i];
    st->toeheel_ratio[i] = m_toeheelRatio.data[i];
    st->controlSwingSupportTime[i] = m_controlSwingSupportTime.data[i];
  }
  for (size_t j = 0; j < st->wrenches.size(); j++) {
    for (size_t i = 0; i < 6; i++) {
      st->ref_wrenches[j][i] = m_ref_force[j].data[i];
    }
  }
  for (size_t i = 0; i < st->limbCOPOffset.size(); i++) {
    st->limbCOPOffset[i](0) = m_limbCOPOffset[i].data.x;
    st->limbCOPOffset[i](1) = m_limbCOPOffset[i].data.y;
    st->limbCOPOffset[i](2) = m_limbCOPOffset[i].data.z;
    st->stikp[i].localCOPPos = st->stikp[i].localp + st->stikp[i].localR * hrp::Vector3(st->limbCOPOffset[i](0), 0, st->limbCOPOffset[i](2));
  }
  st->zmpRef = rel_ref_zmp;
  st->basePos = ref_basePos;
  st->baseRpy = baseRpy;
  st->is_walking = gg_is_walking || gg_is_wheeling;
  st->is_single_walking = gg_is_walking && gg->get_is_single_walking();
  st->sbp_cog_offset = sbp_cog_offset;
}

void AutoBalancer::getTargetParameters()
{
  // joint angles
  for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = m_qRef.data[i];
  }
  fik->setReferenceJointAngles();
  // basepos, rot, zmp
  m_robot->rootLink()->p = input_basePos;
  m_robot->rootLink()->R = input_baseRot;
  m_robot->calcForwardKinematics();
  gg->proc_zmp_weight_map_interpolation();
  gg->set_total_mass(m_robot->totalMass());
  if (control_mode != MODE_IDLE) {
    interpolateLegNamesAndZMPOffsets();
    // Calculate tmp_fix_coords and something
    coordinates tmp_fix_coords;
    // calc convex_hull // assume 2 legs
    {
      std::vector<bool> tmp_contact_states(2);
      std::vector<hrp::Vector3> tmp_ee_pos(2);
      std::vector <hrp::Matrix33> tmp_ee_rot(2);
      for (size_t i = 0; i < 2; i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        tmp_ee_pos[i] = tmpikp.target_p0;
        tmp_ee_rot[i] = tmpikp.target_r0;
        tmp_contact_states[i] = (gg->is_inverse_double_phase ? true : m_contactStates.data[i]);
      }
      gg->get_vertices(support_polygon_vertices);
      gg->calc_convex_hull(support_polygon_vertices, tmp_contact_states, tmp_ee_pos, tmp_ee_rot);
    }
    if ( gg_is_walking || gg_is_wheeling ) {
      gg->set_default_zmp_offsets(default_zmp_offsets);
      hrp::Vector3 act_cog = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cog;
      act_cog.head(2) += sbp_cog_offset.head(2);
      hrp::Vector3 act_cogvel = st->ref_foot_origin_rot * st->act_cogvel;
      hrp::Vector3 act_cmp = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cmp;
      if (gg_is_walking) {
        gg_solved = gg->proc_one_tick(act_cog, act_cogvel, act_cmp);
        st->falling_direction = gg->get_falling_direction();
        gg->get_swing_support_mid_coords(tmp_fix_coords);
      } else { // gg_is_wheeling
        if (!gg->proc_one_tick_wheel(act_cog, act_cogvel)) stopWheeling();
      }
    } else {
      tmp_fix_coords = fix_leg_coords;
    }
    if (!adjust_footstep_interpolator->isEmpty()) {
        calcFixCoordsForAdjustFootstep(tmp_fix_coords);
        fix_leg_coords = tmp_fix_coords;
    }
    // TODO : see explanation in this function
    fixLegToCoords2(tmp_fix_coords);
    fix_leg_coords2 = tmp_fix_coords;

    // Get output parameters and target EE coords
    target_root_p = m_robot->rootLink()->p;
    target_root_R = m_robot->rootLink()->R;
    if ( gg_is_walking ) {
      getOutputParametersForWalking();
    } else {
      getOutputParametersForABC();
    }
    //   Just for ik initial value
    if (control_mode == MODE_SYNC_TO_ABC) {
        fik->current_root_p = target_root_p;
        fik->current_root_R = target_root_R;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if ( std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
                it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
                it->second.target_r0 = it->second.target_link->R * it->second.localR;
            }
        }
    }

    // Calculate other parameters
    updateTargetCoordsForHandFixMode (tmp_fix_coords);
    rotateRefForcesForFixCoords (tmp_fix_coords);
    // TODO : see explanation in this function
    calculateOutputRefForces ();
    // TODO : see explanation in this function
    updateWalkingVelocityFromHandError(tmp_fix_coords);
    calcReferenceJointAnglesForIK();

    calcTouchoffRemainTime();

    // Calculate ZMP, COG, and sbp targets
    hrp::Vector3 tmp_ref_cog(m_robot->calcCM());
    hrp::Vector3 tmp_foot_mid_pos = calcFootMidPosUsingZMPWeightMap ();
    if (gg_is_walking || gg_is_wheeling) {
      prev_orig_cog = orig_cog;
      ref_cog = gg->get_cog();
      orig_cog = ref_cog;
    } else {
      ref_cog = tmp_foot_mid_pos;
    }
    limit_cog(ref_cog);
    ref_cog(2) = tmp_ref_cog(2);
    if (gg_is_walking || gg_is_wheeling) {
      ref_zmp = gg->get_refzmp();
    } else {
      ref_zmp(0) = ref_cog(0);
      ref_zmp(1) = ref_cog(1);
      ref_zmp(2) = tmp_foot_mid_pos(2);
    }
  } else { // MODE_IDLE
      // Update fix_leg_coords based on input basePos and rot if MODE_IDLE
      std::vector<coordinates> tmp_end_coords_list;
      for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
          if ( std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
              tmp_end_coords_list.push_back(coordinates(it->second.target_link->p+it->second.target_link->R*it->second.localPos, it->second.target_link->R*it->second.localR));
          }
      }
      multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
      getOutputParametersForIDLE();
      // Set force
      for (unsigned int i=0; i< m_force.size(); i++) {
          for (unsigned int j=0; j<6; j++) {
              m_force[i].data[j] = m_ref_force[i].data[j];
          }
      }
      is_after_walking = false;
  }
  // Just for stop walking
  if (gg_is_walking && !gg_solved) stopWalking ();
};

void AutoBalancer::getOutputParametersForWalking ()
{
    for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
        size_t idx = contact_states_index_map[it->first];
        // Check whether "it->first" ee_name is included in leg_names. leg_names is equivalent to "swing" + "support" in gg.
        if (std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end()) {
            // Set EE coords
            gg->get_swing_support_ee_coords_from_ee_name(it->second.target_p0, it->second.target_r0, it->first);
            // Set contactStates
            m_contactStates.data[idx] = gg->get_current_support_state_from_ee_name(it->first);
            // Set controlSwingSupportTime
            m_controlSwingSupportTime.data[idx] = gg->get_current_swing_time_from_ee_name(it->first);
            // Set limbCOPOffset
            hrp::Vector3 tmpzmpoff(m_limbCOPOffset[idx].data.x, m_limbCOPOffset[idx].data.y, m_limbCOPOffset[idx].data.z);
            gg->get_swing_support_foot_zmp_offsets_from_ee_name(tmpzmpoff, it->first);
            m_limbCOPOffset[idx].data.x = tmpzmpoff(0);
            m_limbCOPOffset[idx].data.y = tmpzmpoff(1);
            m_limbCOPOffset[idx].data.z = tmpzmpoff(2);
            // Set toe heel ratio which can be used force moment distribution
            double tmp = m_toeheelRatio.data[idx];
            gg->get_current_toe_heel_ratio_from_ee_name(tmp, it->first);
            m_toeheelRatio.data[idx] = tmp;
        } else { // Not included in leg_names
            // Set EE coords
            it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
            it->second.target_r0 = it->second.target_link->R * it->second.localR;
            // contactStates is OFF other than leg_names
            m_contactStates.data[idx] = false;
            // controlSwingSupportTime is not used while double support period, 1.0 is neglected
            m_controlSwingSupportTime.data[idx] = 1.0;
            // Set limbCOPOffset
            m_limbCOPOffset[idx].data.x = default_zmp_offsets[idx](0);
            m_limbCOPOffset[idx].data.y = default_zmp_offsets[idx](1);
            m_limbCOPOffset[idx].data.z = default_zmp_offsets[idx](2);
            // Set toe heel ratio which can be used force moment distribution
            m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
        }
    }
};

void AutoBalancer::getOutputParametersForABC ()
{
    // double support by default
    for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
        size_t idx = contact_states_index_map[it->first];
        // Set EE coords.
        //   If not included in leg_names, calculate EE coords because of not being used in GaitGenerator.
        //   Otherwise, keep previous EE coords derived from GaitGenerator or initial value.
        if ( std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end() ) {
            it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
            it->second.target_r0 = it->second.target_link->R * it->second.localR;
        }
        // Set contactStates
        std::vector<std::string>::const_iterator dst = std::find_if(leg_names.begin(), leg_names.end(), (boost::lambda::_1 == it->first));
        if (dst != leg_names.end()) {
            m_contactStates.data[idx] = true;
        } else {
            m_contactStates.data[idx] = false;
        }
        // controlSwingSupportTime is not used while double support period, 1.0 is neglected
        m_controlSwingSupportTime.data[idx] = 1.0;
        // Set limbCOPOffset
        m_limbCOPOffset[idx].data.x = default_zmp_offsets[idx](0);
        m_limbCOPOffset[idx].data.y = default_zmp_offsets[idx](1);
        m_limbCOPOffset[idx].data.z = default_zmp_offsets[idx](2);
        // Set toe heel ratio is not used while double support
        m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
    }
};

void AutoBalancer::getOutputParametersForIDLE ()
{
    // Set contactStates and controlSwingSupportTime
    if (m_optionalData.data.length() >= contact_states_index_map.size()*2) {
        // current optionalData is contactstates x limb and controlSwingSupportTime x limb
        //   If contactStates in optionalData is 1.0, m_contactStates is true. Otherwise, false.
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            m_contactStates.data[contact_states_index_map[it->first]] = isOptionalDataContact(it->first);
            m_controlSwingSupportTime.data[contact_states_index_map[it->first]] = m_optionalData.data[contact_states_index_map[it->first]+contact_states_index_map.size()];
        }
        // If two feet have no contact, force set double support contact
        if ( !m_contactStates.data[contact_states_index_map["rleg"]] && !m_contactStates.data[contact_states_index_map["lleg"]] ) {
            m_contactStates.data[contact_states_index_map["rleg"]] = true;
            m_contactStates.data[contact_states_index_map["lleg"]] = true;
        }
    }
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        size_t idx = contact_states_index_map[it->first];
        // Set limbCOPOffset
        m_limbCOPOffset[idx].data.x = 0;
        m_limbCOPOffset[idx].data.y = 0;
        m_limbCOPOffset[idx].data.z = 0;
        // Set toe heel ratio is not used while double support
        m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
    }
};

void AutoBalancer::interpolateLegNamesAndZMPOffsets()
{
    if (!zmp_offset_interpolator->isEmpty()) {
      double *default_zmp_offsets_output = new double[ikp.size()*3];
      zmp_offset_interpolator->get(default_zmp_offsets_output, true);
      for (size_t i = 0; i < ikp.size(); i++)
        for (size_t j = 0; j < 3; j++)
          default_zmp_offsets[i](j) = default_zmp_offsets_output[i*3+j];
      delete[] default_zmp_offsets_output;
      if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] default_zmp_offsets (interpolated)" << std::endl;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            std::cerr << "[" << m_profile.instance_name << "]   " << it->first << " = " << default_zmp_offsets[contact_states_index_map[it->first]].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        }
      }
    }
    if (!leg_names_interpolator->isEmpty()) {
        leg_names_interpolator->get(&leg_names_interpolator_ratio, true);
    }else {
        leg_names_interpolator_ratio = 1.0;
    }
};

void AutoBalancer::calcFixCoordsForAdjustFootstep (coordinates& tmp_fix_coords)
{
    double tmp = 0.0;
    adjust_footstep_interpolator->get(&tmp, true);
    //std::cerr << "[" << m_profile.instance_name << "] adjust ratio " << tmp << std::endl;
    ikp["rleg"].target_p0 = (1-tmp) * ikp["rleg"].adjust_interpolation_org_p0 + tmp*ikp["rleg"].adjust_interpolation_target_p0;
    ikp["lleg"].target_p0 = (1-tmp) * ikp["lleg"].adjust_interpolation_org_p0 + tmp*ikp["lleg"].adjust_interpolation_target_p0;
    rats::mid_rot(ikp["rleg"].target_r0, tmp, ikp["rleg"].adjust_interpolation_org_r0, ikp["rleg"].adjust_interpolation_target_r0);
    rats::mid_rot(ikp["lleg"].target_r0, tmp, ikp["lleg"].adjust_interpolation_org_r0, ikp["lleg"].adjust_interpolation_target_r0);
    coordinates tmprc, tmplc;
    tmprc.pos = ikp["rleg"].target_p0;
    tmprc.rot = ikp["rleg"].target_r0;
    tmplc.pos = ikp["lleg"].target_p0;
    tmplc.rot = ikp["lleg"].target_r0;
    rats::mid_coords(tmp_fix_coords, 0.5, tmprc, tmplc);
    //fix_leg_coords = tmp_fix_coords;
};

void AutoBalancer::rotateRefForcesForFixCoords (coordinates& tmp_fix_coords)
{
    /* update ref_forces ;; StateHolder's absolute -> AutoBalancer's absolute */
    for (size_t i = 0; i < m_ref_forceIn.size(); i++) {
      // hrp::Matrix33 eeR;
      // hrp::Link* parentlink;
      // hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
      // if (sensor) parentlink = sensor->link;
      // else parentlink = m_vfs[sensor_names[i]].link;
      // for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      //     if (it->second.target_link->name == parentlink->name) eeR = parentlink->R * it->second.localR;
      // }
      // End effector frame
      //ref_forces[i] = eeR * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
      // world frame
      ref_forces[i] = tmp_fix_coords.rot * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
      ref_moments[i] = tmp_fix_coords.rot * hrp::Vector3(m_ref_force[i].data[3], m_ref_force[i].data[4], m_ref_force[i].data[5]);
    }
    sbp_offset = tmp_fix_coords.rot * hrp::Vector3(sbp_offset);
};

void AutoBalancer::updateTargetCoordsForHandFixMode (coordinates& tmp_fix_coords)
{
    // Move hand for hand fix mode
    //   If arms' ABCIKparam.is_active is true, move hands according to cog velocity.
    //   If is_hand_fix_mode is false, no hand fix mode and move hands according to cog velocity.
    //   If is_hand_fix_mode is true, hand fix mode and do not move hands in Y axis in tmp_fix_coords.rot.
    if (gg_is_walking) {
        // hand control while walking = solve hand ik with is_hand_fix_mode and solve hand ik without is_hand_fix_mode
        bool is_hand_control_while_walking = false;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if ( it->second.is_active && std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()
                 && it->first.find("arm") != std::string::npos ) {
                is_hand_control_while_walking = true;
            }
        }
        if (is_hand_control_while_walking) {
        //if (false) { // Disabled temporarily
            // Store hand_fix_initial_offset in the initialization of walking
            if (is_hand_fix_initial) hand_fix_initial_offset = tmp_fix_coords.rot.transpose() * (hrp::Vector3(gg->get_cog()(0), gg->get_cog()(1), tmp_fix_coords.pos(2)) - tmp_fix_coords.pos);
            is_hand_fix_initial = false;
            hrp::Vector3 dif_p = hrp::Vector3(gg->get_cog()(0), gg->get_cog()(1), tmp_fix_coords.pos(2)) - tmp_fix_coords.pos - tmp_fix_coords.rot * hand_fix_initial_offset;
            if (is_hand_fix_mode) {
                dif_p = tmp_fix_coords.rot.transpose() * dif_p;
                dif_p(1) = 0;
                dif_p = tmp_fix_coords.rot * dif_p;
            }
            for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
                if ( it->second.is_active && std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()
                     && it->first.find("arm") != std::string::npos ) {
                    it->second.target_p0 = it->second.target_p0 + dif_p;
                    it->second.handfix_target_p0 = it->second.target_p0;
                }
            }
            prev_orig_dif_p = orig_dif_p;
            orig_dif_p = dif_p;
        }
    } else if (is_after_walking) {
      std::vector<double> start_pos(3), start_vel(3), goal_pos(3);
      double tmp_time = 5.0;
      for (size_t i = 0; i < 3; i++) {
        start_pos[i] = orig_dif_p(i);
        start_vel[i] = (orig_dif_p(i) - prev_orig_dif_p(i)) / m_dt;
        goal_pos[i] = 0.0;
      }
      hand_fix_interpolator->set(start_pos.data(), start_vel.data());
      hand_fix_interpolator->setGoal(goal_pos.data(), tmp_time, true);
    }
    if (!hand_fix_interpolator->isEmpty()) {
      std::vector<double> tmp_v(3);
      hand_fix_interpolator->get(tmp_v.data(), true);
      for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        if ( it->second.is_active && std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()
             && it->first.find("arm") != std::string::npos ) {
          for (size_t i = 0; i < 3; i++) {
            it->second.target_p0(i) = it->second.target_p0(i) + tmp_v[i];
          }
        }
      }
    }
};

void AutoBalancer::calculateOutputRefForces ()
{
    // TODO : need to be updated for multicontact and other walking
    if (leg_names.size() == 2) {
        std::vector<hrp::Vector3> ee_pos;
        for (size_t i = 0 ; i < leg_names.size(); i++) {
            ABCIKparam& tmpikp = ikp[leg_names[i]];
            ee_pos.push_back(tmpikp.target_p0 + tmpikp.target_r0 * default_zmp_offsets[i]);
        }
        double alpha = (ref_zmp - ee_pos[1]).norm() / ((ee_pos[0] - ref_zmp).norm() + (ee_pos[1] - ref_zmp).norm());
        if (alpha>1.0) alpha = 1.0;
        if (alpha<0.0) alpha = 0.0;
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "] alpha:" << alpha << std::endl;
        }
        double mg = m_robot->totalMass() * gg->get_gravitational_acceleration();
        m_force[0].data[2] = alpha * mg;
        m_force[1].data[2] = (1-alpha) * mg;
    }
    if ( use_force == MODE_REF_FORCE_WITH_FOOT || use_force == MODE_REF_FORCE_RFU_EXT_MOMENT ) { // TODO : use other use_force mode. This should be depends on Stabilizer distribution mode.
        distributeReferenceZMPToWrenches (ref_zmp);
    }
    prev_ref_zmp = ref_zmp;
};

hrp::Vector3 AutoBalancer::calcFootMidPosUsingZMPWeightMap ()
{
    hrp::Vector3 tmp_foot_mid_pos(hrp::Vector3::Zero());
    std::map<leg_type, std::string> leg_type_map = gg->get_leg_type_map();
    std::map<leg_type, double> zmp_weight_map = gg->get_zmp_weight_map();
    double sum_of_weight = 0.0;
    for (size_t i = 0; i < leg_names.size(); i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        // for foot_mid_pos
        std::map<leg_type, std::string>::const_iterator dst = std::find_if(leg_type_map.begin(), leg_type_map.end(), (&boost::lambda::_1->* &std::map<leg_type, std::string>::value_type::second == leg_names[i]));
        tmp_foot_mid_pos += (tmpikp.target_p0 + tmpikp.target_r0 * default_zmp_offsets[i]) * zmp_weight_map[dst->first];
        sum_of_weight += zmp_weight_map[dst->first];
    }
    tmp_foot_mid_pos *= (1.0 / sum_of_weight);
    return tmp_foot_mid_pos;
};

void AutoBalancer::updateWalkingVelocityFromHandError (coordinates& tmp_fix_coords)
{
    // TODO : check frame and robot state for this calculation
    if ( gg_is_walking && gg->get_lcg_count() == gg->get_overwrite_check_timing()+2 ) {
        hrp::Vector3 vel_htc(calc_vel_from_hand_error(tmp_fix_coords));
        gg->set_offset_velocity_param(vel_htc(0), vel_htc(1) ,vel_htc(2));
    }//  else {
    //     if ( gg_is_walking && gg->get_lcg_count() == static_cast<size_t>(gg->get_default_step_time()/(2*m_dt))-1) {
    //         gg->set_offset_velocity_param(0,0,0);
    //     }
    // }
};

void AutoBalancer::calcReferenceJointAnglesForIK ()
{
    fik->overwrite_ref_ja_index_vec.clear();
    // Fix for toe joint
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        if (it->second.is_active && it->second.has_toe_joint && gg->get_use_toe_joint()) {
            int i = it->second.target_link->jointId;
            if (gg->get_swing_leg_names().front() == it->first) {
                fik->qrefv[i] = fik->qrefv[i] + -1 * gg->get_foot_dif_rot_angle();
            }
            fik->overwrite_ref_ja_index_vec.push_back(i);
        }
    }
};

hrp::Matrix33 AutoBalancer::OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2)
{
  hrp::Vector3 vv = axis1.cross(axis2);
  if (fabs(vv.norm()-0.0) < 1e-5) {
    return rot;
  } else {
    Eigen::AngleAxis<double> tmpr(std::asin(vv.norm()), vv.normalized());
    return tmpr.toRotationMatrix() * rot;
  }
}

void AutoBalancer::fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot)
{
  // get current foot mid pos + rot
  std::vector<coordinates> foot_coords;
  for (size_t i = 0; i < leg_names.size(); i++) {
      ABCIKparam& tmpikp = ikp[leg_names[i]];
      if (leg_names[i].find("leg") != std::string::npos) foot_coords.push_back(coordinates((tmpikp.target_link->p + tmpikp.target_link->R * tmpikp.localPos),
                                                                                           (tmpikp.target_link->R * tmpikp.localR)));
  }
  coordinates current_foot_mid_coords;
  multi_mid_coords(current_foot_mid_coords, foot_coords);
  hrp::Vector3 current_foot_mid_pos = current_foot_mid_coords.pos;
  hrp::Matrix33 current_foot_mid_rot = current_foot_mid_coords.rot;
  // fix root pos + rot to fix "coords" = "current_foot_mid_xx"
  hrp::Matrix33 tmpR (fix_rot * current_foot_mid_rot.transpose());
  m_robot->rootLink()->p = fix_pos + tmpR * (m_robot->rootLink()->p - current_foot_mid_pos);
  rats::rotm3times(m_robot->rootLink()->R, tmpR, m_robot->rootLink()->R);
  m_robot->calcForwardKinematics();
}

void AutoBalancer::fixLegToCoords2 (coordinates& tmp_fix_coords)
{
    // Tempolarily modify tmp_fix_coords
    // This will be removed after seq outputs adequate waistRPY discussed in https://github.com/fkanehiro/hrpsys-base/issues/272
    // Snap input tmp_fix_coords to XY plan projection.
    {
      hrp::Vector3 ex = hrp::Vector3::UnitX();
      hrp::Vector3 ez = hrp::Vector3::UnitZ();
      hrp::Vector3 xv1 (tmp_fix_coords.rot * ex);
      xv1(2) = 0.0;
      xv1.normalize();
      hrp::Vector3 yv1(ez.cross(xv1));
      tmp_fix_coords.rot(0,0) = xv1(0); tmp_fix_coords.rot(1,0) = xv1(1); tmp_fix_coords.rot(2,0) = xv1(2);
      tmp_fix_coords.rot(0,1) = yv1(0); tmp_fix_coords.rot(1,1) = yv1(1); tmp_fix_coords.rot(2,1) = yv1(2);
      tmp_fix_coords.rot(0,2) = ez(0); tmp_fix_coords.rot(1,2) = ez(1); tmp_fix_coords.rot(2,2) = ez(2);
    }
    fixLegToCoords(tmp_fix_coords.pos, tmp_fix_coords.rot);
}

// assume biped walking
void AutoBalancer::calcTouchoffRemainTime()
{
  bool is_last_double = (gg->get_footstep_index() == gg->get_step_num() - 1);
  for (size_t i = 0; i < 2; i++) {
    if (gg_is_walking) {
      touchoff_remain_time[i] = gg->get_remain_count() * m_dt + (m_contactStates.data[i == 0 ? 1 : 0] &&
                                                     ((gg->is_before_step_phase() && gg->get_cur_leg() != i && (!is_last_double || (is_last_double && gg->get_remain_count() == 1))) ||
                                                      ((!m_contactStates.data[i] || !gg->is_before_step_phase()) && gg->get_cur_leg() == i && !is_last_double)) ?
                                                     gg->get_default_step_time() + m_dt : 0);
      if (st->stikp.size() > i) st->stikp[i].touchoff_remain_time = touchoff_remain_time[i];
    } else {
      touchoff_remain_time[i] = 0.0;
    }
  }
}

void AutoBalancer::stopFootForEarlyTouchDown ()
{
  bool is_last_double = (gg->get_footstep_index() == gg->get_step_num() - 1);
  for (size_t i = 0; i < 2; i++) {
    if (gg_is_walking) {
      if (gg->get_footstep_index() > 0 && !is_last_double && st->act_contact_states[contact_states_index_map[leg_names[i]]]) {
        if (!is_foot_touch[i] && !gg->is_before_step_phase()) {
          double tmp_ratio = 1.0;
          touchdown_transition_interpolator[leg_names[i]]->clear();
          touchdown_transition_interpolator[leg_names[i]]->set(&tmp_ratio);
          ikp[leg_names[i]].target_p0 = touchdown_foot_pos[i];
          tmp_ratio = 0.0;
          touchdown_transition_interpolator[leg_names[i]]->setGoal(&tmp_ratio, touchoff_remain_time[i], true);
          is_foot_touch[i] = true;
        }
      }
    }
    if (!touchdown_transition_interpolator[leg_names[i]]->isEmpty()) {
      double tmp_ratio = 0.0;
      touchdown_transition_interpolator[leg_names[i]]->setGoal(&tmp_ratio, touchoff_remain_time[i], true);
      touchdown_transition_interpolator[leg_names[i]]->get(&tmp_ratio, true);
      ikp[leg_names[i]].target_p0 = touchdown_foot_pos[i] * tmp_ratio + ikp[leg_names[i]].target_p0 * (1.0 - tmp_ratio);
    } else {
      is_foot_touch[i] = false;
      touchdown_foot_pos[i] = ikp[leg_names[i]].target_p0;
    }
  }
}
// TODO: move to fik.h
void AutoBalancer::limbStretchAvoidanceControl () {
  std::vector<hrp::Vector3> tmp_p;
  std::vector<std::string> tmp_name;
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->first.find("leg") != std::string::npos) {
      tmp_p.push_back(it->second.target_p0 + it->second.target_r0 * it->second.localPos);
      tmp_name.push_back(it->first);
    }
  }
  double tmp_d_root_height = 0.0, prev_d_root_height = fik->d_root_height;
  if (fik->use_limb_stretch_avoidance) {
    for (size_t i = 0; i < tmp_p.size(); i++) {
      // Check whether inside limb length limitation
      hrp::Link* parent_link = m_robot->link(fik->ikp[tmp_name[i]].parent_name);
      hrp::Vector3 rel_target_p = tmp_p[i] - parent_link->p; // position from parent to target link (world frame)
      double limb_length_limitation = fik->ikp[tmp_name[i]].max_limb_length - fik->ikp[tmp_name[i]].limb_length_margin;
      double tmp = limb_length_limitation * limb_length_limitation - rel_target_p(0) * rel_target_p(0) - rel_target_p(1) * rel_target_p(1);
      if (rel_target_p.norm() > limb_length_limitation && tmp >= 0) {
        tmp_d_root_height = std::min(tmp_d_root_height, rel_target_p(2) + std::sqrt(tmp));
      }
    }
    // Change root link height depending on limb length
    fik->d_root_height = tmp_d_root_height == 0.0 ? (- 1/fik->limb_stretch_avoidance_time_const * fik->d_root_height * m_dt + fik->d_root_height) : tmp_d_root_height;
  } else {
    fik->d_root_height = - 1/fik->limb_stretch_avoidance_time_const * fik->d_root_height * m_dt + fik->d_root_height;
  }
  vlimit(fik->d_root_height, prev_d_root_height + fik->limb_stretch_avoidance_vlimit[0], prev_d_root_height + fik->limb_stretch_avoidance_vlimit[1]);
  target_root_p(2) += fik->d_root_height;
  ref_cog(2) += fik->d_root_height;
}
void AutoBalancer::solveFullbodyIK ()
{
  // set desired natural pose and pullback gain
  for(int i=0;i<m_robot->numJoints();i++) {
      fik->q_ref(i) = m_qRef.data[i];
      if (is_natural_walk && gg_is_walking) {
        double arm_off = (ikp["rleg"].target_r0.transpose() * (ikp["lleg"].target_p0 - ikp["rleg"].target_p0))(0) * (deg2rad(arm_swing_deg)/0.15);
        if (m_robot->joint(i)->name == "R_SHOULDER_P" || m_robot->joint(i)->name == "RARM_JOINT1") fik->q_ref(i) -= arm_off;
        if (m_robot->joint(i)->name == "L_SHOULDER_P" || m_robot->joint(i)->name == "LARM_JOINT1") fik->q_ref(i) += arm_off;
      }
  }
  fik->revertRobotStateToCurrentAll();
  {
    hrp::Vector3 tmpcog = m_robot->calcCM();
    if (gg_is_walking && use_act_states) {
      ref_cog.head(2) = (tmpcog + (ref_cog -  (st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cog))).head(2);
    }
  }
  // calc sbp_cog_offset
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, ref_zmp(2));
  sbp_cog_offset(2) = 0.0;

  if (is_stop_early_foot) stopFootForEarlyTouchDown();

  limbStretchAvoidanceControl();

    std::vector<IKConstraint> ik_tgt_list;
    {
        IKConstraint tmp;
        tmp.target_link_name = "WAIST";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = target_root_p;// will be ignored by selection_vec
        tmp.targetRpy = hrp::rpyFromRot(target_root_R);
        tmp.constraint_weight << 0,0,0,1e-5,1e-5,1e-5; // COMMON
        ik_tgt_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = ikp["rleg"].target_link->name;
        tmp.localPos = ikp["rleg"].localPos;
        tmp.localR = ikp["rleg"].localR;
        tmp.targetPos = ikp["rleg"].target_p0;
        tmp.targetRpy = hrp::rpyFromRot(ikp["rleg"].target_r0);
        tmp.constraint_weight << 1,1,1,1,1,1;
        ik_tgt_list.push_back(tmp);
    }{
        IKConstraint tmp;
        tmp.target_link_name = ikp["lleg"].target_link->name;
        tmp.localPos = ikp["lleg"].localPos;
        tmp.localR = ikp["lleg"].localR;
        tmp.targetPos = ikp["lleg"].target_p0;
        tmp.targetRpy = hrp::rpyFromRot(ikp["lleg"].target_r0);
        tmp.constraint_weight << 1,1,1,1,1,1;
        ik_tgt_list.push_back(tmp);
    }
    if (ikp.size() >= 4) {
      if (ikp["rarm"].is_active || ikp["larm"].is_active) {
        double tmp_const = (ikp["rarm"].is_active ? 1e-1 : 1e-8) * transition_interpolator_ratio;
        IKConstraint tmp;
        tmp.target_link_name = ikp["rarm"].target_link->name;
        tmp.localPos = ikp["rarm"].localPos;
        tmp.localR = ikp["rarm"].localR;
        tmp.targetPos = ikp["rarm"].target_p0;
        tmp.targetRpy = hrp::rpyFromRot(ikp["rarm"].target_r0);
        tmp.constraint_weight << tmp_const,tmp_const,tmp_const,tmp_const,tmp_const,tmp_const;
        ik_tgt_list.push_back(tmp);
      }
      if (ikp["rarm"].is_active || ikp["larm"].is_active) {
        double tmp_const = (ikp["larm"].is_active ? 1e-1 : 1e-8) * transition_interpolator_ratio;
        IKConstraint tmp;
        tmp.target_link_name = ikp["larm"].target_link->name;
        tmp.localPos = ikp["larm"].localPos;
        tmp.localR = ikp["larm"].localR;
        tmp.targetPos = ikp["larm"].target_p0;
        tmp.targetRpy = hrp::rpyFromRot(ikp["larm"].target_r0);
        tmp.constraint_weight << tmp_const,tmp_const,tmp_const,tmp_const,tmp_const,tmp_const;
        ik_tgt_list.push_back(tmp);
      }
    }
    // collision aoidance
    if (use_collision_avoidance) {
      for (size_t i = 0; i < 2; i++) {
        if (fik->arm_leg_collision[i].is_collision) {
          IKConstraint tmp;
          double tmp_weight = fik->arm_leg_collision[i].calcWeight(1e-8);
          tmp.target_link_name = m_robot->joint(fik->arm_leg_collision[i].target_id)->name;
          tmp.localPos = fik->arm_leg_collision[i].target_localp;
          tmp.targetPos = m_robot->joint(fik->arm_leg_collision[i].base_id)->p + m_robot->joint(fik->arm_leg_collision[i].base_id)->R * fik->arm_leg_collision[i].base_localp
            + fik->arm_leg_collision[i].dir * (fik->arm_leg_collision[i].safe_d0 + fik->arm_leg_collision[i].safe_d_offset);
          tmp.constraint_weight << tmp_weight,tmp_weight,tmp_weight,0,0,0;
          ik_tgt_list.push_back(tmp);
        }
      }
    }
    {
        IKConstraint tmp;
        tmp.target_link_name = "COM";
        tmp.localPos = hrp::Vector3::Zero();
        tmp.localR = hrp::Matrix33::Identity();
        tmp.targetPos = ref_cog - sbp_cog_offset;// COM height will not be constraint
        hrp::Vector3 tmp_tau = gg->get_flywheel_tau();
        tmp_tau = st->vlimit(tmp_tau, -1000, 1000);
        tmp.targetRpy = hrp::Vector3::Zero();

        hrp::Vector3 prev_momentum = fik->cur_momentum_around_COM_filtered;
        m_tmp.data[25] = prev_momentum(0);
        m_tmp.data[26] = prev_momentum(1);
        if (gg->get_use_roll_flywheel()) tmp.targetRpy(0) = (prev_momentum + tmp_tau * m_dt)(0);//reference angular momentum
        if (gg->get_use_pitch_flywheel()) tmp.targetRpy(1) = (prev_momentum + tmp_tau * m_dt)(1);//reference angular momentum
        double roll_weight, pitch_weight, fly_weight = 1e-3, normal_weight = 1e-7, weight_fly_interpolator_time = 1.0, weight_normal_interpolator_time = 1.5;
        if (is_natural_walk) normal_weight = 0.0;
        if (ikp.size() >= 4 && (ikp["rarm"].is_active || ikp["larm"].is_active)) fly_weight = 1e-6;
        // roll
        if (gg->get_use_roll_flywheel()) {
          if (!prev_roll_state) {
            if (roll_weight_interpolator->isEmpty()) roll_weight = normal_weight;
            else roll_weight_interpolator->get(&roll_weight, true);
            roll_weight_interpolator->set(&roll_weight);
            roll_weight_interpolator->setGoal(&fly_weight, weight_fly_interpolator_time, true);
          }
          if (roll_weight_interpolator->isEmpty()) roll_weight = fly_weight;
          else roll_weight_interpolator->get(&roll_weight, true);
        } else {
          if (prev_roll_state) {
            if (roll_weight_interpolator->isEmpty()) roll_weight = fly_weight;
            else roll_weight_interpolator->get(&roll_weight, true);
            roll_weight_interpolator->set(&roll_weight);
            roll_weight_interpolator->setGoal(&normal_weight, weight_normal_interpolator_time, true);
          }
          if (roll_weight_interpolator->isEmpty()) roll_weight = normal_weight;
          else roll_weight_interpolator->get(&roll_weight, true);
        }
        // pitch
        if (gg->get_use_pitch_flywheel()) {
          if (!prev_pitch_state) {
            if (pitch_weight_interpolator->isEmpty()) pitch_weight = normal_weight;
            else pitch_weight_interpolator->get(&pitch_weight, true);
            pitch_weight_interpolator->set(&pitch_weight);
            pitch_weight_interpolator->setGoal(&fly_weight, weight_fly_interpolator_time, true);
          }
          if (pitch_weight_interpolator->isEmpty()) pitch_weight = fly_weight;
          else pitch_weight_interpolator->get(&pitch_weight, true);
        } else {
          if (prev_pitch_state) {
            if (pitch_weight_interpolator->isEmpty()) pitch_weight = fly_weight;
            else pitch_weight_interpolator->get(&pitch_weight, true);
            pitch_weight_interpolator->set(&pitch_weight);
            pitch_weight_interpolator->setGoal(&normal_weight, weight_normal_interpolator_time, true);
          }
          if (pitch_weight_interpolator->isEmpty()) pitch_weight = normal_weight;
          else pitch_weight_interpolator->get(&pitch_weight, true);
        }
        if (!cog_constraint_interpolator->isEmpty()) {
          cog_constraint_interpolator->get(&cog_z_constraint, true);
        }
        tmp.constraint_weight << 1,1,cog_z_constraint,roll_weight*transition_interpolator_ratio,pitch_weight*transition_interpolator_ratio,0; // consider angular momentum (COMMON)

        // 上半身関節角のq_refへの緩い拘束
        double upper_weight, fly_ratio = 0.0, normal_ratio = 2e-6;
        if (is_natural_walk) normal_ratio = 1e-5;
        if (ikp.size() >= 4 && (ikp["rarm"].is_active || ikp["larm"].is_active)) normal_ratio = 0.0;
        if (gg->get_use_roll_flywheel() || gg->get_use_pitch_flywheel()) {
          if (!prev_roll_state && !prev_pitch_state) {
            if (angular_momentum_interpolator->isEmpty()) upper_weight = normal_ratio;
            else angular_momentum_interpolator->get(&upper_weight, true);
            angular_momentum_interpolator->set(&upper_weight);
            angular_momentum_interpolator->setGoal(&fly_ratio, weight_fly_interpolator_time, true);
          }
          if (angular_momentum_interpolator->isEmpty()) upper_weight = fly_ratio;
          else angular_momentum_interpolator->get(&upper_weight, true);
        } else {
          if (prev_roll_state || prev_pitch_state) {
            if (angular_momentum_interpolator->isEmpty()) upper_weight = fly_ratio;
            else angular_momentum_interpolator->get(&upper_weight, true);
            angular_momentum_interpolator->set(&upper_weight);
            angular_momentum_interpolator->setGoal(&normal_ratio, weight_normal_interpolator_time, true);
          }
          if (angular_momentum_interpolator->isEmpty()) upper_weight = normal_ratio;
          else angular_momentum_interpolator->get(&upper_weight, true);
        }
        fik->q_ref_constraint_weight.fill(upper_weight);

        prev_roll_state = gg->get_use_roll_flywheel();
        prev_pitch_state = gg->get_use_pitch_flywheel();
        fik->q_ref_constraint_weight.segment(fik->ikp["rleg"].group_indices.back(), fik->ikp["rleg"].group_indices.size()).fill(0);
        fik->q_ref_constraint_weight.segment(fik->ikp["lleg"].group_indices.back(), fik->ikp["lleg"].group_indices.size()).fill(0);
        if(m_robot->link("RARM_F_JOINT0") != NULL) fik->q_ref_constraint_weight[m_robot->link("RARM_F_JOINT0")->jointId] = 1e-3;
        if(m_robot->link("RARM_F_JOINT1") != NULL) fik->q_ref_constraint_weight[m_robot->link("RARM_F_JOINT1")->jointId] = 1e-3;
        if(m_robot->link("LARM_F_JOINT0") != NULL) fik->q_ref_constraint_weight[m_robot->link("LARM_F_JOINT0")->jointId] = 1e-3;
        if(m_robot->link("LARM_F_JOINT1") != NULL) fik->q_ref_constraint_weight[m_robot->link("LARM_F_JOINT1")->jointId] = 1e-3;
        if(m_robot->link("HEAD_JOINT0") != NULL) fik->q_ref_constraint_weight[m_robot->link("HEAD_JOINT0")->jointId] = 1e-3;
        if(m_robot->link("HEAD_JOINT1") != NULL) fik->q_ref_constraint_weight[m_robot->link("HEAD_JOINT1")->jointId] = 1e-3;

//        tmp.rot_precision = 1e-1;//angular momentum precision
        ik_tgt_list.push_back(tmp);
    }
    // knee stretch protection
    if(m_robot->link("RLEG_JOINT3") != NULL) m_robot->link("RLEG_JOINT3")->llimit = deg2rad(5);
    if(m_robot->link("LLEG_JOINT3") != NULL) m_robot->link("LLEG_JOINT3")->llimit = deg2rad(5);
    if(m_robot->link("R_KNEE_P") != NULL) m_robot->link("R_KNEE_P")->llimit = deg2rad(5);
    if(m_robot->link("L_KNEE_P") != NULL) m_robot->link("L_KNEE_P")->llimit = deg2rad(5);
    // reduce chest joint move
    if(m_robot->link("HEAD_JOINT0") != NULL) fik->dq_weight_all(m_robot->link("HEAD_JOINT0")->jointId) = 10;
    if(m_robot->link("HEAD_JOINT1") != NULL) fik->dq_weight_all(m_robot->link("HEAD_JOINT1")->jointId) = 10;
    // reduce chest joint move
    if(m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT0")->jointId) = 10;
    if(m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT1")->jointId) = 10;
    if(m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT2")->jointId) = 10;
    if(m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(m_robot->link("CHEST_Y")->jointId) = 10;
    if(m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(m_robot->link("CHEST_P")->jointId) = 10;
    // disable toe joint move
    if(m_robot->link("RLEG_JOINT6") != NULL) fik->dq_weight_all(m_robot->link("RLEG_JOINT6")->jointId) = 1e6;
    if(m_robot->link("LLEG_JOINT6") != NULL) fik->dq_weight_all(m_robot->link("LLEG_JOINT6")->jointId) = 1e6;
    // fik->dq_weight_all.tail(3).fill(1e1);//ベースリンク回転変位の重みは1e1以下は暴れる？
    fik->rootlink_rpy_llimit << deg2rad(-15), deg2rad(-30), -DBL_MAX;
    fik->rootlink_rpy_ulimit << deg2rad(15), deg2rad(45), DBL_MAX;

  int loop_result = 0;
  const int IK_MAX_LOOP = 1;
  loop_result = fik->solveFullbodyIKLoop(m_robot, ik_tgt_list, IK_MAX_LOOP);
}

void AutoBalancer::solveSimpleFullbodyIK ()
{
  // Set ik target params
  fik->target_root_p = target_root_p;
  fik->target_root_R = target_root_R;
  for ( std::map<std::string, FullbodyInverseKinematicsSolver::IKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
      it->second.target_p0 = ikp[it->first].target_p0;
      it->second.target_r0 = ikp[it->first].target_r0;
  }
  fik->ratio_for_vel = transition_interpolator_ratio * leg_names_interpolator_ratio;
//  fik->current_tm = m_qRef.tm;
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      fik->ikp[it->first].is_ik_enable = it->second.is_active;
  }
  // Revert
  fik->revertRobotStateToCurrent();
  // TODO : SBP calculation is outside of solve ik?
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, ref_zmp(2));
  hrp::Vector3 dif_cog = tmp_input_sbp - ref_cog;

  // Solve IK
  fik->solveFullbodyIK (dif_cog, transition_interpolator->isEmpty());
  for (size_t i = 0; i < 2; i++) { // ik loop
    dif_cog = m_robot->calcCM() - ref_cog - dif_ref_act_cog;
    fik->solveSimpleFullbodyIKLoop(dif_cog, transition_interpolator->isEmpty());
  }
}

void AutoBalancer::limit_cog (hrp::Vector3& cog)
{
  if (transition_interpolator->isEmpty() && !gg_is_walking && !gg_is_wheeling && is_after_walking) {
    std::vector<double> start_pos(3), start_vel(3), goal_pos(3);
    double tmp_time = 5.0;
    for (size_t i = 0; i < 3; i++) {
      start_pos[i] = orig_cog(i);
      start_vel[i] = (orig_cog(i) - prev_orig_cog(i)) / m_dt;
      goal_pos[i] = cog(i);
    }
    limit_cog_interpolator->set(start_pos.data(), start_vel.data());
    limit_cog_interpolator->setGoal(goal_pos.data(), tmp_time, true);
    is_after_walking = false;
  }
  if (!limit_cog_interpolator->isEmpty()) {
    std::vector<double> tmp_v(3);
    double tmp_time = limit_cog_interpolator->get_remain_time();
    for (size_t i = 0; i < 3; i++) {
      tmp_v[i] = cog(i);
    }
    limit_cog_interpolator->setGoal(tmp_v.data(), tmp_time, true);
    limit_cog_interpolator->get(tmp_v.data(), true);
    for (size_t i = 0; i < 3; i++) {
      cog(i) = tmp_v[i];
    }
  }
}

bool AutoBalancer::vlimit(double& ret, const double llimit_value, const double ulimit_value)
{
  if (ret > ulimit_value) {
    ret = ulimit_value;
    return false;
  } else if (ret < llimit_value) {
    ret = llimit_value;
    return false;
  }
  return true;
};


/*
  RTC::ReturnCode_t AutoBalancer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void AutoBalancer::startABCparam(const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  std::cerr << "[" << m_profile.instance_name << "] start auto balancer mode" << std::endl;
  Guard guard(m_mutex);
  double tmp_ratio = 0.0;
  transition_interpolator->clear();
  transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 1.0;
  transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
  prev_ref_zmp = ref_zmp;
  prev_roll_state = false;
  prev_pitch_state = false;
  prev_orig_dif_p = hrp::Vector3::Zero();
  gg->set_is_emergency_step(false);
  d_wheel_angle = 0.0;
  start_d_wheel_angle = 0.0;
  angular_momentum_interpolator->clear();
  roll_weight_interpolator->clear();
  pitch_weight_interpolator->clear();
  limit_cog_interpolator->clear();
  fik->resetCollision();
  hand_fix_interpolator->clear();
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    it->second.is_active = false;
  }
  for (size_t i = 0; i < 2; i++) {
    touchdown_transition_interpolator[leg_names[i]]->clear();
  }

  for (size_t i = 0; i < limbs.length(); i++) {
    ABCIKparam& tmp = ikp[std::string(limbs[i])];
    tmp.is_active = true;
    std::cerr << "[" << m_profile.instance_name << "]   limb [" << std::string(limbs[i]) << "]" << std::endl;
  }

  control_mode = MODE_SYNC_TO_ABC;
}

void AutoBalancer::stopABCparam()
{
  std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode" << std::endl;
  //Guard guard(m_mutex);
  double tmp_ratio = 1.0;
  transition_interpolator->clear();
  transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 0.0;
  transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
  control_mode = MODE_SYNC_TO_IDLE;
}

void AutoBalancer::stopABCparamEmergency()
{
  std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode for emergency" << std::endl;
  double tmp_ratio = 1.0, tmp_time = (gg->is_emergency_touch_wall && is_emergency_touch_wall_mode) ? 0.8 : 0.8;
  emergency_transition_interpolator->clear();
  emergency_transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 0.0;
  emergency_transition_interpolator->setGoal(&tmp_ratio, tmp_time, true);
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    diff_q[i] = m_robot->joint(i)->q - m_qRef.data[i];
  }
  control_mode = MODE_IDLE;
  gg->clear_footstep_nodes_list();
  for ( std::map<std::string, interpolator*>::iterator it = touchdown_transition_interpolator.begin(); it != touchdown_transition_interpolator.end(); it++ ) {
    it->second->clear();
  }
  gg_is_walking = false;
  gg_is_wheeling = false;
}

bool AutoBalancer::startWalking ()
{
  if ( control_mode != MODE_ABC ) {
    std::cerr << "[" << m_profile.instance_name << "] Cannot start walking without MODE_ABC. Please startAutoBalancer." << std::endl;
    return false;
  }
  if ( gg->use_act_states && st->control_mode != Stabilizer::MODE_ST ) {
    std::cerr << "[" << m_profile.instance_name << "] Cannot start walking without MODE_ST. Please startStabilizer." << std::endl;
    return false;
  }
  hrp::Vector3 act_cog = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cog;
  act_cog.head(2) += sbp_cog_offset.head(2);
  hrp::Vector3 act_cogvel = st->ref_foot_origin_rot * st->act_cogvel;
  hrp::Vector3 act_cmp = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cmp;
  {
    Guard guard(m_mutex);
    fik->resetIKFailParam();
    std::vector<std::string> init_swing_leg_names(gg->get_footstep_front_leg_names());
    std::vector<std::string> tmp_all_limbs(leg_names);
    std::vector<std::string> init_support_leg_names;
    std::sort(tmp_all_limbs.begin(), tmp_all_limbs.end());
    std::sort(init_swing_leg_names.begin(), init_swing_leg_names.end());
    std::set_difference(tmp_all_limbs.begin(), tmp_all_limbs.end(),
                        init_swing_leg_names.begin(), init_swing_leg_names.end(),
                        std::back_inserter(init_support_leg_names));
    std::vector<step_node> init_support_leg_steps, init_swing_leg_dst_steps;
    for (std::vector<std::string>::iterator it = init_support_leg_names.begin(); it != init_support_leg_names.end(); it++)
        init_support_leg_steps.push_back(step_node(*it, coordinates(ikp[*it].target_p0, ikp[*it].target_r0), 0, 0, 0, 0));
    for (std::vector<std::string>::iterator it = init_swing_leg_names.begin(); it != init_swing_leg_names.end(); it++)
        init_swing_leg_dst_steps.push_back(step_node(*it, coordinates(ikp[*it].target_p0, ikp[*it].target_r0), 0, 0, 0, 0));
    gg->set_default_zmp_offsets(default_zmp_offsets);
    gg->initialize_gait_parameter(act_cog, ref_cog, init_support_leg_steps, init_swing_leg_dst_steps);
  }
  is_hand_fix_initial = true;
  while ( !gg->proc_one_tick(act_cog, act_cogvel, act_cmp) );
  {
    Guard guard(m_mutex);
    gg_is_walking = gg_solved = true;
    is_after_walking = true;
    limit_cog_interpolator->clear();
  }
  return true;
}

void AutoBalancer::stopWalking ()
{
  std::vector<coordinates> tmp_end_coords_list;
  for (std::vector<string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) {
      if ((*it).find("leg") != std::string::npos) tmp_end_coords_list.push_back(coordinates(ikp[*it].target_p0, ikp[*it].target_r0));
  }
  multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
  fixLegToCoords(fix_leg_coords.pos, fix_leg_coords.rot);
  gg->clear_footstep_nodes_list();
  gg_is_walking = false;
}

void AutoBalancer::stopWheeling ()
{
  std::vector<coordinates> tmp_end_coords_list;
  for (std::vector<string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) {
      if ((*it).find("leg") != std::string::npos) tmp_end_coords_list.push_back(coordinates(ikp[*it].target_p0, ikp[*it].target_r0));
  }
  multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
  fixLegToCoords(fix_leg_coords.pos, fix_leg_coords.rot);
  gg_is_wheeling = false;
}

bool AutoBalancer::startAutoBalancer (const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  if (control_mode == MODE_IDLE) {
    if (gg_is_walking) {
      // std::cerr << "[" << m_profile.instance_name << "] goStop before startAutoBalancer because gg_is_walking is true" << std::endl;
      // goStop();
      std::cerr << "[" << m_profile.instance_name << "] Cannot startAutobalancer while gg_is_waking is true. Please goStop or wait for ending previous goPos." << std::endl;
      return false;
    }
    fik->resetIKFailParam();
    startABCparam(limbs);
    waitABCTransition();
    return true;
  } else {
    return false;
  }
}

bool AutoBalancer::stopAutoBalancer ()
{
  if (control_mode == MODE_ABC) {
    stopABCparam();
    waitABCTransition();
    return true;
  } else {
    return false;
  }
}

void AutoBalancer::startStabilizer(void)
{
  st->startStabilizer();
}

void AutoBalancer::stopStabilizer(void)
{
  st->stopStabilizer();
}

void AutoBalancer::waitABCTransition()
{
  while (!transition_interpolator->isEmpty()) usleep(1000);
  usleep(1000);
}
bool AutoBalancer::goPos(const double& x, const double& y, const double& th)
{
    //  if ( !gg_is_walking && !is_stop_mode) {
  if ( !is_stop_mode) {
    gg->set_all_limbs(leg_names);
    coordinates start_ref_coords;
    std::vector<coordinates> initial_support_legs_coords;
    std::vector<leg_type> initial_support_legs;
    bool is_valid_gait_type = calc_inital_support_legs(y, initial_support_legs_coords, initial_support_legs, start_ref_coords);
    if (is_valid_gait_type == false) return false;
    gg->set_vel_foot_offset(start_ref_coords.rot.transpose() * (ikp["rleg"].target_p0 - start_ref_coords.pos), RLEG);
    gg->set_vel_foot_offset(start_ref_coords.rot.transpose() * (ikp["lleg"].target_p0 - start_ref_coords.pos), LLEG);
    bool ret = gg->go_pos_param_2_footstep_nodes_list(x, y, th,
                                                      initial_support_legs_coords, // Dummy if gg_is_walking
                                                      start_ref_coords,            // Dummy if gg_is_walking
                                                      initial_support_legs,        // Dummy if gg_is_walking
                                                      (!gg_is_walking)); // If gg_is_walking, initialize. Otherwise, not initialize and overwrite footsteps.
    if (!ret) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot goPos because of invalid timing." << std::endl;
    }
    if ( !gg_is_walking ) { // Initializing
        ret = startWalking();
    }
    return ret;
  } else {
    std::cerr << "[" << m_profile.instance_name << "] Cannot goPos while stopping mode." << std::endl;
    return false;
  }
}
bool AutoBalancer::goWheel(const double& x, const double& tm)
{
  // initialize wheel node
  gg->set_all_limbs(leg_names);
  coordinates start_ref_coords;
  std::vector<coordinates> initial_support_legs_coords; // dummy
  std::vector<leg_type> initial_support_legs; // dummy
  bool is_valid_gait_type = calc_inital_support_legs(0, initial_support_legs_coords, initial_support_legs, start_ref_coords);
  if (is_valid_gait_type == false) return false;
  // gg->set_vel_foot_offset(start_ref_coords.rot.transpose() * (ikp["rleg"].target_p0 - start_ref_coords.pos), RLEG);
  // gg->set_vel_foot_offset(start_ref_coords.rot.transpose() * (ikp["lleg"].target_p0 - start_ref_coords.pos), LLEG);
  bool ret = gg->go_wheel_param_2_wheel_nodes_list(x, tm, start_ref_coords);

  // initialize wheel generation
  hrp::Vector3 act_cog = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cog;
  act_cog.head(2) += sbp_cog_offset.head(2);
  gg->initialize_wheel_parameter(act_cog, ref_cog);

  gg_is_wheeling = true;

  return true;
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  if (!is_stop_mode) {
    gg->set_all_limbs(leg_names);
    bool ret = true;
    if (gg_is_walking && gg_solved) {
      gg->set_velocity_param(vx, vy, vth);
      // std::vector<double> tmp_v(3), target_v(3);
      // target_v[0]  = vx;
      // target_v[1]  = vy;
      // target_v[2]  = vth;
      // double transition_time = 5.0;
      // if (go_vel_interpolator->isEmpty()) {
      //   go_vel_interpolator->clear();
      //   go_vel_interpolator->setGoal(target_v.data(), transition_time, true);
      // } else {
      //   go_vel_interpolator->setGoal(target_v.data(), transition_time, true);
      // }
      // go_vel_interpolator->get(tmp_v.data(), true);
      // gg->set_velocity_param(tmp_v[0], tmp_v[1], tmp_v[2]);
    } else {
      coordinates ref_coords;
      ref_coords.pos = (ikp["rleg"].target_p0+ikp["lleg"].target_p0)*0.5;
      mid_rot(ref_coords.rot, 0.5, ikp["rleg"].target_r0, ikp["lleg"].target_r0);
      std::vector<leg_type> current_legs;
      switch(gait_type) {
      case BIPED:
        current_legs.assign (1, vy > 0 ? RLEG : LLEG);
        break;
      case TROT:
        current_legs = (vy > 0 ? boost::assign::list_of(RLEG)(LARM) : boost::assign::list_of(LLEG)(RARM)).convert_to_container < std::vector<leg_type> > ();
        break;
      case PACE:
        current_legs = (vy > 0 ? boost::assign::list_of(RLEG)(RARM) : boost::assign::list_of(LLEG)(LARM)).convert_to_container < std::vector<leg_type> > ();
        break;
      case CRAWL:
        std::cerr << "[" << m_profile.instance_name << "] crawl walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
      case GALLOP:
        /* at least one leg shoud be in contact */
        std::cerr << "[" << m_profile.instance_name << "] gallop walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
      default: break;
      }
      gg->set_vel_foot_offset(ref_coords.rot.transpose() * (ikp["rleg"].target_p0 - ref_coords.pos), RLEG);
      gg->set_vel_foot_offset(ref_coords.rot.transpose() * (ikp["lleg"].target_p0 - ref_coords.pos), LLEG);
      gg->initialize_velocity_mode(ref_coords, vx, vy, vth, current_legs);
      ret = startWalking();
    }
    return ret;
  } else {
    std::cerr << "[" << m_profile.instance_name << "] Cannot goVelocity while stopping mode." << std::endl;
    return false;
  }
}

bool AutoBalancer::goStop ()
{
  go_vel_interpolator->clear();
  gg->finalize_velocity_mode();
  waitFootSteps();
  return true;
}

bool AutoBalancer::emergencyStop ()
{
  std::cerr << "[" << m_profile.instance_name << "] emergencyStop" << std::endl;
  // is_stop_mode = true;
  gg->emergency_stop();
  waitFootSteps();
  return true;
}

bool AutoBalancer::releaseEmergencyStop ()
{
  if (is_stop_mode) {
      std::cerr << "[" << m_profile.instance_name << "] releaseEmergencyStop" << std::endl;
      is_stop_mode = false;
  }
  return true;
}

bool AutoBalancer::setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
{
  OpenHRP::AutoBalancerService::StepParamsSequence spss;
  spss.length(fss.length());
  // If gg_is_walking is false, initial footstep will be double support. So, set 0 for step_height and toe heel angles.
  // If gg_is_walking is true, do not set to 0.
  for (size_t i = 0; i < spss.length(); i++) {
      spss[i].sps.length(fss[i].fs.length());
      for (size_t j = 0; j < spss[i].sps.length(); j++) {
          spss[i].sps[j].step_height = ((!gg_is_walking && i==0) ? 0.0 : gg->get_default_step_height());
          spss[i].sps[j].step_time = gg->get_default_step_time();
          spss[i].sps[j].toe_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_toe_angle());
          spss[i].sps[j].heel_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_heel_angle());
      }
  }
  return setFootStepsWithParam(fss, spss, overwrite_fs_idx);
}

bool AutoBalancer::setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
{
    if (!is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] setFootStepsList" << std::endl;

        // Initial footstep Snapping
        coordinates tmpfs, fstrans;
        step_node initial_support_step, initial_input_step;
        {
            std::vector<step_node> initial_support_steps;
            if (gg_is_walking) {
                if (overwrite_fs_idx <= 0) {
                    std::cerr << "[" << m_profile.instance_name << "]   Invalid overwrite index = " << overwrite_fs_idx << std::endl;
                    return false;
                }
                if (!gg->get_footstep_nodes_by_index(initial_support_steps, overwrite_fs_idx-1)) {
                    std::cerr << "[" << m_profile.instance_name << "]   Invalid overwrite index = " << overwrite_fs_idx << std::endl;
                    return false;
                }
            } else {
                // If walking, snap initial leg to current ABC foot coords.
                for (size_t i = 0; i < fss[0].fs.length(); i++) {
                    ABCIKparam& tmpikp = ikp[std::string(fss[0].fs[i].leg)];
                    initial_support_steps.push_back(step_node(std::string(fss[0].fs[i].leg), coordinates(tmpikp.target_p0, tmpikp.target_r0), 0, 0, 0, 0));
                }
            }
            initial_support_step = initial_support_steps.front(); /* use only one leg for representation */
        }
        {
            std::map<leg_type, std::string> leg_type_map = gg->get_leg_type_map();
            for (size_t i = 0; i < fss[0].fs.length(); i++) {
                if (std::string(fss[0].fs[i].leg) == leg_type_map[initial_support_step.l_r]) {
                    coordinates tmp;
                    memcpy(tmp.pos.data(), fss[0].fs[i].pos, sizeof(double)*3);
                    tmp.rot = (Eigen::Quaternion<double>(fss[0].fs[i].rot[0], fss[0].fs[i].rot[1], fss[0].fs[i].rot[2], fss[0].fs[i].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
                    initial_input_step = step_node(std::string(fss[0].fs[i].leg), tmp, 0, 0, 0, 0);
                }
            }
        }

        // Get footsteps
        std::vector< std::vector<coordinates> > fs_vec_list;
        std::vector< std::vector<std::string> > leg_name_vec_list;
        for (size_t i = 0; i < fss.length(); i++) {
            std::vector<coordinates> fs_vec;
            std::vector<std::string> leg_name_vec;
            for (size_t j = 0; j < fss[i].fs.length(); j++) {
                std::string leg(fss[i].fs[j].leg);
                if (std::find(leg_names.begin(), leg_names.end(), leg) != leg_names.end()) {
                    memcpy(tmpfs.pos.data(), fss[i].fs[j].pos, sizeof(double)*3);
                    tmpfs.rot = (Eigen::Quaternion<double>(fss[i].fs[j].rot[0], fss[i].fs[j].rot[1], fss[i].fs[j].rot[2], fss[i].fs[j].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
                    initial_input_step.worldcoords.transformation(fstrans, tmpfs);
                    tmpfs = initial_support_step.worldcoords;
                    tmpfs.transform(fstrans);
                } else {
                    std::cerr << "[" << m_profile.instance_name << "]   No such target : " << leg << std::endl;
                    return false;
                }
                leg_name_vec.push_back(leg);
                fs_vec.push_back(tmpfs);
            }
            leg_name_vec_list.push_back(leg_name_vec);
            fs_vec_list.push_back(fs_vec);
        }
        if (spss.length() != fs_vec_list.size()) {
            std::cerr << "[" << m_profile.instance_name << "]   StepParam length " << spss.length () << " != Footstep length " << fs_vec_list.size() << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] print footsteps " << std::endl;
        std::vector< std::vector<step_node> > fnsl;
        for (size_t i = 0; i < fs_vec_list.size(); i++) {
            if (!(gg_is_walking && i == 0)) { // If initial footstep, e.g., not walking, pass user-defined footstep list. If walking, pass cdr footsteps in order to neglect initial double support leg.
                std::vector<step_node> tmp_fns;
                for (size_t j = 0; j < fs_vec_list.at(i).size(); j++) {
                    tmp_fns.push_back(step_node(leg_name_vec_list[i][j], fs_vec_list[i][j], spss[i].sps[j].step_height, spss[i].sps[j].step_time, spss[i].sps[j].toe_angle, spss[i].sps[j].heel_angle));
                }
                fnsl.push_back(tmp_fns);
            }
        }
        bool ret = true;
        if (gg_is_walking) {
            std::cerr << "[" << m_profile.instance_name << "]  Set overwrite footsteps" << std::endl;
            gg->set_overwrite_foot_steps_list(fnsl);
            gg->set_overwrite_foot_step_index(overwrite_fs_idx);
        } else {
            std::cerr << "[" << m_profile.instance_name << "]  Set normal footsteps" << std::endl;
            gg->set_foot_steps_list(fnsl);
            ret = startWalking();
        }
        return ret;
    } else {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setFootSteps while walking." << std::endl;
        return false;
    }
}

void AutoBalancer::waitFootSteps()
{
  //while (gg_is_walking) usleep(10);
  while (gg_is_walking || !transition_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  gg->set_offset_velocity_param(0,0,0);
}

void AutoBalancer::waitFootStepsEarly(const double tm)
{
  if (!gg_is_walking) { return;}
  while ( !gg->is_finalizing(tm)|| !transition_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalancer::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  std::cerr << "[" << m_profile.instance_name << "] setGaitGeneratorParam" << std::endl;
  if (i_param.stride_parameter.length() == 4) { // Support old stride_parameter definitions
      gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3],
                                i_param.stride_parameter[1]*0.5, i_param.stride_parameter[2]*0.5);
  } else {
      gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3],
                                i_param.stride_parameter[4], i_param.stride_parameter[5]);
  }
  std::vector<hrp::Vector3> off;
  for (size_t i = 0; i < i_param.leg_default_translate_pos.length(); i++) {
      off.push_back(hrp::Vector3(i_param.leg_default_translate_pos[i][0], i_param.leg_default_translate_pos[i][1], i_param.leg_default_translate_pos[i][2]));
  }
  gg->set_leg_default_translate_pos(off);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  gg->set_default_double_support_ratio_before(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_ratio_after(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_static_ratio_before(i_param.default_double_support_static_ratio/2.0);
  gg->set_default_double_support_static_ratio_after(i_param.default_double_support_static_ratio/2.0);
  gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio/2.0);
  // gg->set_default_double_support_ratio_before(i_param.default_double_support_ratio_before);
  // gg->set_default_double_support_ratio_after(i_param.default_double_support_ratio_after);
  // gg->set_default_double_support_static_ratio_before(i_param.default_double_support_static_ratio_before);
  // gg->set_default_double_support_static_ratio_after(i_param.default_double_support_static_ratio_after);
  // gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio_before);
  // gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio_after);
  // gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio_swing_before);
  // gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio_swing_after);
  if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::SHUFFLING) {
    gg->set_default_orbit_type(SHUFFLING);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOID) {
    gg->set_default_orbit_type(CYCLOID);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::RECTANGLE) {
    gg->set_default_orbit_type(RECTANGLE);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::STAIR) {
    gg->set_default_orbit_type(STAIR);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOIDDELAY) {
    gg->set_default_orbit_type(CYCLOIDDELAY);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOIDDELAYKICK) {
    gg->set_default_orbit_type(CYCLOIDDELAYKICK);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CROSS) {
    gg->set_default_orbit_type(CROSS);
  }
  gg->set_swing_trajectory_delay_time_offset(i_param.swing_trajectory_delay_time_offset);
  gg->set_swing_trajectory_final_distance_weight(i_param.swing_trajectory_final_distance_weight);
  gg->set_swing_trajectory_time_offset_xy2z(i_param.swing_trajectory_time_offset_xy2z);
  gg->set_stair_trajectory_way_point_offset(hrp::Vector3(i_param.stair_trajectory_way_point_offset[0], i_param.stair_trajectory_way_point_offset[1], i_param.stair_trajectory_way_point_offset[2]));
  gg->set_rectangle_trajectory_way_point_offset(hrp::Vector3(i_param.rectangle_trajectory_way_point_offset[0], i_param.rectangle_trajectory_way_point_offset[1], i_param.rectangle_trajectory_way_point_offset[2]));
  gg->set_cycloid_delay_kick_point_offset(hrp::Vector3(i_param.cycloid_delay_kick_point_offset[0], i_param.cycloid_delay_kick_point_offset[1], i_param.cycloid_delay_kick_point_offset[2]));
  gg->set_gravitational_acceleration(i_param.gravitational_acceleration);
  gg->set_toe_angle(i_param.toe_angle);
  gg->set_heel_angle(i_param.heel_angle);
  gg->set_toe_pos_offset_x(i_param.toe_pos_offset_x);
  gg->set_heel_pos_offset_x(i_param.heel_pos_offset_x);
  gg->set_toe_zmp_offset_x(i_param.toe_zmp_offset_x);
  gg->set_heel_zmp_offset_x(i_param.heel_zmp_offset_x);
  gg->set_toe_check_thre(i_param.toe_check_thre);
  gg->set_heel_check_thre(i_param.heel_check_thre);
  std::vector<double> tmp_ratio(i_param.toe_heel_phase_ratio.get_buffer(), i_param.toe_heel_phase_ratio.get_buffer()+i_param.toe_heel_phase_ratio.length());
  std::cerr << "[" << m_profile.instance_name << "]   "; // for set_toe_heel_phase_ratio
  gg->set_toe_heel_phase_ratio(tmp_ratio);
  gg->set_use_toe_joint(i_param.use_toe_joint);
  gg->set_use_toe_heel_transition(i_param.use_toe_heel_transition);
  gg->set_use_toe_heel_auto_set(i_param.use_toe_heel_auto_set);
  gg->set_zmp_weight_map(boost::assign::map_list_of<leg_type, double>(RLEG, i_param.zmp_weight_map[0])(LLEG, i_param.zmp_weight_map[1])(RARM, i_param.zmp_weight_map[2])(LARM, i_param.zmp_weight_map[3]));
  gg->set_optional_go_pos_finalize_footstep_num(i_param.optional_go_pos_finalize_footstep_num);
  gg->set_overwritable_footstep_index_offset(i_param.overwritable_footstep_index_offset);
  gg->set_leg_margin(i_param.leg_margin);
  gg->set_safe_leg_margin(i_param.safe_leg_margin);
  gg->set_vertices_from_leg_margin();
  gg->set_stride_limitation_for_circle_type(i_param.stride_limitation_for_circle_type);
  gg->set_overwritable_stride_limitation(i_param.overwritable_stride_limitation);
  gg->set_use_stride_limitation(i_param.use_stride_limitation);
  gg->set_footstep_modification_gain(i_param.footstep_modification_gain);
  gg->set_modify_footsteps(i_param.modify_footsteps);
  gg->set_min_time_mgn(i_param.min_time_mgn);
  gg->set_min_time(i_param.min_time);
  gg->set_cp_check_margin(i_param.cp_check_margin);
  gg->set_margin_time_ratio(i_param.margin_time_ratio);
  if (i_param.stride_limitation_type == OpenHRP::AutoBalancerService::SQUARE) {
    gg->set_stride_limitation_type(SQUARE);
  } else if (i_param.stride_limitation_type == OpenHRP::AutoBalancerService::CIRCLE) {
    gg->set_stride_limitation_type(CIRCLE);
  }
  gg->set_act_vel_ratio(i_param.act_vel_ratio);
  gg->set_use_disturbance_compensation(i_param.use_disturbance_compensation);
  gg->set_dc_gain(i_param.dc_gain);
  gg->set_dcm_offset(i_param.dcm_offset);
  gg->set_emergency_step_time(i_param.emergency_step_time);
  gg->use_act_states = st->use_act_states = i_param.use_act_states;
  gg->set_use_act_states();
  gg->is_interpolate_zmp_in_double = i_param.is_interpolate_zmp_in_double;
  gg->is_stable_go_stop_mode = i_param.is_stable_go_stop_mode;
  gg->use_flywheel_balance = i_param.use_flywheel_balance;
  gg->zmp_delay_time_const = i_param.zmp_delay_time_const;
  gg->overwritable_max_time = i_param.overwritable_max_time;
  gg->fg_zmp_cutoff_freq = i_param.fg_zmp_cutoff_freq;
  gg->cp_filter->setCutOffFreq(i_param.fg_cp_cutoff_freq);
  gg->set_sum_d_footstep_thre(hrp::Vector3(i_param.sum_d_footstep_thre[0], i_param.sum_d_footstep_thre[1], i_param.sum_d_footstep_thre[2]));
  gg->set_footstep_check_delta(hrp::Vector3(i_param.footstep_check_delta[0], i_param.footstep_check_delta[1], i_param.footstep_check_delta[2]));
  gg->debug_set_landing_height = i_param.debug_set_landing_height;
  gg->debug_landing_height = i_param.debug_landing_height;
  for (size_t i = 0; i < 2; i++) {
    gg->debug_landing_height_xrange[i] = i_param.debug_landing_height_xrange[i];
  }
  for (size_t i = 0; i < 2; i++) {
    gg->front_edge_offset_of_steppable_region[i] = i_param.front_edge_offset_of_steppable_region[i];
  }
  gg->is_slow_stair_mode = i_param.is_slow_stair_mode;
  gg->stair_step_time = i_param.stair_step_time;

  // print
  gg->print_param(std::string(m_profile.instance_name));
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3], i_param.stride_parameter[4], i_param.stride_parameter[5]);
  std::vector<hrp::Vector3> off;
  gg->get_leg_default_translate_pos(off);
  i_param.leg_default_translate_pos.length(off.size());
  for (size_t i = 0; i < i_param.leg_default_translate_pos.length(); i++) {
      i_param.leg_default_translate_pos[i].length(3);
      i_param.leg_default_translate_pos[i][0] = off[i](0);
      i_param.leg_default_translate_pos[i][1] = off[i](1);
      i_param.leg_default_translate_pos[i][2] = off[i](2);
  }
  i_param.default_step_time = gg->get_default_step_time();
  i_param.default_step_height = gg->get_default_step_height();
  i_param.default_double_support_ratio_before = gg->get_default_double_support_ratio_before();
  i_param.default_double_support_ratio_after = gg->get_default_double_support_ratio_after();
  i_param.default_double_support_static_ratio_before = gg->get_default_double_support_static_ratio_before();
  i_param.default_double_support_static_ratio_after = gg->get_default_double_support_static_ratio_after();
  i_param.default_double_support_ratio_swing_before = gg->get_default_double_support_ratio_swing_before();
  i_param.default_double_support_ratio_swing_after = gg->get_default_double_support_ratio_swing_after();
  i_param.default_double_support_ratio = i_param.default_double_support_ratio_before + i_param.default_double_support_ratio_after;
  i_param.default_double_support_static_ratio = i_param.default_double_support_static_ratio_before + i_param.default_double_support_static_ratio_after;
  if (gg->get_default_orbit_type() == SHUFFLING) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::SHUFFLING;
  } else if (gg->get_default_orbit_type() == CYCLOID) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOID;
  } else if (gg->get_default_orbit_type() == RECTANGLE) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::RECTANGLE;
  } else if (gg->get_default_orbit_type() == STAIR) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::STAIR;
  } else if (gg->get_default_orbit_type() == CYCLOIDDELAY) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOIDDELAY;
  } else if (gg->get_default_orbit_type() == CYCLOIDDELAYKICK) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOIDDELAYKICK;
  } else if (gg->get_default_orbit_type() == CROSS) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CROSS;
  }

  hrp::Vector3 tmpv = gg->get_stair_trajectory_way_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.stair_trajectory_way_point_offset[i] = tmpv(i);
  tmpv = gg->get_rectangle_trajectory_way_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.rectangle_trajectory_way_point_offset[i] = tmpv(i);
  tmpv = gg->get_cycloid_delay_kick_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.cycloid_delay_kick_point_offset[i] = tmpv(i);
  i_param.swing_trajectory_delay_time_offset = gg->get_swing_trajectory_delay_time_offset();
  i_param.swing_trajectory_final_distance_weight = gg->get_swing_trajectory_final_distance_weight();
  i_param.swing_trajectory_time_offset_xy2z = gg->get_swing_trajectory_time_offset_xy2z();
  i_param.gravitational_acceleration = gg->get_gravitational_acceleration();
  i_param.toe_angle = gg->get_toe_angle();
  i_param.heel_angle = gg->get_heel_angle();
  i_param.toe_pos_offset_x = gg->get_toe_pos_offset_x();
  i_param.heel_pos_offset_x = gg->get_heel_pos_offset_x();
  i_param.toe_zmp_offset_x = gg->get_toe_zmp_offset_x();
  i_param.heel_zmp_offset_x = gg->get_heel_zmp_offset_x();
  i_param.toe_check_thre = gg->get_toe_check_thre();
  i_param.heel_check_thre = gg->get_heel_check_thre();
  std::vector<double> ratio(gg->get_NUM_TH_PHASES(),0.0);
  gg->get_toe_heel_phase_ratio(ratio);
  for (int i = 0; i < gg->get_NUM_TH_PHASES(); i++) i_param.toe_heel_phase_ratio[i] = ratio[i];
  i_param.use_toe_joint = gg->get_use_toe_joint();
  i_param.use_toe_heel_transition = gg->get_use_toe_heel_transition();
  i_param.use_toe_heel_auto_set = gg->get_use_toe_heel_auto_set();
  std::map<leg_type, double> tmp_zmp_weight_map = gg->get_zmp_weight_map();
  i_param.zmp_weight_map[0] = tmp_zmp_weight_map[RLEG];
  i_param.zmp_weight_map[1] = tmp_zmp_weight_map[LLEG];
  i_param.zmp_weight_map[2] = tmp_zmp_weight_map[RARM];
  i_param.zmp_weight_map[3] = tmp_zmp_weight_map[LARM];
  i_param.optional_go_pos_finalize_footstep_num = gg->get_optional_go_pos_finalize_footstep_num();
  i_param.overwritable_footstep_index_offset = gg->get_overwritable_footstep_index_offset();
  for (size_t i=0; i<4; i++) {
    i_param.leg_margin[i] = gg->get_leg_margin(i);
  }
  for (size_t i=0; i<4; i++) {
    i_param.safe_leg_margin[i] = gg->get_safe_leg_margin(i);
  }
  for (size_t i=0; i<5; i++) {
    i_param.stride_limitation_for_circle_type[i] = gg->get_stride_limitation_for_circle_type(i);
  }
  for (size_t i=0; i<5; i++) {
    i_param.overwritable_stride_limitation[i] = gg->get_overwritable_stride_limitation(i);
  }
  i_param.use_stride_limitation = gg->get_use_stride_limitation();
  i_param.footstep_modification_gain = gg->get_footstep_modification_gain();
  i_param.modify_footsteps = gg->get_modify_footsteps();
  i_param.min_time_mgn = gg->get_min_time_mgn();
  i_param.min_time = gg->get_min_time();
  for (size_t i=0; i<2; i++) {
    i_param.cp_check_margin[i] = gg->get_cp_check_margin(i);
  }
  i_param.margin_time_ratio = gg->get_margin_time_ratio();
  if (gg->get_stride_limitation_type() == SQUARE) {
    i_param.stride_limitation_type = OpenHRP::AutoBalancerService::SQUARE;
  } else if (gg->get_stride_limitation_type() == CIRCLE) {
    i_param.stride_limitation_type = OpenHRP::AutoBalancerService::CIRCLE;
  }
  i_param.act_vel_ratio = gg->get_act_vel_ratio();
  i_param.use_disturbance_compensation = gg->get_use_disturbance_compensation();
  i_param.dc_gain = gg->get_dc_gain();
  i_param.dcm_offset = gg->get_dcm_offset();
  for (size_t i = 0; i < 3; i++) {
    i_param.emergency_step_time[i] = gg->get_emergency_step_time(i);
  }
  i_param.use_act_states = gg->use_act_states;
  i_param.is_interpolate_zmp_in_double = gg->is_interpolate_zmp_in_double;
  i_param.is_stable_go_stop_mode = gg->is_stable_go_stop_mode;
  i_param.use_flywheel_balance = gg->use_flywheel_balance;
  i_param.zmp_delay_time_const = gg->zmp_delay_time_const;
  i_param.overwritable_max_time = gg->overwritable_max_time;
  i_param.fg_zmp_cutoff_freq = gg->fg_zmp_cutoff_freq;
  i_param.fg_cp_cutoff_freq = gg->cp_filter->getCutOffFreq();
  {
    hrp::Vector3 thre, delta;
    gg->get_sum_d_footstep_thre(thre);
    gg->get_footstep_check_delta(delta);
    for (size_t i = 0; i < 3; i++) {
      i_param.sum_d_footstep_thre[i] = thre(i);
      i_param.footstep_check_delta[i] = delta(i);
    }
  }
  i_param.debug_set_landing_height = gg->debug_set_landing_height;
  i_param.debug_landing_height = gg->debug_landing_height;
  for (size_t i = 0; i < 2; i++) {
    i_param.debug_landing_height_xrange[i] = gg->debug_landing_height_xrange[i];
  }
  for (size_t i = 0; i < 2; i++) {
    i_param.front_edge_offset_of_steppable_region[i] = gg->front_edge_offset_of_steppable_region[i];
  }
  i_param.is_slow_stair_mode = gg->is_slow_stair_mode;
  i_param.stair_step_time = gg->stair_step_time;
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name << "] setAutoBalancerParam" << std::endl;
  double *default_zmp_offsets_array = new double[ikp.size()*3];
  for (size_t i = 0; i < ikp.size(); i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets_array[i*3+j] = i_param.default_zmp_offsets[i][j];
  zmp_transition_time = i_param.zmp_transition_time;
  adjust_footstep_transition_time = i_param.adjust_footstep_transition_time;
  if (zmp_offset_interpolator->isEmpty()) {
      zmp_offset_interpolator->clear();
      zmp_offset_interpolator->setGoal(default_zmp_offsets_array, zmp_transition_time, true);
  } else if (control_mode == MODE_IDLE) {
      zmp_offset_interpolator->setGoal(default_zmp_offsets_array, zmp_transition_time, true);
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets cannot be set because interpolating." << std::endl;
  }
  if (control_mode == MODE_IDLE) {
    switch (i_param.use_force_mode) {
    case OpenHRP::AutoBalancerService::MODE_NO_FORCE:
        use_force = MODE_NO_FORCE;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE:
        use_force = MODE_REF_FORCE;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT:
        use_force = MODE_REF_FORCE_WITH_FOOT;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT:
        use_force = MODE_REF_FORCE_RFU_EXT_MOMENT;
        break;
    default:
        break;
    }
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   use_force_mode cannot be changed to [" << i_param.use_force_mode << "] during MODE_ABC, MODE_SYNC_TO_IDLE or MODE_SYNC_TO_ABC." << std::endl;
  }
  graspless_manip_mode = i_param.graspless_manip_mode;
  graspless_manip_arm = std::string(i_param.graspless_manip_arm);
  for (size_t j = 0; j < 3; j++)
      graspless_manip_p_gain[j] = i_param.graspless_manip_p_gain[j];
  for (size_t j = 0; j < 3; j++)
      graspless_manip_reference_trans_coords.pos[j] = i_param.graspless_manip_reference_trans_pos[j];
  graspless_manip_reference_trans_coords.rot = (Eigen::Quaternion<double>(i_param.graspless_manip_reference_trans_rot[0],
                                                                          i_param.graspless_manip_reference_trans_rot[1],
                                                                          i_param.graspless_manip_reference_trans_rot[2],
                                                                          i_param.graspless_manip_reference_trans_rot[3]).normalized().toRotationMatrix()); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
  transition_time = i_param.transition_time;
  std::vector<std::string> cur_leg_names, dst_leg_names;
  cur_leg_names = leg_names;
  for (size_t i = 0; i < i_param.leg_names.length(); i++) {
      dst_leg_names.push_back(std::string(i_param.leg_names[i]));
  }
  std::sort(cur_leg_names.begin(), cur_leg_names.end());
  std::sort(dst_leg_names.begin(), dst_leg_names.end());
  if (cur_leg_names != dst_leg_names) {
      if (leg_names_interpolator->isEmpty()) {
          leg_names.clear();
          leg_names = dst_leg_names;
          if (control_mode == MODE_ABC) {
              double tmp_ratio = 0.0;
              leg_names_interpolator->set(&tmp_ratio);
              tmp_ratio = 1.0;
              leg_names_interpolator->setGoal(&tmp_ratio, 5.0, true);
              control_mode = MODE_SYNC_TO_ABC;
          }
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   leg_names cannot be set because interpolating." << std::endl;
  }
  if (!gg_is_walking) {
      is_hand_fix_mode = i_param.is_hand_fix_mode;
      std::cerr << "[" << m_profile.instance_name << "]   is_hand_fix_mode = " << is_hand_fix_mode << std::endl;
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   is_hand_fix_mode cannot be set in (gg_is_walking = true). Current is_hand_fix_mode is " << (is_hand_fix_mode?"true":"false") << std::endl;
  }
  if (control_mode == MODE_IDLE) {
      for (size_t i = 0; i < i_param.end_effector_list.length(); i++) {
          std::map<std::string, ABCIKparam>::iterator it = ikp.find(std::string(i_param.end_effector_list[i].leg));
          memcpy(it->second.localPos.data(), i_param.end_effector_list[i].pos, sizeof(double)*3);
          it->second.localR = (Eigen::Quaternion<double>(i_param.end_effector_list[i].rot[0], i_param.end_effector_list[i].rot[1], i_param.end_effector_list[i].rot[2], i_param.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
  }
  if (i_param.default_gait_type == OpenHRP::AutoBalancerService::BIPED) {
      gait_type = BIPED;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::TROT) {
      gait_type = TROT;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::PACE) {
      gait_type = PACE;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::CRAWL) {
      gait_type = CRAWL;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::GALLOP) {
      gait_type = GALLOP;
  }
  // Ref force balancing
  std::cerr << "[" << m_profile.instance_name << "] Ref force balancing" << std::endl;
  if ( use_force == MODE_REF_FORCE_WITH_FOOT && control_mode != MODE_IDLE ) {
      std::cerr << "[" << m_profile.instance_name << "]   additional_force_applied_point_offset and additional_force_applied_link_name cannot be updated during MODE_REF_FORCE_WITH_FOOT and non-MODE_IDLE"<< std::endl;
  } else if ( !m_robot->link(std::string(i_param.additional_force_applied_link_name)) ) {
      std::cerr << "[" << m_profile.instance_name << "]   Invalid link name for additional_force_applied_link_name = " << i_param.additional_force_applied_link_name << std::endl;
  } else {
      additional_force_applied_link = m_robot->link(std::string(i_param.additional_force_applied_link_name));
      for (size_t i = 0; i < 3; i++) {
          additional_force_applied_point_offset(i) = i_param.additional_force_applied_point_offset[i];
      }
      std::cerr << "[" << m_profile.instance_name << "]   Link name for additional_force_applied_link_name = " << additional_force_applied_link->name << ", additional_force_applied_point_offset = " << additional_force_applied_point_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
  }

  for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << it->first << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localpos = " << it->second.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localR = " << it->second.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
  }

  std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets = ";
  for (size_t i = 0; i < ikp.size() * 3; i++) {
      std::cerr << default_zmp_offsets_array[i] << " ";
  }
  std::cerr << std::endl;
  delete[] default_zmp_offsets_array;
  std::cerr << "[" << m_profile.instance_name << "]   use_force_mode = " << getUseForceModeString() << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_mode = " << graspless_manip_mode << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_arm = " << graspless_manip_arm << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_p_gain = " << graspless_manip_p_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_pos = " << graspless_manip_reference_trans_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_rot = " << graspless_manip_reference_trans_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   transition_time = " << transition_time << "[s], zmp_transition_time = " << zmp_transition_time << "[s], adjust_footstep_transition_time = " << adjust_footstep_transition_time << "[s]" << std::endl;
  for (std::vector<std::string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) std::cerr << "[" << m_profile.instance_name << "]   leg_names [" << *it << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   default_gait_type = " << gait_type << std::endl;
  // FIK
  fik->move_base_gain = i_param.move_base_gain;
  fik->pos_ik_thre = i_param.pos_ik_thre;
  fik->rot_ik_thre = i_param.rot_ik_thre;
  fik->printParam();
  // IK limb parameters
  fik->setIKParam(ee_vec, i_param.ik_limb_parameters);
  // Limb stretch avoidance
  fik->use_limb_stretch_avoidance = i_param.use_limb_stretch_avoidance;
  fik->limb_stretch_avoidance_time_const = i_param.limb_stretch_avoidance_time_const;
  for (size_t i = 0; i < 2; i++) {
    fik->limb_stretch_avoidance_vlimit[i] = i_param.limb_stretch_avoidance_vlimit[i];
  }
  for (size_t i = 0; i < fik->ikp.size(); i++) {
    fik->ikp[ee_vec[i]].limb_length_margin = i_param.limb_length_margin[i];
  }
  is_emergency_step_mode = i_param.is_emergency_step_mode;
  is_emergency_touch_wall_mode = i_param.is_emergency_touch_wall_mode;
  use_collision_avoidance = i_param.use_collision_avoidance;
  if (control_mode == MODE_IDLE) {
    ik_mode = i_param.ik_mode;
    std::cerr << "[" << m_profile.instance_name << "]   ik_mode = " << ik_mode << std::endl;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   ik_mode cannot be set in (control_mode != MODE_IDLE). Current ik_mode is " << ik_mode << std::endl;
  }
  if (cog_constraint_interpolator->isEmpty()) {
    double tmp_time = 1.0;
    cog_constraint_interpolator->clear();
    cog_constraint_interpolator->set(&cog_z_constraint);
    cog_constraint_interpolator->setGoal(&i_param.cog_z_constraint, tmp_time, true);
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   cog_z_constraint cannot be set because interpolating." << std::endl;
  }
  touch_wall_retrieve_time = i_param.touch_wall_retrieve_time;
  is_natural_walk = i_param.is_natural_walk;
  is_stop_early_foot = i_param.is_stop_early_foot;
  arm_swing_deg = i_param.arm_swing_deg;
  debug_read_steppable_region = i_param.debug_read_steppable_region;
  return true;
};

bool AutoBalancer::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  i_param.default_zmp_offsets.length(ikp.size());
  for (size_t i = 0; i < ikp.size(); i++) {
      i_param.default_zmp_offsets[i].length(3);
      for (size_t j = 0; j < 3; j++) {
          i_param.default_zmp_offsets[i][j] = default_zmp_offsets[i](j);
      }
  }
  switch(control_mode) {
  case MODE_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_IDLE; break;
  case MODE_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_ABC; break;
  case MODE_SYNC_TO_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_IDLE; break;
  case MODE_SYNC_TO_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_ABC; break;
  default: break;
  }
  switch(use_force) {
  case MODE_NO_FORCE: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_NO_FORCE; break;
  case MODE_REF_FORCE: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE; break;
  case MODE_REF_FORCE_WITH_FOOT: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT; break;
  case MODE_REF_FORCE_RFU_EXT_MOMENT: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT; break;
  default: break;
  }
  i_param.graspless_manip_mode = graspless_manip_mode;
  i_param.graspless_manip_arm = graspless_manip_arm.c_str();
  for (size_t j = 0; j < 3; j++)
      i_param.graspless_manip_p_gain[j] = graspless_manip_p_gain[j];
  for (size_t j = 0; j < 3; j++)
      i_param.graspless_manip_reference_trans_pos[j] = graspless_manip_reference_trans_coords.pos[j];
  Eigen::Quaternion<double> qt(graspless_manip_reference_trans_coords.rot);
  i_param.graspless_manip_reference_trans_rot[0] = qt.w();
  i_param.graspless_manip_reference_trans_rot[1] = qt.x();
  i_param.graspless_manip_reference_trans_rot[2] = qt.y();
  i_param.graspless_manip_reference_trans_rot[3] = qt.z();
  i_param.transition_time = transition_time;
  i_param.zmp_transition_time = zmp_transition_time;
  i_param.adjust_footstep_transition_time = adjust_footstep_transition_time;
  i_param.leg_names.length(leg_names.size());
  for (size_t i = 0; i < leg_names.size(); i++) i_param.leg_names[i] = leg_names.at(i).c_str();
  i_param.is_hand_fix_mode = is_hand_fix_mode;
  i_param.end_effector_list.length(ikp.size());
  {
      size_t i = 0;
      for (std::map<std::string, ABCIKparam>::const_iterator it = ikp.begin(); it != ikp.end(); it++) {
          copyRatscoords2Footstep(i_param.end_effector_list[i],
                                  coordinates(it->second.localPos, it->second.localR));
          i_param.end_effector_list[i].leg = it->first.c_str();
          i++;
      }
  }
  switch(gait_type) {
  case BIPED:  i_param.default_gait_type = OpenHRP::AutoBalancerService::BIPED;  break;
  case TROT:   i_param.default_gait_type = OpenHRP::AutoBalancerService::TROT;   break;
  case PACE:   i_param.default_gait_type = OpenHRP::AutoBalancerService::PACE;   break;
  case CRAWL:  i_param.default_gait_type = OpenHRP::AutoBalancerService::CRAWL;  break;
  case GALLOP: i_param.default_gait_type = OpenHRP::AutoBalancerService::GALLOP; break;
  default: break;
  }
  // FIK
  i_param.move_base_gain = fik->move_base_gain;
  i_param.pos_ik_thre = fik->pos_ik_thre;
  i_param.rot_ik_thre = fik->rot_ik_thre;
  // IK limb parameters
  fik->getIKParam(ee_vec, i_param.ik_limb_parameters);
  // Limb stretch avoidance
  i_param.use_limb_stretch_avoidance = fik->use_limb_stretch_avoidance;
  i_param.limb_stretch_avoidance_time_const = fik->limb_stretch_avoidance_time_const;
  i_param.limb_length_margin.length(fik->ikp.size());
  for (size_t i = 0; i < 2; i++) {
    i_param.limb_stretch_avoidance_vlimit[i] = fik->limb_stretch_avoidance_vlimit[i];
  }
  for (size_t i = 0; i < ikp.size(); i++) {
    i_param.limb_length_margin[i] = fik->ikp[ee_vec[i]].limb_length_margin;
  }
  i_param.additional_force_applied_link_name = additional_force_applied_link->name.c_str();
  for (size_t i = 0; i < 3; i++) {
      i_param.additional_force_applied_point_offset[i] = additional_force_applied_point_offset(i);
  }
  i_param.is_emergency_step_mode = is_emergency_step_mode;
  i_param.is_emergency_touch_wall_mode = is_emergency_touch_wall_mode;
  i_param.ik_mode = ik_mode;
  i_param.use_collision_avoidance = use_collision_avoidance;
  i_param.cog_z_constraint = cog_z_constraint;
  i_param.touch_wall_retrieve_time = touch_wall_retrieve_time;
  i_param.is_natural_walk = is_natural_walk;
  i_param.is_stop_early_foot = is_stop_early_foot;
  i_param.arm_swing_deg = arm_swing_deg;
  i_param.debug_read_steppable_region = debug_read_steppable_region;
  return true;
};

void AutoBalancer::setStabilizerParam(const OpenHRP::AutoBalancerService::StabilizerParam& i_param)
{
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name << "] setStabilizerParam" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    st->k_tpcc_p[i] = i_param.k_tpcc_p[i];
    st->k_tpcc_x[i] = i_param.k_tpcc_x[i];
    st->k_brot_p[i] = i_param.k_brot_p[i];
    st->k_brot_tc[i] = i_param.k_brot_tc[i];
  }
  std::cerr << "[" << m_profile.instance_name << "]  TPCC" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   k_tpcc_p  = [" << st->k_tpcc_p[0] << ", " <<  st->k_tpcc_p[1] << "], k_tpcc_x  = [" << st->k_tpcc_x[0] << ", " << st->k_tpcc_x[1] << "], k_brot_p  = [" << st->k_brot_p[0] << ", " << st->k_brot_p[1] << "], k_brot_tc = [" << st->k_brot_tc[0] << ", " << st->k_brot_tc[1] << "]" << std::endl;
  // for (size_t i = 0; i < 2; i++) {
  //   k_run_b[i] = i_param.k_run_b[i];
  //   d_run_b[i] = i_param.d_run_b[i];
  //   m_tau_x[i].setup(i_param.tdfke[0], i_param.tdftc[0], dt);
  //   m_tau_y[i].setup(i_param.tdfke[0], i_param.tdftc[0], dt);
  //   m_f_z.setup(i_param.tdfke[1], i_param.tdftc[1], dt);
  // }
  // m_torque_k[0] = i_param.k_run_x;
  // m_torque_k[1] = i_param.k_run_y;
  // m_torque_d[0] = i_param.d_run_x;
  // m_torque_d[1] = i_param.d_run_y;
  // std::cerr << "[" << m_profile.instance_name << "]  RUNST" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   m_torque_k  = [" << m_torque_k[0] << ", " <<  m_torque_k[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   m_torque_d  = [" << m_torque_d[0] << ", " <<  m_torque_d[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   k_run_b  = [" << k_run_b[0] << ", " <<  k_run_b[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   d_run_b  = [" << d_run_b[0] << ", " <<  d_run_b[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]  EEFM" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    st->eefm_k1[i] = i_param.eefm_k1[i];
    st->eefm_k2[i] = i_param.eefm_k2[i];
    st->eefm_k3[i] = i_param.eefm_k3[i];
    st->eefm_zmp_delay_time_const[i] = i_param.eefm_zmp_delay_time_const[i];
    st->ref_zmp_aux(i) = i_param.eefm_ref_zmp_aux[i];
    st->eefm_body_attitude_control_gain[i] = i_param.eefm_body_attitude_control_gain[i];
    st->eefm_body_attitude_control_time_const[i] = i_param.eefm_body_attitude_control_time_const[i];
    st->ref_cp(i) = i_param.ref_capture_point[i];
    st->act_cp(i) = i_param.act_capture_point[i];
    st->cp_offset(i) = i_param.cp_offset[i];
  }
  bool is_damping_parameter_ok = true;
  if ( i_param.eefm_pos_damping_gain.length () == st->stikp.size() &&
       i_param.eefm_pos_time_const_support.length () == st->stikp.size() &&
       i_param.eefm_pos_compensation_limit.length () == st->stikp.size() &&
       i_param.eefm_swing_pos_spring_gain.length () == st->stikp.size() &&
       i_param.eefm_swing_pos_time_const.length () == st->stikp.size() &&
       i_param.eefm_rot_damping_gain.length () == st->stikp.size() &&
       i_param.eefm_rot_time_const.length () == st->stikp.size() &&
       i_param.eefm_rot_compensation_limit.length () == st->stikp.size() &&
       i_param.eefm_swing_rot_spring_gain.length () == st->stikp.size() &&
       i_param.eefm_swing_rot_time_const.length () == st->stikp.size() &&
       i_param.eefm_ee_moment_limit.length () == st->stikp.size() &&
       i_param.eefm_ee_forcemoment_distribution_weight.length () == st->stikp.size()) {
    is_damping_parameter_ok = true;
    for (size_t j = 0; j < st->stikp.size(); j++) {
      for (size_t i = 0; i < 3; i++) {
        st->stikp[j].eefm_pos_damping_gain(i) = i_param.eefm_pos_damping_gain[j][i];
        st->stikp[j].eefm_pos_time_const_support(i) = i_param.eefm_pos_time_const_support[j][i];
        st->stikp[j].eefm_swing_pos_spring_gain(i) = i_param.eefm_swing_pos_spring_gain[j][i];
        st->stikp[j].eefm_swing_pos_time_const(i) = i_param.eefm_swing_pos_time_const[j][i];
        st->stikp[j].eefm_rot_damping_gain(i) = i_param.eefm_rot_damping_gain[j][i];
        st->stikp[j].eefm_rot_time_const(i) = i_param.eefm_rot_time_const[j][i];
        st->stikp[j].eefm_swing_rot_spring_gain(i) = i_param.eefm_swing_rot_spring_gain[j][i];
        st->stikp[j].eefm_swing_rot_time_const(i) = i_param.eefm_swing_rot_time_const[j][i];
        st->stikp[j].eefm_ee_moment_limit(i) = i_param.eefm_ee_moment_limit[j][i];
        st->stikp[j].eefm_ee_forcemoment_distribution_weight(i) = i_param.eefm_ee_forcemoment_distribution_weight[j][i];
        st->stikp[j].eefm_ee_forcemoment_distribution_weight(i+3) = i_param.eefm_ee_forcemoment_distribution_weight[j][i+3];
      }
      st->stikp[j].eefm_pos_compensation_limit = i_param.eefm_pos_compensation_limit[j];
      st->stikp[j].eefm_rot_compensation_limit = i_param.eefm_rot_compensation_limit[j];
    }
  } else {
    is_damping_parameter_ok = false;
  }
  for (size_t i = 0; i < 3; i++) {
    st->eefm_swing_pos_damping_gain(i) = i_param.eefm_swing_pos_damping_gain[i];
    st->eefm_swing_rot_damping_gain(i) = i_param.eefm_swing_rot_damping_gain[i];
  }
  st->eefm_pos_time_const_swing = i_param.eefm_pos_time_const_swing;
  st->eefm_pos_transition_time = i_param.eefm_pos_transition_time;
  st->eefm_pos_margin_time = i_param.eefm_pos_margin_time;
  st->szd->set_leg_inside_margin(i_param.eefm_leg_inside_margin);
  st->szd->set_leg_outside_margin(i_param.eefm_leg_outside_margin);
  st->szd->set_leg_front_margin(i_param.eefm_leg_front_margin);
  st->szd->set_leg_rear_margin(i_param.eefm_leg_rear_margin);
  st->szd->set_vertices_from_margin_params();

  if (i_param.eefm_support_polygon_vertices_sequence.length() != st->stikp.size()) {
    std::cerr << "[" << m_profile.instance_name << "]   eefm_support_polygon_vertices_sequence cannot be set. Length " << i_param.eefm_support_polygon_vertices_sequence.length() << " != " << st->stikp.size() << std::endl;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   eefm_support_polygon_vertices_sequence set" << std::endl;
    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
    for (size_t ee_idx = 0; ee_idx < i_param.eefm_support_polygon_vertices_sequence.length(); ee_idx++) {
      std::vector<Eigen::Vector2d> tvec;
      for (size_t v_idx = 0; v_idx < i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(); v_idx++) {
        tvec.push_back(Eigen::Vector2d(i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0],
                                       i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1]));
      }
      support_polygon_vec.push_back(tvec);
    }
    st->szd->set_vertices(support_polygon_vec);
    st->szd->print_vertices(std::string(m_profile.instance_name));
  }
  st->eefm_use_force_difference_control = i_param.eefm_use_force_difference_control;
  st->eefm_use_swing_damping = i_param.eefm_use_swing_damping;
  for (size_t i = 0; i < 3; ++i) {
    st->eefm_swing_damping_force_thre[i] = i_param.eefm_swing_damping_force_thre[i];
    st->eefm_swing_damping_moment_thre[i] = i_param.eefm_swing_damping_moment_thre[i];
  }
  st->act_cogvel_filter->setCutOffFreq(i_param.eefm_cogvel_cutoff_freq);
  st->szd->set_wrench_alpha_blending(i_param.eefm_wrench_alpha_blending);
  st->szd->set_alpha_cutoff_freq(i_param.eefm_alpha_cutoff_freq);
  st->eefm_gravitational_acceleration = i_param.eefm_gravitational_acceleration;
  for (size_t i = 0; i < st->stikp.size(); i++) {
    st->stikp[i].target_ee_diff_p_filter->setCutOffFreq(i_param.eefm_ee_error_cutoff_freq);
    st->stikp[i].limb_length_margin = i_param.limb_length_margin[i];
  }
  st->setBoolSequenceParam(st->is_ik_enable, i_param.is_ik_enable, std::string("is_ik_enable"));
  st->setBoolSequenceParamWithCheckContact(st->is_feedback_control_enable, i_param.is_feedback_control_enable, std::string("is_feedback_control_enable"));
  st->setBoolSequenceParam(st->is_zmp_calc_enable, i_param.is_zmp_calc_enable, std::string("is_zmp_calc_enable"));
  st->emergency_check_mode = i_param.emergency_check_mode;

  st->transition_time = i_param.transition_time;
  st->cop_check_margin = i_param.cop_check_margin;
  for (size_t i = 0; i < st->cp_check_margin.size(); i++) {
    st->cp_check_margin[i] = i_param.cp_check_margin[i];
 }
  st->szd->set_vertices_from_margin_params(st->cp_check_margin);
  for (size_t i = 0; i < st->tilt_margin.size(); i++) {
    st->tilt_margin[i] = i_param.tilt_margin[i];
  }
  st->contact_decision_threshold = i_param.contact_decision_threshold;
  st->is_estop_while_walking = i_param.is_estop_while_walking;
  st->use_limb_stretch_avoidance = i_param.use_limb_stretch_avoidance;
  st->use_zmp_truncation = i_param.use_zmp_truncation;
  st->use_force_sensor = i_param.use_force_sensor;
  st->use_footguided_stabilizer = i_param.use_footguided_stabilizer;
  gg->footguided_balance_time_const = st->footguided_balance_time_const = i_param.footguided_balance_time_const;
  for (size_t i = 0; i < 2; i++) {
    st->limb_stretch_avoidance_vlimit[i] = i_param.limb_stretch_avoidance_vlimit[i];
    st->root_rot_compensation_limit[i] = i_param.root_rot_compensation_limit[i];
  }
  st->detection_count_to_air = static_cast<int>(i_param.detection_time_to_air / m_dt);
  if (st->control_mode == Stabilizer::MODE_IDLE) {
    for (size_t i = 0; i < i_param.end_effector_list.length(); i++) {
      std::vector<Stabilizer::STIKParam>::iterator it = std::find_if(st->stikp.begin(), st->stikp.end(), (&boost::lambda::_1->* &std::vector<Stabilizer::STIKParam>::value_type::ee_name == std::string(i_param.end_effector_list[i].leg)));
      memcpy(it->localp.data(), i_param.end_effector_list[i].pos, sizeof(double)*3);
      it->localR = (Eigen::Quaternion<double>(i_param.end_effector_list[i].rot[0], i_param.end_effector_list[i].rot[1], i_param.end_effector_list[i].rot[2], i_param.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
    }
  } else {
    std::cerr << "[" << m_profile.instance_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
  }
  for (std::vector<Stabilizer::STIKParam>::const_iterator it = st->stikp.begin(); it != st->stikp.end(); it++) {
    std::cerr << "[" << m_profile.instance_name << "]  End Effector [" << it->ee_name << "]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   localpos = " << it->localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   localR = " << it->localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", "    [", "]")) << std::endl;
  }
  if (i_param.foot_origin_offset.length () != 2) {
    std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset cannot be set. Length " << i_param.foot_origin_offset.length() << " != " << 2 << std::endl;
  } else if (st->control_mode != Stabilizer::MODE_IDLE) {
    std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset cannot be set. Current control_mode is " << st->control_mode << std::endl;
  } else {
    for (size_t i = 0; i < i_param.foot_origin_offset.length(); i++) {
      st->foot_origin_offset[i](0) = i_param.foot_origin_offset[i][0];
      st->foot_origin_offset[i](1) = i_param.foot_origin_offset[i][1];
      st->foot_origin_offset[i](2) = i_param.foot_origin_offset[i][2];
    }
  }
  std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset is ";
  for (size_t i = 0; i < 2; i++) {
    std::cerr << st->foot_origin_offset[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"));
  }
  std::cerr << "[m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_k1  = [" << st->eefm_k1[0] << ", " << st->eefm_k1[1] << "], eefm_k2  = [" << st->eefm_k2[0] << ", " << st->eefm_k2[1] << "], eefm_k3  = [" << st->eefm_k3[0] << ", " << st->eefm_k3[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_zmp_delay_time_const  = [" << st->eefm_zmp_delay_time_const[0] << ", " << st->eefm_zmp_delay_time_const[1] << "][s], eefm_ref_zmp_aux  = [" << st->ref_zmp_aux(0) << ", " << st->ref_zmp_aux(1) << "][m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_body_attitude_control_gain  = [" << st->eefm_body_attitude_control_gain[0] << ", " << st->eefm_body_attitude_control_gain[1] << "], eefm_body_attitude_control_time_const  = [" << st->eefm_body_attitude_control_time_const[0] << ", " << st->eefm_body_attitude_control_time_const[1] << "][s]" << std::endl;
  if (is_damping_parameter_ok) {
    for (size_t j = 0; j < st->stikp.size(); j++) {
      std::cerr << "[" << m_profile.instance_name << "]   [" << st->stikp[j].ee_name << "] eefm_rot_damping_gain = "
                << st->stikp[j].eefm_rot_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                << ", eefm_rot_time_const = "
                << st->stikp[j].eefm_rot_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                << "[s]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   [" << st->stikp[j].ee_name << "] eefm_pos_damping_gain = "
                << st->stikp[j].eefm_pos_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                << ", eefm_pos_time_const_support = "
                << st->stikp[j].eefm_pos_time_const_support.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                << "[s]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   [" << st->stikp[j].ee_name << "] "
                << "eefm_pos_compensation_limit = " << st->stikp[j].eefm_pos_compensation_limit << "[m], "
                << "eefm_rot_compensation_limit = " << st->stikp[j].eefm_rot_compensation_limit << "[rad], "
                << "eefm_ee_moment_limit = " << st->stikp[j].eefm_ee_moment_limit.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[Nm]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   [" << st->stikp[j].ee_name << "] "
                << "eefm_swing_pos_spring_gain = " << st->stikp[j].eefm_swing_pos_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                << "eefm_swing_pos_time_const = " << st->stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                << "eefm_swing_rot_spring_gain = " << st->stikp[j].eefm_swing_rot_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                << "eefm_swing_pos_time_const = " << st->stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   [" << st->stikp[j].ee_name << "] "
                << "eefm_ee_forcemoment_distribution_weight = " << st->stikp[j].eefm_ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "" << std::endl;
    }
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   eefm damping parameters cannot be set because of invalid param." << std::endl;
  }
  std::cerr << "[" << m_profile.instance_name << "]   eefm_pos_transition_time = " << st->eefm_pos_transition_time << "[s], eefm_pos_margin_time = " << st->eefm_pos_margin_time << "[s] eefm_pos_time_const_swing = " << st->eefm_pos_time_const_swing << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   cogvel_cutoff_freq = " << st->act_cogvel_filter->getCutOffFreq() << "[Hz]" << std::endl;
  st->szd->print_params(std::string(m_profile.instance_name));
  std::cerr << "[" << m_profile.instance_name << "]   eefm_gravitational_acceleration = " << st->eefm_gravitational_acceleration << "[m/s^2], eefm_use_force_difference_control = " << (st->eefm_use_force_difference_control? "true":"false") << ", eefm_use_swing_damping = " << (st->eefm_use_swing_damping? "true":"false") << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_ee_error_cutoff_freq = " << st->stikp[0].target_ee_diff_p_filter->getCutOffFreq() << "[Hz]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]  COMMON" << std::endl;
  if (st->control_mode == Stabilizer::MODE_IDLE) {
    st->st_algorithm = i_param.st_algorithm;
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm changed to [" << st->getStabilizerAlgorithmString(st->st_algorithm) << "]" << std::endl;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm cannot be changed to [" << st->getStabilizerAlgorithmString(st->st_algorithm) << "] during MODE_AIR or MODE_ST." << std::endl;
  }
  std::cerr << "[" << m_profile.instance_name << "]   emergency_check_mode changed to [" << (st->emergency_check_mode == OpenHRP::AutoBalancerService::NO_CHECK?"NO_CHECK": (st->emergency_check_mode == OpenHRP::AutoBalancerService::COP?"COP":"CP") ) << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   transition_time = " << st->transition_time << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   cop_check_margin = " << st->cop_check_margin << "[m], "
            << "cp_check_margin = [" << st->cp_check_margin[0] << ", " << st->cp_check_margin[1] << ", " << st->cp_check_margin[2] << ", " << st->cp_check_margin[3] << "] [m], "
            << "tilt_margin = [" << st->tilt_margin[0] << ", " << st->tilt_margin[1] << "] [rad]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   contact_decision_threshold = " << st->contact_decision_threshold << "[N], detection_time_to_air = " << st->detection_count_to_air * m_dt << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   root_rot_compensation_limit = [" << st->root_rot_compensation_limit[0] << " " << st->root_rot_compensation_limit[1] << "][rad]" << std::endl;
  // IK limb parameters
  std::cerr << "[" << m_profile.instance_name << "]  IK limb parameters" << std::endl;
  bool is_ik_limb_parameter_valid_length = true;
  if (i_param.ik_limb_parameters.length() != st->jpe_v.size()) {
    is_ik_limb_parameter_valid_length = false;
    std::cerr << "[" << m_profile.instance_name << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << i_param.ik_limb_parameters.length() << ", desired = " << st->jpe_v.size() << ")" << std::endl;
  } else {
    for (size_t i = 0; i < st->jpe_v.size(); i++) {
      if (st->jpe_v[i]->numJoints() != i_param.ik_limb_parameters[i].ik_optional_weight_vector.length())
        is_ik_limb_parameter_valid_length = false;
    }
    if (is_ik_limb_parameter_valid_length) {
      for (size_t i = 0; i < st->jpe_v.size(); i++) {
        const OpenHRP::AutoBalancerService::IKLimbParameters& ilp = i_param.ik_limb_parameters[i];
        std::vector<double> ov;
        ov.resize(st->jpe_v[i]->numJoints());
        for (size_t j = 0; j < st->jpe_v[i]->numJoints(); j++) {
          ov[j] = ilp.ik_optional_weight_vector[j];
        }
        st->jpe_v[i]->setOptionalWeightVector(ov);
        st->jpe_v[i]->setSRGain(ilp.sr_gain);
        st->stikp[i].avoid_gain = ilp.avoid_gain;
        st->stikp[i].reference_gain = ilp.reference_gain;
        st->jpe_v[i]->setManipulabilityLimit(ilp.manipulability_limit);
        st->stikp[i].ik_loop_count = ilp.ik_loop_count; // unsigned short -> size_t, value not change
      }
    } else {
      std::cerr << "[" << m_profile.instance_name << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
      for (size_t i = 0; i < st->jpe_v.size(); i++) {
        std::cerr << i_param.ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
      }
      std::cerr << "], desired = [";
      for (size_t i = 0; i < st->jpe_v.size(); i++) {
        std::cerr << st->jpe_v[i]->numJoints() << ", ";
      }
      std::cerr << "])" << std::endl;
    }
  }
  if (is_ik_limb_parameter_valid_length) {
    std::cerr << "[" << m_profile.instance_name << "]   ik_optional_weight_vectors = ";
    for (size_t i = 0; i < st->jpe_v.size(); i++) {
      std::vector<double> ov;
      ov.resize(st->jpe_v[i]->numJoints());
      st->jpe_v[i]->getOptionalWeightVector(ov);
      std::cerr << "[";
      for (size_t j = 0; j < st->jpe_v[i]->numJoints(); j++) {
        std::cerr << ov[j] << " ";
      }
      std::cerr << "]";
    }
    std::cerr << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   sr_gains = [";
    for (size_t i = 0; i < st->jpe_v.size(); i++) {
      std::cerr << st->jpe_v[i]->getSRGain() << ", ";
    }
    std::cerr << "]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   avoid_gains = [";
    for (size_t i = 0; i < st->stikp.size(); i++) {
      std::cerr << st->stikp[i].avoid_gain << ", ";
    }
    std::cerr << "]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   reference_gains = [";
    for (size_t i = 0; i < st->stikp.size(); i++) {
      std::cerr << st->stikp[i].reference_gain << ", ";
    }
    std::cerr << "]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   manipulability_limits = [";
    for (size_t i = 0; i < st->jpe_v.size(); i++) {
      std::cerr << st->jpe_v[i]->getManipulabilityLimit() << ", ";
    }
    std::cerr << "]" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   ik_loop_count = [";
    for (size_t i = 0; i < st->stikp.size(); i++) {
      std::cerr << st->stikp[i].ik_loop_count << ", ";
    }
    std::cerr << "]" << std::endl;
  }
  // joint servo control parameters
  std::cerr << "[" << m_profile.instance_name << "]  joint servo control parameters" << std::endl;
  if (control_mode == MODE_IDLE) {
      // !TORQUE -> TORQUE
      if( st->joint_control_mode != OpenHRP::RobotHardwareService::TORQUE && i_param.joint_control_mode == OpenHRP::RobotHardwareService::TORQUE ){
          for(size_t i = 0; i < st->stikp.size(); i++) {
              Stabilizer::STIKParam& ikp = st->stikp[i];
              ikp.eefm_pos_damping_gain *= 1000;
              ikp.eefm_rot_damping_gain *= 1000;
          }
          st->eefm_swing_pos_damping_gain *= 1000;
          st->eefm_swing_rot_damping_gain *= 1000;
      }
      // TORQUE -> !TORQUE
      if( st->joint_control_mode == OpenHRP::RobotHardwareService::TORQUE && i_param.joint_control_mode != OpenHRP::RobotHardwareService::TORQUE ){
          for(size_t i = 0; i < st->stikp.size(); i++) {
              Stabilizer::STIKParam& ikp = st->stikp[i];
              ikp.eefm_pos_damping_gain /= 1000;
              ikp.eefm_rot_damping_gain /= 1000;
          }
          st->eefm_swing_pos_damping_gain /= 1000;
          st->eefm_swing_rot_damping_gain /= 1000;
      }
      st->joint_control_mode = i_param.joint_control_mode;
      std::cerr << "[" << m_profile.instance_name << "]   joint_control_mode changed" << std::endl;
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   joint_control_mode cannot be changed during MODE_AIR or MODE_ST." << std::endl;
  }
  st->swing2landing_transition_time = i_param.swing2landing_transition_time;
  st->landing_phase_time = i_param.landing_phase_time;
  st->landing2support_transition_time = i_param.landing2support_transition_time;
  st->support_phase_min_time = i_param.support_phase_min_time;
  st->support2swing_transition_time = i_param.support2swing_transition_time;
  bool is_joint_servo_control_parameter_valid_length = true;
  if ( i_param.joint_servo_control_parameters.length() == st->stikp.size() ) {
      for (size_t i = 0; i < st->stikp.size(); i++) {
          const OpenHRP::AutoBalancerService::JointServoControlParameter& jscp = i_param.joint_servo_control_parameters[i];
          if ( st->stikp[i].support_pgain.size() == i_param.joint_servo_control_parameters[i].support_pgain.length() &&
               st->stikp[i].support_dgain.size() == i_param.joint_servo_control_parameters[i].support_dgain.length() &&
               st->stikp[i].landing_pgain.size() == i_param.joint_servo_control_parameters[i].landing_pgain.length() &&
               st->stikp[i].landing_dgain.size() == i_param.joint_servo_control_parameters[i].landing_dgain.length() &&
               st->stikp[i].swing_pgain.size() == i_param.joint_servo_control_parameters[i].swing_pgain.length() &&
               st->stikp[i].swing_dgain.size() == i_param.joint_servo_control_parameters[i].swing_dgain.length() ) {
              for (size_t j = 0; j < st->stikp[i].support_pgain.size(); j++) {
                  st->stikp[i].support_pgain(j) = i_param.joint_servo_control_parameters[i].support_pgain[j];
                  st->stikp[i].support_dgain(j) = i_param.joint_servo_control_parameters[i].support_dgain[j];
                  st->stikp[i].landing_pgain(j) = i_param.joint_servo_control_parameters[i].landing_pgain[j];
                  st->stikp[i].landing_dgain(j) = i_param.joint_servo_control_parameters[i].landing_dgain[j];
                  st->stikp[i].swing_pgain(j) = i_param.joint_servo_control_parameters[i].swing_pgain[j];
                  st->stikp[i].swing_dgain(j) = i_param.joint_servo_control_parameters[i].swing_dgain[j];
              }
          } else is_joint_servo_control_parameter_valid_length = false;
      }
  } else {
      is_joint_servo_control_parameter_valid_length = false;
  }
  if ( is_joint_servo_control_parameter_valid_length ) {
      std::cerr << "[" << m_profile.instance_name << "]   support_pgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].support_pgain.transpose() << "],";
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   support_dgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].support_dgain.transpose() << "],";
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   landing_pgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].landing_pgain.transpose() << "],";
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   landing_dgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].landing_dgain.transpose() << "],";
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   swing_pgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].swing_pgain.transpose() << "],";
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   swing_dgain = [";
      for (size_t i = 0; i < st->stikp.size(); i++) std::cerr << "[" << st->stikp[i].swing_dgain.transpose() << "],";
      std::cerr << "]" << std::endl;
  } else std::cerr << "[" << m_profile.instance_name << "]   servo gain parameters cannot be set because of invalid param." << std::endl;
}

void AutoBalancer::getStabilizerParam(OpenHRP::AutoBalancerService::StabilizerParam& i_param)
{
  std::cerr << "[" << m_profile.instance_name << "] getStabilizerParam" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    // i_param.k_run_b[i] = k_run_b[i];
    // i_param.d_run_b[i] = d_run_b[i];
    //m_tau_x[i].setup(i_param.tdfke[0], i_param.tdftc[0], dt);
    //m_tau_y[i].setup(i_param.tdfke[0], i_param.tdftc[0], dt);
    //m_f_z.setup(i_param.tdfke[1], i_param.tdftc[1], dt);
    i_param.k_tpcc_p[i] = st->k_tpcc_p[i];
    i_param.k_tpcc_x[i] = st->k_tpcc_x[i];
    i_param.k_brot_p[i] = st->k_brot_p[i];
    i_param.k_brot_tc[i] = st->k_brot_tc[i];
  }
  // i_param.k_run_x = m_torque_k[0];
  // i_param.k_run_y = m_torque_k[1];
  // i_param.d_run_x = m_torque_d[0];
  // i_param.d_run_y = m_torque_d[1];
  for (size_t i = 0; i < 2; i++) {
    i_param.eefm_k1[i] = st->eefm_k1[i];
    i_param.eefm_k2[i] = st->eefm_k2[i];
    i_param.eefm_k3[i] = st->eefm_k3[i];
    i_param.eefm_zmp_delay_time_const[i] = st->eefm_zmp_delay_time_const[i];
    i_param.eefm_ref_zmp_aux[i] = st->ref_zmp_aux(i);
    i_param.eefm_body_attitude_control_time_const[i] = st->eefm_body_attitude_control_time_const[i];
    i_param.eefm_body_attitude_control_gain[i] = st->eefm_body_attitude_control_gain[i];
    i_param.ref_capture_point[i] = st->ref_cp(i);
    i_param.act_capture_point[i] = st->act_cp(i);
    i_param.cp_offset[i] = st->cp_offset(i);
  }
  i_param.eefm_pos_time_const_support.length(st->stikp.size());
  i_param.eefm_pos_damping_gain.length(st->stikp.size());
  i_param.eefm_pos_compensation_limit.length(st->stikp.size());
  i_param.eefm_swing_pos_spring_gain.length(st->stikp.size());
  i_param.eefm_swing_pos_time_const.length(st->stikp.size());
  i_param.eefm_rot_time_const.length(st->stikp.size());
  i_param.eefm_rot_damping_gain.length(st->stikp.size());
  i_param.eefm_rot_compensation_limit.length(st->stikp.size());
  i_param.eefm_swing_rot_spring_gain.length(st->stikp.size());
  i_param.eefm_swing_rot_time_const.length(st->stikp.size());
  i_param.eefm_ee_moment_limit.length(st->stikp.size());
  i_param.eefm_ee_forcemoment_distribution_weight.length(st->stikp.size());
  for (size_t j = 0; j < st->stikp.size(); j++) {
    i_param.eefm_pos_damping_gain[j].length(3);
    i_param.eefm_pos_time_const_support[j].length(3);
    i_param.eefm_swing_pos_spring_gain[j].length(3);
    i_param.eefm_swing_pos_time_const[j].length(3);
    i_param.eefm_rot_damping_gain[j].length(3);
    i_param.eefm_rot_time_const[j].length(3);
    i_param.eefm_swing_rot_spring_gain[j].length(3);
    i_param.eefm_swing_rot_time_const[j].length(3);
    i_param.eefm_ee_moment_limit[j].length(3);
    i_param.eefm_ee_forcemoment_distribution_weight[j].length(6);
    for (size_t i = 0; i < 3; i++) {
      i_param.eefm_pos_damping_gain[j][i] = st->stikp[j].eefm_pos_damping_gain(i);
      i_param.eefm_pos_time_const_support[j][i] = st->stikp[j].eefm_pos_time_const_support(i);
      i_param.eefm_swing_pos_spring_gain[j][i] = st->stikp[j].eefm_swing_pos_spring_gain(i);
      i_param.eefm_swing_pos_time_const[j][i] = st->stikp[j].eefm_swing_pos_time_const(i);
      i_param.eefm_rot_damping_gain[j][i] = st->stikp[j].eefm_rot_damping_gain(i);
      i_param.eefm_rot_time_const[j][i] = st->stikp[j].eefm_rot_time_const(i);
      i_param.eefm_swing_rot_spring_gain[j][i] = st->stikp[j].eefm_swing_rot_spring_gain(i);
      i_param.eefm_swing_rot_time_const[j][i] = st->stikp[j].eefm_swing_rot_time_const(i);
      i_param.eefm_ee_moment_limit[j][i] = st->stikp[j].eefm_ee_moment_limit(i);
      i_param.eefm_ee_forcemoment_distribution_weight[j][i] = st->stikp[j].eefm_ee_forcemoment_distribution_weight(i);
      i_param.eefm_ee_forcemoment_distribution_weight[j][i+3] = st->stikp[j].eefm_ee_forcemoment_distribution_weight(i+3);
    }
    i_param.eefm_pos_compensation_limit[j] = st->stikp[j].eefm_pos_compensation_limit;
    i_param.eefm_rot_compensation_limit[j] = st->stikp[j].eefm_rot_compensation_limit;
  }
  for (size_t i = 0; i < 3; i++) {
    i_param.eefm_swing_pos_damping_gain[i] = st->eefm_swing_pos_damping_gain(i);
    i_param.eefm_swing_rot_damping_gain[i] = st->eefm_swing_rot_damping_gain(i);
  }
  i_param.eefm_pos_time_const_swing = st->eefm_pos_time_const_swing;
  i_param.eefm_pos_transition_time = st->eefm_pos_transition_time;
  i_param.eefm_pos_margin_time = st->eefm_pos_margin_time;
  i_param.eefm_leg_inside_margin = st->szd->get_leg_inside_margin();
  i_param.eefm_leg_outside_margin = st->szd->get_leg_outside_margin();
  i_param.eefm_leg_front_margin = st->szd->get_leg_front_margin();
  i_param.eefm_leg_rear_margin = st->szd->get_leg_rear_margin();

  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
  st->szd->get_vertices(support_polygon_vec);
  i_param.eefm_support_polygon_vertices_sequence.length(support_polygon_vec.size());
  for (size_t ee_idx = 0; ee_idx < support_polygon_vec.size(); ee_idx++) {
    i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(support_polygon_vec[ee_idx].size());
    for (size_t v_idx = 0; v_idx < support_polygon_vec[ee_idx].size(); v_idx++) {
      i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0] = support_polygon_vec[ee_idx][v_idx](0);
      i_param.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1] = support_polygon_vec[ee_idx][v_idx](1);
    }
  }

  i_param.eefm_cogvel_cutoff_freq = st->act_cogvel_filter->getCutOffFreq();
  i_param.eefm_wrench_alpha_blending = st->szd->get_wrench_alpha_blending();
  i_param.eefm_alpha_cutoff_freq = st->szd->get_alpha_cutoff_freq();
  i_param.eefm_gravitational_acceleration = st->eefm_gravitational_acceleration;
  i_param.eefm_ee_error_cutoff_freq = st->stikp[0].target_ee_diff_p_filter->getCutOffFreq();
  i_param.eefm_use_force_difference_control = st->eefm_use_force_difference_control;
  i_param.eefm_use_swing_damping = st->eefm_use_swing_damping;
  for (size_t i = 0; i < 3; ++i) {
    i_param.eefm_swing_damping_force_thre[i] = st->eefm_swing_damping_force_thre[i];
    i_param.eefm_swing_damping_moment_thre[i] = st->eefm_swing_damping_moment_thre[i];
  }
  i_param.is_ik_enable.length(st->is_ik_enable.size());
  for (size_t i = 0; i < st->is_ik_enable.size(); i++) {
    i_param.is_ik_enable[i] = st->is_ik_enable[i];
  }
  i_param.is_feedback_control_enable.length(st->is_feedback_control_enable.size());
  for (size_t i = 0; i < st->is_feedback_control_enable.size(); i++) {
    i_param.is_feedback_control_enable[i] = st->is_feedback_control_enable[i];
  }
  i_param.is_zmp_calc_enable.length(st->is_zmp_calc_enable.size());
  for (size_t i = 0; i < st->is_zmp_calc_enable.size(); i++) {
    i_param.is_zmp_calc_enable[i] = st->is_zmp_calc_enable[i];
  }

  i_param.foot_origin_offset.length(2);
  for (size_t i = 0; i < i_param.foot_origin_offset.length(); i++) {
    i_param.foot_origin_offset[i].length(3);
    i_param.foot_origin_offset[i][0] = st->foot_origin_offset[i](0);
    i_param.foot_origin_offset[i][1] = st->foot_origin_offset[i](1);
    i_param.foot_origin_offset[i][2] = st->foot_origin_offset[i](2);
  }
  i_param.st_algorithm = st->st_algorithm;
  i_param.transition_time = st->transition_time;
  i_param.cop_check_margin = st->cop_check_margin;
  for (size_t i = 0; i < st->cp_check_margin.size(); i++) {
    i_param.cp_check_margin[i] = st->cp_check_margin[i];
  }
  for (size_t i = 0; i < st->tilt_margin.size(); i++) {
    i_param.tilt_margin[i] = st->tilt_margin[i];
  }
  i_param.contact_decision_threshold = st->contact_decision_threshold;
  i_param.is_estop_while_walking = st->is_estop_while_walking;
  switch(st->control_mode) {
  case Stabilizer::MODE_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_STIDLE; break;
  case Stabilizer::MODE_AIR: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_AIR; break;
  case Stabilizer::MODE_ST: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_ST; break;
  case Stabilizer::MODE_SYNC_TO_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_STIDLE; break;
  case Stabilizer::MODE_SYNC_TO_AIR: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_AIR; break;
  default: break;
  }
  i_param.emergency_check_mode = st->emergency_check_mode;
  i_param.end_effector_list.length(st->stikp.size());
  i_param.use_limb_stretch_avoidance = st->use_limb_stretch_avoidance;
  i_param.use_zmp_truncation = st->use_zmp_truncation;
  i_param.use_force_sensor = st->use_force_sensor;
  i_param.use_footguided_stabilizer = st->use_footguided_stabilizer;
  i_param.footguided_balance_time_const = st->footguided_balance_time_const;
  i_param.limb_stretch_avoidance_time_const = st->limb_stretch_avoidance_time_const;
  i_param.limb_length_margin.length(st->stikp.size());
  i_param.detection_time_to_air = st->detection_count_to_air * m_dt;
  for (size_t i = 0; i < 2; i++) {
    i_param.limb_stretch_avoidance_vlimit[i] = st->limb_stretch_avoidance_vlimit[i];
    i_param.root_rot_compensation_limit[i] = st->root_rot_compensation_limit[i];
  }
  for (size_t i = 0; i < st->stikp.size(); i++) {
    const rats::coordinates cur_ee = rats::coordinates(st->stikp.at(i).localp, st->stikp.at(i).localR);
    OpenHRP::AutoBalancerService::Footstep ret_ee;
    // position
    memcpy(ret_ee.pos, cur_ee.pos.data(), sizeof(double)*3);
    // rotation
    Eigen::Quaternion<double> qt(cur_ee.rot);
    ret_ee.rot[0] = qt.w();
    ret_ee.rot[1] = qt.x();
    ret_ee.rot[2] = qt.y();
    ret_ee.rot[3] = qt.z();
    // name
    ret_ee.leg = st->stikp.at(i).ee_name.c_str();
    // set
    i_param.end_effector_list[i] = ret_ee;
    i_param.limb_length_margin[i] = st->stikp[i].limb_length_margin;
  }
  i_param.ik_limb_parameters.length(st->jpe_v.size());
  for (size_t i = 0; i < st->jpe_v.size(); i++) {
    OpenHRP::AutoBalancerService::IKLimbParameters& ilp = i_param.ik_limb_parameters[i];
    ilp.ik_optional_weight_vector.length(st->jpe_v[i]->numJoints());
    std::vector<double> ov;
    ov.resize(st->jpe_v[i]->numJoints());
    st->jpe_v[i]->getOptionalWeightVector(ov);
    for (size_t j = 0; j < st->jpe_v[i]->numJoints(); j++) {
      ilp.ik_optional_weight_vector[j] = ov[j];
    }
    ilp.sr_gain = st->jpe_v[i]->getSRGain();
    ilp.avoid_gain = st->stikp[i].avoid_gain;
    ilp.reference_gain = st->stikp[i].reference_gain;
    ilp.manipulability_limit = st->jpe_v[i]->getManipulabilityLimit();
    ilp.ik_loop_count = st->stikp[i].ik_loop_count; // size_t -> unsigned short, value may change, but ik_loop_count is small value and value not change
  }
  i_param.swing2landing_transition_time = st->swing2landing_transition_time;
  i_param.landing_phase_time = st->landing_phase_time;
  i_param.landing2support_transition_time = st->landing2support_transition_time;
  i_param.support_phase_min_time = st->support_phase_min_time;
  i_param.support2swing_transition_time = st->support2swing_transition_time;
  i_param.joint_control_mode = st->joint_control_mode;
  i_param.joint_servo_control_parameters.length(st->stikp.size());
  for (size_t i = 0; i < st->stikp.size(); i++) {
      OpenHRP::AutoBalancerService::JointServoControlParameter& jscp = i_param.joint_servo_control_parameters[i];
      jscp.support_pgain.length(st->stikp[i].support_pgain.size());
      jscp.support_dgain.length(st->stikp[i].support_dgain.size());
      jscp.landing_pgain.length(st->stikp[i].landing_pgain.size());
      jscp.landing_dgain.length(st->stikp[i].landing_dgain.size());
      jscp.swing_pgain.length(st->stikp[i].swing_pgain.size());
      jscp.swing_dgain.length(st->stikp[i].swing_dgain.size());
      for (size_t j=0; j < st->stikp[i].support_pgain.size(); j++) {
          jscp.support_pgain[j] = st->stikp[i].support_pgain(j);
          jscp.support_dgain[j] = st->stikp[i].support_dgain(j);
          jscp.landing_pgain[j] = st->stikp[i].landing_pgain(j);
          jscp.landing_dgain[j] = st->stikp[i].landing_dgain(j);
          jscp.swing_pgain[j] = st->stikp[i].swing_pgain(j);
          jscp.swing_dgain[j] = st->stikp[i].swing_dgain(j);
      }
  }
}

void AutoBalancer::copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs)
{
  memcpy(out_fs.pos, in_fs.pos.data(), sizeof(double)*3);
  Eigen::Quaternion<double> qt(in_fs.rot);
  out_fs.rot[0] = qt.w();
  out_fs.rot[1] = qt.x();
  out_fs.rot[2] = qt.y();
  out_fs.rot[3] = qt.z();
};

bool AutoBalancer::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param)
{
  copyRatscoords2Footstep(i_param.support_leg_coords, gg->get_support_leg_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_coords, gg->get_swing_leg_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_src_coords, gg->get_swing_leg_src_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_dst_coords, gg->get_swing_leg_dst_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.dst_foot_midcoords, gg->get_dst_foot_midcoords());
  if (gg->get_support_leg_names().front() == "rleg") {
    i_param.support_leg = OpenHRP::AutoBalancerService::RLEG;
  } else {
    i_param.support_leg = OpenHRP::AutoBalancerService::LLEG;
  }
  switch ( gg->get_current_support_states().front() ) {
  case BOTH: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::BOTH; break;
  case RLEG: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::RLEG; break;
  case LLEG: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::LLEG; break;
  default: break;
  }
  return true;
};

bool AutoBalancer::adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep)
{
  std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
            << "] adjustFootSteps" << std::endl;
  if (control_mode == MODE_ABC && !gg_is_walking && adjust_footstep_interpolator->isEmpty()) {
      Guard guard(m_mutex);
      //
      hrp::Vector3 eepos, org_mid_rpy, target_mid_rpy;
      hrp::Matrix33 eerot, tmprot;
      coordinates org_mid_coords, target_mid_coords;
      // Get org coords
      ikp["rleg"].adjust_interpolation_org_p0 = ikp["rleg"].target_p0;
      ikp["lleg"].adjust_interpolation_org_p0 = ikp["lleg"].target_p0;
      ikp["rleg"].adjust_interpolation_org_r0 = ikp["rleg"].target_r0;
      ikp["lleg"].adjust_interpolation_org_r0 = ikp["lleg"].target_r0;
      mid_coords(org_mid_coords, 0.5,
                 coordinates(ikp["rleg"].adjust_interpolation_org_p0, ikp["rleg"].adjust_interpolation_org_r0),
                 coordinates(ikp["lleg"].adjust_interpolation_org_p0, ikp["lleg"].adjust_interpolation_org_r0));
      org_mid_rpy = hrp::rpyFromRot(org_mid_coords.rot);
      // Get target coords
      //   Input : ee coords
      //   Output : link coords
      memcpy(eepos.data(), rfootstep.pos, sizeof(double)*3);
      eerot = (Eigen::Quaternion<double>(rfootstep.rot[0], rfootstep.rot[1], rfootstep.rot[2], rfootstep.rot[3])).normalized().toRotationMatrix(); // rtc:
      ikp["rleg"].adjust_interpolation_target_r0 = eerot;
      ikp["rleg"].adjust_interpolation_target_p0 = eepos;
      memcpy(eepos.data(), lfootstep.pos, sizeof(double)*3);
      eerot = (Eigen::Quaternion<double>(lfootstep.rot[0], lfootstep.rot[1], lfootstep.rot[2], lfootstep.rot[3])).normalized().toRotationMatrix(); // rtc:
      ikp["lleg"].adjust_interpolation_target_r0 = eerot;
      ikp["lleg"].adjust_interpolation_target_p0 = eepos;
      mid_coords(target_mid_coords, 0.5,
                 coordinates(ikp["rleg"].adjust_interpolation_target_p0, ikp["rleg"].adjust_interpolation_target_r0),
                 coordinates(ikp["lleg"].adjust_interpolation_target_p0, ikp["lleg"].adjust_interpolation_target_r0));
      coordinates rtrans, ltrans;
      target_mid_coords.transformation(rtrans, coordinates(ikp["rleg"].adjust_interpolation_target_p0, ikp["rleg"].adjust_interpolation_target_r0));
      target_mid_coords.transformation(ltrans, coordinates(ikp["lleg"].adjust_interpolation_target_p0, ikp["lleg"].adjust_interpolation_target_r0));
      target_mid_rpy = hrp::rpyFromRot(target_mid_coords.rot);
      // Fix target pos => org pos, target yaw => org yaw
      target_mid_rpy(2) = org_mid_rpy(2);
      target_mid_coords.rot = hrp::rotFromRpy(target_mid_rpy);
      target_mid_coords.pos = org_mid_coords.pos;
      // Calculate rleg and lleg coords
      coordinates tmpc;
      tmpc = target_mid_coords;
      tmpc.transform(rtrans);
      ikp["rleg"].adjust_interpolation_target_p0 = tmpc.pos;
      ikp["rleg"].adjust_interpolation_target_r0 = tmpc.rot;
      tmpc = target_mid_coords;
      tmpc.transform(ltrans);
      ikp["lleg"].adjust_interpolation_target_p0 = tmpc.pos;
      ikp["lleg"].adjust_interpolation_target_r0 = tmpc.rot;
      // Set interpolator
      adjust_footstep_interpolator->clear();
      double tmp = 0.0;
      adjust_footstep_interpolator->set(&tmp);
      tmp = 1.0;
      adjust_footstep_interpolator->setGoal(&tmp, adjust_footstep_transition_time, true);
  }
  while (!adjust_footstep_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  return true;
};

bool AutoBalancer::getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx)
{
    std::cerr << "[" << m_profile.instance_name << "] getRemainingFootstepSequence" << std::endl;
    o_footstep = new OpenHRP::AutoBalancerService::FootstepSequence;
    if (gg_is_walking) {
        std::vector< std::vector<step_node> > fsnl = gg->get_remaining_footstep_nodes_list();
        o_current_fs_idx = gg->get_footstep_index();
        o_footstep->length(fsnl.size());
        for (size_t i = 0; i < fsnl.size(); i++) {
            o_footstep[i].leg = (fsnl[i].front().l_r==RLEG?"rleg":"lleg");
            copyRatscoords2Footstep(o_footstep[i], fsnl[i].front().worldcoords);
        }
    }
    return true;
};

bool AutoBalancer::getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep)
{
    std::cerr << "[" << m_profile.instance_name << "] getGoPosFootstepsSequence" << std::endl;
    o_footstep = new OpenHRP::AutoBalancerService::FootstepsSequence;
    if (gg_is_walking) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot call getGoPosFootstepsSequence in walking" << std::endl;
        return false;
    } else {
        gg->set_all_limbs(leg_names);
        std::vector< std::vector<step_node> > new_footstep_nodes_list;
        coordinates start_ref_coords;
        std::vector<coordinates> initial_support_legs_coords;
        std::vector<leg_type> initial_support_legs;
        bool is_valid_gait_type = calc_inital_support_legs(y, initial_support_legs_coords, initial_support_legs, start_ref_coords);
        if (is_valid_gait_type == false) return false;
        /* go_pos_param_2_footstep_nodes_list_core is const member function  */
        gg->go_pos_param_2_footstep_nodes_list_core (x, y, th,
                                                     initial_support_legs_coords, start_ref_coords, initial_support_legs,
                                                     new_footstep_nodes_list, true, 0);
        o_footstep->length(new_footstep_nodes_list.size());
        for (size_t i = 0; i < new_footstep_nodes_list.size(); i++) {
            o_footstep[i].fs.length(new_footstep_nodes_list.at(i).size());
            for (size_t j = 0; j < new_footstep_nodes_list.at(i).size(); j++) {
                leg_type tmp_leg_type = new_footstep_nodes_list.at(i).at(j).l_r;
                o_footstep[i].fs[j].leg = ((tmp_leg_type == RLEG) ? "rleg":
                                           (tmp_leg_type == LLEG) ? "lleg":
                                           (tmp_leg_type == RARM) ? "rarm":
                                           "larm");
                copyRatscoords2Footstep(o_footstep[i].fs[j], new_footstep_nodes_list.at(i).at(j).worldcoords);
            }
        }
        return true;
    }
};

void AutoBalancer::static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height)
{
  hrp::Vector3 target_sbp = hrp::Vector3(0, 0, 0);
  hrp::Vector3 tmpcog;
  if (!gg_is_walking || !use_act_states) {
    tmpcog = m_robot->calcCM();
    dif_ref_act_cog = hrp::Vector3::Zero();
  } else {
    tmpcog = st->ref_foot_origin_pos + st->ref_foot_origin_rot * st->act_cog;
    dif_ref_act_cog = m_robot->calcCM() - tmpcog;
  }
  if ( use_force == MODE_NO_FORCE ) {
    tmp_input_sbp = tmpcog + sbp_cog_offset;
  } else {
    calc_static_balance_point_from_forces(target_sbp, tmpcog, ref_com_height);
    tmp_input_sbp = target_sbp - sbp_offset;
    sbp_cog_offset = tmp_input_sbp - tmpcog;
  }
};

void AutoBalancer::calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height)
{
  hrp::Vector3 denom, nume;
  /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
  double mass = m_robot->totalMass();
  double mg = mass * gg->get_gravitational_acceleration();
  hrp::Vector3 total_sensor_ref_force = hrp::Vector3::Zero();
  for (size_t i = 0; i < ref_forces.size(); i++) {
      total_sensor_ref_force += ref_forces[i];
  }
  hrp::Vector3 total_nosensor_ref_force = mg * hrp::Vector3::UnitZ() - total_sensor_ref_force; // total ref force at the point without sensors, such as torso
  hrp::Vector3 tmp_ext_moment = fix_leg_coords2.pos.cross(total_nosensor_ref_force) + fix_leg_coords2.rot * hrp::Vector3(m_refFootOriginExtMoment.data.x, m_refFootOriginExtMoment.data.y, m_refFootOriginExtMoment.data.z);
  // For MODE_REF_FORCE_RFU_EXT_MOMENT, store previous root position to calculate influence from tmp_ext_moment while walking (basically, root link moves while walking).
  //   Calculate values via fix_leg_coords2 relative/world values.
  static hrp::Vector3 prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p-fix_leg_coords2.pos);
  //   If not is_hold_value (not hold value), update prev_additional_force_applied_pos
  if ( !m_refFootOriginExtMomentIsHoldValue.data ) {
      prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p-fix_leg_coords2.pos);
  }
  hrp::Vector3 tmp_prev_additional_force_applied_pos = fix_leg_coords2.rot * prev_additional_force_applied_pos + fix_leg_coords2.pos;
  // Calculate SBP
  for (size_t j = 0; j < 2; j++) {
    nume(j) = mg * tmpcog(j);
    denom(j) = mg;
    if ( use_force == MODE_REF_FORCE_RFU_EXT_MOMENT ) {
        //nume(j) += (j==0 ? tmp_ext_moment(1):-tmp_ext_moment(0));
        nume(j) += (tmp_prev_additional_force_applied_pos(j)-additional_force_applied_link->p(j))*total_nosensor_ref_force(2) + (j==0 ? tmp_ext_moment(1):-tmp_ext_moment(0));
        denom(j) -= total_nosensor_ref_force(2);
    } else {
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            // Check leg_names. leg_names is assumed to be support limb for locomotion, cannot be used for manipulation. If it->first is not included in leg_names, use it for manipulation and static balance point calculation.
            if (std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()) {
                size_t idx = contact_states_index_map[it->first];
                // Force applied point is assumed as end effector
                hrp::Vector3 fpos = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
                nume(j) += ( (fpos(2) - ref_com_height) * ref_forces[idx](j) - fpos(j) * ref_forces[idx](2) );
                nume(j) += (j==0 ? ref_moments[idx](1):-ref_moments[idx](0));
                denom(j) -= ref_forces[idx](2);
            }
        }
        if ( use_force == MODE_REF_FORCE_WITH_FOOT ) {
            hrp::Vector3 fpos(additional_force_applied_link->p+additional_force_applied_point_offset);
            nume(j) += ( (fpos(2) - ref_com_height) * total_nosensor_ref_force(j) - fpos(j) * total_nosensor_ref_force(2) );
            denom(j) -= total_nosensor_ref_force(2);
        }
    }
    sb_point(j) = nume(j) / denom(j);
  }
  sb_point(2) = ref_com_height;
};

#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

hrp::Vector3 AutoBalancer::calc_vel_from_hand_error (const coordinates& tmp_fix_coords)
{
  if (graspless_manip_mode) {
    hrp::Vector3 dp,dr;
    coordinates ref_hand_coords(gg->get_dst_foot_midcoords()), act_hand_coords;
    ref_hand_coords.transform(graspless_manip_reference_trans_coords); // desired arm coords
    hrp::Vector3 foot_pos(gg->get_dst_foot_midcoords().pos);
    if ( graspless_manip_arm == "arms" ) {
        hrp::Vector3 rarm_pos = ikp["rarm"].target_p0;
        hrp::Vector3 larm_pos = ikp["larm"].target_p0;
        act_hand_coords.pos = (rarm_pos+larm_pos)/2.0;
        hrp::Vector3 act_y = larm_pos-rarm_pos;
        act_y(2) = 0;
        act_y.normalize();
        hrp::Vector3 ref_y(ref_hand_coords.rot * hrp::Vector3::UnitY());
        ref_y(2) = 0;
        ref_y.normalize();
        dr = hrp::Vector3(0,0,(hrp::Vector3(ref_y.cross(act_y))(2) > 0 ? 1.0 : -1.0) * std::acos(ref_y.dot(act_y))); // fix for rotation
    } else {
      ABCIKparam& tmpikp = ikp[graspless_manip_arm];
      act_hand_coords = rats::coordinates(tmpikp.target_p0,
                                          tmpikp.target_r0);
      rats::difference_rotation(dr, ref_hand_coords.rot, act_hand_coords.rot);
      dr(0) = 0; dr(1) = 0;
    }
    dp = act_hand_coords.pos - ref_hand_coords.pos
        + dr.cross(hrp::Vector3(foot_pos - act_hand_coords.pos));
    dp(2) = 0;
    hrp::Matrix33 foot_mt(gg->get_dst_foot_midcoords().rot.transpose());
    //alias(dp) = foot_mt * dp;
    hrp::Vector3 dp2 = foot_mt * dp;
    //alias(dr) = foot_mt * dr;
    return hrp::Vector3(graspless_manip_p_gain[0] * dp2(0)/gg->get_default_step_time(),
                        graspless_manip_p_gain[1] * dp2(1)/gg->get_default_step_time(),
                        graspless_manip_p_gain[2] * rad2deg(dr(2))/gg->get_default_step_time());
  } else {
    return hrp::Vector3::Zero();
  }
};

bool AutoBalancer::calc_inital_support_legs(const double& y, std::vector<coordinates>& initial_support_legs_coords, std::vector<leg_type>& initial_support_legs, coordinates& start_ref_coords) {
    switch(gait_type) {
    case BIPED:
        initial_support_legs_coords.assign (1,
                                            y > 0 ?
                                            coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0)
                                            : coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0));
        initial_support_legs.assign (1, y > 0 ? RLEG : LLEG);
        break;
    case TROT:
        initial_support_legs_coords = (y > 0 ?
                                       boost::assign::list_of(coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0))(coordinates(ikp["larm"].target_p0, ikp["larm"].target_r0))
                                       : boost::assign::list_of(coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0))(coordinates(ikp["rarm"].target_p0, ikp["rarm"].target_r0))).convert_to_container < std::vector<coordinates> > ();
        initial_support_legs = (y > 0 ? boost::assign::list_of(RLEG)(LARM) : boost::assign::list_of(LLEG)(RARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case PACE:
        initial_support_legs_coords = (y > 0 ?
                                       boost::assign::list_of(coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0))(coordinates(ikp["rarm"].target_p0, ikp["rarm"].target_r0))
                                       : boost::assign::list_of(coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0))(coordinates(ikp["larm"].target_p0, ikp["larm"].target_r0))).convert_to_container < std::vector<coordinates> > ();
        initial_support_legs = (y > 0 ? boost::assign::list_of(RLEG)(RARM) : boost::assign::list_of(LLEG)(LARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case CRAWL:
        std::cerr << "[" << m_profile.instance_name << "] crawl walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    case GALLOP:
        /* at least one leg shoud be in contact */
        std::cerr << "[" << m_profile.instance_name << "] gallop walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    default: break;
    }
    start_ref_coords.pos = (ikp["rleg"].target_p0+ikp["lleg"].target_p0)*0.5;
    mid_rot(start_ref_coords.rot, 0.5, ikp["rleg"].target_r0, ikp["lleg"].target_r0);
    return true;
};

// TODO : Use same code as ZMPDistributor.h
// Solve A * x = b => x = W A^T (A W A^T)-1 b
// => x = W^{1/2} Pinv(A W^{1/2}) b
// Copied from ZMPDistributor.h
void calcWeightedLinearEquation(hrp::dvector& ret, const hrp::dmatrix& A, const hrp::dmatrix& W, const hrp::dvector& b)
{
    hrp::dmatrix W2 = hrp::dmatrix::Zero(W.rows(), W.cols());
    for (size_t i = 0; i < W.rows(); i++) W2(i,i) = std::sqrt(W(i,i));
    hrp::dmatrix Aw = A*W2;
    hrp::dmatrix Aw_inv = hrp::dmatrix::Zero(A.cols(), A.rows());
    hrp::calcPseudoInverse(Aw, Aw_inv);
    ret = W2 * Aw_inv * b;
    //ret = W2 * Aw.colPivHouseholderQr().solve(b);
};

void AutoBalancer::distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp)
{
    // apply inverse system
    // TODO : fix 0.055 (zmp delay)
    hrp::Vector3 tmp_ref_zmp = _ref_zmp + 0.055 * (_ref_zmp - prev_ref_zmp) / m_dt;

    std::vector<hrp::Vector3> cop_pos;
    std::vector<double> limb_gains;
    for (size_t i = 0 ; i < leg_names.size(); i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        cop_pos.push_back(tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localR * default_zmp_offsets[i]);
        limb_gains.push_back(m_contactStates.data[contact_states_index_map[leg_names[i]]] ? 1.0 : 0.0);
    }
    size_t ee_num = leg_names.size();
    size_t state_dim = 6*ee_num;
    size_t total_wrench_dim = 5;
    // size_t total_fz = m_robot->totalMass() * gg->get_gravitational_acceleration();
    size_t total_fz = m_ref_force[0].data[2]+m_ref_force[1].data[2];
    //size_t total_wrench_dim = 3;
    hrp::dmatrix Wmat = hrp::dmatrix::Identity(state_dim/2, state_dim/2);
    hrp::dmatrix Gmat = hrp::dmatrix::Zero(total_wrench_dim, state_dim/2);
    // Set Gmat
    //   Fill Fz
    for (size_t j = 0; j < ee_num; j++) {
        if (total_wrench_dim == 3) {
            Gmat(0,3*j+2) = 1.0;
        } else {
            for (size_t k = 0; k < 3; k++) Gmat(k,3*j+k) = 1.0;
        }
    }
    //   Fill Nx and Ny
    for (size_t i = 0; i < total_wrench_dim; i++) {
        for (size_t j = 0; j < ee_num; j++) {
            if ( i == total_wrench_dim-2 ) { // Nx
                Gmat(i,3*j+1) = -(cop_pos[j](2) - tmp_ref_zmp(2));
                Gmat(i,3*j+2) = (cop_pos[j](1) - tmp_ref_zmp(1));
            } else if ( i == total_wrench_dim-1 ) { // Ny
                Gmat(i,3*j) = (cop_pos[j](2) - tmp_ref_zmp(2));
                Gmat(i,3*j+2) = -(cop_pos[j](0) - tmp_ref_zmp(0));
            }
        }
    }
    // Set Wmat
    for (size_t j = 0; j < ee_num; j++) {
        for (size_t i = 0; i < 3; i++) {
            if (ee_num == 2)
                Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j] * (i==2? 1.0 : 0.01);
            else
                Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j];
        }
    }
    // Ret is wrench around cop_pos
    //   f_cop = f_ee
    //   n_ee = (cop_pos - ee_pos) x f_cop + n_cop
    hrp::dvector ret(state_dim/2);
    hrp::dvector total_wrench = hrp::dvector::Zero(total_wrench_dim);
    total_wrench(total_wrench_dim-5) = m_ref_force[0].data[0]+m_ref_force[1].data[0];
    total_wrench(total_wrench_dim-4) = m_ref_force[0].data[1]+m_ref_force[1].data[1];
    total_wrench(total_wrench_dim-3) = total_fz;
    calcWeightedLinearEquation(ret, Gmat, Wmat, total_wrench);
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] distributeReferenceZMPToWrenches" << std::endl;
    }
    for (size_t i = 0 ; i < leg_names.size(); i++) {
        size_t fidx = contact_states_index_map[leg_names[i]];
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        hrp::Vector3 f_ee(ret(3*i), ret(3*i+1), ret(3*i+2));
        //hrp::Vector3 tmp_ee_pos = tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localPos;
        hrp::Vector3 tmp_ee_pos = tmpikp.target_p0;
        hrp::Vector3 n_ee = (cop_pos[i]-tmp_ee_pos).cross(f_ee); // n_cop = 0
        m_force[fidx].data[0] = f_ee(0);
        m_force[fidx].data[1] = f_ee(1);
        m_force[fidx].data[2] = f_ee(2);
        m_force[fidx].data[3] = n_ee(0);
        m_force[fidx].data[4] = n_ee(1);
        m_force[fidx].data[5] = n_ee(2);
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "]   "
                      << "ref_force  [" << leg_names[i] << "] " << f_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                      << "ref_moment [" << leg_names[i] << "] " << n_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        }
    }
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "]   Gmat = " << Gmat.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   total_wrench = " << total_wrench.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        hrp::dvector tmp(total_wrench.size());
        tmp = Gmat*ret;
        std::cerr << "[" << m_profile.instance_name << "]   Gmat*ret = " << tmp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   (Gmat*ret-total_wrench) = " << (tmp-total_wrench).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   ret = " << ret.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   Wmat(diag) = [";
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t i = 0; i < 3; i++) {
                std::cerr << Wmat(i+j*3, i+j*3) << " ";
            }
        }
        std::cerr << "]" << std::endl;
    }
};

std::string AutoBalancer::getUseForceModeString ()
{
    switch (use_force) {
    case OpenHRP::AutoBalancerService::MODE_NO_FORCE:
        return "MODE_NO_FORCE";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE:
        return "MODE_REF_FORCE";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT:
        return "MODE_REF_FORCE_WITH_FOOT";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT:
        return "MODE_REF_FORCE_RFU_EXT_MOMENT";
    default:
        return "";
    }
};

//
extern "C"
{

    void AutoBalancerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(autobalancer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<AutoBalancer>,
                                 RTC::Delete<AutoBalancer>);
    }

};
