// -*- C++ -*-
/*!
 * @file  ZMPDistributor.h
 * @brief ZMP distribution
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ZMP_DISTRIBUTOR_H
#define ZMP_DISTRIBUTOR_H

#include <hrpModel/Body.h>
#include <iostream>
#include "../ImpedanceController/JointPathEx.h"

class FootSupportPolygon
{
    std::vector<std::vector<Eigen::Vector2d> > foot_vertices; // RLEG, LLEG
public:
    FootSupportPolygon () {};
    bool inside_foot (size_t idx)
    {
        return true;
    };
    Eigen::Vector2d get_foot_vertex (const size_t foot_idx, const size_t vtx_idx)
    {
        return foot_vertices[foot_idx][vtx_idx];
    };
    void set_vertices (const std::vector<std::vector<Eigen::Vector2d> >& vs) { foot_vertices = vs; };
    void get_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { vs = foot_vertices; };
    void print_vertices (const std::string& str)
    {
        for (size_t i = 0; i < foot_vertices.size(); i++) {
            std::cerr << "[" << str << "]   vs = ";
            for (size_t j = 0; j < foot_vertices[i].size(); j++) {
                std::cerr << "[" << foot_vertices[i][j](0) << " " << foot_vertices[i][j](1) << "] ";
            }
            std::cerr << "[m]" << std::endl;;
        }
    }
};

//

class SimpleZMPDistributor
{
    FootSupportPolygon fs;
    double leg_inside_margin, leg_front_margin, leg_rear_margin, wrench_alpha_blending;
public:
    SimpleZMPDistributor () : wrench_alpha_blending (0.5)
    {
    };

    inline bool is_inside_foot (const hrp::Vector3& leg_pos, const bool is_lleg)
    {
        if (is_lleg) return leg_pos(1) >= -1 * leg_inside_margin;
        else return leg_pos(1) <= leg_inside_margin;
    };
    inline bool is_front_of_foot (const hrp::Vector3& leg_pos)
    {
        return leg_pos(0) >= leg_front_margin;
    };
    inline bool is_rear_of_foot (const hrp::Vector3& leg_pos)
    {
        return leg_pos(0) <= -1 * leg_rear_margin;
    };
    void print_params (const std::string& str)
    {
        std::cerr << "[" << str << "]   leg_inside_margin = " << leg_inside_margin << "[m], leg_front_margin = " << leg_front_margin << "[m], leg_rear_margin = " << leg_rear_margin << "[m]" << std::endl;
        std::cerr << "[" << str << "]   wrench_alpha_blending = " << wrench_alpha_blending << std::endl;
    }
    void print_vertices (const std::string& str)
    {
        fs.print_vertices(str);
    };
    // setter
    void set_wrench_alpha_blending (const double a) { wrench_alpha_blending = a; };
    void set_leg_front_margin (const double a) { leg_front_margin = a; };
    void set_leg_rear_margin (const double a) { leg_rear_margin = a; };
    void set_leg_inside_margin (const double a) { leg_inside_margin = a; };
    void set_vertices (const std::vector<std::vector<Eigen::Vector2d> >& vs)
    {
        fs.set_vertices(vs);
        // leg_inside_margin = fs.get_foot_vertex(0, 0)(1);
        // leg_front_margin = fs.get_foot_vertex(0, 0)(0);
        // leg_rear_margin = std::fabs(fs.get_foot_vertex(0, 3)(0));
    };
    void set_vertices_from_margin_params ()
    {
        std::vector<std::vector<Eigen::Vector2d> > vec;
        std::vector<Eigen::Vector2d> tvec;
        tvec.push_back(Eigen::Vector2d(leg_front_margin, leg_inside_margin));
        tvec.push_back(Eigen::Vector2d(leg_front_margin, -1*leg_inside_margin));
        tvec.push_back(Eigen::Vector2d(leg_rear_margin, -1*leg_inside_margin));
        tvec.push_back(Eigen::Vector2d(leg_rear_margin, leg_inside_margin));
        vec.push_back(tvec);
        vec.push_back(tvec);
        set_vertices(vec);
    };
    // getter
    double get_wrench_alpha_blending () { return wrench_alpha_blending; };
    double get_leg_front_margin () { return leg_front_margin; };
    double get_leg_rear_margin () { return leg_rear_margin; };
    double get_leg_inside_margin () { return leg_inside_margin; };
    void get_vertices (std::vector<std::vector<Eigen::Vector2d> >& vs) { fs.get_vertices(vs); };
    //
    double calcAlpha (const hrp::Vector3& tmprefzmp,
                      const std::vector<hrp::Vector3>& ee_pos,
                      const std::vector<hrp::Matrix33>& ee_rot)
    {
        double alpha;
        hrp::Vector3 l_local_zmp = ee_rot[1].transpose() * (tmprefzmp-ee_pos[1]);
        hrp::Vector3 r_local_zmp = ee_rot[0].transpose() * (tmprefzmp-ee_pos[0]);
        // std::cerr << "a " << l_local_zmp(0) << " " << l_local_zmp(1) << std::endl;
        // std::cerr << "b " << r_local_zmp(0) << " " << r_local_zmp(1) << std::endl;
        if ( is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) { // new_refzmp is inside lfoot
            alpha = 0.0;
        } else if ( is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) { // new_refzmp is inside rfoot
            alpha = 1.0;
        } else {
            hrp::Vector3 ledge_foot;
            hrp::Vector3 redge_foot;
            // lleg
            if (is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(leg_front_margin, l_local_zmp(1), 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(leg_front_margin, -1 * leg_inside_margin, 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(l_local_zmp(0), -1 * leg_inside_margin, 0.0);
            } else if (!is_inside_foot(l_local_zmp, true) && is_rear_of_foot(l_local_zmp)) {
                ledge_foot = hrp::Vector3(-1 * leg_rear_margin, -1 * leg_inside_margin, 0.0);
            } else {
                ledge_foot = hrp::Vector3(-1 * leg_rear_margin, l_local_zmp(1), 0.0);
            }
            ledge_foot = ee_rot[1] * ledge_foot + ee_pos[1];
            // rleg
            if (is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(leg_front_margin, r_local_zmp(1), 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(leg_front_margin, leg_inside_margin, 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(r_local_zmp(0), leg_inside_margin, 0.0);
            } else if (!is_inside_foot(r_local_zmp, false) && is_rear_of_foot(r_local_zmp)) {
                redge_foot = hrp::Vector3(-1 * leg_rear_margin, leg_inside_margin, 0.0);
            } else {
                redge_foot = hrp::Vector3(-1 * leg_rear_margin, r_local_zmp(1), 0.0);
            }
            redge_foot = ee_rot[0] * redge_foot + ee_pos[0];
            // calc alpha
            hrp::Vector3 difp = redge_foot - ledge_foot;
            alpha = difp.dot(tmprefzmp-ledge_foot)/difp.squaredNorm();
        }
        return alpha;
    };

    void distributeZMPToForceMoments (hrp::Vector3* ref_foot_force, hrp::Vector3* ref_foot_moment,
                                      const std::vector<hrp::Vector3>& ee_pos,
                                      const std::vector<hrp::Vector3>& cop_pos,
                                      const std::vector<hrp::Matrix33>& ee_rot,
                                      const hrp::Vector3& new_refzmp, const hrp::Vector3& ref_zmp,
                                      const double total_fz, const bool printp = true, const std::string& print_str = "")
    {
        //double fz_alpha =  calcAlpha(hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos), ee_pos, ee_rot);
        double fz_alpha =  calcAlpha(ref_zmp, ee_pos, ee_rot);
        double alpha = calcAlpha(new_refzmp, ee_pos, ee_rot);
        fz_alpha = wrench_alpha_blending * fz_alpha + (1-wrench_alpha_blending) * alpha;
        ref_foot_force[0] = hrp::Vector3(0,0, fz_alpha * total_fz);
        ref_foot_force[1] = hrp::Vector3(0,0, (1-fz_alpha) * total_fz);

        hrp::Vector3 tau_0 = hrp::Vector3::Zero();
#if 0
        double gamma = fz_alpha;
        double beta = m_wrenches[1].data[2] / ( m_wrenches[0].data[2] + m_wrenches[1].data[2] );
        beta = isnan(beta) ? 0 : beta;
        double steepness = 8; // change ration from alpha to beta (steepness >= 4)
        double r = - 1/(1+exp(-6*steepness*(gamma-1+1/steepness))) + 1/(1+exp(-6*steepness*(gamma-1/steepness)));
        fz_alpha = r * beta + ( 1 - r ) * gamma;
        //       alpha = fz_alpha;

        ref_foot_force[0] = hrp::Vector3(0,0, fz_alpha * total_fz);
        ref_foot_force[1] = hrp::Vector3(0,0, (1-fz_alpha) * total_fz);
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "] slip st parameters" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   " << ref_foot_force[1](2) << " " << ref_foot_force[0](2) << "   a:"<< steepness << " beta:" << beta << " gamma:" << gamma << " r:" << r << " fz_alpha:" << fz_alpha <<  " alpha:" << alpha << std::endl;
        }
#endif

        for (size_t i = 0; i < 2; i++) {
            tau_0 -= (cop_pos[i] - new_refzmp).cross(ref_foot_force[i]);
        }
        {
            // Foot-distribution-coords frame =>
            hrp::Vector3 foot_dist_coords_y = (cop_pos[1] - cop_pos[0]); // e_y'
            foot_dist_coords_y(2) = 0.0;
            foot_dist_coords_y.normalize();
            hrp::Vector3 foot_dist_coords_x = hrp::Vector3(foot_dist_coords_y.cross(hrp::Vector3::UnitZ())); // e_x'
            hrp::Matrix33 foot_dist_coords_rot;
            foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
            foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
            foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
            foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
            foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
            foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
            foot_dist_coords_rot(0,2) = 0;
            foot_dist_coords_rot(1,2) = 0;
            foot_dist_coords_rot(2,2) = 1;
            hrp::Vector3 tau_0_f = foot_dist_coords_rot.transpose() * tau_0; // tau_0'
            // x
            //         // right
            //         if (tau_0_f(0) > 0) ref_foot_moment[0](0) = tau_0_f(0);
            //         else ref_foot_moment[0](0) = 0;
            //         // left
            //         if (tau_0_f(0) > 0) ref_foot_moment[1](0) = 0;
            //         else ref_foot_moment[1](0) = tau_0_f(0);
            ref_foot_moment[0](0) = tau_0_f(0) * alpha;
            ref_foot_moment[1](0) = tau_0_f(0) * (1-alpha);
            // y
            ref_foot_moment[0](1) = tau_0_f(1) * alpha;
            ref_foot_moment[1](1) = tau_0_f(1) * (1-alpha);
            ref_foot_moment[0](2) = ref_foot_moment[1](2) = 0.0;
            // <= Foot-distribution-coords frame
            // Convert foot-distribution-coords frame => actual world frame
            ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
            ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
        }
#if 0
        {
            // Foot-distribution-coords frame =>
            hrp::Vector3 foot_dist_coords_y = (cop_pos[1] - cop_pos[0]); // e_y'
            foot_dist_coords_y(2) = 0.0;
            foot_dist_coords_y.normalize();
            hrp::Vector3 foot_dist_coords_x = hrp::Vector3(foot_dist_coords_y.cross(hrp::Vector3::UnitZ())); // e_x'
            hrp::Matrix33 foot_dist_coords_rot;
            foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
            foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
            foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
            foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
            foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
            foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
            foot_dist_coords_rot(0,2) = 0;
            foot_dist_coords_rot(1,2) = 0;
            foot_dist_coords_rot(2,2) = 1;
            tau_0 = hrp::Vector3::Zero();
            //
            hrp::dvector fvec(3);
            fvec(0) = total_fz;
            fvec(1) = tau_0(0);
            fvec(2) = tau_0(1);
            hrp::dmatrix Gmat(3,6);
            Gmat(0,0) = 1.0; Gmat(0,1) = 0.0; Gmat(0,2) = 0.0;
            Gmat(0,3) = 1.0; Gmat(0,4) = 0.0; Gmat(0,5) = 0.0;
            Gmat(1,0) = (cop_pos[0](1)-new_refzmp(1));
            Gmat(1,1) = 1.0;
            Gmat(1,2) = 0.0;
            Gmat(1,3) = (cop_pos[1](1)-new_refzmp(1));
            Gmat(1,4) = 1.0;
            Gmat(1,5) = 0.0;
            Gmat(2,0) = -(cop_pos[0](0)-new_refzmp(0));
            Gmat(2,1) = 0.0;
            Gmat(2,2) = 1.0;
            Gmat(2,3) = -(cop_pos[1](0)-new_refzmp(0));
            Gmat(2,4) = 0.0;
            Gmat(2,5) = 1.0;
            hrp::dmatrix Wmat(6,6);
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 6; j++) {
                    Wmat(i,j) = 0.0;
                }
            }
            double beta_r =0 , beta_l =0;
            double kk = 16.0;
            if (alpha > 0.5) beta_r = std::pow((2*alpha-1.0), kk);
            else beta_r = 0;
            if (alpha < 0.5) beta_l = std::pow((2*alpha-1.0), kk);
            else beta_l = 0;
            Wmat(0,0) = alpha;
            Wmat(1,1) = beta_r;
            Wmat(2,2) = alpha;
            Wmat(3,3) = (1-alpha);
            Wmat(5,5) = (1-alpha);
            Wmat(4,4) = beta_l;
            // if (printp) {
            //     std::cerr << Wmat << std::endl;
            // }
            hrp::dmatrix Gmat_ret(6,3);
            hrp::calcSRInverse(Gmat, Gmat_ret, 0.0, Wmat);
            hrp::dvector fmvec(6);
            fmvec = Gmat_ret* fvec;
            ref_foot_force[0] = hrp::Vector3(0,0,fmvec(0));
            ref_foot_force[1] = hrp::Vector3(0,0,fmvec(3));
            ref_foot_moment[0] = hrp::Vector3(fmvec(1),fmvec(2),0);
            ref_foot_moment[1] = hrp::Vector3(fmvec(4),fmvec(5),0);
            // <= Foot-distribution-coords frame
            // Convert foot-distribution-coords frame => actual world frame
            ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
            ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
        }
#endif

        if (printp) {
            std::cerr << "[" << print_str << "]   alpha = " << alpha << ", fz_alpha = " << fz_alpha << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "total_tau    = " << hrp::Vector3(tau_0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_force_R  = " << hrp::Vector3(ref_foot_force[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_force_L  = " << hrp::Vector3(ref_foot_force[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_moment_R = " << hrp::Vector3(ref_foot_moment[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << print_str << "]   "
                      << "ref_moment_L = " << hrp::Vector3(ref_foot_moment[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        }
    };
};

#endif // ZMP_DISTRIBUTOR_H
