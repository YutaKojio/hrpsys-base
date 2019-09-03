// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testLimbTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <fstream>
#include "../LimbTrajectoryGenerator.h"

namespace
{

std::vector<hrp::Vector3> rleg_contact_points;
std::vector<hrp::Vector3> lleg_contact_points;

void forwardFootstep(std::vector<hrp::ConstraintsWithCount>& constraints_list,
                     const double stride, const size_t step_limb,
                     const size_t start_count, const size_t step_count)
{
    constraints_list.reserve(constraints_list.size() + 2);

    const std::vector<hrp::LinkConstraint>& cur_constraints = constraints_list.back().constraints;

    {
        // Swinging phase
        hrp::ConstraintsWithCount constraint_count;
        constraint_count.start_count = start_count;
        constraint_count.constraints = cur_constraints;
        constraint_count.constraints[step_limb].setConstraintType(hrp::LinkConstraint::FLOAT);

        constraints_list.push_back(constraint_count);
    }

    {
        // Double support phase
        hrp::ConstraintsWithCount constraint_count;
        constraint_count.start_count = start_count + step_count;
        constraint_count.constraints = cur_constraints;
        constraint_count.constraints[step_limb].targetPos() += hrp::Vector3(stride, 0, 0);

        constraints_list.push_back(constraint_count);
    }
}

void addConstraints(std::vector<hrp::ConstraintsWithCount>& constraints_list, const size_t start_count,
                    const int leg_mask = 0b11, const double rleg_weight = 1.0, const double lleg_weight = 1.0)
{
    if (!(leg_mask & 0b11)) return;

    hrp::ConstraintsWithCount constraint_count;
    constraint_count.start_count = start_count;

    {
        hrp::LinkConstraint rleg_constraint(0);
        for (const auto& point : rleg_contact_points) rleg_constraint.addLinkContactPoint(point);
        rleg_constraint.calcLinkRepresentativePoint();
        rleg_constraint.setWeight(rleg_weight);
        if (!(leg_mask & 0b10)) {
            rleg_constraint.setConstraintType(hrp::LinkConstraint::FLOAT);
        }
        constraint_count.constraints.push_back(rleg_constraint);
    }

    {
        hrp::LinkConstraint lleg_constraint(1);
        for (const auto& point : lleg_contact_points) lleg_constraint.addLinkContactPoint(point);
        lleg_constraint.calcLinkRepresentativePoint();
        lleg_constraint.setWeight(lleg_weight);
        if (!(leg_mask & 0b01)) {
            lleg_constraint.setConstraintType(hrp::LinkConstraint::FLOAT);
        }
        constraint_count.constraints.push_back(lleg_constraint);
    }

    constraints_list.push_back(constraint_count);
}

}

int main(int argc, char **argv)
{
    // same as testRefZMP
    constexpr double dt = 0.002;

    bool use_gnuplot = true;
    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    rleg_contact_points.emplace_back(0.15, 0.05, 0);
    rleg_contact_points.emplace_back(0.15, 0.15, 0);
    rleg_contact_points.emplace_back(-0.15, 0.15, 0);
    rleg_contact_points.emplace_back(-0.15, 0.05, 0);

    lleg_contact_points.emplace_back(0.15, -0.05, 0);
    lleg_contact_points.emplace_back(0.15, -0.15, 0);
    lleg_contact_points.emplace_back(-0.15, -0.15, 0);
    lleg_contact_points.emplace_back(-0.15, -0.05, 0);

    std::vector<hrp::ConstraintsWithCount> constraints_list;

    size_t start_count = 0;
    constexpr size_t SUPPORT_COUNT = static_cast<size_t>(1.0 / dt);
    constexpr size_t STEP_COUNT = static_cast<size_t>(1.0 / dt);

    addConstraints(constraints_list, start_count);

    start_count += SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 0, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 1, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 0, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 1, start_count, STEP_COUNT);

    size_t next_turning_count = 0;
    int index = -1;
    std::vector<hrp::LimbTrajectoryGenerator> ltg(2); // TODO: 使い方変わるかも
    ltg[0].setPos(constraints_list[0].constraints[0].getLinkRepresentativePoint());
    ltg[0].setPos(constraints_list[0].constraints[1].getLinkRepresentativePoint());
    constexpr double FOOTSTEP_HEIGHT = 0.15;
    const size_t FINISH = start_count + STEP_COUNT + SUPPORT_COUNT;

    const std::string fname("/tmp/testLimbTrajectory.dat");
    std::ofstream ofs(fname);

    for (size_t count = 0; count < FINISH; ++count) {
        if (count == next_turning_count) {
            ++index;
            if (index + 1 < constraints_list.size()) next_turning_count = constraints_list[index + 1].start_count;

            for (size_t i = 0; i < constraints_list[index].constraints.size(); ++i) {
                if (constraints_list[index].constraints[i].getConstraintType() == hrp::LinkConstraint::FLOAT) {
                    ltg[i].calcViaPoints(hrp::LimbTrajectoryGenerator::CYCLOIDDELAY, constraints_list,
                                         constraints_list[index].constraints[i].getLinkId(),
                                         constraints_list[index].start_count,
                                         FOOTSTEP_HEIGHT);

                    const std::vector<hrp::ViaPoint>& via_points = ltg[i].getViaPoints();
                    // TODO: debug
                    std::cerr << "size: " << via_points.size() << ", via point: \n";
                    for (size_t j = 0; j < via_points.size(); ++j) {
                        std::cerr << via_points[j].point.transpose() << std::endl;
                    }
                }
            }
        }

        for (size_t i = 0; i < ltg.size(); ++i) {
            ltg[i].calcTrajectory(count, dt);
            const hrp::Vector3& pos = ltg[i].getPos();
            ofs << pos[0] << " " << pos[1] << " " << pos[2] << " ";
        }
        ofs << std::endl;
    }

    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 2, 1\n");
        fprintf(gp, "set title \"Pos\"\n");
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Right\"\n", fname.c_str());
        fprintf(gp, "plot \"%s\" using 4:6 with lines title \"Left\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }


    return 0;
}