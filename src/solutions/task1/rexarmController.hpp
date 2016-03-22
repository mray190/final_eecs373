#ifndef REXARM_H
#define REXARM_H

#include <cmath>
#include <iostream>
#include <queue>
#include <vector>
#include <unistd.h>

// LCM includes
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/dynamixel_command_list_t.hpp>
#include <lcmtypes/dynamixel_command_t.hpp>
#include <lcmtypes/dynamixel_status_list_t.hpp>
#include <lcmtypes/dynamixel_status_t.hpp>
#include <lcmtypes/target_t.hpp>
#include <lcmtypes/target_xy_t.hpp>
#include <lcmtypes/tic_tac_toe_turn_t.hpp>

// Header files
#include "Handler.hpp"

struct Rexarm
{
	dynamixel_command_list_t goal;
	dynamixel_status_list_t status;
	std::vector<double> dist;
};

void pick_up_and_place (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm);

void pick_up_go_home_and_place (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm);

void update_arm (Rexarm &rexarm, target_t target);

void update_theta_1 (Rexarm &rexarm, target_t target);

void update_theta_2 (Rexarm &rexarm, target_t target);

void update_theta_3 (Rexarm &rexarm, target_t target);

void update_theta_4 (Rexarm &rexarm, target_t target);

void go_home (Rexarm &rexarm);

void grab (Rexarm &rexarm);

void drop (Rexarm &rexarm);

void open (Rexarm &rexarm);

void high_torque (Rexarm &rexarm, unsigned id);

void low_torque (Rexarm &rexarm, unsigned id);

void update_status (Rexarm &rexarm, Handler &handler);

void wait (Rexarm &rexarm, Handler& handler);

/**************************************************************************
*	For fun                                                               *
**************************************************************************/

void one_hand_clap (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm);

void rexarm_shake (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm);

#endif /* REXARM_H */
