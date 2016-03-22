#include "Handler.hpp"

/**************************************************************************
*	Getters                                                               *
**************************************************************************/

dynamixel_command_list_t Handler::get_command_list() const
{
	return commandList;
}

dynamixel_status_list_t Handler::get_status_list() const
{
	return statusList;
}

target_t Handler::get_target() const
{
	return target;
}

target_xy_t Handler::get_target_xy() const
{
	return targetXY;
}

tic_tac_toe_turn_t Handler::get_our_turn() const
{
	return ourTurn;
}

tic_tac_toe_turn_t Handler::get_their_turn() const
{
	return theirTurn;
}

bool Handler::get_command_list_recieved() const
{
	return commandListRecieved;
}

bool Handler::get_status_list_recieved() const
{
	return statusListRecieved;
}

bool Handler::get_target_recieved() const
{
	return targetRecieved;
}

bool Handler::get_target_xy_recieved() const
{
	return targetXYRecieved;
}

bool Handler::get_our_recieved() const
{
	return ourTurnRecieved;
}

bool Handler::get_their_recieved() const
{
	return theirTurnRecieved;
}

/**************************************************************************
*	Public handler functions                                              *
**************************************************************************/

void Handler::handleCommandList (const lcm::ReceiveBuffer* rbuf,
						const std::string& chan,
						const dynamixel_command_list_t* msg)
{
	commandList.len = msg->len;
	commandList.commands = msg->commands;
	commandListRecieved = 1;
}

void Handler::handleStatusList (const lcm::ReceiveBuffer* rbuf,
						const std::string& chan,
						const dynamixel_status_list_t* msg)
{
	statusList.len = msg->len;
	statusList.statuses = msg->statuses;
	statusListRecieved = 1;
}

void Handler::handleTarget (const lcm::ReceiveBuffer* rbuf,
						const std::string& chan,
						const target_t* msg)
{
	target.utime = msg->utime;
	target.r = msg->r;
	target.theta = msg->theta;
	targetRecieved = 1;
}

void Handler::handleTargetXY (const lcm::ReceiveBuffer* rbuf,
						const std::string& chan,
						const target_xy_t* msg)
{
	targetXY.utime = msg->utime;
	targetXY.x = msg->x;
	targetXY.y = msg->y;
	targetXYRecieved = 1;
}

void Handler::handleOurTurn (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const tic_tac_toe_turn_t* msg)
{
	ourTurn.utime = msg->utime;
	ourTurn.turn_number = msg->turn_number;
	ourTurnRecieved = 1; 
}

void Handler::handleTheirTurn (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const tic_tac_toe_turn_t* msg)
{
	theirTurn.utime = msg->utime;
	theirTurn.turn_number = msg->turn_number;
	theirTurnRecieved = 1;
}

