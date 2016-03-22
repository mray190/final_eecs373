#ifndef HANDLER_H
#define HANDLER_H

#include <iostream>

// LCM includes
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/dynamixel_command_list_t.hpp>
#include <lcmtypes/dynamixel_command_t.hpp>
#include <lcmtypes/dynamixel_status_list_t.hpp>
#include <lcmtypes/dynamixel_status_t.hpp>
#include <lcmtypes/target_t.hpp>
#include <lcmtypes/target_xy_t.hpp>
#include <lcmtypes/tic_tac_toe_turn_t.hpp>

class Handler 
{
public:
	Handler()
	: commandListRecieved(0)
	, statusListRecieved(0)
	, targetRecieved(0)
	, targetXYRecieved(0)
	, ourTurnRecieved(0)
	, theirTurnRecieved(0) {}

	/**************************************************************************
	*	Getters                                                               *
	**************************************************************************/

	dynamixel_command_list_t get_command_list() const;
	dynamixel_status_list_t get_status_list() const;
	target_t get_target() const;
	target_xy_t get_target_xy() const;
	tic_tac_toe_turn_t get_our_turn() const;	
	tic_tac_toe_turn_t get_their_turn() const;

	bool get_command_list_recieved() const;
	bool get_status_list_recieved() const;
	bool get_target_recieved() const;
	bool get_target_xy_recieved() const;
	bool get_our_recieved() const;	
	bool get_their_recieved() const;

	/**************************************************************************
	*	Public handler functions                                              *
	*   Handler functions update our member variables as LCM message are      *
	*	recieved. They are defined in Handlers.cpp.            	 	     	  *
	**************************************************************************/

	void handleCommandList (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const dynamixel_command_list_t* msg);

	void handleStatusList (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const dynamixel_status_list_t* msg);

	void handleTarget (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const target_t* msg);

	void handleTargetXY (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const target_xy_t* msg);

	void handleOurTurn (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const tic_tac_toe_turn_t* msg);

	void handleTheirTurn (const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const tic_tac_toe_turn_t* msg);

private:
	/**************************************************************************
	*	Private Member Variables                                              *
	**************************************************************************/
	
	dynamixel_command_list_t commandList;
	dynamixel_status_list_t statusList;
	target_t target;
	target_xy_t targetXY;
	tic_tac_toe_turn_t ourTurn;
	tic_tac_toe_turn_t theirTurn;

	bool commandListRecieved;
	bool statusListRecieved;
	bool targetRecieved;
	bool targetXYRecieved;
	bool ourTurnRecieved;
	bool theirTurnRecieved;
};

#endif /* HANDLER_H */
