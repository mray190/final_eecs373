#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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
#include "common/timestamp.h"
#include "rexarmController2.hpp"
#include "Handler.hpp"

Handler handler;

std::string ourChannel;
std::string theirChannel;

target_t cartesian_to_polar (double x, double y);
void *runLCM(void* id);

using namespace std;

int main (int argc, char **argv)
{
	// Intialize team and ball positions
	target_t ball;
	vector<target_t> ballVec;
	string team;
	cin >> team;
	cout << "team: " << team << "\n";

	double rexarmX, rexarmY, ballX, ballY;
	cin >> rexarmX >> rexarmY;
	cout << "Rexarm position: " << rexarmX << " " << rexarmY << "\n";

	while (cin >> ballX >> ballY)
	{
		ball = cartesian_to_polar(ballX - rexarmX, ballY - rexarmY);
		cout << "R: " << ball.r << " theta: " << ball.theta << "\n";
		ballVec.push_back(ball);
	}

	if (team == "red")
	{
		ourChannel = "RED_TURN";
		theirChannel = "GREEN_TURN";
	}	
	else
	{
		ourChannel = "GREEN_TURN";
		theirChannel = "RED_TURN";
	}

	// LCM and thread setup
	lcm::LCM lcm;
	if (!lcm.good()) exit(1);

	pthread_t thread;
	int rc;

	rc = pthread_create(&thread, NULL, runLCM, NULL);
	if (rc) 
	{
		std::cout << "Error: Unable to create thread for lcm.\n"; 
		exit(1);
	}

	// Rest of setup	
	const int numServos = 6;
	const int hz = 30;
	Rexarm rexarm;

	// These values work with Arm 6
	// Distances in meters
	rexarm.dist.push_back(0.1000); //0.1180 to the ground
	rexarm.dist.push_back(0.0984);
	rexarm.dist.push_back(0.0989);
	rexarm.dist.push_back(0.0981);
	rexarm.dist.push_back(0.0821);	

	rexarm.goal.len = numServos;

	// Intialize all joint commands to go home	
	for (int id = 0; id < numServos; ++id) 
	{
		dynamixel_command_t temp;
  		
		temp.utime = utime_now ();
        temp.position_radians = 0.0;
        temp.speed = 0.06;
        temp.max_torque = 0.60;

		rexarm.goal.commands.push_back(temp);
	}

	rexarm.goal.commands[5].position_radians = 1.00;

	//rexarm.goal.commands[0].speed = 0.15;
	//rexarm.goal.commands[5].speed = 0.12;
	
	lcm.publish("ARM_COMMAND2", &(rexarm.goal));
	wait(rexarm, handler);
	
	tic_tac_toe_turn_t ourTurn;
	tic_tac_toe_turn_t theirTurn;
		
	//bool won = 0;
	//bool lost = 0;
	//bool tied = 0;

	ourTurn.utime = 0;
	ourTurn.turn_number = 0;

	while(!ballVec.empty())
	{	
		if (handler.get_their_recieved())
		{	
			theirTurn = handler.get_their_turn();
			target_t nextBall = ballVec.back();

			if ((team == "red" && ourTurn.turn_number == theirTurn.turn_number)
				|| ((team == "green" && ourTurn.turn_number != theirTurn.turn_number)) )
			{
				pick_up_go_home_and_place(rexarm, nextBall, handler, lcm);
				ballVec.pop_back();
				ourTurn.turn_number += 1;
			}
		}

	lcm.publish(ourChannel, &ourTurn);
		usleep (1000000/hz);
	}

	
    return 0;
}

// converts (x,y) to (r,theta)
target_t cartesian_to_polar (double x, double y)
{
	target_t result;
	result.utime = 0;	
	result.r = sqrt(pow(x,2) + pow(y,2));
	result.theta = atan(y/x);
	return result;
}

void *runLCM(void* id) 
{
	lcm::LCM lcm;
	if (!lcm.good()) exit(1);

	lcm.subscribe("ARM_COMMAND2", &Handler::handleCommandList, &handler);
	lcm.subscribe("ARM_STATUS2", &Handler::handleStatusList, &handler);
	lcm.subscribe("TARGET", &Handler::handleTarget, &handler);
	lcm.subscribe("TARGET_XY", &Handler::handleTargetXY, &handler);
	//lcm.subscribe(ourChannel, &Handler::handleOurTurn, &handler);	
	lcm.subscribe(theirChannel, &Handler::handleTheirTurn, &handler);

	while(0 == lcm.handle());

	pthread_exit(NULL);	
}
