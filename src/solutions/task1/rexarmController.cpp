#include "rexarmController.hpp"

void pick_up_and_place (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm)
{
	double counterClockwise = 0.075; // Can't go to that exact spot because we'd just hit the ball
	double aboveBoard = 0.042; // Trick to think the ground is a little closer than it is
	
	update_status (rexarm, handler);
	
	// Go home to start
	go_home(rexarm);	
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Go slightly counter-clockwise of ball and grab	
	update_arm(rexarm, target);
	rexarm.goal.commands[0].position_radians += counterClockwise; 
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	grab(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Move away from the balls
	rexarm.goal.commands[1].position_radians = 0.0;
	//rexarm.goal.commands[3].position_radians = 0.0;
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	
	while(!handler.get_target_recieved());
	std::cout << "Target message recieved\n";
	target = handler.get_target();

	// Go to the middle of the board
	rexarm.dist[0] -= aboveBoard;
	rexarm.dist[0] -= aboveBoard;
	target_t middle;
	middle.r = 0.070;
	middle.theta = M_PI;
	update_arm(rexarm, middle);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Go slightly above target position and drop
	rexarm.dist[0] += aboveBoard;
	update_arm(rexarm, target);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	drop(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Go back to the middle of the board
	update_arm(rexarm, middle);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	rexarm.goal.commands[0].position_radians = 0.0;
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	
	// Go back home
	go_home(rexarm);
	open(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	rexarm.dist[0] += aboveBoard;
}

void pick_up_go_home_and_place (Rexarm &rexarm, target_t target, Handler &handler, lcm::LCM &lcm)
{
	double counterClockwise = 0.075; // Can't go to that exact spot because we'd just hit the ball
	double aboveBoard = 0.042; // Trick to think the ground is a little closer than it is
	
	update_status (rexarm, handler);
	
	// Go home to start
	go_home(rexarm);	
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Go slightly counter-clockwise of ball and grab	
	update_arm(rexarm, target);
	rexarm.goal.commands[0].position_radians += counterClockwise;
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	grab(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	rexarm.goal.commands[1].position_radians = 0.0;
	//rexarm.goal.commands[3].position_radians = 0.0;
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	while(!handler.get_target_recieved());
	std::cout << "Target message recieved\n";
	target = handler.get_target();

	// Go back home to start
	go_home(rexarm);	
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	// Go slightly above target position and drop
	rexarm.dist[0] -= aboveBoard;
	update_arm(rexarm, target);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	drop(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	
	// Go back home
	go_home(rexarm);
	open(rexarm);
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	rexarm.dist[0] += aboveBoard;
}

// This function is described in the Inverse Kinematics
// slides (Lecture 10) from EECS 467 Winter 2016
void update_arm (Rexarm &rexarm, target_t target)
{	
	double m = 0.0;
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;
	double maxRange = 0.175; //0.18026
	double wristTiltDist = 0.0;	
	double maxExtra = rexarm.dist[3] + rexarm.dist[4] - 0.020; //0.020 is the radius of the ball

	std::cout << "R: " << target.r << " Theta:" << target.theta << "\n";
	
	rexarm.goal.commands[0].position_radians = target.theta;
	rexarm.goal.commands[3].position_radians = 0.0;

	// d4 + h - d1	
	double a = rexarm.dist[3] + rexarm.dist[4] - rexarm.dist[0];

	// R^2 and (d4 + h - d1)^2
	double asquared = pow(a,2);

	maxRange = sqrt(pow((rexarm.dist[1] + rexarm.dist[2]), 2) - asquared);

	if(target.r > maxRange)
	{
		wristTiltDist = (target.r - maxRange) + 0.000001;
		target.r = maxRange - 0.000001;
	}

	double rsquared = pow(target.r,2);
	
	// M, alpha, beta, and gamma are all intermediate values
	m = sqrt(rsquared + asquared);

	alpha = atan2(a, target.r);
	beta = acos((pow(rexarm.dist[1],2) - pow(rexarm.dist[2],2) + pow(m,2)) / (2.0*m*rexarm.dist[1]));
	gamma = acos((pow(rexarm.dist[1],2) + pow(rexarm.dist[2],2) - pow(m,2)) / (2*rexarm.dist[1]*rexarm.dist[2]));
	
	// Set joint angles along the arm to bend it to pick up the ball
	rexarm.goal.commands[1].position_radians = M_PI_2 - alpha - beta;
	rexarm.goal.commands[2].position_radians = M_PI - gamma;
	rexarm.goal.commands[3].position_radians = M_PI - rexarm.goal.commands[1].position_radians 
										- rexarm.goal.commands[2].position_radians;
	if (wristTiltDist > maxExtra)
	{
		std::cout << "Target is out of Rexarm's range\n";
		return;
	} else if (wristTiltDist != 0.0) {
		rexarm.goal.commands[3].position_radians = M_PI - acos(wristTiltDist/(rexarm.dist[4]+rexarm.dist[3])) + 10.0/180.0*M_PI;
	}
}

void update_theta_1 (Rexarm &rexarm, target_t target)
{
	rexarm.goal.commands[0].position_radians = target.theta;
}

void update_theta_2 (Rexarm &rexarm, target_t target)
{
	double m = 0.0;
	double alpha = 0.0;
	double beta = 0.0;

	// d4 + h - d1	
	double a = rexarm.dist[3] + rexarm.dist[4] - (rexarm.dist[0]);

	// R^2 and (d4 + h - d1)^2
	double rsquared = pow(target.r,2);
	double asquared = pow(a,2);
	
	// M, alpha, beta, and gamma are all intermediate values
	m = sqrt(rsquared + asquared);
	alpha = atan2(a, target.r);
	beta = acos((pow(rexarm.dist[1],2) - pow(rexarm.dist[2],2) + pow(m,2)) / (2.0*m*rexarm.dist[1]));
	
	// Set joint angles along the arm to bend it to pick up the ball
	rexarm.goal.commands[1].position_radians = M_PI_2 - alpha - beta;
}

void update_theta_3 (Rexarm &rexarm, target_t target)
{
	double m = 0.0;
	double gamma = 0.0;

	// d4 + h - d1	
	double a = rexarm.dist[3] + rexarm.dist[4] - (rexarm.dist[0]);

	// R^2 and (d4 + h - d1)^2
	double rsquared = pow(target.r,2);
	double asquared = pow(a,2);
	
	// M, alpha, beta, and gamma are all intermediate values
	m = sqrt(rsquared + asquared);
	gamma = acos((pow(rexarm.dist[1],2) + pow(rexarm.dist[2],2) - pow(m,2)) / (2*rexarm.dist[1]*rexarm.dist[2]));

	rexarm.goal.commands[2].position_radians = M_PI - gamma;
}

void update_theta_4 (Rexarm &rexarm, target_t target)
{
	double m = 0.0;
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;

	// d4 + h - d1	
	double a = rexarm.dist[3] + rexarm.dist[4] - (rexarm.dist[0]);

	// R^2 and (d4 + h - d1)^2
	double rsquared = pow(target.r,2);
	double asquared = pow(a,2);
	
	// M, alpha, beta, and gamma are all intermediate values
	m = sqrt(rsquared + asquared);
	alpha = atan2(a, target.r);
	beta = acos((pow(rexarm.dist[1],2) - pow(rexarm.dist[2],2) + pow(m,2)) / (2.0*m*rexarm.dist[1]));
	gamma = acos((pow(rexarm.dist[1],2) + pow(rexarm.dist[2],2) - pow(m,2)) / (2*rexarm.dist[1]*rexarm.dist[2]));
	
	// Set joint angles along the arm to bend it to pick up the ball
	double theta2 = M_PI_2 - alpha - beta;
	double theta3 = M_PI - gamma;
	rexarm.goal.commands[3].position_radians = M_PI - theta2 - theta3;
}

void go_home (Rexarm &rexarm)
{
	rexarm.goal.commands[0].position_radians = 0.0;
	rexarm.goal.commands[1].position_radians = 0.0;
	rexarm.goal.commands[2].position_radians = 0.0;
	rexarm.goal.commands[3].position_radians = 0.0;
	rexarm.goal.commands[4].position_radians = 0.0;
}

void grab (Rexarm &rexarm)
{
	rexarm.goal.commands[5].position_radians = 1.50;
}

void drop (Rexarm &rexarm)
{
	rexarm.goal.commands[5].position_radians = 1.30;
}

void open (Rexarm &rexarm)
{
	rexarm.goal.commands[5].position_radians = 0.70;
}


void high_torque (Rexarm &rexarm, unsigned id)
{
	rexarm.goal.commands[id].max_torque = 0.9;
}

void low_torque (Rexarm &rexarm, unsigned id)
{
	rexarm.goal.commands[id].max_torque = 0.6;
}

void update_status (Rexarm &rexarm, Handler &handler)
{
	if(handler.get_status_list_recieved())
	{	
		rexarm.status.len = handler.get_status_list().len;		
		rexarm.status.statuses = handler.get_status_list().statuses;	
	}
}

void wait (Rexarm &rexarm, Handler& handler)
{	
	std::queue<dynamixel_status_list_t> recentStatuses;
	while (1)
	{		
		bool weveArrived = 1;		
		update_status (rexarm, handler);	
		if(handler.get_status_list_recieved())
		{	
			size_t statusQueSize = recentStatuses.size();
			//std:: cout << "Queue size is " << statusQueSize << ": ";				
			for (int id = 0; id < rexarm.status.len; ++id) {									
				double diff = 0.06;				
				if (statusQueSize < 60) {				
					diff = rexarm.goal.commands[id].position_radians - rexarm.status.statuses[id].position_radians;
					diff = std::abs(diff);
				}
				else if (statusQueSize >= 60)
				{	
					diff = recentStatuses.front().statuses[id].position_radians 
						- rexarm.status.statuses[id].position_radians;
					diff = std::abs(diff);
					recentStatuses.pop();
				}		
				if (diff >= 0.02) weveArrived = 0;
				
			}
			recentStatuses.push(rexarm.status);
			//std::cout << "\n";
				
			if (weveArrived)
			{
				break;
			}
			usleep(20000);
		}
	}
}

/**************************************************************************
*	For fun                                                               *
**************************************************************************/

void one_hand_clap (Rexarm &rexarm, target_t ball, Handler &handler, lcm::LCM &lcm)
{
	// Go home to start
	go_home(rexarm);	
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);
	
	rexarm.goal.commands[5].speed = 0.3;

	std::cout << "Bravo!\n";
		
	while(1)
	{
		rexarm.goal.commands[5].position_radians = 2.10;
		lcm.publish("ARM_COMMAND", &(rexarm.goal));
		usleep(350000);
		
		rexarm.goal.commands[5].position_radians = 1.15;
		lcm.publish("ARM_COMMAND", &(rexarm.goal));
		usleep(350000);
	}
}

void rexarm_shake (Rexarm &rexarm, target_t ball, Handler &handler, lcm::LCM &lcm)
{
	// Go home to start
	go_home(rexarm);	
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	rexarm.goal.commands[1].speed = 0.1;

	rexarm.goal.commands[0].position_radians = 3.10;
	rexarm.goal.commands[1].position_radians = 1.30;
	rexarm.goal.commands[5].position_radians = 1.50;
	lcm.publish("ARM_COMMAND", &(rexarm.goal));
	wait(rexarm, handler);

	std::cout << "Good game! Better luck next time\n";

	while(1)
	{
		rexarm.goal.commands[1].position_radians = 1.30;
		lcm.publish("ARM_COMMAND", &(rexarm.goal));
		usleep(400000);
		
		rexarm.goal.commands[1].position_radians = 1.60;
		lcm.publish("ARM_COMMAND", &(rexarm.goal));
		usleep(400000);
	}

}



