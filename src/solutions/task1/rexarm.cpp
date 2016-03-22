#include "rexarm.h"
#include <vector>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/dynamixel_command_list_t.hpp>
#include <lcmtypes/dynamixel_command_t.hpp>
#include <lcmtypes/dynamixel_status_list_t.hpp>
#include <lcmtypes/dynamixel_status_list_t.hpp>
#include <lcmtypes/dynamixel_status_t.hpp>

#include "common/timestamp.h"

//using namespace eecs467;

Rexarm::Rexarm()
{
    thetas.resize(NUM_SERVOS);
}
   


void Rexarm::setThetas(double theta,double radius)
{

}

void Rexarm::run()
{
    dynamixel_command_list_t commandList;
    commandList.commands.resize(NUM_SERVOS);

    lcm::LCM rexarmLcm;
	//loop through the thetas and create command messages
    while(1)
    {
		for(int i =0; i< thetas.size(); i++)
		{
		    commandList.commands[i].utime = utime_now ();
		    commandList.commands[i].position_radians = thetas[i];
		    commandList.commands[i].speed = 0.05;
		    commandList.commands[i].max_torque = 0.55;
		   
		}
    
		//publish commandList message
		std::cout<<"About to publish!"<<std::endl;
		rexarmLcm.publish("ARM_COMMAND", &commandList);
		usleep(1000000/hz);
    }

}
