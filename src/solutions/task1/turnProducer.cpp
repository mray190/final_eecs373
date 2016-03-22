#include <cstdlib>
#include <iostream>
#include <istream>

// a0
#include <common/timestamp.h>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/tic_tac_toe_turn_t.hpp>

using namespace std;

int main(int argc, char** argv) {
	system("stty raw -echo");
		
	lcm::LCM lcm;
	if (!lcm.good()) return 1;

	tic_tac_toe_turn_t theirTurn;
	theirTurn.utime = utime_now();
	theirTurn.turn_number = -1;
	
	char c;
	while(cin >> c) {
		theirTurn.utime = utime_now();		
		theirTurn.turn_number += 1;		
		if (c == 'c') break;	
		if (c == 'r') lcm.publish("RED_TURN", &theirTurn);	
		if (c == 'g') lcm.publish("GREEN_TURN", &theirTurn);
		cout << "Turn: " << theirTurn.turn_number << "\r\n";

	}	

	system("stty sane");
	return 0;
}
