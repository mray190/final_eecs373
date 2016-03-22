#ifndef BOARD_STATE_H
#define BOARD_STATE_H

#include <iostream>
#include "lcmtypes/maebot_pose_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "maebot/maebot_channels.h"
#include "common/timestamp.h"
#include <stdlib.h>
#include <unistd.h>
//#include <thread>
#include <mutex>
#include <stack>
#include <utility>
#include <math.h>
#include "Utilities.hpp"
#include "lcmtypes/target_t.hpp"
#include <algorithm>

// imagesource
#include "eecs467/vx_utils.h"

using namespace std;

class Board_State {
public:

	Board_State();

	Pixel bot_left;

	int grid[3][3];

	void setID(int _myID, int _oppID) {
		myID = _myID;
		oppID = _oppID;
	}

	void initState(vector<Blob>&);
	void printBoard();
	void pickNextMove();
	void sendCoord();

private:
	int myID, oppID;
	int orientation;

	int next_move_x;
	int next_move_y;

	float golden_ratio; //mm per 1 pixel

  static bool compareX(const Blob& i,const Blob& j) { return i.center.x < j.center.x; }
  static bool compareY(const Blob& i,const Blob& j) { return i.center.y < j.center.y; }
//	static bool compareX(const Blob& i,const Blob& j);
//	static bool compareY(const Blob& i,const Blob& j);

  	bool canWin(int, int);
  	int ballsOnBoard();
};

#endif /* BOARD_STATE_H */
