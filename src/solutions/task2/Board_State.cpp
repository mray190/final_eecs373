#include "Board_State.hpp"

Board_State::Board_State() {}

void Board_State::sendCoord() {

	double dy, dx;
	double scalar = 55;
	if (orientation==0) {
		dy = ((2-next_move_y)*60+32)*golden_ratio;
		dx = (1-next_move_x)*scalar*golden_ratio;
	} else if (orientation==1) {
		dy = ((next_move_x)*60+32)*golden_ratio;
		dx = (1-next_move_y)*scalar*golden_ratio;
	} else if (orientation==2) {
		dy = ((next_move_y)*60+32)*golden_ratio;
		dx = -(1-next_move_x)*scalar*golden_ratio;
	} else {
		dy = ((2-next_move_x)*60+32)*golden_ratio;
		dx = -(1-next_move_y)*scalar*golden_ratio;
	}
	double r = (sqrt(dy*dy+dx*dx)+20.0)/1000.0;
	double theta = atan2(dx,dy)+M_PI;

	lcm::LCM lcm_m;
	if (!lcm_m.good()) return;
	target_t target;
	target.utime = utime_now();
	target.r = (r+20.0)/1000.0;
	if (theta<3.14) {
		if (r < 100.0) {
			target.theta = theta*1.25;
		} else {
			target.theta = theta*1.05;

		}
	} else
		target.theta = theta;
	cout << "r: " << r << " theta: " << target.theta << endl;
	lcm_m.publish("TARGET", &target);
}

void Board_State::initState(vector<Blob>& blobs) {
	if (blobs.size()<6) return;
	int sum = 0;
	for (int i=0; i<blobs.size(); i++) {
		if (blobs.at(i).type==SQUARE) sum++;
	}
	if (sum!=6) return;

	std::sort (blobs.begin(), blobs.end(), compareX);

	if (blobs.at(2).type==SQUARE && blobs.at(blobs.size()-3).type==SQUARE) {
		if (blobs.at(2).cluster_id==myID || blobs.at(1).cluster_id==myID || blobs.at(0).cluster_id==myID) {
			orientation = 2;
		} else {
			orientation = 0;
		}

		if (compareY(blobs.at(0),blobs.at(1))) {
			if (compareY(blobs.at(0),blobs.at(2))) bot_left = blobs.at(0).center;
			else  bot_left = blobs.at(2).center;
		} else {
			if (compareY(blobs.at(1),blobs.at(2))) bot_left = blobs.at(1).center;
			else  bot_left = blobs.at(2).center;
		}

	} else {
		std::sort (blobs.begin(), blobs.end(), compareY);
		// if (blobs.at(2).cluster_id==myID || blobs.at(1).cluster_id==myID || blobs.at(0).cluster_id==myID) {
		// 	orientation = 3;
		// }else {
		// 	orientation = 1;
		// }
		if (myID==2) {
			orientation = 3;
		}else {
			orientation = 1;
		}


		if (compareX(blobs.at(0),blobs.at(1))) {
			if (compareX(blobs.at(0),blobs.at(2))) bot_left = blobs.at(0).center;
			else  bot_left = blobs.at(2).center;
		} else {
			if (compareX(blobs.at(1),blobs.at(2))) bot_left = blobs.at(1).center;
			else  bot_left = blobs.at(2).center;
		}

	}

	cout << "Orientation: " << orientation << endl;
	int rect_width;

	// if(orientation == 2){
	// 	bot_left.x = 35;
	// 	bot_left.y = 35;
	// }

//		rect_width = ((blobs.at(0).right_pixel.x - blobs.at(0).left_pixel.x) +
//				(blobs.at(1).right_pixel.x - blobs.at(1).left_pixel.x)/2.0);
	rect_width = ((blobs.at(0).right_pixel.x - blobs.at(0).left_pixel.x) +
			(blobs.at(1).right_pixel.x - blobs.at(1).left_pixel.x) +
			(blobs.at(0).top_pixel.y - blobs.at(0).bottom_pixel.y) +
			(blobs.at(1).top_pixel.y - blobs.at(1).bottom_pixel.y))/4.0;

	//golden_ratio = 60.0/rect_width;
	golden_ratio = 1.13208;
	cout << "Golden ratio: " << golden_ratio << endl;
	cout << "Bot left.x: " << bot_left.x<< endl;
	cout << "Bot left.y: " << bot_left.y<< endl;

	for (int x=0; x<3; x++) {
		for (int y=0; y<3; y++) {
			grid[x][y] = 0;
		}
	}

	for (int i=3; i<blobs.size()-3; i++) {
		cout << "Blob: (" << blobs.at(i).center.x << "," << blobs.at(i).center.y << ")\n";
		int x = ((blobs.at(i).center.x - bot_left.x)*golden_ratio-30)/60;
		int y = ((blobs.at(i).center.y - bot_left.y)*golden_ratio-30)/60;
		if(blobs.at(i).type == CIRCLE){
			cout << "plotting circle of type: " << blobs.at(i).cluster_id  << " at " << x << ", " << y <<  endl;
		}
		if (x>2) x = 2;
		if (y>2) y = 2;

		if(blobs.at(i).type == CIRCLE){
			//cout << "plotting circle of type: " << blobs.at(i).cluster_id  << " at " << x << ", " << y <<  endl;
			grid[x][y] = blobs.at(i).cluster_id;
		}
	}

	cout << "Printing board:" << endl;

	printBoard();
	pickNextMove();
	sendCoord();
}

void Board_State::printBoard() {
	cout << grid[0][2] << " | " << grid[1][2] << " | " << grid[2][2] << endl;
	cout << "----------" << endl;
	cout << grid[0][1] << " | " << grid[1][1] << " | " << grid[2][1] << endl;
	cout << "----------" << endl;
	cout << grid[0][0] << " | " << grid[1][0] << " | " << grid[2][0] << endl;
	cout << "----------" << endl;
}

int Board_State::ballsOnBoard() {
	int sum = 0;
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			if (grid[i][j]!=0)
			sum += 1;
		}
	}
	return sum;
}

bool Board_State::canWin(int myID, int oppID) {
	for (int y = 0; y<3; y++) {
		int my_row_sum = 0;
		int opp_row_sum = 0;
		for (int x = 0; x<3; x++) {
			if (grid[x][y]==myID) my_row_sum+=1;
			if (grid[x][y]==oppID) opp_row_sum+=1;
		}
		if (my_row_sum>=2 && opp_row_sum==0) {
			next_move_y = y;
			if (grid[0][y]==0) next_move_x = 0;
			else if (grid[1][y]==0) next_move_x = 1;
			else next_move_x = 2;
			return true;
		}
	}
	for (int x = 0; x<3; x++) {
		int my_row_sum = 0;
		int opp_row_sum = 0;
		for (int y = 0; y<3; y++) {
			if (grid[x][y]==myID) my_row_sum+=1;
			if (grid[x][y]==oppID) opp_row_sum+=1;
		}
		if (my_row_sum>=2 && opp_row_sum==0) {
			next_move_x = x;
			if (grid[x][0]==0) next_move_y = 0;
			else if (grid[x][1]==0) next_move_y = 1;
			else next_move_y = 2;
			return true;
		}
	}
	int my_row_sum = 0;
	int opp_row_sum = 0;
	for (int x = 0; x<3; x++) {
		if (grid[x][x]==myID) my_row_sum+=1;
		if (grid[x][x]==oppID) opp_row_sum+=1;
	}
	if (my_row_sum>=2 && opp_row_sum==0) {
		if (grid[0][0]==0) {
			next_move_y = 0;
			next_move_x = 0;
		}
		else if (grid[1][1]==0) {
			next_move_y = 1;
			next_move_x = 1;
		} else {
			next_move_y = 2;
			next_move_x = 2;
		}
		return true;
	}
	my_row_sum = 0;
	opp_row_sum = 0;
	for (int x = 0; x<3; x++) {
		if (grid[x][2-x]==myID) my_row_sum+=1;
		if (grid[x][2-x]==oppID) opp_row_sum+=1;
	}
	if (my_row_sum>=2 && opp_row_sum==0) {
		if (grid[0][2]==0) {
			next_move_y = 2;
			next_move_x = 0;
		}
		else if (grid[1][1]==0) {
			next_move_y = 1;
			next_move_x = 1;
		} else {
			next_move_y = 0;
			next_move_x = 2;
		}
		return true;
	}

	return false;
}

void Board_State::pickNextMove() {
	if (canWin(myID,oppID)) {
		cout << "Winning move: (" << next_move_x << "," << next_move_y << ")\n";
		return;
	} else if (canWin(oppID,myID)) {
		cout << "Blocking move: (" << next_move_x << "," << next_move_y << ")\n";
		return;
	}
	next_move_x = -1;
	next_move_y = -1;

	int totalBalls = ballsOnBoard();
	cout << "Ball count: " << totalBalls << endl;
	if (totalBalls==0) {
		next_move_x = 1;
		next_move_y = 1;
	} else if (totalBalls==1) {
		if (grid[1][1]==oppID) {
			next_move_x = 0;
			next_move_y = 0;
		} else {
			next_move_x = 1;
			next_move_y = 1;
		}
	} else if (totalBalls==2) {
		if (grid[1][1]==myID) {
			if (grid[0][1]==oppID || grid[2][1]) {
				next_move_y = 0;
				next_move_x = 1;
			} else {
				next_move_y = 1;
				next_move_x = 0;
			}
		} else {
			next_move_x = 0;
			next_move_y = 1;
		}

	} else if (totalBalls==3) {
		if (grid[1][1]==myID) {
			for (int i=0; i<3; i++) {
				for (int j=0; j<3; j++) {
					if (grid[i][j]==0) {
						next_move_y = j;
						next_move_x = i;
						break;
					}
				}
			}
		} else {
			if (grid[0][0]==myID) {
				next_move_y = 0;
				next_move_x = 2;
			} else if (grid[2][0]==myID) {
				next_move_y = 0;
				next_move_x = 0;
			} else if (grid[0][2]==myID) {
				next_move_y = 2;
				next_move_x = 2;
			} else {
				next_move_y = 2;
				next_move_x = 0;
			}
		}
	} else if (totalBalls==4) {
		if (grid[1][1]==oppID) {
			for (int i=0; i<3; i++) {
				for (int j=0; j<3; j++) {
					if (grid[i][j]==0) {
						next_move_y = j;
						next_move_x = i;
						break;
					}
				}
			}
		} else {
			if (grid[1][2]==oppID && grid[2][1]==oppID) {
				next_move_y = 0;
				next_move_x = 0;
			} else if (grid[1][0]==oppID && grid[2][1]==oppID) {
				next_move_y = 0;
				next_move_x = 0;
			} else if (grid[0][1]==oppID && grid[1][2]==oppID) {
				next_move_y = 0;
				next_move_x = 0;
			} else {
				next_move_y = 0;
				next_move_x = 1;
			}
		}
	} else {
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				if (grid[i][j]==0) {
					next_move_y = j;
					next_move_x = i;
					break;
				}
			}
		}

	}
	cout << "Next move: (" << next_move_x << "," << next_move_y << ")\n";
}

