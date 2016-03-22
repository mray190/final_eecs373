#ifndef IMAGECONTROLLER_H
#define IMAGECONTROLLER_H

#include <iostream>
#include "lcmtypes/maebot_pose_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "maebot/maebot_channels.h"
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <stack>
#include <utility>
#include <math.h>
#include <limits>
#include <assert.h>
// imagesource
#include <imagesource/image_u8.h>
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "eecs467/vx_utils.h"
#include "Utilities.hpp"
#include "Board_State.hpp"

using namespace std;

class ImageController {
public:
	ImageController() : mask_set(0), board() {}

	//Image process
	image_u32_t * run(image_u32_t *im0, int _myID);

	//Set the mask set by the user clicks
	void setMask(int nx0, int ny0, int nx1, int ny1);
	int extractBlob(int x, int y, Blob&) ;
	void kMeansClustering() ;
	float calcHSVdist(float H1, float H2, float S1, float S2, float, float) ;
	void blobDetector() ;
	int scoreBlobs(Blob& blob);

	std::vector<Blob> imageBlobs;
	Board_State board;

private:
	int mask_set;
	int x0, y0, x1, y1;

	int myID;

	image_u32_t *im;
	void convertToHSL(image_u32_t *im0);
	void * thread_runner (void* arg);
};

#endif /* IMAGECONTROLLER_H */
