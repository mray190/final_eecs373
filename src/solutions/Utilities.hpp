#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>

using namespace std;

enum shape_type {NONE, CIRCLE, SQUARE};
enum colors{RED, GREEN, BLUE};

struct Pixel{
		Pixel() {}
		Pixel(int x_, int y_):x(x_), y(y_){}
		int x;
		int y;
	};

struct Blob{
	Blob() : left_pixel(4000,0), right_pixel(0,0),
				top_pixel(0,0), bottom_pixel(0,4000) {}
	~Blob(){
	}

	std::vector<Pixel> blobPixels;
	std::vector<Pixel> edgePixels;
	float square_score;
	float circle_score;
	shape_type type;

	int cluster_id;

	Pixel left_pixel;
	Pixel right_pixel;
	Pixel top_pixel;
	Pixel bottom_pixel;
	Pixel center;
};

#endif
