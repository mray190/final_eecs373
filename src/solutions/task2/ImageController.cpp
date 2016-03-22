#include "ImageController.h"
#include <algorithm>

int ignore_index; 
void ImageController::convertToHSL(image_u32_t *im0) { 
	im = image_u32_create(x1-x0, y1-y0);
    for (int y = 0; y < im->height; y++) {
		for (int x = 0; x < im->width; x++) {
		    uint32_t abgr = im0->buf[(y+y0)*im0->stride + (x+x0)];
		    float r = ((uint8_t)((abgr >> 0) & 0xff))/255.;
		    float g = ((uint8_t)((abgr >> 8) & 0xff))/255.;
		    float b = ((uint8_t)((abgr >> 16) & 0xff))/255.; 
		    float M = max(r, max(g,b));
		    float m = min(r, min(g,b));
		    float C = M-m;
		    float S = (C<0.00001) ? 1 : C/M;
		    im->s[y*im->stride + x] = S;

		    float H = 0;
		    float V = M;
		    im->v[y*im->stride + x] = V;
		    if((C>0.00001)){
			    if (M <= r) {
				H = (g-b)/C;
			    } else if (M <= g) {
				H = 2.0 + (b-r)/C;
			    } else if (M <= b) {
				H = 4.0 + (r-g)/C;
			    }
		    }
		    else H = 0; 

		    H *= 60;
		    if (H<0.0) H+= 360;
		    im->buf[y*im->stride + x] = abgr;

		    assert(H <= 360.0);
		    float Hrad = H/180.0*M_PI;
		    im->hue[y*im->stride+x] = Hrad;

		    //RED
		    if (H <= 35 || H>=330) {
		    	im->cluster_id[y*im->stride + x] = 1;

		    //BLUE
		    } else if (H>=120 && H<=320) {
		    	im->cluster_id[y*im->stride + x] = 2;

		    //GREEN
		    } else if (H>=50 && H<=120) {
			    if (S>0.6) {
				    	im->cluster_id[y*im->stride + x] = 3;
				} else {
			    	im->buf[y*im->stride + x] = 0x00000000;
			    }
		    //OTHER
		    } else {
		    	im->buf[y*im->stride + x] = 0x00000000;
		    }
		}
	}
}

float ImageController::calcHSVdist(float H1, float H2, float S1, float S2, float V1, float V2) {
  //Hue goes from 0-360; maybe we should normalize to 0-2pi
  float distance = std::abs(H1-H2);
  if(distance > M_PI/2.0){
    distance = 2.0*M_PI-distance;
  }
  /*if(S1 < 0.2 || S2 < 0.2){
	return (S2-S1)*(S2-S1);
  }*/
  //return distance+std::abs(S2-S1)*2.0*M_PI+std::abs(V2-V1)*2.0*M_PI;
  //return distance+std::abs(S2-S1)+std::abs(V2-V1);
  return distance*distance+(S2-S1)*(S2-S1);//+(V2-V1)*(V2-V1);
}

//TODO: convert all H to range 0->2PI during the above step
//	set the hue buffer in image during the above step 
void ImageController::kMeansClustering() {
  //Initialize clust values to ideal RGB values
	int NUM_ITER = 100;
	int NUM_CLUSTERS = 9;

	float clusterCenter[NUM_CLUSTERS];
	float sCenter[NUM_CLUSTERS];
	float vCenter[NUM_CLUSTERS];
    
	float sumSin[NUM_CLUSTERS];
	float sumCos[NUM_CLUSTERS];
	float count[NUM_CLUSTERS];
	float S[NUM_CLUSTERS];
	float V[NUM_CLUSTERS];


  
  clusterCenter[0] = 80.0/180.0*M_PI;//White background, doens't matter
  sCenter[0] = 0.50; 
  //vCenter[0] = 0.00; 
  
  clusterCenter[1] = 20.0/180.0*M_PI;//...orange = 30 degress, 100%, 100%
  //sCenter[1] = 0.80; 
  //vCenter[1] = 0.50; 
  sCenter[1] = 0.80; 
  //vCenter[1] = 1.00; 
  
  clusterCenter[2] = 80.0/180.0*M_PI;//GREEN...120 degrees
  sCenter[2] = 1.00;
  //vCenter[2] = 0.50; 
  
  clusterCenter[3] = 200.0/180.0*M_PI;//BLUE...degrees
  sCenter[3] = 0.50;
  //vCenter[3] = 0.50; 
  
  clusterCenter[4] = 30.0/180.0*M_PI;//wood color
  sCenter[4] = 0.60;
  //vCenter[4] = 0.70; 

  clusterCenter[5] = 20.0/180.0*M_PI;//...black and shadows
  sCenter[5] = 0.55; 
  //vCenter[5] = 0.00; 
  
  clusterCenter[6] = 120.0/180.0*M_PI;//...other green
  sCenter[6] = 0.50; 
  //vCenter[6] = 0.50; 
  
  clusterCenter[7] = 30.0/180.0*M_PI;//...black and shadows
  sCenter[7] = 0.55; 
  //vCenter[5] = 0.00; 

  clusterCenter[8] = 120.0/180.0*M_PI;//...other other green
  sCenter[6] = 0.80; 
  //vCenter[6] = 0.50;



  //Assign all pixels to a blob
  for(int i = 0; i < NUM_ITER; i++){
    for (int y = 0; y < im->height; y++) {
      for (int x = 0; x < im->width; x++) {
        float curMinDist = std::numeric_limits<float>::max();
        int curMinCluster = 0; 
        for (int k = 0; k < NUM_CLUSTERS; k++) {
          float testDist = calcHSVdist(clusterCenter[k], im->hue[y*im->stride + x], sCenter[k], im->s[y*im->stride + x], vCenter[k], im->v[y*im->stride+x]);
          if(testDist < curMinDist){
                curMinDist = testDist;
                curMinCluster = k;
          }//End if
        }//End clusters
        im->cluster_id[y*im->stride + x] = curMinCluster;
      }//End x
    }//End y

    //Find new center means
    for(int i = 0; i < NUM_CLUSTERS; i++){
	 	sumSin[i] = 0; sumCos[i] = 0; count[i] = 0; S[i] = 0; V[i] = 0;
    }

    for (int y = 0; y < im->height; y++) {
      for (int x = 0; x < im->width; x++) {
        int clusterID = im->cluster_id[y*im->stride + x];
	if(clusterID == 0){
		im->buf[y*im->stride+x] = 0x00ffffff;
	}else if(clusterID == 1){
		im->buf[y*im->stride+x] = (0xff0000ff);
	}else if(clusterID == 2){
		im->buf[y*im->stride+x] = 0xff00ff00;
	}else if(clusterID == 3){
		im->buf[y*im->stride+x] = 0xffff0000;
	}else if(clusterID == 4){
		im->buf[y*im->stride+x] = 0xffaaaacc;
	}else if(clusterID == 5){
		im->buf[y*im->stride+x] = 0xff000000;
	}else if(clusterID == 6){
		im->buf[y*im->stride+x] = 0xffaa00aa;
	}

        count[clusterID] += 1;
        assert(im->hue[y*im->stride + x] <= 2*M_PI);
        assert(im->hue[y*im->stride + x] >= 0);
        sumCos[clusterID] += cos(im->hue[y*im->stride + x]);
        sumSin[clusterID] += sin(im->hue[y*im->stride + x]);
        S[clusterID] += im->s[y*im->stride + x];
        V[clusterID] += im->v[y*im->stride + x];
      }//End for x
    }//End for y
    for(int k = 0; k < NUM_CLUSTERS; k++){
		float x_val = sumCos[k]/count[k];
		float y_val = sumSin[k]/count[k];	
		//cout << "Num pixels[" << k << "]" << count[k] << endl;
		//assert(x_val < 1);
		//assert(y_val < 1);
		clusterCenter[k] = atan2(y_val, x_val);
		if(clusterCenter[k] < 0) clusterCenter[k] += 2*M_PI;
		sCenter[k] = S[k]/count[k];
		vCenter[k] = V[k]/count[k];
    }//End for num clusters
	//cout << endl;
  }//End iter 
	cout << "Final clusters: " << endl;
 
	float max = count[0];	
	int max_index= 0;
    	for(int k = 0; k < NUM_CLUSTERS; k++){
		if(count[k] > max){
			max = count[k];
			max_index = k;
		}
	    	cout << k << ": " << clusterCenter[k]/(M_PI)*180.0 << ", " << sCenter[k] << ", " << vCenter[k] << endl;
    	}//End for num clusters
	ignore_index = max_index;	
	cout << "IGNORING BACKGROUND: " << ignore_index << endl << endl;
   /* for (int y = 0; y < im->height; y++) {
      for (int x = 0; x < im->width; x++) {
	if(im->cluster_id[y*im->stride+x] == 6)
		im->cluster_id[y*im->stride+x] = 2;
	}
   }*/
}//End kmeans

int ImageController::scoreBlobs(Blob& blob) {
	// float numerator = blob.blobPixels.size()*4.0*M_PI;
	// float denominator = blob.edgePixels.size()*blob.edgePixels.size();
	// blob.score = numerator/denominator;
	float numerator = blob.blobPixels.size();
	float denominator = (blob.right_pixel.x - blob.left_pixel.x)*(blob.top_pixel.y - blob.bottom_pixel.y);
	blob.square_score = numerator/denominator;

	float radius = (blob.right_pixel.x - blob.left_pixel.x)*0.5;
	blob.circle_score = blob.blobPixels.size()/(M_PI*radius*radius);

	cout << "Scoring blob with ID: " << blob.cluster_id << "... square: " << blob.square_score << " circle: " << blob.circle_score;
	if ((blob.circle_score < 0.65 || blob.circle_score > 1.35) && (blob.square_score < 0.65 || blob.square_score > 1.35)) {
		cout << "...nothin!" <<  endl;
		return 0;
		}

	blob.center.x = (int)((blob.right_pixel.x - blob.left_pixel.x)*0.5 + blob.left_pixel.x);
	blob.center.y = (int)((blob.top_pixel.y - blob.bottom_pixel.y)*0.5 + blob.bottom_pixel.y);

	// if (blob.square_score>0.9 || blob.cluster_id==3) {
	// 	blob.type = SQUARE;
	// 	cout << "...square! " << endl;
	// } else {
	// 	blob.type = CIRCLE;
	// 	cout << "...circle! " << endl;
	// }

	if (blob.cluster_id==2 && blob.square_score > 0.9) {
		blob.type = SQUARE;
		cout << "...square! " << endl;
	} else if (std::abs(1-blob.circle_score) < std::abs(1-blob.square_score) && blob.cluster_id != 3 && blob.cluster_id != 6){
		blob.type = CIRCLE; 
		// cout << "...circle! " << endl;
	// } else if(blob.cluster_id == 2){ 
	// 	blob.type = CIRCLE; 
		cout << "...circle! " << endl;
	} else {
		blob.type = SQUARE;
		cout << "...square! " << endl;
	}

	return 1;
}


void ImageController::blobDetector() {
int count = 0;
for (int y = 0; y < im->height; y++) {
	for (int x = 0; x < im->width; x++) {
		if(im->blob_id[y*im->stride + x] == 0){
			Blob blob;
			int result = extractBlob(x,y, blob);
			if (result==0) {
				if (scoreBlobs(blob)){ 
					imageBlobs.push_back(blob);
					cout << "adding blob#: " << count++ << " with ID: " << blob.cluster_id << endl;
				}
			}
		}//End if
	}//End for
  }//End for
}//end connected components

int ImageController::extractBlob(int x, int y, Blob& blob) {
	std::stack<Pixel> pixel_stack;
	im->blob_id[y*im->stride + x] = 1;
	blob.cluster_id = im->cluster_id[y*im->stride + x];;
	if (blob.cluster_id != 1 && blob.cluster_id != 2 && blob.cluster_id != 3 && blob.cluster_id != 6) return -1;//If I remove this, it fails every time!? WHY?
	//if (blob.cluster_id == ignore_index || blob.cluster_id == 4) return -1;//If I remove this, it fails every time!? WHY?
	pixel_stack.push(Pixel(x,y));

	while(!pixel_stack.empty()){
		Pixel p = pixel_stack.top();
		pixel_stack.pop();
		blob.blobPixels.push_back(p);

		if (p.x < blob.left_pixel.x) blob.left_pixel = p;
		if (p.x > blob.right_pixel.x) blob.right_pixel = p;
		if (p.y > blob.top_pixel.y) blob.top_pixel = p;
		if (p.y < blob.bottom_pixel.y) blob.bottom_pixel = p;

		for (int i = p.x-1; i < p.x+2; i++){
			if (i<0 || i>im->width) continue;
		  	for (int j = p.y-1; j < p.y+2; j++){
			    if ((p.x == i && p.y == j) || (j<0 || j>im->height)) continue;
			    if((im->blob_id[j*im->stride + i] == 0) && (im->cluster_id[j*im->stride + i] == blob.cluster_id)){
			      im->blob_id[j*im->stride + i] = 1;
			      pixel_stack.push(Pixel(i,j));
			    } else if (im->cluster_id[j*im->stride + i] != blob.cluster_id) {
			    	if (blob.edgePixels.empty()) {
			    		blob.edgePixels.push_back(p);	

			    	} else if (blob.edgePixels.back().x != p.x || blob.edgePixels.back().y != p.y) {
			    		blob.edgePixels.push_back(p);
			    	}
			    }//End if
		  	}//End for
		}//End for
	}//End while

	// if (blob.blobPixels.size()<200) {cout << "size: " <<  blob.blobPixels.size() << endl;}
	if (blob.blobPixels.size()<200) return -2;

	return 0;
}//End extractBlob

void ImageController::setMask(int nx0, int ny0, int nx1, int ny1) {
	mask_set = 1;
	x0 = nx0;
	y0 = ny0;
	x1 = nx1;
	y1 = ny1;
}

void * ImageController::thread_runner (void* arg) {
	//Image processing goes here using image_u32 im
	return NULL;
}

image_u32_t * ImageController::run(image_u32_t *im0, int _myID) {
	myID = _myID;
	//Only convert to HSL if the mask has been set
    if (mask_set==1){
       	convertToHSL(im0);
        kMeansClustering() ;
       	blobDetector();
		if (myID==1) board.setID(1, 2);
		else board.setID(2, 1);
       	board.initState(imageBlobs);
		while(!imageBlobs.empty()){
			imageBlobs.pop_back();
		}
	  	//imageBlobs.clear();
        return im;
    }
    return im0;
}
