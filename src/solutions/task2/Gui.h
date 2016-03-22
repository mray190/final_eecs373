#include <stdio.h>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <common/timestamp.h>
//#include <pthread.h>
#include <unistd.h>
#include <cmath>

#include "ImageController.h"

#include <eecs467/vx_gtk_window_base.hpp>
#include <vx/vxo_text.h>
#include <vx/vxo_pix_coords.h>
#include <vx/vxo_image.h>
#include <vx/vx_object.h>
#include <vx/vx.h>
#include "vx/vx_util.h"

#include "vx/vx_ray3.h"
#include "vx/vxo_mat.h"
#include "vx/vx_colors.h"
#include "vx/vx_world.h"
#include "vx/vx_global.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include <imagesource/image_u8.h>
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "eecs467/vx_utils.h"

#include "lcmtypes/tic_tac_toe_turn_t.hpp"

class VxWindow : public eecs467::VxGtkWindowBase {
public:

    VxWindow(int argc, char** argv) : VxGtkWindowBase(argc, argv, 640, 480, 5), frameCount_(0), turn_number(0) { }

    virtual int onMouseEvent(vx_layer_t*, vx_camera_pos_t*, vx_mouse_event_t*);

    virtual int onKeyEvent(vx_layer_t* layer, vx_key_event_t* event);

    /**
    * Initialize the camera
    * \return int representing the status of the method
    *    0 if no errors
    *   -1 if no cameras detected
    *   -2 if the camera detected could not be opened
    *
    * \post isrc is set to the correct camera source
    **/
    int setup(int);

    /**
    * Default overloaded render function to display the frames of the image source
    **/
    virtual void render(void);

private:
    int turn_number;

    image_source_t *isrc;
    int frameCount_;
    ImageController img_controller;

    double img_scale;
    double left_shift, right_shift;

    int x0, y0, x1, y1;
    int mouse_dejitter;
    int click_count;

    int myID;
};
