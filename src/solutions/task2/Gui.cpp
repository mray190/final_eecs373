#include "Gui.h"

int VxWindow::onKeyEvent(vx_layer_t* layer, vx_key_event_t* event) {
    cout << event->released << " " << event->key_code << endl;
    if (event->released && event->key_code=='n') {
        lcm::LCM lcm_m;
        if (!lcm_m.good()) return 0;
        tic_tac_toe_turn_t turn;
        turn.utime = utime_now();
        turn_number++;
        turn.turn_number = turn_number;
        lcm_m.publish("TIC_TAC_TOE_TURN", &turn);
    }
    return 0;
}

int VxWindow::onMouseEvent(vx_layer_t* layer, vx_camera_pos_t* cameraPosition, vx_mouse_event_t* event) {
    if (event->button_mask==1 && mouse_dejitter==0) {
        click_count++;
       // double* vec3_out = new double[3];
        double vec3_out[3];
        if (click_count==1) {
            vx_ray3_t ray;
            vx_camera_pos_compute_ray(cameraPosition, event->x, event->y, &ray);
            vx_ray3_intersect_xy(&ray, 0, vec3_out);
            x0 = vec3_out[0]/img_scale + left_shift;
            y0 = 2*right_shift - (vec3_out[1]/img_scale + right_shift);
            cout << "Start: (" << x0 << "," << y0 << ")\n";
        } else if (click_count==2) {
            vx_ray3_t ray;
            vx_camera_pos_compute_ray(cameraPosition, event->x, event->y, &ray);
            vx_ray3_intersect_xy(&ray, 0,vec3_out);
            x1 = vec3_out[0]/img_scale + left_shift;
            y1 = 2*right_shift - (vec3_out[1]/img_scale + right_shift);
            cout << "End: (" << x1 << "," << y1 << ")\n";
            img_controller.setMask(x0,y0,x1,y1);
        }
        mouse_dejitter = 1;
    } else if (event->button_mask==0) {
        mouse_dejitter = 0;
    }
    // mouse_position = "<<#000000>>(" + to_string((int)event->x) + "," + to_string((int)event->y) + ")";
    return 0;
}

int VxWindow::setup(int _myID) {

    myID = _myID;

	//Get Camera url
	zarray_t *urls = image_source_enumerate ();
    char *img_url;
 	if (0==zarray_size (urls)) return -1;
    //Change the 0 to 1 to change to the overhead cameras
	zarray_get (urls, 1, &img_url);

    // Set up the imagesource
    isrc = image_source_open (img_url);

    if (isrc == NULL) return -2;
    else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
        for (int i = 0; i < isrc->num_formats (isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format (isrc, i, &ifmt);
            printf ("%3d: %4d x %4d (%s)\n",
                    i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start (isrc);
    }

    //Correctly set the image source
    return 0;
}

void VxWindow::render(void) {

    // Created world buffer
	vx_buffer_t* worldBuf = vx_world_get_buffer(world_, "map");
    // vx_buffer_add_back(worldBuf, vxo_grid());

    //Use the image source if it isn't null for the camera playback
    if (isrc != NULL) {

        //Retrieve the current frame of the image source
        //image_source_data_t *frmd = new image_source_data_t;
        image_source_data_t frmd;
        int res = isrc->get_frame (isrc, &frmd);
        if (res < 0)
            std::cout << "Get frame fail: " << res << "\n";

        else {

            //Convert the current frame to an image_u32
            image_u32_t *im = image_convert_u32 (&frmd);
            image_u32_t *new_im = img_controller.run(im, myID);
            if (new_im != NULL) {

                //Create a vx variable from the image_u32 object
                vx_object_t *vim = vxo_image_from_u32(new_im,
                                                      VXO_IMAGE_FLIPY,
                                                      VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
                img_scale = 2./new_im->width;
                left_shift = new_im->width/2.;
                right_shift = new_im->height/2.;
                vx_buffer_add_back (worldBuf,
                                    vxo_chain (vxo_mat_scale3 (img_scale, img_scale, 1.0),
                                               vxo_mat_translate3 (-left_shift, -right_shift, 0.),
                                               vim));

                //Swap the old frame with the new frame
                vx_buffer_swap (worldBuf);
                //image_u32_destroy (new_im);
            }
        }
        fflush (stdout);
        isrc->release_frame (isrc, &frmd);
    }
    ++frameCount_;
}
