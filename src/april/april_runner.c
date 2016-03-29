#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"

#include "zarray.h"
#include "getopt.h"

#include "../lcmtypes/april_tag_t.h"
#include <lcm/lcm.h>

#define CMD_PRD 100000 //us  -> 5Hz
apriltag_detector_t *td;

void runner() {
    int i;
    char* pnm = "temp.pnm";
    lcm_t * lcm = lcm_create(NULL);
    if (!lcm) return;
    while(1) {
        image_u8_t *image = image_u8_create_from_pnm(pnm);
        if (image != NULL) {
            zarray_t *detections = apriltag_detector_detect(td, image);
            for (i=0; i<zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                april_tag_t tag;
                tag.x = det->c[0];
                tag.y = det->c[1];
                tag.id = det->id;
                tag.tl_x = det->p[2][0]; //2
                tag.tl_y = det->p[2][1]; //2
                tag.tr_x = det->p[3][0]; //3
                tag.tr_y = det->p[3][1]; //3
                tag.br_x = det->p[0][0]; //0
                tag.br_y = det->p[0][1]; //0
                tag.bl_x = det->p[1][0]; //1
                tag.bl_y = det->p[1][1]; //1
                april_tag_t_publish(lcm, "APRIL_TAGS", &tag);
            }
            image_u8_destroy (image);
        }
        usleep (100000);
    }
}

int main(int argc, char** argv) {
    apriltag_family_t *tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    runner();
    return 0;
}
