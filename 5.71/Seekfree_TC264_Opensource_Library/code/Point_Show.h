#ifndef CODE_POINT_SHOW_H_
#define CODE_POINT_SHOW_H_

#include "zf_common_headfile.h"

#define EARTH_RADIUS        (6371000.0)
#define EPSILON             (1e-9)
#define MAP_POINT_MAX       (20)
#define MAP_PI              (3.1415926535898)

typedef struct {
    double lat;
    double lon;
} gps_point;

void map_display_init  (int16 start_x, int16 start_y, int16 width, int16 high);
void map_transition    (gps_point *points, int16 point_num);
void map_display       (uint16 line_color, int16 wp_count);
void tft180_draw_yuan  (uint16 x, uint16 y, uint16 R, const uint16 color);

#endif
