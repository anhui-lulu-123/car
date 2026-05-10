#include "Point_Show.h"
#include <math.h>
#include <string.h>

static int16 disp_x   = 0;
static int16 disp_y   = 0;
static int16 disp_w   = 0;
static int16 disp_h   = 0;
static int16 pt_num   = 0;

typedef struct { double x; double y; } plane_pt;
typedef struct { double x; double y; } screen_pt;

static plane_pt  plane_pts [MAP_POINT_MAX];
static screen_pt screen_pts[MAP_POINT_MAX];

static void to_plane(gps_point *in)
{
    double mlat = (MAP_PI / 180.0) * EARTH_RADIUS;
    double mlon = mlat * cos(in[0].lat * MAP_PI / 180.0);
    plane_pts[0].x = 0;
    plane_pts[0].y = 0;
    for(int i = 1; i < pt_num; i++) {
        plane_pts[i].x = (in[i].lon - in[0].lon) * mlon;
        plane_pts[i].y = (in[i].lat - in[0].lat) * mlat;
    }
}

static void to_screen(void)
{
    double min_x = plane_pts[0].x, max_x = plane_pts[0].x;
    double min_y = plane_pts[0].y, max_y = plane_pts[0].y;

    for(int i = 1; i < pt_num; i++) {
        if(plane_pts[i].x < min_x) min_x = plane_pts[i].x;
        if(plane_pts[i].x > max_x) max_x = plane_pts[i].x;
        if(plane_pts[i].y < min_y) min_y = plane_pts[i].y;
        if(plane_pts[i].y > max_y) max_y = plane_pts[i].y;
    }

    double scale = 1.0, x_off = 0, y_off = 0;
    if(max_x - min_x > EPSILON && max_y - min_y > EPSILON) {
        double sx = (double)disp_w / (max_x - min_x);
        double sy = (double)disp_h / (max_y - min_y);
        scale = sx < sy ? sx : sy;
        if(scale == sx) y_off = ((double)disp_h - (max_y - min_y) * scale) / 2.0;
        else            x_off = ((double)disp_w - (max_x - min_x) * scale) / 2.0;
    }

    for(int i = 0; i < pt_num; i++) {
        screen_pts[i].x =           (plane_pts[i].x - min_x) * scale + x_off;
        screen_pts[i].y = disp_h - ((plane_pts[i].y - min_y) * scale + y_off); // 北朝上
    }
}

void map_display_init(int16 start_x, int16 start_y, int16 width, int16 high)
{
    disp_x = start_x;
    disp_y = start_y;
    disp_w = width - 1;
    disp_h = high  - 1;
}

void map_transition(gps_point *points, int16 point_num)
{
    pt_num = point_num;
    memset(plane_pts,  0, sizeof(plane_pts));
    memset(screen_pts, 0, sizeof(screen_pts));
    to_plane(points);
    to_screen();
}

void tft180_draw_yuan(uint16 x, uint16 y, uint16 R, const uint16 color)
{
    if(R == 0) return;
    tft180_draw_point(x, y, color);
    for(int16 a = 0; a <= 8; a++) {
        int32 cx = (int32)x + R * cos(a * MAP_PI / 4);
        int32 cy = (int32)y + R * sin(a * MAP_PI / 4);
        if(cx >= 0 && cy >= 0) {
            tft180_draw_point((uint16)cx, (uint16)cy, color);
            tft180_draw_point((uint16)(2*(int32)x - cx), (uint16)cy, color);
            tft180_draw_point((uint16)cx, (uint16)(2*(int32)y - cy), color);
            tft180_draw_point((uint16)(2*(int32)x - cx), (uint16)(2*(int32)y - cy), color);
        }
    }
}

void map_display(uint16 line_color, int16 wp_count)
{
    // 画连线
    for(int i = 0; i < pt_num - 1; i++) {
        tft180_draw_line(
            (uint16)screen_pts[i].x   + disp_x, (uint16)screen_pts[i].y   + disp_y,
            (uint16)screen_pts[i+1].x + disp_x, (uint16)screen_pts[i+1].y + disp_y,
            line_color);
    }
    // 画航点圆圈（仅航点，不含当前位置）
    for(int i = 0; i < wp_count; i++) {
        tft180_draw_yuan((uint16)screen_pts[i].x + disp_x,
                         (uint16)screen_pts[i].y + disp_y,
                         2, RGB565_BLACK);
    }
    // 当前位置用红色实心点标出（最后一个点）
    if(pt_num > wp_count) {
        int last = pt_num - 1;
        tft180_draw_yuan((uint16)screen_pts[last].x + disp_x,
                         (uint16)screen_pts[last].y + disp_y,
                         3, RGB565_RED);
    }
}
