#ifndef GRAPHS_H
#define GRAPHS_H

#include "vex.h"
#include "lvgl/lvgl.h" 
#include "lv_conf.h"
#include <vector>
#include <utility>

// External declaration of the image symbol from 29295D_logo.c
extern const lv_img_dsc_t logo_img;

class BrainGUI {
public:
    static void init();
    static void updatePIDGraph(float setpoint, float actual);
    static void updateDriverCurve();
    static void drawBezierPath(const std::vector<std::pair<double, double>>& points);
    static void displayLogo();

private:
    static lv_obj_t* main_screen;
    static lv_obj_t* pid_chart;
    static lv_obj_t* driver_chart;
    static lv_obj_t* path_canvas;
};

#endif