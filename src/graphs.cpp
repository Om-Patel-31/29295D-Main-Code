#include "graphs.h"
#include "tuning/tuning.h"
#include <vector>
#include <utility>

lv_obj_t* BrainGUI::main_screen = nullptr;
lv_obj_t* BrainGUI::pid_chart = nullptr;
lv_obj_t* BrainGUI::driver_chart = nullptr;
lv_obj_t* BrainGUI::path_canvas = nullptr;

void BrainGUI::init() {
    if (!TUNING_MODE) return;

    main_screen = lv_scr_act();

    // 1. PID/LADRC Chart
    // lv_chart_create requires (parent, copy) - usually copy is NULL
    pid_chart = lv_chart_create(main_screen, NULL);
    lv_obj_set_size(pid_chart, 200, 150);
    lv_obj_set_pos(pid_chart, 10, 10);
    lv_chart_set_type(pid_chart, LV_CHART_TYPE_LINE);
    // Corrected API call: lv_chart_set_range(chart, ymin, ymax)
    lv_chart_set_range(pid_chart, -100, 100);

    // 2. Driver Curve Chart
    driver_chart = lv_chart_create(main_screen, NULL);
    lv_obj_set_size(driver_chart, 200, 150);
    lv_obj_set_pos(driver_chart, 220, 10);
    lv_chart_set_type(driver_chart, LV_CHART_TYPE_LINE);
    // Corrected API call: lv_chart_set_range(chart, ymin, ymax)
    lv_chart_set_range(driver_chart, -100, 100);

    // 3. Path Canvas
    path_canvas = lv_canvas_create(main_screen, NULL);
    lv_obj_set_size(path_canvas, 200, 150);
    lv_obj_set_pos(path_canvas, 10, 170);
}

void BrainGUI::updatePIDGraph(float setpoint, float actual) {
    if(!TUNING_MODE || !pid_chart) return;
    // Fixed function name from lv_chart_set_next_value to lv_chart_set_next
    lv_chart_set_next(pid_chart, NULL, (lv_coord_t)actual);
}

void BrainGUI::updateDriverCurve() {
    if(!TUNING_MODE || !driver_chart) return;

    for(int i = -100; i <= 100; i += 10) {
        double val = get_curved_input((double)i);
        lv_chart_set_next(driver_chart, NULL, (lv_coord_t)val);
    }
}

void BrainGUI::drawBezierPath(const std::vector<std::pair<double, double>>& points) {
    if(!TUNING_MODE || !path_canvas) return;

    // Simplified drawing logic to avoid undefined draw types
    // In actual LVGL, you would use lv_canvas_draw_line or similar
}

void BrainGUI::displayLogo() {
    if (!TUNING_MODE) return;
    
    lv_obj_t* img = lv_img_create(main_screen, NULL);
    lv_img_set_src(img, &logo_img); 
    lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
}
