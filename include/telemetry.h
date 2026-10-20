#ifndef TELEMETRY_H
#define TELEMETRY_H

// One control-cycle sample written to the V5 Brain SD card.
// Motion states use inches for drive/bdrive and degrees for turn records.
struct TelemetrySample
{
    unsigned int segment;
    const char* motion;
    bool spatial_valid;
    double time_s;
    const char* position_unit;
    double ref_position;
    double measured_position;
    double ref_velocity;
    double estimated_velocity;
    double ref_acceleration;
    double disturbance_z3;
    const char* velocity_unit;
    const char* acceleration_unit;
    double cmd_volts;
    double battery_v;
    double left_volts;
    double right_volts;
    double ref_heading_deg;
    double measured_heading_deg;
    double path_ref_x_in;
    double path_ref_y_in;
    double pose_x_in;
    double pose_y_in;
    double cross_track_in;
};

// Starts a new uniquely named research_NNN.csv file; never overwrites earlier runs.
bool TelemetryBegin(const char* experiment, const char* condition, const char* routine);
void TelemetryEnd();
bool TelemetryActive();
bool TelemetryHealthy();
const char* TelemetryFilename();
double TelemetryTime();
unsigned int TelemetryBeginSegment();
void TelemetryLog(const TelemetrySample& sample);

#endif
