#include "telemetry.h"
#include "robot.h"
#include "vex.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdint.h>

namespace {
const int kBufferSize = 4096;
const int kFlushThreshold = 3072;
uint8_t buffer[kBufferSize];
int used = 0;
char filename[32] = {0};
char experiment_label[48] = {0};
char condition_label[48] = {0};
char routine_label[48] = {0};
bool active = false;
bool healthy = false;
unsigned int segment = 0;
vex::timer run_clock;

void copy_label(char* destination, size_t capacity, const char* source)
{
    if (capacity == 0) return;
    size_t out = 0;
    if (source) {
        while (source[out] && out + 1 < capacity) {
            const char c = source[out];
            destination[out] = ((c >= 'a' && c <= 'z') ||
                                (c >= 'A' && c <= 'Z') ||
                                (c >= '0' && c <= '9') ||
                                c == '_' || c == '-') ? c : '_';
            ++out;
        }
    }
    destination[out] = '\0';
    if (out == 0) snprintf(destination, capacity, "unspecified");
}

bool flush_buffer()
{
    if (used == 0) return true;
    const int32_t written = Brain.SDcard.appendfile(filename, buffer, used);
    if (written != used) {
        healthy = false;
        active = false;
        return false;
    }
    used = 0;
    return true;
}

bool append_bytes(const uint8_t* bytes, size_t length)
{
    if (!active || !healthy || !bytes || length > sizeof(buffer)) return false;
    if (used + static_cast<int>(length) > kBufferSize && !flush_buffer()) return false;
    std::memcpy(buffer + used, bytes, length);
    used += static_cast<int>(length);
    if (used >= kFlushThreshold) return flush_buffer();
    return true;
}

bool append_text(const char* text)
{
    if (!text) {
        healthy = false;
        active = false;
        return false;
    }
    return append_bytes(reinterpret_cast<const uint8_t*>(text), std::strlen(text));
}
}

bool TelemetryBegin(const char* experiment, const char* condition, const char* routine)
{
    TelemetryEnd();
    active = false;
    healthy = false;
    used = 0;
    filename[0] = '\0';
    if (!Brain.SDcard.isInserted()) return false;

    char candidate[32];
    bool found_name = false;
    for (unsigned int index = 1; index <= 999; ++index) {
        snprintf(candidate, sizeof(candidate), "research_%03u.csv", index);
        if (!Brain.SDcard.exists(candidate)) {
            found_name = true;
            break;
        }
    }
    if (!found_name) return false;

    copy_label(experiment_label, sizeof(experiment_label), experiment);
    copy_label(condition_label, sizeof(condition_label), condition);
    copy_label(routine_label, sizeof(routine_label), routine);
    uint8_t header[] =
        "experiment,condition,routine,run_segment,motion,time_s,position_unit,"
        "ref_position,meas_position,ref_velocity,est_velocity,ref_acceleration,"
        "disturb_z3,velocity_unit,acceleration_unit,axis_cmd_volts,battery_v,"
        "left_volts,right_volts,ref_heading_deg,measured_heading_deg,"
        "path_ref_x_in,path_ref_y_in,pose_x_in,pose_y_in,cross_track_in\n";
    const int32_t header_size = static_cast<int32_t>(sizeof(header) - 1);
    if (Brain.SDcard.savefile(candidate, header, header_size) != header_size) return false;

    if (!Brain.SDcard.exists(candidate)) {
        healthy = false;
        return false;
    }

    snprintf(filename, sizeof(filename), "%s", candidate);
    active = true;
    healthy = true;
    segment = 0;
    used = 0;
    run_clock.reset();
    char metadata[160];
    const int metadata_size = snprintf(metadata, sizeof(metadata),
        "%s,%s,%s,,,,,,,,,,,,,,,,,,,,,,,\n",
        experiment_label, condition_label, routine_label);
    if (metadata_size < 0 || metadata_size >= static_cast<int>(sizeof(metadata)) ||
        !append_text(metadata) || !healthy || !flush_buffer()) {
        active = false;
        healthy = false;
        return false;
    }
    return true;
}

void TelemetryEnd()
{
    if (active && healthy) flush_buffer();
    active = false;
}

bool TelemetryActive() { return active; }
bool TelemetryHealthy() { return healthy; }
const char* TelemetryFilename() { return filename; }
double TelemetryTime() { return run_clock.time(vex::sec); }
unsigned int TelemetryBeginSegment() { return ++segment; }

void TelemetryLog(const TelemetrySample& s)
{
    if (!active || !healthy) return;
    const double values[] = {
        s.time_s, s.ref_position, s.measured_position, s.ref_velocity,
        s.estimated_velocity, s.ref_acceleration, s.disturbance_z3,
        s.cmd_volts, s.battery_v, s.left_volts, s.right_volts,
        s.ref_heading_deg, s.measured_heading_deg
    };
    if (s.spatial_valid &&
        (!std::isfinite(s.path_ref_x_in) || !std::isfinite(s.path_ref_y_in) ||
         !std::isfinite(s.pose_x_in) || !std::isfinite(s.pose_y_in) ||
         !std::isfinite(s.cross_track_in))) {
        healthy = false;
        active = false;
        return;
    }
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        if (!std::isfinite(values[i])) {
            healthy = false;
            active = false;
            return;
        }
    }

    char row[512];
    char spatial_fields[160];
    if (s.spatial_valid) {
        snprintf(spatial_fields, sizeof(spatial_fields),
            "%.4f,%.4f,%.4f,%.4f,%.4f", s.path_ref_x_in, s.path_ref_y_in,
            s.pose_x_in, s.pose_y_in, s.cross_track_in);
    } else {
        snprintf(spatial_fields, sizeof(spatial_fields), ",,,,");
    }
    const int length = snprintf(row, sizeof(row),
        "%s,%s,%s,%u,%s,%.4f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n",
        experiment_label, condition_label, routine_label, s.segment,
        s.motion ? s.motion : "unknown", s.time_s,
        s.position_unit ? s.position_unit : "unknown",
        s.ref_position, s.measured_position, s.ref_velocity,
        s.estimated_velocity, s.ref_acceleration, s.disturbance_z3,
        s.velocity_unit ? s.velocity_unit : "unknown",
        s.acceleration_unit ? s.acceleration_unit : "unknown",
        s.cmd_volts, s.battery_v, s.left_volts, s.right_volts,
        s.ref_heading_deg, s.measured_heading_deg, spatial_fields);
    if (length < 0 || length >= static_cast<int>(sizeof(row))) {
        healthy = false;
        active = false;
        return;
    }
    if (!append_text(row)) {
        healthy = false;
        active = false;
    }
}
