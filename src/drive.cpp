#include "drive.h"
#include "config.h"
#include <algorithm>
#include <cmath>

// The controller combines independent translation and heading LADRC loops.
// It never includes VEX headers; the hardware layer supplies measured pose and
// battery voltage in SI-like project units (mm, rad, and V).
namespace {
double AvailableDriveVoltage(double supply_voltage)
{
    // A missing battery reading uses the configured 12 V limit rather than
    // forcing a false zero command.
    if (!std::isfinite(supply_voltage) || supply_voltage <= 0.0) {
        return config::max_voltage;
    }
    return std::min(config::max_voltage, supply_voltage);
}
}

double WrapToNear(double angle, double reference)
{
    // Select the equivalent angle nearest the local path heading to avoid a
    // discontinuity when inertial heading crosses +/-pi rad.
    const double pi = std::acos(-1.0);
    return reference + std::remainder(angle - reference, 2.0 * pi);
}

DriveController::DriveController(double lwc,double lb0,double hwc,double hb0)
    :path_(),profile_(),longitudinal_(lwc,lb0),heading_(hwc,hb0),elapsed_(0),
     hold_time_(0),active_(false),faulted_(false){}
bool DriveController::StartPath(const BezierPath& path, const Pose2& measured,
                                double vmax,double amax,double jmax)
{
    // StartPath resets lifecycle state before validation, so a rejected
    // restart cannot leave an earlier path active.
    active_ = false;
    faulted_ = false;
    if (!path.valid() ||
        !ValidateScurveLimits(amax, jmax, longitudinal_.input_gain(),
                              longitudinal_.bandwidth(), config::max_voltage))
        return false;
    path_=path;
    const double kappa = path_.max_curvature();
    // v^2*kappa is lateral acceleration; this limit converts the configured
    // lateral acceleration ceiling into a safe path speed in mm/s.
    const double curvature_limit = kappa > 0.0
        ? std::sqrt(config::lateral_acceleration_max / kappa)
        : vmax;
    const double effective_vmax = std::min(vmax, curvature_limit);
    active_=profile_.Configure(path_.length(),effective_vmax,amax,jmax);
    elapsed_=0;
    hold_time_=0;
    const double projected = path_.ProjectDistance({measured.x, measured.y});
    const double s0 = std::max(0.0, std::min(path_.length(), projected));
    const double heading_reference = path_.HeadingAt(s0);
    longitudinal_.Reset(s0,0);
    heading_.Reset(WrapToNear(measured.heading, heading_reference),0);
    longitudinal_.SetAppliedInput(0.0);
    heading_.SetAppliedInput(0.0);
    return active_;
}
void DriveController::Reset(const Pose2& p){longitudinal_.Reset(0,0);heading_.Reset(p.heading,0);longitudinal_.SetAppliedInput(0.0);heading_.SetAppliedInput(0.0);elapsed_=0;hold_time_=0;active_=false;faulted_=false;}
DriveVoltages DriveController::Update(double dt,const Pose2& measured,
                                       double supply_voltage)
{
    // Until final hold completes, each update computes references, feedback,
    // voltage mixing, and applied-input feedback for the next ESO update.
    if (!active_) return {0, 0};
    const bool profile_done = elapsed_ >= profile_.duration();
    const double final_heading = path_.HeadingAt(path_.length());
    ScurveState ref = profile_done
        ? ScurveState{path_.length(), 0.0, 0.0, true}
        : profile_.Sample(elapsed_);
    double measured_s=path_.ProjectDistance({measured.x,measured.y});
    double theta=path_.HeadingAt(ref.position), omega=ref.velocity*path_.CurvatureAt(ref.position);
    double u_long=longitudinal_.Update(dt,measured_s,ref.position,ref.velocity,ref.acceleration);
    const double measured_heading = WrapToNear(measured.heading, theta);
    double u_turn=heading_.Update(dt,measured_heading,theta,omega,0);
    if (!longitudinal_.valid() || !heading_.valid()) {
        faulted_ = true;
        active_ = false;
        return {0, 0};
    }
    if (profile_done) {
        const double position_error = std::fabs(path_.length() - measured_s);
        const double heading_error = std::fabs(
            WrapToNear(measured.heading, final_heading) - final_heading);
        if (position_error <= config::position_tolerance_mm &&
            heading_error <= config::heading_tolerance_rad) {
            hold_time_ += dt;
        } else {
            hold_time_ = 0.0;
        }
        if (hold_time_ >= config::settle_time_s) {
            active_ = false;
        }
    }
    const double voltage_limit = AvailableDriveVoltage(supply_voltage);
    // The heading loop uses U_turn, so positive turn increases right voltage
    // and decreases left voltage according to the documented plant convention.
    const double left_request = u_long - u_turn;
    const double right_request = u_long + u_turn;
    const double peak = std::max(std::fabs(left_request), std::fabs(right_request));
    // Common-mode scaling preserves the requested left/right ratio while
    // ensuring neither motor command exceeds the available voltage.
    const bool saturated = peak > voltage_limit;
    const double scale = saturated ? voltage_limit / peak : 1.0;
    const double left = left_request * scale;
    const double right = right_request * scale;
    longitudinal_.SetAppliedInput(0.5 * (left + right));
    heading_.SetAppliedInput(0.5 * (right - left));
    // LIMITATION: no stall timeout exists; a saturated stalled path remains
    // active until its caller's autonomous timeout or another fault occurs.
    if(!profile_done && !saturated) elapsed_+=dt;
    return {left,right};
}
