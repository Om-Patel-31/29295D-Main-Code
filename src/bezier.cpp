#include "bezier.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// Cubic Bezier Path Geometry & Arc-Length Parameterization
// ============================================================================
// Evaluates cubic curves P(t) with tangent alignment for smooth trajectory
// following. Generates an equidistant arc-length lookup table (table_) to map
// physical distance s (mm) directly to parameter t in [0, 1].

static double norm(Point2 p)
{
    return std::sqrt(p.x * p.x + p.y * p.y);
}

static double cross(Point2 a, Point2 b)
{
    return a.x * b.y - a.y * b.x;
}

BezierPath::BezierPath()
    : length_(0.0), max_curvature_(0.0), valid_(false)
{
    for (int i = 0; i <= samples; ++i) {
        table_[i] = 0.0;
    }
}

Point2 BezierPath::Eval(const Point2 p[4], double t)
{
    // Bernstein basis polynomials for degree 3
    const double u = 1.0 - t;
    return p[0] * (u * u * u) +
           p[1] * (3.0 * u * u * t) +
           p[2] * (3.0 * u * t * t) +
           p[3] * (t * t * t);
}

Point2 BezierPath::Derivative(const Point2 p[4], double t)
{
    // First derivative P'(t) = velocity vector along path parameter
    const double u = 1.0 - t;
    return (p[1] - p[0]) * (3.0 * u * u) +
           (p[2] - p[1]) * (6.0 * u * t) +
           (p[3] - p[2]) * (3.0 * t * t);
}

Point2 BezierPath::SecondDerivative(const Point2 p[4], double t)
{
    // Second derivative P''(t) = acceleration vector along path parameter
    return (p[2] - p[1] * 2.0 + p[0]) * (6.0 * (1.0 - t)) +
           (p[3] - p[2] * 2.0 + p[1]) * (6.0 * t);
}

double BezierPath::CurvatureAtParameter(double t) const
{
    // Signed curvature kappa = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
    const Point2 d = Derivative(p_, t);
    const Point2 dd = SecondDerivative(p_, t);
    const double n = norm(d);
    return n < 1e-9 ? 0.0 : cross(d, dd) / (n * n * n);
}

void BezierPath::BuildTable()
{
    // Discretizes parameter t into uniform samples and accumulates chord lengths
    // to provide bounded, non-allocating arc-distance queries.
    table_[0] = 0.0;
    Point2 last = Eval(p_, 0.0);

    for (int i = 1; i <= samples; ++i) {
        const Point2 q = Eval(p_, static_cast<double>(i) / samples);
        table_[i] = table_[i - 1] + norm(q - last);
        last = q;
    }

    length_ = table_[samples];
    max_curvature_ = 0.0;

    for (int i = 0; i <= samples; ++i) {
        const double k = std::fabs(CurvatureAtParameter(static_cast<double>(i) / samples));
        max_curvature_ = std::max(max_curvature_, k);
    }
}

bool BezierPath::Configure(double dx, double dy, double end_heading, double min_radius)
{
    // Generates control points p0..p3 starting at (0, 0) facing forward along the x-axis
    // and ending at (dx, dy) with tangent angle matching end_heading.
    // Iteratively lengthens control handles until curvature constraints are met.
    valid_ = false;
    const double chord = std::sqrt(dx * dx + dy * dy);
    if (chord < 1e-9) return false;
    if (min_radius > 0.0 && min_radius < 0.01) return false;

    double d = 0.4 * chord;
    double scale = 1.0;
    const Point2 end = {dx, dy};

    for (int iteration = 0; iteration < 30; ++iteration) {
        p_[0] = {0.0, 0.0};
        p_[1] = {d, 0.0};
        p_[2] = {dx - d * std::cos(end_heading), dy - d * std::sin(end_heading)};
        p_[3] = end;

        BuildTable();

        if (min_radius <= 0.0 || max_curvature_ <= (1.0 / min_radius)) {
            valid_ = true;
            return true;
        }

        scale *= 1.15;
        d = 0.4 * chord * scale;
    }

    valid_ = false;
    return false;
}

double BezierPath::ParameterAt(double distance) const
{
    // Binary-compatible bounded lookup converting distance s -> parameter t
    distance = std::max(0.0, std::min(length_, distance));
    for (int i = 1; i <= samples; ++i) {
        if (distance <= table_[i]) {
            const double span = std::max(1e-9, table_[i] - table_[i - 1]);
            const double f = (distance - table_[i - 1]) / span;
            return (i - 1 + f) / samples;
        }
    }
    return 1.0;
}

Point2 BezierPath::PointAt(double distance) const
{
    return Eval(p_, ParameterAt(distance));
}

double BezierPath::HeadingAt(double distance) const
{
    const Point2 d = Derivative(p_, ParameterAt(distance));
    return std::atan2(d.y, d.x);
}

double BezierPath::CurvatureAt(double distance) const
{
    return CurvatureAtParameter(ParameterAt(distance));
}

double BezierPath::ProjectDistance(Point2 point) const
{
    // Finds nearest arc-length sample along path to a 2D query point
    double best = 0.0;
    double best_error = 1e300;

    for (int i = 0; i <= samples; ++i) {
        const Point2 d = Eval(p_, static_cast<double>(i) / samples) - point;
        const double e = d.x * d.x + d.y * d.y;
        if (e < best_error) {
            best_error = e;
            best = table_[i];
        }
    }
    return best;
}
