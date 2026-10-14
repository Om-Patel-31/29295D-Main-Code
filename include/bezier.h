#ifndef BEZIER_H
#define BEZIER_H

// Cubic Bezier path geometry in mm and rad. Configure derives control points
// from a displacement and final heading, then stores an arc-length lookup table.

struct Point2
{
    // Coordinates are expressed in the path frame: x forward and y lateral,
    // both in mm.
    double x;
    double y;
    Point2 operator+(const Point2& p) const { return {x + p.x, y + p.y}; }
    Point2 operator-(const Point2& p) const { return {x - p.x, y - p.y}; }
    Point2 operator*(double k) const { return {x * k, y * k}; }
};

class BezierPath
{
public:
    // samples is the fixed lookup-table resolution. PointAt and HeadingAt
    // consume arc distance in mm rather than the Bezier parameter t.
    static const int samples = 200;
    BezierPath();
    bool Configure(double dx, double dy, double end_heading = 0.0,
                   double minimum_radius = 0.0);
    Point2 PointAt(double distance) const;
    double HeadingAt(double distance) const;
    double CurvatureAt(double distance) const;
    double ProjectDistance(Point2 point) const;
    double length() const { return length_; }
    double max_curvature() const { return max_curvature_; }
    bool valid() const { return valid_; }

private:
    // p_ contains the four cubic control points; table_ maps normalized
    // parameter samples to accumulated arc length in mm.
    Point2 p_[4];
    double table_[samples + 1];
    double length_, max_curvature_;
    bool valid_;
    double ParameterAt(double distance) const;
    void BuildTable();
    double CurvatureAtParameter(double t) const;
    static Point2 Eval(const Point2 p[4], double t);
    static Point2 Derivative(const Point2 p[4], double t);
    static Point2 SecondDerivative(const Point2 p[4], double t);
};

#endif
