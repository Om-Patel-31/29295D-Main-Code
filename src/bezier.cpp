#include "bezier.h"
#include <algorithm>
#include <cmath>

// Geometry is evaluated in the path frame. Curvature is signed 1/mm and
// arc-length queries are converted to the cubic parameter through table_.
static double norm(Point2 p) { return std::sqrt(p.x*p.x+p.y*p.y); }
static double cross(Point2 a, Point2 b) { return a.x*b.y-a.y*b.x; }

BezierPath::BezierPath():length_(0),max_curvature_(0),valid_(false)
{ for(int i=0;i<=samples;++i)table_[i]=0; }

Point2 BezierPath::Eval(const Point2 p[4], double t)
{
    double u=1-t; return p[0]*(u*u*u)+p[1]*(3*u*u*t)+p[2]*(3*u*t*t)+p[3]*(t*t*t);
}
Point2 BezierPath::Derivative(const Point2 p[4], double t)
{
    double u=1-t; return (p[1]-p[0])*(3*u*u)+(p[2]-p[1])*(6*u*t)+(p[3]-p[2])*(3*t*t);
}
Point2 BezierPath::SecondDerivative(const Point2 p[4], double t)
{ return (p[2]-p[1]*2+p[0])*(6*(1-t))+(p[3]-p[2]*2+p[1])*(6*t); }

double BezierPath::CurvatureAtParameter(double t) const
{
    Point2 d=Derivative(p_,t), dd=SecondDerivative(p_,t);
    double n=norm(d); return n < 1e-9 ? 0.0 : cross(d,dd)/(n*n*n);
}
void BezierPath::BuildTable()
{
    // Uniform parameter samples approximate arc length and provide a bounded
    // lookup cost for the controller's repeated projection queries.
    table_[0]=0; Point2 last=Eval(p_,0);
    for(int i=1;i<=samples;++i){Point2 q=Eval(p_,double(i)/samples);
        table_[i]=table_[i-1]+norm(q-last); last=q;}
    length_=table_[samples]; max_curvature_=0;
    for(int i=0;i<=samples;++i) max_curvature_=std::max(max_curvature_,std::fabs(CurvatureAtParameter(double(i)/samples)));
}
bool BezierPath::Configure(double dx,double dy,double end_heading,double min_radius)
{
    // p1 and p2 align the start/end tangents with the requested displacement
    // and final heading. Increasing their handle length reduces curvature.
    valid_=false; double chord=std::sqrt(dx*dx+dy*dy); if(chord<1e-9)return false;
    if (min_radius > 0.0 && min_radius < 0.01) return false;
    double d=0.4*chord, scale=1.0; Point2 end={dx,dy};
    for(int iteration=0;iteration<30;++iteration) {
        p_[0]={0,0}; p_[1]={d,0}; p_[2]={dx-d*std::cos(end_heading),dy-d*std::sin(end_heading)}; p_[3]=end;
        BuildTable(); if(min_radius<=0 || max_curvature_<=1.0/min_radius){valid_=true;return true;}
        scale*=1.15; d=0.4*chord*scale;
    }
    valid_=false; return false;
}
double BezierPath::ParameterAt(double distance) const
{
    distance=std::max(0.0,std::min(length_,distance));
    for(int i=1;i<=samples;++i) if(distance<=table_[i]){
        double f=(distance-table_[i-1])/std::max(1e-9,table_[i]-table_[i-1]);
        return (i-1+f)/samples;
    } return 1.0;
}
Point2 BezierPath::PointAt(double distance) const { return Eval(p_,ParameterAt(distance)); }
double BezierPath::HeadingAt(double distance) const { Point2 d=Derivative(p_,ParameterAt(distance)); return std::atan2(d.y,d.x); }
double BezierPath::CurvatureAt(double distance) const { return CurvatureAtParameter(ParameterAt(distance)); }
double BezierPath::ProjectDistance(Point2 point) const
{
    // Projection is a nearest sampled point, so it is bounded and allocation
    // free; its resolution is limited by the fixed samples constant.
    double best=0.0, best_error=1e300;
    for(int i=0;i<=samples;++i) {
        Point2 d=Eval(p_,double(i)/samples)-point;
        double e=d.x*d.x+d.y*d.y;
        if(e<best_error){best_error=e;best=table_[i];}
    }
    return best;
}
