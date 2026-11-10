#pragma once
// 纯几何算法，不依赖ROS或第三方数值库；单位为米。
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slicer_navigation {
struct Point2 {
  double x = 0., y = 0.;
  Point2() = default;
  Point2(double x_, double y_) : x(x_), y(y_) {}
  Point2 operator+(const Point2& b) const { return {x+b.x, y+b.y}; }
  Point2 operator-(const Point2& b) const { return {x-b.x, y-b.y}; }
  Point2 operator*(double s) const { return {x*s, y*s}; }
};
inline double norm(const Point2& p) { return std::hypot(p.x,p.y); }
inline double dot(const Point2& a,const Point2& b) { return a.x*b.x+a.y*b.y; }
inline double segmentDistance(const Point2& p,const Point2& a,const Point2& b) {
  const Point2 v=b-a;
  const double vv=dot(v,v);
  const double t=vv>0. ? std::max(0.,std::min(1.,dot(p-a,v)/vv)) : 0.;
  return norm(p-(a+v*t));
}

inline std::vector<Point2> simplifyRDP(const std::vector<Point2>& p,double epsilon) {
  if(p.size()<3) return p;
  std::vector<bool> keep(p.size(),false);
  keep.front()=keep.back()=true;
  std::vector<std::pair<size_t,size_t>> stack{{0,p.size()-1}};
  while(!stack.empty()) {
    const auto span=stack.back(); stack.pop_back();
    double farthest=epsilon; size_t split=span.first;
    for(size_t i=span.first+1;i<span.second;++i) {
      const double d=segmentDistance(p[i],p[span.first],p[span.second]);
      if(d>farthest) { farthest=d; split=i; }
    }
    if(split!=span.first) {
      keep[split]=true;
      stack.emplace_back(span.first,split); stack.emplace_back(split,span.second);
    }
  }
  std::vector<Point2> result;
  for(size_t i=0;i<p.size();++i) if(keep[i]) result.push_back(p[i]);
  return result;
}

inline std::vector<Point2> controlPolygon(const std::vector<Point2>& raw,
                                          double epsilon,double spacing) {
  const auto key=simplifyRDP(raw,epsilon);
  std::vector<Point2> out;
  if(key.empty()) return out;
  out.push_back(key.front());
  for(size_t i=1;i<key.size();++i) {
    const auto delta=key[i]-key[i-1];
    const int count=std::max(1,static_cast<int>(std::ceil(norm(delta)/spacing)));
    for(int j=1;j<=count;++j) out.push_back(key[i-1]+delta*(double(j)/count));
  }
  // 少于4个顶点时细分最长边，而不是重复端点，避免零切向量。
  while(out.size()>1 && out.size()<4) {
    size_t longest=1;
    for(size_t i=2;i<out.size();++i)
      if(norm(out[i]-out[i-1])>norm(out[longest]-out[longest-1])) longest=i;
    out.insert(out.begin()+longest,(out[longest-1]+out[longest])*.5);
  }
  return out;
}

class CubicBSpline {
 public:
  explicit CubicBSpline(std::vector<Point2> controls) : controls_(std::move(controls)) {
    if(controls_.size()<4) throw std::invalid_argument("cubic spline needs four controls");
    const size_t m=controls_.size();
    knots_.resize(m+4);
    for(size_t i=0;i<knots_.size();++i)
      knots_[i]=i<=3 ? 0. : (i>=m ? 1. : double(i-3)/double(m-3));
  }
  Point2 at(double u) const { return evaluate(controls_,knots_,3,u); }
  Point2 derivative(double u) const { return differentiated(1,u); }
  Point2 secondDerivative(double u) const { return differentiated(2,u); }
  const std::vector<double>& knots() const { return knots_; }
  double derivativeBound() const {
    double maximum=0.;
    for(size_t i=0;i+1<controls_.size();++i)
      maximum=std::max(maximum,3.*norm(controls_[i+1]-controls_[i])/(knots_[i+4]-knots_[i+1]));
    return maximum;
  }
 private:
  static Point2 evaluate(const std::vector<Point2>& c,const std::vector<double>& t,int degree,double u) {
    u=std::max(0.,std::min(1.,u));
    size_t span=c.size()-1;
    if(u<1.) span=std::upper_bound(t.begin()+degree,t.begin()+c.size()+1,u)-t.begin()-1;
    std::vector<Point2> d(degree+1);
    for(int j=0;j<=degree;++j) d[j]=c[span-degree+j];
    for(int r=1;r<=degree;++r) for(int j=degree;j>=r;--j) {
      const size_t i=span-degree+j;
      const double denominator=t[i+degree-r+1]-t[i];
      const double alpha=denominator>0. ? (u-t[i])/denominator : 0.;
      d[j]=d[j-1]*(1.-alpha)+d[j]*alpha;
    }
    return d[degree];
  }
  Point2 differentiated(int order,double u) const {
    auto c=controls_; auto t=knots_; int degree=3;
    for(int r=0;r<order;++r) {
      std::vector<Point2> dc;
      for(size_t i=0;i+1<c.size();++i)
        dc.push_back((c[i+1]-c[i])*(double(degree)/(t[i+degree+1]-t[i+1])));
      c=std::move(dc); t=std::vector<double>(t.begin()+1,t.end()-1); --degree;
    }
    return evaluate(c,t,degree,u);
  }
  std::vector<Point2> controls_;
  std::vector<double> knots_;
};

struct PathSample { Point2 position; double yaw=0., u=0.; };
struct SmoothResult {
  std::vector<PathSample> path;
  int refinement=0;
  double length=0.;
};

// 采用解析导数计算航向；先建立细密弧长表，再反查参数u，最终点仍在原样条上。
inline bool sampleSpline(const CubicBSpline& curve,double output_step,double check_step,
                         const std::function<bool(const PathSample&)>& valid,
                         SmoothResult& result) {
  const double bound=curve.derivativeBound();
  if(!std::isfinite(bound) || bound<=1e-10) return false;
  const double fine_step=std::min(check_step,output_step*.2);
  const size_t count=std::max(size_t(16),static_cast<size_t>(std::ceil(bound/fine_step)));
  std::vector<double> lengths(count+1,0.);
  Point2 previous=curve.at(0.);
  for(size_t i=0;i<=count;++i) {
    const double u=double(i)/count;
    const Point2 p=curve.at(u),d=curve.derivative(u);
    // 非正则曲线（尖点/零导数）不冒充曲率连续路径，交回原规划器处理。
    if(norm(d)<1e-9 || !std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    const PathSample s{p,std::atan2(d.y,d.x),u};
    if(!valid(s)) return false;
    if(i) lengths[i]=lengths[i-1]+norm(p-previous);
    previous=p;
  }
  result.length=lengths.back();
  if(result.length<1e-8) return false;
  result.path.clear();
  const size_t samples=std::max(size_t(1),static_cast<size_t>(std::ceil(result.length/output_step)));
  size_t segment=1;
  for(size_t i=0;i<=samples;++i) {
    const double distance=std::min(result.length,double(i)*output_step);
    while(segment<count && lengths[segment]<distance) ++segment;
    const double ds=lengths[segment]-lengths[segment-1];
    const double fraction=ds>0. ? (distance-lengths[segment-1])/ds : 0.;
    const double u=i==samples ? 1. : (double(segment-1)+fraction)/count;
    const auto d=curve.derivative(u);
    result.path.push_back({curve.at(u),std::atan2(d.y,d.x),u});
  }
  return true;
}

inline bool smoothPath(const std::vector<Point2>& input,double epsilon,double control_spacing,
                       double output_step,double check_step,int refinements,
                       const std::function<bool(const PathSample&)>& valid,SmoothResult& result) {
  std::vector<Point2> raw;
  for(const auto& p:input)
    if(raw.empty() || norm(p-raw.back())>1e-9) raw.push_back(p);
  if(raw.size()<2) return false;
  for(int attempt=0;attempt<=refinements;++attempt) {
    // 内切碰撞时逐级靠回原折线；只改变控制点，不拼接破坏C2的局部折线段。
    const double e=attempt==refinements ? 0. : epsilon*std::pow(.5,attempt);
    const double spacing=control_spacing*std::pow(.5,attempt);
    CubicBSpline curve(controlPolygon(raw,e,spacing));
    SmoothResult candidate;
    if(sampleSpline(curve,output_step,check_step,valid,candidate)) {
      candidate.refinement=attempt; result=std::move(candidate); return true;
    }
  }
  return false;  // 调用方完整返回Dijkstra原路径，不阻断导航、不拼接不连续曲线。
}
}  // namespace slicer_navigation
