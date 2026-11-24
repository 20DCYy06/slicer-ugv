#include <slicer_navigation/clearance_preference.h>
#include <slicer_navigation/bspline_core.h>
#include <slicer_navigation/route_continuity.h>
#include <cstdlib>
#include <iostream>
using namespace slicer_navigation;
void require(bool good,const char* message) {
  if(!good) { std::cerr<<"FAIL: "<<message<<std::endl; std::exit(1); }
}
int main() {
  std::vector<std::uint8_t> map(21*21, 0);
  const auto at = [&](int x, int y) -> std::uint8_t& { return map[y*21+x]; };
  at(10,10) = 254;
  at(12,10) = 255;
  at(13,10) = 171;
  preferClearance(map.data(), 21, 21, .05, .45, .14, 4.5, 173);
  require(at(10,10)==254 && at(12,10)==255, "preference preserves lethal and unknown");
  require(at(13,10)>=171 && at(13,10)<=173, "preference respects the traversable threshold");
  require(at(16,10)>at(18,10) && at(18,10)>0 && at(20,10)==0,
          "wider planning preference decays to no extra cost");
  const auto observed=[](const Point2& p) { return p.x < .6 ? 0 : 255; };
  const std::vector<Point2> route{{0,0},{.3,0},{1,0}};
  require(std::isfinite(routeCost(route, .025, 0., observed)), "route may reach an unknown goal");
  require(std::isfinite(routeCost(route, .025, .3, observed)), "connector entirely observed");
  require(!std::isfinite(routeCost(route, .025, .7, observed)), "reuse cannot reconnect through unknown space");
  const auto obstruction=[](const Point2& p) { return p.x>.4 && p.x<.48 ? 254 : 0; };
  require(!std::isfinite(routeCost(route, .025, 0., obstruction)), "new obstacle invalidates old route");
  const auto near_obstacle = [](const Point2& p) { return p.x > .4 ? 200 : 0; };
  require(!std::isfinite(routeCost(route, .025, 0., near_obstacle, 173)),
          "route reuse must respect the active inflation contour");
  require(preferPreviousRoute(4.2, 4., .35), "near equal routes keep direction");
  require(!preferPreviousRoute(4.5, 4., .35), "meaningfully better route replaces old route");
  const auto free=[](const PathSample&) { return true; };
  std::vector<Point2> stairs;
  for(int i=0;i<21;++i) stairs.emplace_back(i*.05, .002*(i%2));
  require(simplifyRDP(stairs,.01).size()==2,"RDP must remove stair noise");
  auto control=controlPolygon({{0,0},{1,0},{1,1},{2,1}},.02,.2);
  CubicBSpline curve(control);
  require(norm(curve.at(0.)-Point2(0.,0.))<1e-12,"exact start");
  require(norm(curve.at(1.)-Point2(2.,1.))<1e-12,"exact goal");
  for(double u:curve.knots()) if(u>0. && u<1.) {
    const double h=1e-8;
    require(norm(curve.at(u+h)-curve.at(u-h))<1e-5,"C0 at knots");
    require(norm(curve.derivative(u+h)-curve.derivative(u-h))<1e-4,"C1 at knots");
    require(norm(curve.secondDerivative(u+h)-curve.secondDerivative(u-h))<1e-3,"C2 at knots");
  }
  SmoothResult result;
  require(smoothPath({{0,0},{1,0},{1,1},{2,1}},.03,.2,.05,.025,2,free,result),"smooth L/S bends");
  for(size_t i=1;i<result.path.size();++i) {
    const double distance=norm(result.path[i].position-result.path[i-1].position);
    require(distance<=.0501,"arc sampling must not exceed spacing");
    if(i+1<result.path.size()) require(distance>.046,"arc sampling near requested spacing");
    require(std::isfinite(result.path[i].yaw),"finite tangent heading");
  }
  // An obstacle filling the inside of a right-angle corridor catches inner-cutting.
  const auto corridor=[](const PathSample& s) {
    return !(s.position.x<.98 && s.position.y>.02);
  };
  const std::vector<Point2> corner{{0,0},{1,0},{1,1}};
  require(smoothPath(corner,.03,.4,.05,.01,3,corridor,result),"refine unsafe corner");
  require(result.refinement>0,"collision must actually trigger refinement");
  for(const auto& s:result.path) require(corridor(s),"output must be safe after refinement");
  const auto blocked=[](const PathSample& s) { return s.position.x<.4; };
  require(!smoothPath({{0,0},{1,0}},.03,.2,.05,.01,2,blocked,result),"reject blocked spline for raw fallback");
  require(smoothPath({{0,0},{0,0},{.001,0}},.03,.2,.05,.01,2,free,result),"short path and duplicate points");
  require(!smoothPath({{0,0},{0,0}},.03,.2,.05,.01,2,free,result),"stationary goal has no tangent");
  // Reverse-facing geometric path is legal: smoother never imposes a turning radius or speed rule.
  require(smoothPath({{1,0},{0,0}},.03,.2,.05,.01,2,free,result),"reverse direction geometry");
  require(std::abs(std::abs(result.path[1].yaw)-std::acos(-1.))<1e-9,"analytic heading follows tangent");
  std::cout<<"BSPLINE_GEOMETRY_CHECK_PASS: endpoints, C2, arc sampling, collision refinement, fallback, degeneracy"<<std::endl;
}
