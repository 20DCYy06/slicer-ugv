#include <slicer_navigation/continuity_cost.h>
#include <slicer_navigation/bspline_core.h>
#include <cstdlib>
#include <iostream>
using namespace slicer_navigation;
void require(bool good,const char* message) {
  if(!good) { std::cerr<<"FAIL: "<<message<<std::endl; std::exit(1); }
}
void checkContinuityCosts() {
  const ContinuityCostOptions options;
  require(validContinuityOptions(options), "default continuity options valid");
  auto invalid = options;
  invalid.decay_length_m = 0.;
  require(!validContinuityOptions(invalid), "positive decay length required");
  invalid = options;
  invalid.weight = std::numeric_limits<double>::quiet_NaN();
  require(!validContinuityOptions(invalid), "nonfinite cost rejected");

  const std::vector<Point2> previous{{0,0},{2,0},{2,1},{0,1},{0,0},{2,0}};
  const auto suffix = remainingRoute(previous, {1.,.1}, .3);
  require(suffix.size()==6 && norm(suffix[0]-Point2(1.,0.))<1e-12 &&
          norm(suffix[1]-Point2(2.,0.))<1e-12, "trim at earliest equal projection");
  const auto turn = remainingRoute({{0,0},{1,0},{1,1}}, {1.1,.5}, .3);
  require(turn.size()==2 && norm(turn[0]-Point2(1.,.5))<1e-12,
          "remaining route excludes passed corner");
  require(remainingRoute({{0,0},{1,0}}, {.7,.31}, .3).empty(),
          "discard old bias outside rejoin distance");
  require(remainingRoute(previous, {2.,0.}, .3).size()==5, "no duplicate at projected vertex");

  constexpr int width = 31, height = 21;
  std::vector<std::uint8_t> measured(width*height, 0), search(measured.size());
  const auto cell = [](int x, int y) { return y*width+x; };
  measured[cell(8,5)] = 254;
  measured[cell(16,5)] = 253;
  measured[cell(10,6)] = 255;
  measured[cell(20,5)] = 255;
  measured[cell(15,10)] = 186;
  measured[cell(16,10)] = 252;
  const auto before = measured;
  const auto hard = hardContinuityMask(measured.data(), width, height, .1, options.min_clearance_m);
  require(hard[cell(0,6)]==1 && hard[cell(30,6)]==1 && hard[cell(8,20)]==1,
          "hard mask outlines the grid");
  require(hard[cell(8,5)]==1 && hard[cell(16,5)]==1, "physical lethal and inscribed cells block");
  require(hard[cell(10,6)]==1 && hard[cell(10,7)]==0, "use Euclidean centre clearance even for unknown");
  require(hard[cell(16,6)]==0 && hard[cell(15,10)]==0, "inflation values do not seed clearance");

  const Point2 origin(-.05,-.05), start(.4,1.);
  const std::vector<Point2> reference{{.4,1.},{2.8,1.}};
  encodeContinuityCosts(measured.data(), hard, width, height, .1, origin, start,
                        reference, options, search.data());
  require(measured == before, "source safety costmap remains unchanged");
  require(search[cell(8,5)]==254 && search[cell(16,5)]==253 && search[cell(10,6)]==254 &&
          search[cell(20,5)]==255, "preserve hard classes and unblocked unknown");
  require(search[cell(15,10)]==32 && search[cell(16,10)]==44, "186 and 252 remain soft");
  require(search[cell(6,10)]==0 && search[cell(6,14)]==16 && search[cell(25,14)]==7,
          "meters-based lateral bias decays with distance from start");
  require(search[cell(6,1)]==search[cell(6,19)], "equal lateral magnitudes produce equal bias");

  auto no_preference = options;
  no_preference.weight = 0.;
  encodeContinuityCosts(measured.data(), hard, width, height, .1, origin, start,
                        reference, no_preference, search.data());
  require(search[cell(6,14)]==0 && search[cell(15,10)]==32, "zero continuity still encodes obstacle cost");
}

int main() {
  checkContinuityCosts();
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
