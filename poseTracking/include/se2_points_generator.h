#include "manif/SE2.h"
#include <vector>

namespace manif {

std::vector<manif::SE2d>
generateSE2PointsOnHeightShape(const unsigned int k)
{
  // Generate some k points on 8-shaped curve
  std::vector<manif::SE2d> states;
  states.reserve(k);

  const double x = std::cos(0);
  const double y = std::sin(0)/2;
  //const double y = std::sin(0);

  states.emplace_back(x,y,MANIF_PI/2);

  double t = 0;
  for (unsigned int i=1; i<k; ++i)
  {
    t += MANIF_PI*2. / double(k);

    const double xi = std::cos(t);
    const double yi = std::sin(2.*t) / 2.;
    //const double yi = std::sin(t);

    const double thi = std::atan2(yi-states.back().y(),
                                  xi-states.back().x());

    // cpp特性新函数，先构造临时对象然后将
    states.emplace_back(xi,yi,thi);
  }

  return states;
}

} 
