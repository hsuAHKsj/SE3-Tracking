#include "manif/SE3.h"
#include "se3_points_generator.h"
#include "manif/algorithms/interpolation.h"
#include "manif/impl/se3/SE3_base.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: .se2_interpolation <k> <i> <p>\n";
    std::cout << "\t with k: number of initial points on the 8-shaped curve.\n";
    std::cout << "\t with i: interpolation algorithm to use.\n";
    std::cout << "\t with p: number of points to generate between consecutive points of the initial curve.\n";
    std::cout << "\t Interpolation algorithm are : 0-Slerp / 1-Cubic / 2-CN-Smooth.\n";
    std::cout << "\t Please mind the Matlab script 'plot_interpolation.m' for visualizing the results exported in a csv file.\n";
    std::cout << "\t To export to csv, e.g.: ./se2_interpolation 9 2 40 > se2_interp_cnsmooth.csv \n";
    return EXIT_SUCCESS;
  }

  int k, i, p;

  k = atoi(argv[1]);
  i = atoi(argv[2]);
  p = atoi(argv[3]);

  manif::INTERP_METHOD interp_method;
  switch (i) {
  case 0:
    interp_method = manif::INTERP_METHOD::SLERP;
    break;
  case 1:
    interp_method = manif::INTERP_METHOD::CUBIC;
    break;
  case 2:
    interp_method = manif::INTERP_METHOD::CNSMOOTH;
    break;
  default:
    std::cerr << "Interpolation method 'i' must be in [0,2] !\n";
    return EXIT_FAILURE;
    break;
  }

  // 产生 平面内的8个点
  const auto points = generateSE3PointsOnHeightShape(k);
  //vector<manif::SE3d> points;
  // Interpolate between k-points
  // between each consecutive points
  // of the initial curve.

  std::vector<manif::SE3d> interpolated;

  // Initial point with Tangent t0 = 0

  manif::SE3Tangentd t0 = manif::SE3Tangentd::Zero();
  manif::SE3Tangentd t1 = points[1].rminus(points[0]);

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(points[0], points[1],
                  double(j)/double(p+1),
                  interp_method,
                  t0, t1
        )
    );
  }

  // Intermediate points

  for (std::size_t n=1; n<points.size()-1; ++n)
  {
    const manif::SE3d& s0 = points[ n ];
    const manif::SE3d& s1 = points[n+1];

    t0 = points[ n ] - points[n-1];
    t1 = points[n+1] - points[ n ];

    for (int m=1; m<=p; ++m)
    {
      interpolated.push_back(
        interpolate(s0, s1,
                    static_cast<double>(m)/(p+1),
                    interp_method,
                    t0, t1
        )
      );
    }
  }

  // Close the loop
  // 
  // Final point with Tangent t1 = 0

  const manif::SE3d& s0 = points.back();
  const manif::SE3d& s1 = points[0];

  t0 = points.back() - points[points.size()-2];
  t1 = manif::SE3Tangentd::Zero();

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(s0, s1,
                  static_cast<double>(j)/(p+1),
                  interp_method,
                  t0, t1
      )
    );
  }

  // Print in terminal

  using namespace std;
  // 打开并写文件
  ofstream oFileM;

  std::string pathName = "./data/se3_interpolation.csv";
  oFileM.open(pathName, ios::out | ios::trunc);
  oFileM << k << ", " << i << ", " << p << ", " << 0 << ", " << 0 << ", " << 0 << ", " << 0 << "\n";

  for (const auto& point : points)
  {
      oFileM << point.x() << ","
          << point.y() << ","
          << point.z() << ","
            << point.coeffs()(3) << ","
            << point.coeffs()(4) << ","
            << point.coeffs()(5) << ","
            << point.coeffs()(6) << "\n";
  }

  for (const auto& interp : interpolated)
  {
      oFileM << interp.x() << ","
          << interp.y() << ","
          << interp.z() << ","
          << interp.coeffs()(3) << ","
          << interp.coeffs()(4) << ","
          << interp.coeffs()(5) << ","
          << interp.coeffs()(6) << "\n";
  }

  return EXIT_SUCCESS;
}
