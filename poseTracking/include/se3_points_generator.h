#include "manif/SE3.h"
#include <vector>

std::vector<manif::SE3d> generateSE3PointsOnHeightShape(const unsigned int k)
{
    // Generate some k points on 8-shaped curve
    std::vector<manif::SE3d> states;
    states.reserve(k);

    const double x = std::cos(0);
    const double y = std::sin(0) / 2;
    const double z = sqrt(1 - std::sqrt(x * x + y * y));

    //const double y = std::sin(0);
    // xyz 的方向是 z 轴的方向
    // x的方向是 std::atan2(yi - states.back().y(), xi - states.back().x())

    // 1 . 单位化 x y z
    Eigen::Vector3d zAxis = Eigen::Vector3d(x, y, z).normalized();

    // 2 . 计算 x 轴在0平面内的方向，并计算出 y轴的方向
    double tho = MANIF_PI / 2;
    double x_u = cos(tho);
    double x_v = sin(tho);
    double x_z = 0;

    Eigen::Vector3d xAxis_temp = Eigen::Vector3d(x_u, x_v, x_z).normalized();

    Eigen::Vector3d yAxis = zAxis.cross(xAxis_temp).normalized();

    // 3. 计算 x 轴的方向
    Eigen::Vector3d xAxis = yAxis.cross(zAxis).normalized();

    Eigen::Isometry3d Tr;
    Eigen::Matrix3d rotMat;
    rotMat.col(0) = xAxis;
    rotMat.col(1) = yAxis;
    rotMat.col(2) = zAxis;

    Tr.translation() = Eigen::Vector3d(x, y, z);
    Tr.linear() = rotMat;

    //manif::SO3d::Rotation rot;
    states.emplace_back(Tr);

    double t = 0;

    for (unsigned int i = 1; i < k; ++i)
    {
        t += MANIF_PI * 2. / double(k);

        const double xi = std::cos(t);
        const double yi = std::sin(2. * t) / 2.;
        const double zi = sqrt(1 - std::sqrt(xi * xi + yi * yi));

        const double thi = std::atan2(yi - states.back().y(),
            xi - states.back().x());

        double x_ui = cos(thi);
        double x_vi = sin(thi);
        double x_zi = 0;

        Eigen::Vector3d zAxisi = Eigen::Vector3d(xi, yi, zi).normalized();

        Eigen::Vector3d xAxis_tempi = Eigen::Vector3d(x_ui, x_vi, x_zi).normalized();

        Eigen::Vector3d yAxisi = zAxisi.cross(xAxis_tempi).normalized();

        // 3. 计算 x 轴的方向
        Eigen::Vector3d xAxisi = yAxisi.cross(zAxisi).normalized();

        Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rotMati;

        rotMati.col(0) = xAxisi;
        rotMati.col(1) = yAxisi;
        rotMati.col(2) = zAxisi;

        Tri.translation() = Eigen::Vector3d(xi, yi, zi);
        Tri.linear() = rotMati;

        // cpp特性新函数，先构造临时对象然后将
        states.emplace_back(Tri);

    }

    return states;

}