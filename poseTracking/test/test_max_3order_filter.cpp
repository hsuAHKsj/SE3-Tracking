
#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"
#include "se3_points_generator.h"


#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>


using namespace std;

double sign(double x)
{
    if (x > 0)
        return 1;
    else if (x == 0)//һ��Ҫ��==
        return 0;
    else
        return -1;
}

bool se2_pathgen(std::vector<manif::SE3d>& interpolated)
{
    int k, i, p;

    k = 20;
    i = 2;
    p = 40;

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

    // ���� ƽ���ڵ�8����
    const auto points = generateSE3PointsOnHeightShape(k);

    // Interpolate between k-points
    // between each consecutive points
    // of the initial curve.

    // Initial point with Tangent t0 = 0

    manif::SE3Tangentd t0 = manif::SE3Tangentd::Zero();
    manif::SE3Tangentd t1 = points[1].rminus(points[0]);

    for (int j = 1; j <= p; ++j)
    {
        interpolated.push_back(
            interpolate(points[0], points[1],
                double(j) / double(p + 1),
                interp_method,
                t0, t1
            )
        );
    }

    // Intermediate points

    for (std::size_t n = 1; n < points.size() - 1; ++n)
    {
        const manif::SE3d& s0 = points[n];
        const manif::SE3d& s1 = points[n + 1];

        t0 = points[n] - points[n - 1];
        t1 = points[n + 1] - points[n];

        for (int m = 1; m <= p; ++m)
        {
            interpolated.push_back(
                interpolate(s0, s1,
                    static_cast<double>(m) / (p + 1),
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

    t0 = points.back() - points[points.size() - 2];
    t1 = manif::SE3Tangentd::Zero();

    for (int j = 1; j <= p; ++j)
    {
        interpolated.push_back(
            interpolate(s0, s1,
                static_cast<double>(j) / (p + 1),
                interp_method,
                t0, t1
            )
        );
    }

    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
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

    oFileM.close();
    return EXIT_SUCCESS;
}


// ������ ����켣��Ϣ

void save(std::string fileName, std::vector<manif::SE3d> SE2_path)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& interp : SE2_path)
    {
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << "\n";
    }

}


int main(int argc, char** argv)
{
    // ��ȡ���ɵ�Ŀ��켣
    std::vector<manif::SE3d> interpolated;

    // ������ɹ켣�ɹ���
    // �����ʼ���������и���

    std::vector<manif::SE3d> se3_pos_v;

    double k_p = 1;
    double delta_T = 0.04;


    if (se2_pathgen(interpolated) == EXIT_SUCCESS)
    {
        // ��ʼ���켣λ��
        Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
        manif::SE3d se3_pos(Tri);
        //// ��ʼ���켣�ٶ�
        //manif::SE2Tangentd se2_vel = manif::SE2Tangentd::Zero();

        // ��ʼ���켣��һʱ�̵Ľ��ٶ�ʸ��
        manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();

        // ������һ���ڵļ��ٶ�
        manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();


        for (int i = 0; i < interpolated.size() - 1; i++)
        {
            // ����Ŀ��λ��
            manif::SE3d tar_pose = interpolated[i];

            // mc = ma.rplus(mb.rminus(ma) * t);

            // ����Ŀ���ٶ�
            manif::SE3Tangentd tar_vel = (interpolated[i + 1].rminus(interpolated[i])) / delta_T;
            // 
            // ���㵱ǰ 

            // ��������ʲ����ٶ�
            manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + tar_vel;

            // ��������ʲ����ļ��ٶ�
            manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

            // ��������ʲ����ļӼ��ٶ�
            manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;


            // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
            double rot_vec_bound = 0.6;
            double rot_acc_bound = 3;
            double rot_jerk_bound = 30;

            //// ������������ٶ��˶��£���һ��������ٶȵ�ģ
            manif::SE3Tangentd applied_acc = se3_acc_claw;
            if (se3_jerk.ang().norm() > rot_jerk_bound)
            {
                manif::SE3Tangentd applied_jerk = se3_jerk / se3_jerk.ang().norm() * rot_jerk_bound;
                //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
                applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
            }
            //// ��������ٶ�����һ��������ٶȵ�ģ
            manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
            if (applied_acc.ang().norm() > rot_acc_bound)
            {
                applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;
                //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
                applied_vel = se3_vel_t_1 + applied_acc * delta_T;
            }
            double coeffi = 1;
            // ����ٶȴ����ٽ�ֵ���Ҽ��ٶ�ʹ����һ�����ٶȵ�ģ�����ô��һ���ڼ�������ļ��ٶȵ�ģ����Ҫ�𽥼�Сֱ���㡣
            if ((applied_vel.ang().norm() > 0.95 * rot_vec_bound) && ((applied_acc.ang()/applied_acc.ang().norm()).dot(se3_vel_t_1.ang()/ se3_vel_t_1.ang().norm()) < 1))
            {
                if ((applied_vel.ang().norm() >= rot_vec_bound))
                {
                    applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
                }
                else
                {   // ���ǽ����ٶ����Ƶ�0
                    double temp_coeff = 1 - ((applied_vel.ang().norm() - 0.95 * rot_vec_bound) / (0.05 * rot_vec_bound));
                    applied_acc.ang() = applied_acc.ang() * (temp_coeff <= 0 ? 0: temp_coeff);
                    applied_vel = se3_vel_t_1 + applied_acc * delta_T;
                }

            }

            // ����λ��
            se3_pos = se3_pos.rplus(applied_vel * delta_T);

            // ����켣��Ϣ
            se3_pos_v.push_back(se3_pos);

            // ����ʵ�ʵļ��ٶ���Ϣ
            se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

            // �����ٶ�ʸ��
            se3_vel_t_1 = applied_vel;


        }
    }

    std::string fileName = "SE3_PP_control_path";
    save(fileName, se3_pos_v);

    system("pause");
    return 0;
}