
#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"
#include "se2_points_generator.h"

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

bool se2_pathgen(std::vector<manif::SE2d> &interpolated)
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
    const auto points = manif::generateSE2PointsOnHeightShape(k);

    // Interpolate between k-points
    // between each consecutive points
    // of the initial curve.

    // Initial point with Tangent t0 = 0

    manif::SE2Tangentd t0 = manif::SE2Tangentd::Zero();
    manif::SE2Tangentd t1 = points[1].rminus(points[0]);

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
        const manif::SE2d& s0 = points[n];
        const manif::SE2d& s1 = points[n + 1];

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

    const manif::SE2d& s0 = points.back();
    const manif::SE2d& s1 = points[0];

    t0 = points.back() - points[points.size() - 2];
    t1 = manif::SE2Tangentd::Zero();

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

    std::string pathName = "./data/se2_interpolation.csv";
    oFileM.open(pathName, ios::out | ios::trunc);
    oFileM << k << ", " << i << ", " << p << "\n";

    for (const auto& point : points)
    {
        oFileM << point.x() << ","
            << point.y() << ","
            << point.angle() << "\n";
    }

    for (const auto& interp : interpolated)
    {
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.angle() << "\n";
    }

    oFileM.close();
    return EXIT_SUCCESS;
}


// ������ ����켣��Ϣ

void save(std::string fileName, std::vector<manif::SE2d> SE2_path)
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
            << interp.angle() << "\n";
    }

}


int main(int argc, char** argv)
{
	// ��ȡ���ɵ�Ŀ��켣
    std::vector<manif::SE2d> interpolated;
    
    // ������ɹ켣�ɹ���
    // �����ʼ���������и���

    std::vector<manif::SE2d> se2_pos_v;

    
    double k_p = 1;
    double k_d = 0;
    double delta_T = 0.04;


    if (se2_pathgen(interpolated) == EXIT_SUCCESS)
    {
        // ��ʼ���켣λ��
        manif::SE2d se2_pos(0, 0, 0);
        //// ��ʼ���켣�ٶ�
        //manif::SE2Tangentd se2_vel = manif::SE2Tangentd::Zero();

        // ��ʼ���켣��һʱ�̵Ľ��ٶ�ʸ��
        manif::SE2Tangentd se2_vel_t_1 = manif::SE2Tangentd::Zero();

        for (int i = 0; i < interpolated.size() - 1; i++)
        {
            // ����Ŀ��λ��
            manif::SE2d tar_pose = interpolated[i];

            // mc = ma.rplus(mb.rminus(ma) * t);
            
            // ����Ŀ���ٶ�
            manif::SE2Tangentd tar_vel = (interpolated[i + 1].rminus(interpolated[i])) / delta_T;
            // 
            // ���㵱ǰ 
            
            //manif::SE2Tangentd delta_se_vel = se2_error * 0.5;

            // ��������ʲ����ٶ�
            manif::SE2Tangentd se2_vel_cLaw = tar_pose.rminus(se2_pos)* k_p + tar_vel;

            // ��������ʲ����ĽǼ��ٶ�
            manif::SE2Tangentd se2_acc = (se2_vel_cLaw - se2_vel_t_1) / delta_T;

            // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
            double rot_vec_bound = 0.6;
            double rot_acc_bound = 3;

            // ������ٶȹ��󣬶�����ʸ�����нض�
            manif::SE2Tangentd applied_vel = se2_vel_cLaw;
            if (fabs(se2_acc.angle()) > rot_acc_bound)
            {
                manif::SE2Tangentd applied_acc = se2_acc / fabs(se2_acc.angle()) * rot_acc_bound;
                //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);

                //double rot_acc = sign(se2_acc.angle()) * rot_acc_bound;
                applied_vel = se2_vel_t_1 + applied_acc * delta_T;
            }
            // ����ٶȹ���Լ���ٶ�
            if (fabs(applied_vel.angle()) > rot_vec_bound)
            {
                applied_vel = applied_vel / fabs(applied_vel.angle()) * rot_vec_bound;
            }

            // ����λ��
            se2_pos = se2_pos.rplus(applied_vel * delta_T);

            // ����켣��Ϣ
            se2_pos_v.push_back(se2_pos);

            // �����ٶ�ʸ��
            se2_vel_t_1 = applied_vel;
        }

        std::string fileName = "P_control_path";
        save(fileName, se2_pos_v);
    }

    system("pause");
    return 0;
}