
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include "manif/SE3.h"
#include "manif/impl/lie_group_base.h"
#define PI (3.1415926535897932346f)

#include "udp_reciever.h"

void save(std::string fileName, std::vector<manif::SE3d> SE3_path)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& interp : SE3_path)
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

// 1. ����������Ϣ
// 2. ��̬��Ϣ
// 3. �ٶ���Ϣ
// 4. ���ٶ���Ϣ

void data_interpreter(imuDataPack iData, manif::SE3d &pose, manif::SE3Tangentd &vel, manif::SE3Tangentd &acc)
{
    // ��ȡ�Ƕ�
    double psi = iData.angle[0] * PI / 180.;
    //���Բ�Ʒ�ֲ� ��Ʒ�����Ƕ���ŷ���ǵģ�ŷ���ǵ���ת˳����zyx˳��
    
    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    cout << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;
    //// ��̬
    Eigen::Matrix3d poseEi = 
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
        Eigen::AngleAxisd (phi, Eigen::Vector3d::UnitZ()).matrix());

    //cout << Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix() << endl;
    
    cout << poseEi << endl << endl;;

    // ������ٶ�
    Eigen::Matrix3d eul2Avel_m;
    eul2Avel_m << 1, 0, -sin(theta),
        0, cos(psi), sin(psi)* cos(theta),
        0, -sin(psi), cos(psi)* cos(theta);
                
    // ����
    Eigen::Vector3d eulVel = Eigen::Vector3d(iData.w[0], iData.w[1], iData.w[2]);
    Eigen::Vector3d Avel = eul2Avel_m * eulVel;

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);

    //// ������ٶ�
    vel.setZero();
    vel.ang() = Avel;
}

int main()
{
    // 1. ��ʼ�� ��������̬��Ϣ
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(Tri);

    // �������̬�켣��Ϣ
    std::vector<manif::SE3d> se3_pos_v;
    std::vector<manif::SE3d> se3_tarpose_v;


    // �����ʲ���
    double k_p = 1;
    // ʱ����
    double delta_T = 0.04;

    // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
    double rot_vec_bound = 0.6;
    double rot_acc_bound = 3;

    // ʹ��windows��¼ʱ����Ϣ

    // ��ʼ���켣��һʱ�̵Ľ��ٶ�ʸ��
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();

    // 2. ��ʼ��Ӳ��ͨѶ
    imu_com imu;

    // ��׼����ʱ��

    // 3. ��ѭ��ʹ�ÿ����ʣ�����¼������Ϣ
    double start, end;
    start = clock();

    imuDataPack iData;

    int iter = 0;
    while (imu.get_imu_data(iData) && iter < 1000)
    {
        iter++;
        // TODO���Եõ���λ����Ϣ���д���
        end = clock();
        delta_T = end - start;
        start = end;

        // TODO:���ñ�����Ŀ��λ�ˣ�ʹ����ʵλ���滻��
        manif::SE3d pose;
        manif::SE3Tangentd vel;
        manif::SE3Tangentd acc;
        data_interpreter(iData, pose, vel, acc);

        manif::SE3d tar_pose = pose;

        // mc = ma.rplus(mb.rminus(ma) * t);

        // TODO:���ñ�����Ŀ���ٶȣ�ʹ����ʵ�ٶ��滻��
        manif::SE3Tangentd tar_vel = vel;

        // ��������ʲ����ٶ�
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + tar_vel;

        // ��������ʲ����ĽǼ��ٶȣ��滻ʵ��ʱ�䣬�飩
        manif::SE3Tangentd se3_acc = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // ������ٶȹ��󣬶�����ʸ�����нض�
        manif::SE3Tangentd applied_vel = se3_vel_cLaw;
        if (se3_acc.ang().norm() > rot_acc_bound)
        {
            manif::SE3Tangentd applied_acc = se3_acc / se3_acc.ang().norm() * rot_acc_bound;
            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        // ����ٶȹ���Լ���ٶ�
        if (applied_vel.ang().norm() > rot_vec_bound)
        {
            applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
        }

        // ����λ��
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        // ����켣��Ϣ
        se3_pos_v.push_back(se3_pos);
        se3_tarpose_v.push_back(tar_pose);

        // �����ٶ�ʸ��
        se3_vel_t_1 = applied_vel;
    }

    // ʹ��windows��¼ʱ����Ϣ
    std::string fileName = "SE3_P_control_path";
    save(fileName, se3_pos_v);

    // ʹ��windows��¼ʱ����Ϣ
    std::string imufileName = "SE3_imuData";
    save(imufileName, se3_tarpose_v);

    system("pause");
    return 0;
}


