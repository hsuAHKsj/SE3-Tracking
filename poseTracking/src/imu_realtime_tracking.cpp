
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <Eigen/Core>
#include <Eigen/Dense>

#include <thread>
#include <mutex>
#include "manif/SE3.h"
#include "manif/impl/lie_group_base.h"
#include "udp_reciever.h"

#define PI (3.1415926535897932346f)
#define RECORD_TIME 300

std::mutex m;
bool returnFlag;

imuDataPack iData;
typedef vector<double> timeStamps;


// �����̶߳�ȡ������λ����Ϣ
void imu_record_pose_info()
{
    imu_com imu;

    //std::lock_guard<std::mutex> lockGuard(m);
    // ����imu��Ϣ�����浽ȫ�ֱ�������
    imuDataPack iData_temp;

    while (returnFlag == false)
    {
        imu.get_imu_data(iData_temp);
        m.lock();
        iData = iData_temp;
        m.unlock();
    }

    return;
}

void save(std::string fileName, std::vector<manif::SE3d> SE3_path);

void saveTime(std::string fileName, timeStamps time);

void data_interpreter(const imuDataPack& iData, manif::SE3d& pose)
{
    // ��ȡ�Ƕ�
    double psi = iData.angle[0] * PI / 180.;
    //���Բ�Ʒ�ֲ� ��Ʒ�����Ƕ���ŷ���ǵģ�ŷ���ǵ���ת˳����zyx˳��

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;

    // ��̬
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);
}

int main()
{
    //////////////// �������� /////////////////
    // �������̬�켣��Ϣ
    std::vector<manif::SE3d> se3_pos_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<double> time_v;
    std::vector<double> T_v;

    //////////////// ����������Ϣ/////////////////
    // �����ʲ���
    double k_p = 1;
    // ʱ����
    double delta_T = 0.04; // 2ms����һ��

    // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
    double rot_vec_bound = 0.6;
    double rot_acc_bound = 3;

    //////////////// ���ݲɼ��߳� /////////////////

    // ����imu�߳�,���ݿ�ʼ���ɼ�
    thread thread_IMU(imu_record_pose_info);
    returnFlag = false;

    //////////////// �����˳�ʼ״̬ /////////////////

    // 1. �����˳�ʼ״̬
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(Tri);

    // 2. ��̬ϵͳ��һʱ���ٶ�
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // ������һ���ڵ�Ŀ���ٶ�
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // ������һ���ڵļ��ٶ�
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();

    //////////////// ��̬ϵͳ���� /////////////////

    int iter = 0;

    bool init_flag = false;

    clock_t start, finish;
    start = clock();
    double Times;

    double Sleep_delta_T = 0.01;
    while (iter++ < RECORD_TIME)
    {
        //// �����ڵ�״̬ ////
        manif::SE3d pose;

        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose); // ���ڴ���л�ȡ

        //// �����״̬�쳣 ////

        // �õ���һ���������ݣ����ǲſ�ʼ��ʼ����
        if (init_flag == false && pose.isApprox(manif::SE3d::Identity()))
        {
            Sleep(Sleep_delta_T * 1000); // ˯��2ms
            continue;
        }
        else
        {
            init_flag = true;
        }

        //// ������Ŀ��״̬ ////
        manif::SE3d tar_pose = pose;
        manif::SE3Tangentd tar_vel;

           // ��¼ʱ��


        // ���������û����һʱ��λ�ã���˵���ڳ�ʱ�̣��ٶ�����Ϊ0��������ڳ�ʱ������ٶ�
        if (se3_tarpose_v.size() == 0)
        {
            tar_vel = manif::SE3Tangentd::Zero();
        }
        else
        {
            // ������ݸ���һʱ�����ݷǳ��ӽ�������ʹ��0 ���ٶȵĹ����˶�
            // ���λ�ò�÷ǳ�Զ�������·��������Ŀ��Ҳ���п����ǲ�������
            if (tar_pose.isApprox(*se3_tarpose_v.rbegin()))
            {
                tar_vel = se3_Tarvel_t_1;
            }
            else
            {
                tar_vel = (tar_pose.rminus(*se3_tarpose_v.rbegin())) / delta_T;
            }
        }
        ////////////// �����ʱ�д ///////////////////
        // ��������ʲ����ٶ�
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * (k_p) + tar_vel;

        // ��������ʲ����ĽǼ��ٶ�
        manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // ��������ʲ����ļӼ��ٶ�
        manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;

        // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
        double rot_jerk_bound = 30;

        //// ������ٶȹ��󣬶�����ʸ�����нض�
        manif::SE3Tangentd applied_acc = se3_acc_claw;

        if (se3_jerk.ang().norm() > rot_jerk_bound)
        {
            manif::SE3Tangentd applied_jerk = se3_jerk / (se3_jerk.ang().norm()) * rot_jerk_bound;

            applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
        }
        manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        if (applied_acc.ang().norm() > rot_acc_bound)
        {
            //double coeffi = 1;
            //// ����ٽ��߽�
            //if (applied_vel.ang().norm() > 0.95 * rot_vec_bound)
            //{
            //    // �������ƽ����ٶ����Ƶ�0
            //    coeffi = ((applied_vel.ang().norm() - 0.95 * rot_vec_bound) / (0.05 * rot_vec_bound));
            //}

            applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        double coeffi = 1;
        // ����ٶȴ����ٽ�ֵ���Ҽ��ٶ�ʹ����һ�����ٶȵ�ģ�����ô��һ���ڼ�������ļ��ٶȵ�ģ����Ҫ�𽥼�Сֱ���㡣
        if ((applied_vel.ang().norm() > 0.95 * rot_vec_bound) && ((applied_acc.ang() / applied_acc.ang().norm()).dot(se3_vel_t_1.ang() / se3_vel_t_1.ang().norm()) < 1))
        {
            if ((applied_vel.ang().norm() >= rot_vec_bound))
            {
                applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
            }
            else
            {   // ���ǽ����ٶ����Ƶ�0
                double temp_coeff = 1 - ((applied_vel.ang().norm() - 0.95 * rot_vec_bound) / (0.05 * rot_vec_bound));
                applied_acc.ang() = applied_acc.ang() * (temp_coeff <= 0 ? 0 : temp_coeff);
                applied_vel = se3_vel_t_1 + applied_acc * delta_T;
            }
        }

        // ���»�����λ��
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        // ����켣��Ϣ
        se3_pos_v.push_back(se3_pos);
        se3_tarpose_v.push_back(tar_pose);
        time_v.push_back((clock() - start) / 1000.);

        // ����ʵ�ʵļ��ٶ���Ϣ
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // �����ٶ�ʸ��
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = tar_vel;

        Sleep(Sleep_delta_T * 1000); // ˯��2ms

    }

    // ʹ��windows��¼ʱ����Ϣ
    std::string imufileName = "SE3_imuData";
    save(imufileName, se3_tarpose_v);

    // ʹ��windows��¼ʱ����Ϣ
    std::string fileName = "SE3_P_control_path";
    save(fileName, se3_pos_v);

    //// ����ʱ������
    std::string timefileName = "Time_T";
    saveTime(timefileName, time_v);

    returnFlag = true;

    thread_IMU.join();

    system("pause");
    return 0;
}


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
    oFileM.close();
}

void saveTime(std::string fileName, timeStamps time)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < time.size(); i++)
    {
        oFileM << time.at(i) << "\n";
    }
}