#include <mutex>
#include "manif/SE3.h"
#include "manif/impl/lie_group_base.h"
#include "udp_reciever.h"
#include "manif/SE3.h"
#include <vector>
#include <fstream>

#define PI (3.1415926535897932346f)
std::mutex m;

imuDataPack iData;
bool returnFlag;


// �����̶߳�ȡ������λ����Ϣ
void imu_record_pose_info()
{
    imu_com imu;

    std::lock_guard<std::mutex> lockGuard(m);
    // ����imu��Ϣ�����浽ȫ�ֱ�������
    imuDataPack iData_temp;

    while (returnFlag == false)
    {
        imu.get_imu_data(iData_temp);
        iData = iData_temp;
    }

    return;
}

// д�ĵ�
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

void data_interpreter(const imuDataPack & iData, manif::SE3d& pose)
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


// ������ 
int main()
{
    // 0. ���������߳�
    thread th_IMU(imu_record_pose_info);

    returnFlag = false;

    //Sleep(10);

    int iter = 0;

    std::vector<manif::SE3d> iData_v;
    iData_v.reserve(500);

    // 1. ���յ�ʱ������ȡȫ�ֱ���iData������

    // ����ʱ����
    double delta_T = 2;
    while (iter++ < 300)
    {
        // 1.1 ��ȡ���洢����
        manif::SE3d pose_loc;
        cout << "Time: " << delta_T * iter / 1000;
        data_interpreter(iData, pose_loc);
        iData_v.push_back(pose_loc);

        Sleep(delta_T);
        // 1.2 ������Ϣ
    }

    // ʹ��windows��¼ʱ����Ϣ
    std::string fileName = "test_multuithread_reading";
    save(fileName, iData_v);
    // 2. ������֪ͨ�̣߳��Զ��˳�

    returnFlag = true;

    th_IMU.join();

    system("pause");
    return 0;
}
