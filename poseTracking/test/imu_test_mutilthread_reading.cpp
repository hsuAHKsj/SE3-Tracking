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


// 独立线程读取并保存位置信息
void imu_record_pose_info()
{
    imu_com imu;

    std::lock_guard<std::mutex> lockGuard(m);
    // 保存imu信息，并存到全局变量里面
    imuDataPack iData_temp;

    while (returnFlag == false)
    {
        imu.get_imu_data(iData_temp);
        iData = iData_temp;
    }

    return;
}

// 写文档
void save(std::string fileName, std::vector<manif::SE3d> SE3_path)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
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
    // 获取角度
    double psi = iData.angle[0] * PI / 180.;
    //来自产品手册 产品测量角度是欧拉角的，欧拉角的旋转顺序书zyx顺序

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;
    
    // 姿态
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);
}


// 主函数 
int main()
{
    // 0. 开启监听线程
    thread th_IMU(imu_record_pose_info);

    returnFlag = false;

    //Sleep(10);

    int iter = 0;

    std::vector<manif::SE3d> iData_v;
    iData_v.reserve(500);

    // 1. 按照等时间间隔读取全局变量iData的数据

    // 定义时间间隔
    double delta_T = 2;
    while (iter++ < 300)
    {
        // 1.1 读取并存储数据
        manif::SE3d pose_loc;
        cout << "Time: " << delta_T * iter / 1000;
        data_interpreter(iData, pose_loc);
        iData_v.push_back(pose_loc);

        Sleep(delta_T);
        // 1.2 保存信息
    }

    // 使用windows记录时刻信息
    std::string fileName = "test_multuithread_reading";
    save(fileName, iData_v);
    // 2. 满周期通知线程，自动退出

    returnFlag = true;

    th_IMU.join();

    system("pause");
    return 0;
}
