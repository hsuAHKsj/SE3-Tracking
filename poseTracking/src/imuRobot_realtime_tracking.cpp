
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

#include <hansDriver/conciseController.h>

#define PI (3.1415926535897932346f)
#define RECORD_TIME 1000

struct carKinInfo
{
public:
    EcRealVector pcs;
    double vel;
    double acc;
    double jerk;
};


std::mutex m;
std::mutex robot;

bool returnFlag;

imuDataPack iData;
carKinInfo kinSharedInfo;
EcRealVector initPcsPos;
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

void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time);
void saveHardwarePath(std::string fileName, std::vector<manif::SE3d> SE3_path, vector<double> ang_vel, timeStamps time);

void save_time(std::string fileName, std::vector<double>& T_v)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& Time : T_v)
    {
        oFileM << Time << "\n";
    }
}


void data_interpreter(const imuDataPack& iData, manif::SE3d& pose)
{
    // ��ȡ�Ƕ�
    double psi = iData.angle[0] * PI / 180.;
    //���Բ�Ʒ�ֲ� ��Ʒ�����Ƕ���ŷ���ǵģ�ŷ���ǵ���ת˳����zyx˳��

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    //cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;

    // ��̬
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);
}


// Gives back a rotation matrix specified with RPY convention
Eigen::Vector3d GetRPY(Eigen::Isometry3d iso)
{
    Eigen::Matrix3d data = iso.linear();
    Eigen::Vector3d outVec;
    double epsilon = 1E-12;
    outVec(1) = atan2(-data(2,0), sqrt(data(0,0) * data(0,0) + data(1,0) * data(1,0)));
    if (fabs(outVec(1)) > (PI / 2.0 - epsilon)) {
        outVec(2) = atan2(-data(0,1), data(1,1));
        outVec(0) = 0.0;
    }
    else {
        outVec(0) = atan2(data(2,1), data(2,2));
        outVec(2) = atan2(data(1,0), data(0,0));
    }

    return outVec;
}


void RemoteServo(double indServoTime, double indLookaheadTime)
{
    //servoTime = 0.025   # �����ŷ�����Ϊ 25ms��������С������ 15ms
    //	# ����ǰհʱ�䣬ǰհʱ��Խ�󣬹켣Խƽ������Խ�ͺ�
    //	lookaheadTime = 0.2 # ����ǰհʱ��Ϊ 200ms, ������ 0.05s~0.2s ֮��
    std::vector<manif::SE3d> se3_pos_Robot_v;
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<double> ang_vel_v;
    std::vector<double> cmdang_vel_v;
    std::vector<double> Time_v;

    conciseRobotController elfin;
    manif::SE3d pose;
    elfin.start_servo(indServoTime, indLookaheadTime);

    double begin; // ���ȳ߿�ʼʱ��
    begin = clock();

    // ��ʼ���ŷ�Ŀ��
    robot.lock();
    carKinInfo tempkinInfo = kinSharedInfo; 
    robot.unlock();
   
    int i = 0;

    EcRealVector pcs_lo;
    while(returnFlag == false)
    {
        robot.lock();
        carKinInfo tempkinInfo = kinSharedInfo; 
        robot.unlock();

        pcs_lo = tempkinInfo.pcs;
        elfin.servoP(pcs_lo);

        // ��ȡ������ TCP λ��
        EcRealVector curPos;
        elfin.readPcsPos(curPos);

        //// ��ȡ�����˹ؽ��ٶ�
        double robAngVel;
        elfin.getAngularvelocity(robAngVel);

        // ��¼imu������
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose);

        // ���㵱ǰʱ��
        Time_v.push_back((clock() - begin) / 1000.);
        se3_pos_conLaw_v.push_back(manif::SE3d(pcs_lo[0], pcs_lo[1], pcs_lo[2], pcs_lo[3] * PI / 180, pcs_lo[4] * PI / 180, pcs_lo[5] * PI / 180));
        se3_pos_Robot_v.push_back(manif::SE3d(0, 0, 0, curPos[3] * PI / 180, curPos[4] * PI / 180, curPos[5] * PI / 180));
        se3_tarpose_v.push_back(pose);
        ang_vel_v.push_back(robAngVel);
        cmdang_vel_v.push_back(tempkinInfo.vel);

        while (1)
        {
            if ((clock() - begin) >= ((i + 1) * indServoTime * 1000))
            {
                break;
            }
            Sleep(0.001);
        }

        i++;
    }

    // ʹ��windows��¼ʱ����Ϣ
    std::string controlLawFileName = "SE3_P_control_path";
    saveHardwarePath(controlLawFileName, se3_pos_conLaw_v, cmdang_vel_v, Time_v);
    
    // �洢�켣��ʱ���ļ���Ϣ
    std::string realDataFileName = "SE3_P_control_robot_path";
    saveHardwarePath(realDataFileName, se3_pos_Robot_v, ang_vel_v, Time_v);

    // �洢imu����
    std::string imuDataFileName = "SE3_imuData";
    savePath(imuDataFileName, se3_tarpose_v, Time_v);
}

// �ѿ����ռ�켣��Ϣ�������ٶȣ����ٶ�


int main()
{
    //////////////// �������� /////////////////
    // �������̬�켣��Ϣ
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<manif::SE3d> se3_pos_Robot_v;

    std::vector<double> ang_vel_v;
    std::vector<double> Time_v;
    std::vector<double> T_v;

    //////////////// ����������Ϣ/////////////////
    // �����ʲ���
    double k_p = 1;


    // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
    double rot_vec_bound = 0.5;
    double rot_acc_bound = 1;
    double rot_jerk_bound = 10;

    //////////////// ���ݲɼ��߳� /////////////////
    
    // ����imu�߳�,���ݿ�ʼ���ɼ�
    thread thread_IMU(imu_record_pose_info);
    returnFlag = false;
    
    //////////////// �����˳�ʼ״̬ /////////////////
    // 2. ��̬ϵͳ��һʱ���ٶ�
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // ������һ���ڵ�Ŀ���ٶ�
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // ������һ���ڵļ��ٶ�
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();

    //////////////// ��̬ϵͳ���� /////////////////
    // �߳���
    //std::lock_guard<std::mutex> lockGuard(m);

    int iter = 0; 

    //bool init_flag = false;

    double Times;

    HRIF_Connect(0, "10.0.0.27", 10003);
    // �½�����
    conciseRobotController elfin;

    // ���˶���ָ��Ŀ��λ��
    EcRealVector initJoint = {0, 0, 90, 0, 90, 0};
    elfin.moveJ(initJoint, 360, 50);

    // ��ȡ��ʼλ��
    elfin.readPcsPos(initPcsPos);

    // ��ʼ�ŷ�����
    double indServoTime = 0.03;
    double delta_T = indServoTime/3; // 2ms����һ��
    double indLookaheadTime = 0.05;

    double begin; // ���ȳ߿�ʼʱ��
    begin = clock();

    EcRealVector curPos; // �����˵�ǰ��ʵλ��

    // ��ʼ�������˳�ʼλ��
    manif::SE3d robInitPose(0, 0, 0, initPcsPos[3] * PI / 180, initPcsPos[4] * PI / 180, initPcsPos[5] * PI / 180);

    // ��ȡimu��ʼλ��
    manif::SE3d imuInitPose;
    do
    {
        m.lock();
        imuDataPack temImuData = iData;
        m.unlock();
        // ���ڴ���л�ȡ
        data_interpreter(temImuData, imuInitPose);
        Sleep(1000);
    } while (imuInitPose.isApprox(manif::SE3d::Identity()));

    // ����imuĿ�겹��λ��
    manif::SE3d imuCompensate =  imuInitPose.inverse()* robInitPose;

    manif::SE3d pose;

    // 1. �����˳�ʼ״̬
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(imuInitPose);

    Eigen::Isometry3d initPoseCompensated = (imuInitPose * imuCompensate).isometry();
    Eigen::Vector3d rpyInitCompensated = GetRPY(initPoseCompensated);

    // ƴװ������Ŀ��λ��
    EcRealVector PoseInit = { initPcsPos[0], initPcsPos[1], initPcsPos[2], rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };
    
    
    carKinInfo init_CarKinInfo;
    init_CarKinInfo.pcs = PoseInit;
    init_CarKinInfo.vel = 0;
    init_CarKinInfo.acc = 0;
    init_CarKinInfo.jerk = 0;

    ////// ����servoP��λ
    robot.lock();
    kinSharedInfo = init_CarKinInfo; // ��ȡ��������
    robot.unlock();
    
    thread thread_Robot(RemoteServo, indServoTime, indLookaheadTime);

    //elfin.start_servo(indServoTime, indLookaheadTime);


    while (iter++ < RECORD_TIME)
    {
        //// ���ڴ���л�ȡ�����ڵ�״̬ ////
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose); 

        //// �����״̬�쳣 ////

        //// ������Ŀ��״̬ ////
        manif::SE3d tar_pose = pose; 
        manif::SE3Tangentd tar_vel;


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
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + tar_vel;

        // ��������ʲ����ĽǼ��ٶ�
        manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // ��������ʲ����ļӼ��ٶ�
        manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;


        //// ������ٶȹ��󣬶�����ʸ�����нض�
        manif::SE3Tangentd applied_acc = se3_acc_claw;

        if (se3_jerk.ang().norm() > rot_jerk_bound)
        {
            manif::SE3Tangentd applied_jerk = se3_jerk / (se3_jerk.ang().norm()) * rot_jerk_bound;

            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
        }
        manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        if (applied_acc.ang().norm() > rot_acc_bound)
        {
            // �����һ�����ٶ����ڱ��
            applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;

            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        // ����ٶȻ�����Լ���ٶ�
        if (applied_vel.ang().norm() > rot_vec_bound)
        {
            // �жϼ��ٶ��Ƿ�Ϊ 0�� �����Ϊ0 ʹ�����jerk���ٵ��ٶȱ߽���

            applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
            // ������һ��jerk�Ƿ��ˣ��������Լ��סjerk
            //if(jerk)
        }

        // ���»�����λ��
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        //// ����켣��Ϣ
        //se3_pos_conLaw_v.push_back(se3_pos* imuCompensate);
        se3_tarpose_v.push_back(tar_pose);
        se3_pos_conLaw_v.push_back(se3_pos);
        Time_v.push_back((clock() - begin) / 1000.);

        // ����ʵ�ʵļ��ٶ���Ϣ
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // �����ٶ�ʸ��
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = tar_vel;

        // ƴװ�·�ָ�� imuInitPose.inverse()* robInitPose (����Ӧ���ǲ�����)
        manif::SE3d compensatedPose = se3_pos * imuCompensate;
        Eigen::Isometry3d curse3h = compensatedPose.isometry();
        Eigen::Vector3d rpy = GetRPY(curse3h);

        // ƴװ������Ŀ��λ��
        EcRealVector cmdPose = {initPcsPos[0], initPcsPos[1], initPcsPos[2], rpy(0) * 180 / PI, rpy(1) * 180 / PI, rpy(2) * 180 / PI };

        carKinInfo temp_CarKinInfo;
        temp_CarKinInfo.pcs = cmdPose;
        temp_CarKinInfo.vel = tar_vel.ang().norm();
        temp_CarKinInfo.acc = se3_acc_t_1.ang().norm();
        temp_CarKinInfo.jerk = 0;

        ////// ����servoP��λ
        robot.lock();
        kinSharedInfo = temp_CarKinInfo; // ��ȡ��������
        robot.unlock();

        //{
        //    cout << elfin.servoP(cmdPose) << "\n";

        //    // ��ȡ������ TCP λ��
        //    EcRealVector curPos;
        //    elfin.readPcsPos(curPos);

        //    //// ��ȡ������ TCP �ٶ�
        //    double angVel;
        //    elfin.getAngularvelocity(angVel);
        //    ang_vel_v.push_back(angVel);

        //    // ���㵱ǰʱ��
        //    Time_v.push_back((clock() - begin) / 1000.);
        //    //se3_pos_conLaw_v.push_back(manif::SE3d(cmdPose[0], cmdPose[1], cmdPose[2], cmdPose[3] * PI / 180, cmdPose[4] * PI / 180, cmdPose[5] * PI / 180));
        //    se3_pos_Robot_v.push_back(manif::SE3d(0, 0, 0, curPos[3] * PI / 180, curPos[4] * PI / 180, curPos[5] * PI / 180));
        //    ang_vel_v.push_back(angVel);
        //}


        //elfin.servoP(cmdPose); // ���� servoP ��λ
        //Time_v.push_back((clock() - begin) / 1000.); // ��¼�·�ʱ��ʱ��

        while (1)
        {
            if ((clock() - begin) >= ((iter + 1) * delta_T * 1000))
            {
                break;
            }
            Sleep(0.001);
        }
    }

    // �����˳�˳���
    // 1. ���ı��λ�� 2. �رռ����߳�ȷ��������ȷ�� 3. �Ͽ��������ӣ���imu������ˡ�
    returnFlag = true;

    thread_Robot.join();
    thread_IMU.join();
    HRIF_DisConnect(0);

    // ����ʵ�ʵ�λ����Ϣ
    std::string conLawBeforeTransfer = "SE3_P_control_conLaw_bTrans";
    savePath(conLawBeforeTransfer, se3_pos_conLaw_v, Time_v);

    // ����ʵ�ʵ�λ����Ϣ
    std::string imuBeforeTransfer = "SE3_P_control_imu_bTrans";
    savePath(imuBeforeTransfer, se3_tarpose_v, Time_v);

    return 0;
}


void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < SE3_path.size(); i++)
    {
        manif::SE3d interp = SE3_path.at(i);
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << ","
            << time.at(i) << "\n";
    }
}

void saveHardwarePath(std::string fileName, std::vector<manif::SE3d> SE3_path, vector<double> ang_vel, timeStamps time)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < SE3_path.size(); i++)
    {
        manif::SE3d interp = SE3_path.at(i);
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << ","
            << ang_vel.at(i) << ","
            << time.at(i) << "\n";
    }
}

