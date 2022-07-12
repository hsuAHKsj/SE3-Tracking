
#include <thread>
#include <mutex>
#include "udp_reciever.h"
#include "hansTracker/elfin_PoseTracker.h"
#include <hansDriver/conciseController.h>

#define PI (3.1415926535897932346f)
#define RECORD_TIME 5000

std::mutex m;
bool returnFlag;

imuDataPack iData;

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
        Sleep(0.0001);
    }
    return;
}

EcRealVector data_interpreter(const imuDataPack& iData)
{
    EcRealVector outVec(3);
    outVec[0] = iData.angle[0];
    outVec[1] = iData.angle[1];
    outVec[2] = iData.angle[2];
    return outVec;
}

int main()
{
    /////////////// �������˶�����ʼ��λ ///////////////
    // ����������
    HRIF_Connect(0, "10.0.0.22", 10003);

    conciseRobotController elfin;
    EcRealVector initJoint = { 0, 0, 90, 0, 90, 0 };
    elfin.moveJ(initJoint, 360, 50);

    Elfin5_PoseTracker tracker;
    /////////////// �趨���������� ///////////////
    tracker.settingTrackerProfile(1, 3, 30);

    //////////////// ���ݲɼ��߳̿��� /////////////////
    thread thread_IMU(imu_record_pose_info);
    EcRealVector imuTargetPose = { 0,0,0 };
    do
    {
        m.lock();
        imuDataPack temImuData = iData;
        m.unlock();
        imuTargetPose = data_interpreter(temImuData);

        Sleep(1000);
    } while (imuTargetPose[0] == 0 && imuTargetPose[1] == 0 && imuTargetPose[2] == 0);

    //////////////// �����ŷ������� /////////////////
    returnFlag = false;
    int iter = 0;

    EcRealVector robPCSPos;
    elfin.readPcsPos(robPCSPos);

    EcRealVector tcp;
    elfin.getTCP(tcp);

    double ServoTime = 0.05;
    double indLookaheadTime = 0.05;
    tracker.trackerInitialized(imuTargetPose, robPCSPos, ServoTime);
    elfin.start_servo(ServoTime, indLookaheadTime);
    double begin = clock();
    //////////////// ���¿���������/////////////////
    while (iter++ < RECORD_TIME)
    {
        // �õ�Ŀ����̬
        EcRealVector deltaPose;
        tracker.getDeltaPose(deltaPose);
        
        EcRealVector servoPose = { robPCSPos[0],  robPCSPos[1],  robPCSPos[2],  deltaPose[0],  deltaPose[1],  deltaPose[2] };
        // ƴװ��̬
        cout << elfin.servoP(servoPose, tcp) << endl;
        //////////////// ����Ŀ������ /////////////////
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();

        imuTargetPose = data_interpreter(iData_temp);

        EcRealVector curPCSPos;
        elfin.readPcsPos(curPCSPos);
        if(!tracker.updateTargetPose(imuTargetPose, curPCSPos)) return 0;

        ////////////// �ȵ�������ʱ��Ƭ ///////////////////
        while (1)
        {
            if ((clock() - begin) >= ((iter + 1) * ServoTime * 1000))
            {
                break;
            }
            Sleep(0.00001);
        }
    }

    returnFlag = true;
    thread_IMU.join();
    HRIF_DisConnect(0);
    return 0;
}
