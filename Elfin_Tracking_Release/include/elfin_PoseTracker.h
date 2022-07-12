

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

#include <Eigen/Core>
#include <vector>

typedef double	EcReal;
typedef std::vector<EcReal>               EcRealVector;
typedef std::vector<EcRealVector>         EcRealVectorVector;

#define PI (3.1415926535897932346f)

// Elfin机器人的姿态跟踪控制器
class DLL_API Elfin5_PoseTracker
{
public:
    // 设置伺服控制器轨迹参数
    // 1. 最大速度边界 dVecBound
    // 2. 最大加速度边界 dAccBound
    // 3. 最大加加速度边界 dJerkBound
    void settingTrackerProfile(const double dVecBound, const double dAccBound, const double dJerkBound);

    // 开始启动伺服控制
    // 1. IMU初始位姿 senseInitPose
    // 2. 机器人初始位姿 robInitPose
    // 3. 位姿输出周期 timeCycle
    bool trackerInitialized(const EcRealVector senseInitPose, const EcRealVector robInitPose, const double timeCycle);

    //// 入参1：传感器初始时刻姿态，欧拉角 Rx,Ry,Rz

    // 更新 机器人和imu位姿
    // 1. IMU初始位姿 senseInitPose
    // 2. 机器人初始位姿 robInitPose
    bool updateTargetPose(const EcRealVector targetPose, const EcRealVector robotPose);

    // 获得跟踪器跟踪姿态
    // 1. 算法返回的姿态
    void getDeltaPose(EcRealVector& deltaPose);
};
