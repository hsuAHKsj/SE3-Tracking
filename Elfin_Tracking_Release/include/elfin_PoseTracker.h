

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

// Elfin�����˵���̬���ٿ�����
class DLL_API Elfin5_PoseTracker
{
public:
    // �����ŷ��������켣����
    // 1. ����ٶȱ߽� dVecBound
    // 2. �����ٶȱ߽� dAccBound
    // 3. ���Ӽ��ٶȱ߽� dJerkBound
    void settingTrackerProfile(const double dVecBound, const double dAccBound, const double dJerkBound);

    // ��ʼ�����ŷ�����
    // 1. IMU��ʼλ�� senseInitPose
    // 2. �����˳�ʼλ�� robInitPose
    // 3. λ��������� timeCycle
    bool trackerInitialized(const EcRealVector senseInitPose, const EcRealVector robInitPose, const double timeCycle);

    //// ���1����������ʼʱ����̬��ŷ���� Rx,Ry,Rz

    // ���� �����˺�imuλ��
    // 1. IMU��ʼλ�� senseInitPose
    // 2. �����˳�ʼλ�� robInitPose
    bool updateTargetPose(const EcRealVector targetPose, const EcRealVector robotPose);

    // ��ø�����������̬
    // 1. �㷨���ص���̬
    void getDeltaPose(EcRealVector& deltaPose);
};
