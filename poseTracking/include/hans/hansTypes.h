#ifndef hansTypes_H_
#define hansTypes_H_

#include <string>
#include <vector>
#include "frames.hpp"
#include <Eigen/Eigen>

#pragma warning( disable : 4290 )
#define  Max_ActualAxisCnt 6

typedef bool                    EcBoolean;
typedef double					EcReal;

///  unsigned integer type used to represent the sizes of containers
typedef std::size_t                       EcSizeT;
typedef unsigned int					  EcU32;

///  similar to EcSizeT but for strings
typedef std::string::size_type            EcStringSizeT;

/// a general floating-point vector
typedef std::vector<EcReal>               EcRealVector;

typedef std::vector<EcU32>				  EcU32Vector;

/// a general boolean vector
typedef std::vector<EcBoolean>            EcBooleanVector;

/// a general 2D boolean vector
typedef std::vector<EcBooleanVector>      EcBooleanVectorVector;

/// a general floating-point array
typedef std::vector<EcRealVector>         EcRealVectorVector;

/// a general floating-point 3D array
typedef std::vector<EcRealVectorVector>   EcRealVector3D;

typedef	KDL::Frame						  EcFrame;
typedef KDL::Vector						  EcVector;
typedef KDL::Rotation					  EcRotation;
typedef std::vector<KDL::Frame>			  EcFrameVector;
typedef std::vector<KDL::Vector>	      EcVectorVector;

typedef Eigen::MatrixXd					  EcRealMatrixX;
typedef Eigen::VectorXd					  EcRealVectorX;
typedef Eigen::Quaternionf				  EcQuaternion;


typedef struct EN_PcsElem
{
	EcReal x, y, z, rx, ry, rz;
	EN_PcsElem()
	{
		x = 0;
		y = 0;
		z = 0;
		rx = 0;
		ry = 0;
		rz = 0;
	}

	EN_PcsElem& operator=(const EN_PcsElem& info)
	{
		copy(this, info);
		return *this;
	}

private:
	void copy(EN_PcsElem* pThis, const EN_PcsElem& info)
	{
		pThis->x = info.x;
		pThis->y = info.y;
		pThis->z = info.z;
		pThis->rx = info.rx;
		pThis->ry = info.ry;
		pThis->rz = info.rz;
	}
}EN_PcsElem;

typedef struct EN_AcsElem{
	EcReal j1, j2, j3, j4, j5, j6;
}EN_AcsElem;

typedef enum EN_FrameType
{
	enFrameType_Global,
	enFrameType_Local
}ENFrameType;

typedef enum EN_TrayMode
{
	enTrayMode_Line=0,
	enTrayMode_Square,
	enTrayMode_Box,
	enTrayMode_List
}EN_TrayMode;


typedef struct EN_RobotDH{
	EcReal theta[6];
	EcReal d[6];
	EcReal a[6];
	EcReal alpha[6];
}RobotDH;

// tolerance �����Ƿ�λ���˶�״̬�жϣ�Ŀ��λ���뵱ǰλ�ò�ͬ����Ϊ�����˶��У���ͬ����Ϊ���˶���ɣ�
typedef enum EN_motionState
{
	enMotionState_NotStarted = 0,
	enMotionState_InProgress,
	enMotionState_Succeeded,
	enMotionState_Failed,
}ENmotionState;

typedef enum EN_MoveException
{
	enMoveException_NO_EXCEPTION,
	enMoveException_COLLISION,
	enMoveException_JOINT_LIMIT,
	enMoveException_SINGULARITY,
	enMoveException_GENERAL_STOPPING_CRITERION
}ENMoveException;

// Profile �����ͣ�scurve trape
typedef enum EN_profileType
{
	enProfileType_Scurve = 0,
	enProfileType_Trape
}ENProfileType;

typedef std::vector<EN_PcsElem>			  EcPcsVector;

#define EC_FALSE            0
#define EC_TRUE             1
#define EC_NULL             0

#endif