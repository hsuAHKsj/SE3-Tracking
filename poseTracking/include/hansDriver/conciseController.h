
#include <hansDriver/HR_Pro.h>
#include <iostream>
#include <Windows.h>
#include <hans/elfinKinematics.h>
#include <Eigen/Dense>

typedef double	EcReal;

/// a general floating-point vector
typedef std::vector<EcReal>               EcRealVector;

/// a general floating-point array
typedef std::vector<EcRealVector>         EcRealVectorVector;



class conciseRobotController
{
public:

	elfinKinematics elfin;

	// 运行movej指令
	void moveJ(EcRealVector tarPos, double indAcc = 360, double indVelocity = 50)
	{
		int nMoveType = 0, nIsUseJoint = 1, nIsSeek = 0, nIOBit = 0, nIOState = 0;
		double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0, dAcc = indAcc, dVelocity = indVelocity, dRadius = 0;
		double dJ1 = tarPos[0], dJ2 = tarPos[1], dJ3 = tarPos[2], dJ4 = tarPos[3], dJ5 = tarPos[4], dJ6 = tarPos[5];
		string sTcpName = "TCP", sUcsName = "Base", strCmdID = "Point_0";
		int nRet = 0;
		int nPointNo = 0;

		int boxID = 0;

		nRet = HRIF_MoveJ(boxID,
			dX, dY, dZ, dRx, dRy, dRz,
			dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			sTcpName, sUcsName, dVelocity, dAcc, dRadius,
			nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
		printf("Result of command HRIF_MoveJ: Box-%d Retcode = %d, move to %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", boxID, nRet, strCmdID.c_str(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);

		do
		{
			bool bBlendingDone = false;
			nRet = HRIF_IsBlendingDone(boxID, bBlendingDone);
			if (nRet == 0 && !bBlendingDone)
			{
				cout << "Moving......\r\n";
				Sleep(1000);
				continue;
			}
			else
				break;
		} while (true);
	};


	// 运行movej指令
	void moveP(EcRealVector tarPos, EcRealVector acs, double indAcc = 1, double indVelocity = 0.2)
	{
		int nMoveType = 0, nIsSeek = 0, nIOBit = 0, nIOState = 0;
		double dAcc = indAcc, dVelocity = indVelocity, dRadius = 0;
		double dX = tarPos[0], dY = tarPos[1], dZ = tarPos[2], dRx = tarPos[3], dRy = tarPos[4], dRz = tarPos[5];
		double dJ1 = acs[0], dJ2 = acs[1], dJ3 = acs[2], dJ4 = acs[3], dJ5 = acs[4], dJ6 = acs[5];
		string sTcpName = "TCP", sUcsName = "Base", strCmdID = "Point_0";
		int nRet = 0;
		int nPointNo = 0;

		int boxID = 0;

		nRet = HRIF_MoveL(boxID,
			dX, dY, dZ, dRx, dRy, dRz,
			dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			sTcpName, sUcsName, dVelocity, dAcc, dRadius,
			nIsSeek, nIOBit, nIOState, strCmdID);
		printf("Result of command HRIF_MoveJ: Box-%d Retcode = %d, move to %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", boxID, nRet, strCmdID.c_str(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);

		do
		{
			bool bBlendingDone = false;
			nRet = HRIF_IsBlendingDone(boxID, bBlendingDone);
			if (nRet == 0 && !bBlendingDone)
			{
				cout << "Moving......\r\n";
				Sleep(1000);
				continue;
			}
			else
				break;
		} while (true);
	};

	// 获得当前TCP角速度
	int getAngularvelocity(double& angVel)
	{
		unsigned int boxID = 0;
		EcRealVector tcpVel(6);
		int res = HRIF_ReadActTcpVel(0, tcpVel[0], tcpVel[1], tcpVel[2], tcpVel[3], tcpVel[4], tcpVel[5]);

		Eigen::Vector3d rot = {tcpVel[3], tcpVel[4], tcpVel[5]};
		angVel = rot.norm(); // 获得旋转速度
		return res;
	}

	// 获得当前TCP角速度
	void getAngularVelocityByJd(double& angVel)
	{
		unsigned int boxID = 0;
		Eigen::VectorXd qd(6);
		EcRealVector q(6);
		// 读取关节速度
		HRIF_ReadActJointVel(boxID, qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
		// 读取关节位置
		HRIF_ReadActJointPos(boxID, q[0], q[1], q[2], q[3], q[4], q[5]);
		// 获得雅可比矩阵
		Eigen::MatrixXd Jac;
		elfin.getJacobianMatrix(q,Jac);
		// 计算末端的几何角速度
		Eigen::VectorXd geoVel;
		geoVel  = Jac * qd;

		// 计算几何角速度
		angVel = geoVel.tail(3).norm();
	}

	// 启动伺服指令
	int start_servo(double indServoTime, double indLookaheadTime)
	{
		int boxID = 0;
		double dServoTime = indServoTime;
		double dLookaheadTime = indLookaheadTime;
		return HRIF_StartServo(boxID, dServoTime, dLookaheadTime);
	}

	// 运行 servoj 指令
	int servoj(EcRealVector tarPos)
	{
		int boxID = 0;
		return HRIF_PushServoJ(boxID, tarPos[0], tarPos[1], tarPos[2], tarPos[3], tarPos[4], tarPos[5]);
	};

	// 运行 servoj 指令
	int servoP(const EcRealVector& tarPos)
	{
		vector<double> indCoord(6);

		for (int i = 0; i < tarPos.size(); i++)
		{
			indCoord.at(i) = tarPos[i];
		}
		int boxID = 0;
		vector<double> dCoord = indCoord;
		vector<double> vecUcs = { 0,0,0,0,0,0 };
		vector<double> vecTcp = { 0,0,0,0,0,0 };
		return HRIF_PushServoP(boxID, dCoord, vecUcs, vecTcp);
	};

	int readAcsPos(EcRealVector& tarPos)
	{
		tarPos.resize(6);
		int boxID = 0;
		double dX, dY, dZ, dRx, dRy, dRz;
		double dJ1, dJ2, dJ3, dJ4, dJ5, dJ6;
		double dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz;
		double dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz;
		int res = HRIF_ReadActPos(boxID, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);

		tarPos[0] = dJ1;
		tarPos[1] = dJ2;
		tarPos[2] = dJ3;
		tarPos[3] = dJ4;
		tarPos[4] = dJ5;
		tarPos[5] = dJ6;

		return res;
	}

	int readPcsPos(EcRealVector& tarPos)
	{
		tarPos.resize(6);
		int boxID = 0;
		double dX, dY, dZ, dRx, dRy, dRz;
		double dJ1, dJ2, dJ3, dJ4, dJ5, dJ6;
		double dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz;
		double dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz;
		int res = HRIF_ReadActPos(boxID, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);

		tarPos[0] = dX;
		tarPos[1] = dY;
		tarPos[2] = dZ;
		tarPos[3] = dRx;
		tarPos[4] = dRy;
		tarPos[5] = dRz;
		return res;
	}

};