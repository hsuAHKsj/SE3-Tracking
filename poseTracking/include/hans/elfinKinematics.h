/*
**  1. 完成正逆解计算；
**  2. 雅可比矩阵；
**  3. 逆雅可比矩阵，已知关节速度求空间速度；
*/
#pragma once
#include "hansTypes.h"
#include "frames.hpp"
//#include "ecConstants.h"

#define			 NUMOFSOLVED  			8

typedef enum EN_inverseKineState
{
	ikState_normal = 0,
	ikState_outofLimit,			// immediately stop motio；
	ikState_noSolution,			// need to switch to emergency stop plan;
	ikState_notContinuous,		// need to switch to emergency stop plan;
}ENInverseKineState;
  

class elfinKinematics
{
#define									 NUMJOINTS				6
public:
	elfinKinematics();
	virtual ~elfinKinematics();

	void setRobotDHParameters
		(
		const EcRealVector& kinematcisParam
		);

	void setJointMotionLimit
		(
		const EcRealVector upperJointLimit,
		const EcRealVector lowerJointLimit
		);

	void setToolCoordinateSystem
		(
		const EcFrame& tool
		);

	virtual EcFrame forwardKinematics
		(
		const EcRealVector& jointPositions
		);

	virtual void forwardKinematics2
		(
		const EcRealVector& jointPositions,
		EcFrame& fkFrame
		);


	virtual ENInverseKineState inverseKinematics
		(
		const EcFrame& target,
		const EcRealVector& current,
		EcRealVector& acs
		);

	virtual EcBoolean checkSingularity(const EcRealVector& jointPosition);

	virtual EcReal getJacobiandeterminant(EcRealVector jointPosition);
	virtual	EcReal getJacobianMatrix(EcRealVector jointPosition, Eigen::MatrixXd& Jacobian);

	virtual void getJacobianWithToolMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian);
	virtual void calcJacobianJointsVelocity(EcRealVector jointPosition, EcRealVector EndEffectorVelocity, EcRealVector& jointsVelocity);
	virtual void calcJacobianEndEffectorVelocity(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcRealVector& endEffectorVelocity);

	// 计算Tool Point的Jacobian；
	void getJacobianWithToolPointMatrix(const Eigen::VectorXd& jointPosition, Eigen::MatrixXd& Jacobian);
	//EcBoolean singularityAvoidanceWithDLS(const EcRealVector& qt, const EcFrame& tar_frame, EcRealVector& q_mod);

	// 计算笛卡尔空间速度、加速度、跃度和关节速度、加速度、跃度的关系；
	void calcSpatialVel2JointVel(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, Eigen::VectorXd& jointVel);
	void calcSpatialAcc2JointAcc(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, const Eigen::VectorXd& spatialAcc, Eigen::VectorXd& jointVel, Eigen::VectorXd& jointAcc);
	void calcSpatialJerk2JointJerk(const EcReal deltaT, const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, const Eigen::VectorXd& spatialAcc, const Eigen::VectorXd& spatialJerk, Eigen::VectorXd& jointVel, Eigen::VectorXd& jointAcc, Eigen::VectorXd& jointJerk);

	void calcJointVel2SpatialVel(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, Eigen::VectorXd& spatialVel);
	void calcJointAcc2SpatialAcc(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc, Eigen::VectorXd& spatialVel, Eigen::VectorXd& spatialAcc);
	void calcJointJerk2SpatialJerk(const EcReal deltaT, const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc, const Eigen::VectorXd& jointJerk, Eigen::VectorXd& spatialVel, Eigen::VectorXd& spatialAcc, Eigen::VectorXd& spatialJerk);
	void getJacobianDotWithToolMatrix(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, Eigen::MatrixXd& JacobianDot);

protected:
//private:
	EN_RobotDH m_mk2DH;
	virtual void elfinDHParameters();
	int mk2Solve4Theta23(double q1, EcVector& pw_vector, double q2[2], double q3[2]);
	int mk2Solve4wrist(double q[6], EcFrame& T, int wrist);
	int normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector);
	double vectorMultiplyOperator(const EcVector& v1, const EcVector& v2);
	EcVector vectorCrossOperator(const EcVector& v1, const EcVector& v2);
	virtual EcRealVector matrixToPcs(EcFrame T);

protected:
//private:

	EcReal			m_a2, m_d1, m_d4, m_d6;	//the length of kinematics link
	EcReal			m_compTheta2, m_compTheta3;


	EcRealMatrixX			m_jacobian;
	EcReal					m_determinant;
	EcRealVector			m_upperJointLimit;
	EcRealVector			m_lowerJointLimit;
	EcRealVector			m_joint2PIExpand;
	EcFrame					m_toolFrame;					// 对于这个全局变量，可能需要加锁，避免运动时计算出错；

	std::vector<KDL::Vector> m_zAxis, m_pAxis, m_velAxis;

	EcReal					m_preJacDet;
	EcRealVector			m_preJointPosition;


	EcVectorVector			m_pdotAxis;
	EcVectorVector			m_omega;

	Eigen::MatrixXd	m_preJdot;						// 保存前一周期的Jdot，用于查分计算Jdotdot;

};


