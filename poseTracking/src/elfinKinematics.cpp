#include "elfinKinematics.h"
#include <iostream>
using namespace KDL;




elfinKinematics::elfinKinematics()
{
	//0.208, 0.495, 0.1605, 0.455
	m_d1 = 0.208;
	m_d4 = 0.495;
	m_d6 = 0.1605;
	m_a2 = 0.455;
	elfinDHParameters();
	m_jacobian = EcRealMatrixX(6, 6);

	m_upperJointLimit.assign(6, 2 * KDL::PI);
	m_lowerJointLimit.assign(6, -2 * KDL::PI);
	m_joint2PIExpand.assign(3, 0.0);
	m_toolFrame = EcFrame::Identity();

	m_preJacDet = 1.0;
	m_preJointPosition.assign(6, 0.0);

	KDL::Vector v;
	m_zAxis.assign(NUMJOINTS + 1, v);
	m_pAxis.assign(NUMJOINTS + 1, v);
	m_velAxis.assign(NUMJOINTS + 1, v);

	m_pdotAxis.assign(NUMJOINTS + 1, v);
	m_omega.assign(NUMJOINTS + 1, v);

	m_preJdot.setIdentity(NUMJOINTS, NUMJOINTS);
}

elfinKinematics::~elfinKinematics()
{ 

}


EcRealVector elfinKinematics::matrixToPcs(EcFrame T)
{
	EcRealVector p(6);

	p[0] = T.p(0);
	p[1]= T.p(1);
	p[2] = T.p(2);

	T.M.GetRPY(p[3], p[4], p[5]);

	return p;
}


void elfinKinematics::setRobotDHParameters
(
const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

	elfinDHParameters();
}

void elfinKinematics::setJointMotionLimit
(
const EcRealVector upperJointLimit,
const EcRealVector lowerJointLimit
)
{
	for (int i = 0; i < 6; i++)
	{
		m_upperJointLimit[i] = upperJointLimit[i] + epsilon;
		m_lowerJointLimit[i] = lowerJointLimit[i] - epsilon;
	}
	
	std::cout << "joint motion limit:" << upperJointLimit[0] << "," << upperJointLimit[1] << "," << upperJointLimit[2] << "," << upperJointLimit[3] << "," << upperJointLimit[4] << "," << upperJointLimit[5] << std::endl;
	m_joint2PIExpand.assign(3, 0.0);
	int index[3] = { 0, 3, 5 };
	for (EcSizeT i = 0; i < 3; i++) {
		if (m_upperJointLimit[index[i]] > PI)
			m_joint2PIExpand[i] = 2 * PI;
	}
}

void elfinKinematics::elfinDHParameters()
{
	//EN_RobotDH dh;

	m_mk2DH.theta[0] = 0;
	m_mk2DH.theta[1] = PI / 2;
	m_mk2DH.theta[2] = PI / 2;
	m_mk2DH.theta[3] = 0;
	m_mk2DH.theta[4] = 0;
	m_mk2DH.theta[5] = 0;

	m_mk2DH.d[0] = m_d1;
	m_mk2DH.d[1] = 0;
	m_mk2DH.d[2] = 0;
	m_mk2DH.d[3] = m_d4;
	m_mk2DH.d[4] = 0;
	m_mk2DH.d[5] = m_d6;

	m_mk2DH.a[0] = 0;
	m_mk2DH.a[1] = m_a2;
	m_mk2DH.a[2] = 0;
	m_mk2DH.a[3] = 0;
	m_mk2DH.a[4] = 0;
	m_mk2DH.a[5] = 0;

	m_mk2DH.alpha[0] = PI / 2;
	m_mk2DH.alpha[1] = PI;
	m_mk2DH.alpha[2] = PI / 2;
	m_mk2DH.alpha[3] = -PI / 2;
	m_mk2DH.alpha[4] = PI / 2;
	m_mk2DH.alpha[5] = 0;

	m_compTheta2 = PI / 2;
	m_compTheta3 = PI / 2;
}

double elfinKinematics::vectorMultiplyOperator(const EcVector& v1, const EcVector& v2)
{
	return (v1(0) * v2(0) + v1(1) * v2(1) + v1(2) * v2(2));
}

EcVector elfinKinematics::vectorCrossOperator(const EcVector& v1, const EcVector& v2)
{
	return{ v1(1) * v2(2) - v2(1) * v1(2), v1(2) * v2(0) - v1(0) * v2(2), v1(0) * v2(1) - v2(0) * v1(1) };
}

void elfinKinematics::setToolCoordinateSystem
(
const EcFrame& tool
)
{
	m_toolFrame = tool;
}

EcFrame elfinKinematics::forwardKinematics(const EcRealVector& jointPositions)
{
	EcFrame T;
	double calcAcs[6];
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPositions[i] + m_mk2DH.theta[i];
	}

	for (int i = 0; i < 6; i++)
	{
		T = T * EcFrame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	T = T * m_toolFrame;
	return T;
}

void elfinKinematics::forwardKinematics2(const EcRealVector& q,EcFrame& fkFrame)
{
	EcReal sq0, sq1, sq2, sq3, sq4, sq5, cq0, cq1, cq2, cq3, cq4, cq5;
	sq0 = std::sin(q[0]); cq0 = std::cos(q[0]);
	sq1 = std::sin(q[1]); cq1 = std::cos(q[1]);
	sq2 = std::sin(q[2]); cq2 = std::cos(q[2]);
	sq3 = std::sin(q[3]); cq3 = std::cos(q[3]);
	sq4 = std::sin(q[4]); cq4 = std::cos(q[4]);
	sq5 = std::sin(q[5]); cq5 = std::cos(q[5]);

	fkFrame.M = KDL::Rotation((((sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3)*cq4 + (sq1*cq0*cq2 - sq2*cq0*cq1)*sq4)*cq5 + (-(sq1*sq2*cq0 + cq0*cq1*cq2)*sq3 - sq0*cq3)*sq5, -(((sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3)*cq4 + (sq1*cq0*cq2 - sq2*cq0*cq1)*sq4)*sq5 + (-(sq1*sq2*cq0 + cq0*cq1*cq2)*sq3 - sq0*cq3)*cq5, ((sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3)*sq4 - (sq1*cq0*cq2 - sq2*cq0*cq1)*cq4,
		(((sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0)*cq4 + (sq0*sq1*cq2 - sq0*sq2*cq1)*sq4)*cq5 + (-(sq0*sq1*sq2 + sq0*cq1*cq2)*sq3 + cq0*cq3)*sq5, -(((sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0)*cq4 + (sq0*sq1*cq2 - sq0*sq2*cq1)*sq4)*sq5 + (-(sq0*sq1*sq2 + sq0*cq1*cq2)*sq3 + cq0*cq3)*cq5, ((sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0)*sq4 - (sq0*sq1*cq2 - sq0*sq2*cq1)*cq4,
		((-sq1*sq2 - cq1*cq2)*sq4 + (sq1*cq2 - sq2*cq1)*cq3*cq4)*cq5 - (sq1*cq2 - sq2*cq1)*sq3*sq5, -((-sq1*sq2 - cq1*cq2)*sq4 + (sq1*cq2 - sq2*cq1)*cq3*cq4)*sq5 - (sq1*cq2 - sq2*cq1)*sq3*cq5, -(-sq1*sq2 - cq1*cq2)*cq4 + (sq1*cq2 - sq2*cq1)*sq4*cq3
		);
	fkFrame.p = { m_d6*((sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3)*sq4 - m_d6*(sq1*cq0*cq2 - sq2*cq0*cq1)*cq4 - m_d4*sq1*cq0*cq2 - m_a2*sq1*cq0 + m_d4*sq2*cq0*cq1,
		m_d6*((sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0)*sq4 - m_d6*(sq0*sq1*cq2 - sq0*sq2*cq1)*cq4 - m_d4*sq0*sq1*cq2 - m_a2*sq0*sq1 + m_d4*sq0*sq2*cq1,
		-m_d6*(-sq1*sq2 - cq1*cq2)*cq4 + m_d6*(sq1*cq2 - sq2*cq1)*sq4*cq3 + m_d4*sq1*sq2 + m_d4*cq1*cq2 + m_a2*cq1 + m_d1 };
	fkFrame = fkFrame * m_toolFrame;
}

ENInverseKineState elfinKinematics::inverseKinematics(const EcFrame& target, const EcRealVector& current, EcRealVector& acs)
{
	acs.assign(6, 0.0);
	EcBoolean ret;
	int ret1, ret2;
	ret = true;
	EcFrame T;
	T = target * m_toolFrame.Inverse();
	double L6 = m_d6;
	EcVector end_p_vector = T.p;
	EcVector end_z_vector = { T(0, 2), T(1, 2), T(2, 2) };
	EcVector wrist_p_vector = end_p_vector - L6*end_z_vector;

	double q1[2];
	q1[0] = KDL::atan2(wrist_p_vector(1), wrist_p_vector(0));
	q1[1] = q1[0] + KDL::PI;
	double q2_1[2], q2_2[2], q3_1[2], q3_2[2];

	ret1 = mk2Solve4Theta23(q1[0], wrist_p_vector, q2_1, q3_1);
	ret2 = mk2Solve4Theta23(q1[1], wrist_p_vector, q2_2, q3_2);
	if (ret1 + ret2 > 1)
	{
		acs = current;
		//std::cout << "ik no solution" << std::endl;
		return ikState_noSolution;
	}

	// solve joint 1，2，3；
	double q[][6] = { 
	{ q1[0], q2_1[0], q3_1[0], 0, 0, 0 },
	{ q1[0], q2_1[0], q3_1[0], 0, 0, 0 },
	{ q1[0], q2_1[1], q3_1[1], 0, 0, 0 },
	{ q1[0], q2_1[1], q3_1[1], 0, 0, 0 },
	{ q1[1], q2_2[0], q3_2[0], 0, 0, 0 },
	{ q1[1], q2_2[0], q3_2[0], 0, 0, 0 },
	{ q1[1], q2_2[1], q3_2[1], 0, 0, 0 },
	{ q1[1], q2_2[1], q3_2[1], 0, 0, 0 } };

	for (int i = 0; i < 8; i = i + 2)
	{
		mk2Solve4wrist(q[i], T, 1);
		mk2Solve4wrist(q[i + 1], T, -1);
	}


	EcRealVector tempVector(6);
	EcRealVectorVector jointPositions;
	jointPositions.assign(8, tempVector);

	normalize(current, q, jointPositions);			// regular into ±2pi range


	double faux0, faux1, faux2;
	EcRealVector sum1(NUMOFSOLVED), sum2(NUMOFSOLVED), sum3(NUMOFSOLVED), sum6(NUMOFSOLVED);
	for (int i = 0; i < NUMOFSOLVED; i++)
	{
		faux0 = jointPositions[i][0] - current[0];
		sum1[i] = faux0*faux0;

		faux1 = jointPositions[i][1] - current[1];
		sum2[i] = sum1[i] + faux1*faux1;

		faux2 = jointPositions[i][2] - current[2];
		sum3[i] = sum2[i] + faux2*faux2;

		faux0 = jointPositions[i][3] - current[3];
		faux1 = jointPositions[i][4] - current[4];
		faux2 = jointPositions[i][5] - current[5];
		sum6[i] = sum3[i] + faux0*faux0 + faux1*faux1 + 0.1*faux2*faux2;
	}

	double SumMin = sum6[0];
	int out_index = 0;
	for (int i = 1; i < NUMOFSOLVED; i++)
	{
		if (fabs(SumMin - sum6[i]) < 1e-6)
		{
			if (fabs(sum3[out_index] - sum3[i]) < 1e-6)
			{
				if (fabs(sum2[out_index] - sum2[i]) < 1e-6)
				{
					if (sum1[out_index] > sum1[i])
					{
						SumMin = sum6[i];
						out_index = i;
					}
				}
				else if (sum2[out_index] > sum2[i])
				{
					SumMin = sum6[i];
					out_index = i;
				}
			}
			else if (sum3[out_index] > sum3[i])
			{
				SumMin = sum6[i];
				out_index = i;
			}

		}
		else if (SumMin > sum6[i])
		{
			SumMin = sum6[i];
			out_index = i;
		}
	}

	for (int ii = 0; ii < 6; ii++)
	{
		acs[ii] = jointPositions[out_index][ii];
	}

/**/

	for (int ii = 0; ii < 6; ii++)
	{
		if ((acs[ii] < m_lowerJointLimit[ii] + epsilon) || acs[ii]>m_upperJointLimit[ii]+epsilon)
		{
			//std::cout << "ik out of limit. Joint index: "<< ii << std::endl;
			return ikState_outofLimit;
		}
	}

	double deltaSum = 0.1;
	for (int ii = 0; ii < 6; ii++)
	{
		deltaSum += fabs(acs[ii] - current[ii]);
	}
	if (deltaSum > PI/6)
	{
		return ikState_notContinuous;
	}

	
	return ikState_normal;
}


EcReal elfinKinematics::getJacobiandeterminant(EcRealVector jointPosition)
{
	Eigen::MatrixXd Jacobian(6, 6);
	getJacobianMatrix(jointPosition, Jacobian);
	return Jacobian.determinant();
}


EcReal elfinKinematics::getJacobianMatrix(EcRealVector jointPosition, Eigen::MatrixXd& Jacobian)
{
	Jacobian.resize(NUMJOINTS, NUMJOINTS);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]), s6 = KDL::sin(jointPosition[5]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]), c6 = KDL::cos(jointPosition[5]);

	Jacobian << -m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 + m_d6*(s1*s2*c3 - s1*s3*c2)*c5 + m_d4*s1*s2*c3 + m_a2*s1*s2 - m_d4*s1*s3*c2, -(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3 + m_a2*c2)*c1, (-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3)*c1, -(s2*s3 + c2*c3)*(m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5 - m_d4*s1*s2*c3 + m_d4*s1*s3*c2) + (-s1*s2*c3 + s1*s3*c2)*(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3), (m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5)*(s2*c3 - s3*c2)*s4 + (-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4)*(-(s1*s2*s3 + s1*c2*c3)*s4 + c1*c4), -(m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5)*(-(-s2*s3 - c2*c3)*c5 + (s2*c3 - s3*c2)*s5*c4) + (((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - (s1*s2*c3 - s1*s3*c2)*c5)*(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4),
		m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5 - m_d4*s2*c1*c3 - m_a2*s2*c1 + m_d4*s3*c1*c2, -(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3 + m_a2*c2)*s1, (-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3)*s1, (s2*s3 + c2*c3)*(m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5 - m_d4*s2*c1*c3 + m_d4*s3*c1*c2) - (-s2*c1*c3 + s3*c1*c2)*(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4 + m_d4*s2*s3 + m_d4*c2*c3), -(m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5)*(s2*c3 - s3*c2)*s4 - (-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4)*(-(s2*s3*c1 + c1*c2*c3)*s4 - s1*c4), (m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5)*(-(-s2*s3 - c2*c3)*c5 + (s2*c3 - s3*c2)*s5*c4) - (((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - (s2*c1*c3 - s3*c1*c2)*c5)*(-m_d6*(-s2*s3 - c2*c3)*c5 + m_d6*(s2*c3 - s3*c2)*s5*c4),
		0, (m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5 - m_d4*s1*s2*c3 - m_a2*s1*s2 + m_d4*s1*s3*c2)*s1 + (m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5 - m_d4*s2*c1*c3 - m_a2*s2*c1 + m_d4*s3*c1*c2)*c1, -(m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5 - m_d4*s1*s2*c3 + m_d4*s1*s3*c2)*s1 - (m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5 - m_d4*s2*c1*c3 + m_d4*s3*c1*c2)*c1, -(-s1*s2*c3 + s1*s3*c2)*(m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5 - m_d4*s2*c1*c3 + m_d4*s3*c1*c2) + (-s2*c1*c3 + s3*c1*c2)*(m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5 - m_d4*s1*s2*c3 + m_d4*s1*s3*c2), (m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5)*(-(s2*s3*c1 + c1*c2*c3)*s4 - s1*c4) - (m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5)*(-(s1*s2*s3 + s1*c2*c3)*s4 + c1*c4), (m_d6*((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - m_d6*(s1*s2*c3 - s1*s3*c2)*c5)*(((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - (s2*c1*c3 - s3*c1*c2)*c5) - (((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - (s1*s2*c3 - s1*s3*c2)*c5)*(m_d6*((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - m_d6*(s2*c1*c3 - s3*c1*c2)*c5),
		0, s1, -s1, -s2*c1*c3 + s3*c1*c2, -(s2*s3*c1 + c1*c2*c3)*s4 - s1*c4, ((s2*s3*c1 + c1*c2*c3)*c4 - s1*s4)*s5 - (s2*c1*c3 - s3*c1*c2)*c5,
		0, -c1, c1, -s1*s2*c3 + s1*s3*c2, -(s1*s2*s3 + s1*c2*c3)*s4 + c1*c4, ((s1*s2*s3 + s1*c2*c3)*c4 + s4*c1)*s5 - (s1*s2*c3 - s1*s3*c2)*c5,
		1, 0, 0, s2*s3 + c2*c3, -(s2*c3 - s3*c2)*s4, -(-s2*s3 - c2*c3)*c5 + (s2*c3 - s3*c2)*s5*c4;
	return Jacobian.determinant();
}


void elfinKinematics::getJacobianWithToolMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian)
{
	// 计算关节位置，计算z轴方向，计算坐标系原点位置
	double calcAcs[6];
	Jacobian.resize(6, 6);
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPosition[i] + m_mk2DH.theta[i];
	}
	Frame T = Frame::Identity();
	for (EcSizeT i = 0; i < jointPosition.size(); i++)
	{
		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;
		T = T*Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	// 计算工具坐标系位置
	T = T * m_toolFrame;

	// 计算工具雅可比矩阵
	for (EcSizeT i = 0; i < jointPosition.size(); i++)
	{
		m_velAxis[i] = T.p - m_pAxis[i];
		Vector z = vectorCrossOperator(m_zAxis[i], m_velAxis[i]);
		Jacobian(0, i) = z[0];
		Jacobian(1, i) = z[1];
		Jacobian(2, i) = z[2];

		Jacobian(3, i) = m_zAxis[i][0];
		Jacobian(4, i) = m_zAxis[i][1];
		Jacobian(5, i) = m_zAxis[i][2];
	}
}


EcBoolean elfinKinematics::checkSingularity(const EcRealVector& jointPosition)
{
	// add singularity recognition, compare the determinant of current and solved joint position;
	EcReal det_current = getJacobiandeterminant(jointPosition);
	EcBoolean isSingularity = false;
	/*
	if (fabs(det_current) < 0.001 && fabs(det_current) < fabs(m_preJacDet))
	{
		
		//std::cout << "robot is in singularity position." << "current joint: " << jointPosition[0] << "," << jointPosition[1] << "," << jointPosition[2] << "," << jointPosition[3] << "," << jointPosition[4] << "," << jointPosition[5]
		//	<<", pre Joint: "<< m_preJointPosition[0] << "," << m_preJointPosition[1] << "," << m_preJointPosition[2] << "," << m_preJointPosition[3] << "," << m_preJointPosition[4] << "," << m_preJointPosition[5] << std::endl;
		
		isSingularity = EcTrue;
	}
	*/
	if (fabs(det_current) < 0.001 )
	{
		isSingularity = true;
	}
	m_preJacDet = det_current;
	m_preJointPosition = jointPosition;
	return isSingularity;
}



void elfinKinematics::calcJacobianJointsVelocity(EcRealVector jointPosition, EcRealVector EndEffectorVelocity, EcRealVector& jointsVelocity)
{
	getJacobianWithToolMatrix(jointPosition, m_jacobian);
	//getJacobianMatrix(jointPosition, m_jacobian);
	Eigen::VectorXd jointsVel(6), eeVel(6);
	eeVel << EndEffectorVelocity[0], EndEffectorVelocity[1], EndEffectorVelocity[2], EndEffectorVelocity[3], EndEffectorVelocity[4], EndEffectorVelocity[5];
	jointsVel = m_jacobian.inverse()*eeVel;				//inverse Jacobian

	for (int i = 0; i < 6; i++)
	{
		jointsVelocity[i] = jointsVel[i];
	}
}

void elfinKinematics::calcJacobianEndEffectorVelocity(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcRealVector& endEffectorVelocity)
{
	getJacobianWithToolMatrix(jointPosition, m_jacobian);
	Eigen::VectorXd jointsVel(6), eeVel(6);
	jointsVel << jointVelocity[0], jointVelocity[1], jointVelocity[2], jointVelocity[3], jointVelocity[4], jointVelocity[5];
	eeVel = m_jacobian * jointsVel;
	for (int i = 0; i < 6; i++)
	{
		endEffectorVelocity[i] = eeVel[i];
	}
}



int elfinKinematics::mk2Solve4Theta23(double q1, Vector& pw_vector, double q2[2], double q3[2])
{
	int ret = 0;
	double L2 =m_a2;
	double L3 = m_d4;
	Frame T01;
	T01.M = Rotation(std::cos(q1), 0, std::sin(q1),
		std::sin(q1), 0, -std::cos(q1),
		0, 1, 0);
	T01.p = { 0, 0, m_d1 };
	Vector p1_vector;
	p1_vector = T01.Inverse()*pw_vector;

	double r = KDL::sqrt(p1_vector(0)*p1_vector(0) + p1_vector(1)*p1_vector(1));
	double beta = KDL::atan2(p1_vector(1), p1_vector(0));
	double elem1 = (L2*L2 + r*r - L3*L3) / (2 * r*L2);
	if ((elem1 < -1) || (elem1>1))
	{
		ret = 1;
		return ret;
	}

	double gama = KDL::acos(elem1);

	q2[0] = beta + gama - PI / 2;
	q2[1] = beta - gama - PI / 2;

	double elem2 = (L2*L2 + L3*L3 - r*r) / (2 * L2*L3);
	if ((elem2 < -1) || (elem2>1))
	{
		ret = 1;
		return ret;
	}
	double omega = KDL::acos(elem2);
	q3[0] = -omega + PI;
	q3[1] = omega - PI;

	return ret;
}


int elfinKinematics::normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector)
{
	for (int i = 0; i < 8; i++)
	{
		for (int k = 0; k < 6; k++)
		{
			if (q[i][k] > PI)
			{
				qVector[i][k] = q[i][k] - 2.0 * PI;
			}
			else if (q[i][k] <= -PI)
			{
				qVector[i][k] = q[i][k] + 2.0 * PI;
			}
			else{
				qVector[i][k] = q[i][k];
			}
		}
		
		int index[3] = { 0, 3, 5 };
		for (int jj = 0; jj < 3; jj++)
		{
			int cycleCount = current[index[jj]] / (2 * PI);
			m_joint2PIExpand[jj] = 2 * PI * cycleCount;
			EcReal qsign = m_joint2PIExpand[jj];
			qVector[i][index[jj]] += qsign;

			EcReal	delta = fabs(qVector[i][index[jj]] - current[index[jj]]);
			EcReal deltaPlus2PI = fabs(qVector[i][index[jj]] + 2 * PI - current[index[jj]]);
			EcReal deltaMinus2PI = fabs(qVector[i][index[jj]] - 2 * PI - current[index[jj]]);

			if (fabs(deltaPlus2PI) < deltaMinus2PI)
				qVector[i][index[jj]] = (delta < deltaPlus2PI) ? qVector[i][index[jj]]:qVector[i][index[jj]] + 2 * PI;
			else
				qVector[i][index[jj]] = (delta < deltaMinus2PI) ? qVector[i][index[jj]] : qVector[i][index[jj]] - 2 * PI;


			if (fabs(qVector[i][index[jj]]) > m_upperJointLimit[index[jj]])	
				qVector[i][index[jj]] -= sign(qVector[i][index[jj]]) * 2 * PI;

		}
		
		
	}
	return 0;
}



int elfinKinematics::mk2Solve4wrist(double q[6], Frame& T, int wrist)
{
	int ret = 0;
	Frame T03;
	q[1] = q[1] + m_compTheta2;
	q[2] = q[2] + m_compTheta3;
	/*
	T01 = Frame::DH(m_mk2DH.a[0], m_mk2DH.alpha[0], m_mk2DH.d[0], q[0]);
	T12 = Frame::DH(m_mk2DH.a[1], m_mk2DH.alpha[1], m_mk2DH.d[1], q[1]);
	T23 = Frame::DH(m_mk2DH.a[2], m_mk2DH.alpha[2], m_mk2DH.d[2], q[2]);
	T03 = T01*T12*T23;
	*/

	EcReal sq0, cq0, sq1, cq1, sq2, cq2,sq3,cq3,sq4,cq4;
	sq0 = std::sin(q[0]);	cq0 = std::cos(q[0]);
	sq1 = std::sin(q[1]);	cq1 = std::cos(q[1]);
	sq2 = std::sin(q[2]);	cq2 = std::cos(q[2]);

	T03.M = Rotation(sq1*sq2*cq0 + cq0*cq1*cq2, -sq0, -sq1*cq0*cq2 + sq2*cq0*cq1,
		sq0*sq1*sq2 + sq0*cq1*cq2, cq0, -sq0*sq1*cq2 + sq0*sq2*cq1,
		sq1*cq2 - sq2*cq1, 0, sq1*sq2 + cq1*cq2);


	Vector x3_vector = { T03(0, 0), T03(1, 0), T03(2, 0) };
	Vector y3_vector = { T03(0, 1), T03(1, 1), T03(2, 1) };
	Vector z3_vector = { T03(0, 2), T03(1, 2), T03(2, 2) };

	Vector z5_vector = { T(0, 2), T(1, 2), T(2, 2) };
	Vector z4_vector = vectorCrossOperator(z3_vector, z5_vector);

	if (z4_vector.Norm() < 0.00000001)
	{
		if (wrist == 1)
			q[3] = 0;
		else
			q[3] = -PI;
	}
	else
	{
		double cq4 = wrist*vectorMultiplyOperator(y3_vector, z4_vector);
		double sq4 = -wrist*vectorMultiplyOperator(x3_vector, z4_vector);
		q[3] = KDL::atan2(sq4, cq4);
	}


	sq3 = std::sin(q[3]);	cq3 = std::cos(q[3]);
	Vector x4_vector = { (sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3,
		(sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0,
		(sq1*cq2 - sq2*cq1)*cq3 };
	Vector y4_vector = { sq1*cq0*cq2 - sq2*cq0*cq1,
		sq0*sq1*cq2 - sq0*sq2*cq1,
		-sq1*sq2 - cq1*cq2 };



	double cq5 = -vectorMultiplyOperator(y4_vector, z5_vector);
	double sq5 = vectorMultiplyOperator(x4_vector, z5_vector);
	q[4] = KDL::atan2(sq5, cq5);

	Vector x6_vector = { T(0, 0), T(1, 0), T(2, 0) };

	sq4 = std::sin(q[4]);	cq4 = std::cos(q[4]);
	Vector x5_vector = { ((sq1*sq2*cq0 + cq0*cq1*cq2)*cq3 - sq0*sq3)*cq4 + (sq1*cq0*cq2 - sq2*cq0*cq1)*sq4,
		((sq0*sq1*sq2 + sq0*cq1*cq2)*cq3 + sq3*cq0)*cq4 + (sq0*sq1*cq2 - sq0*sq2*cq1)*sq4,
		(-sq1*sq2 - cq1*cq2)*sq4 + (sq1*cq2 - sq2*cq1)*cq3*cq4 };

	Vector y5_vector = { -(sq1*sq2*cq0 + cq0*cq1*cq2)*sq3 - sq0*cq3,
		-(sq0*sq1*sq2 + sq0*cq1*cq2)*sq3 + cq0*cq3,
		-(sq1*cq2 - sq2*cq1)*sq3 };

	double cq6 = vectorMultiplyOperator(x6_vector, x5_vector);
	double sq6 = vectorMultiplyOperator(x6_vector, y5_vector);
	q[5] = KDL::atan2(sq6, cq6);

	q[1] = q[1] - m_mk2DH.theta[1];
	q[2] = q[2] - m_mk2DH.theta[2];

	return ret;
}


void elfinKinematics::getJacobianWithToolPointMatrix(const Eigen::VectorXd& jointPosition, Eigen::MatrixXd& Jacobian)
{
	double calcAcs[6];
	Jacobian.resize(NUMJOINTS, NUMJOINTS);
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}
	Frame T = Frame::Identity();
	for (EcSizeT i = 0; i < NUMJOINTS; i++)
	{
		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;
		T = T * Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	T = T * m_toolFrame;

	for (EcSizeT i = 0; i < NUMJOINTS; i++)
	{
		m_velAxis[i] = T.p - m_pAxis[i];

		Vector z = vectorCrossOperator(m_zAxis[i], m_velAxis[i]);
		Jacobian(0, i) = z[0];
		Jacobian(1, i) = z[1];
		Jacobian(2, i) = z[2];

		Jacobian(3, i) = m_zAxis[i][0];
		Jacobian(4, i) = m_zAxis[i][1];
		Jacobian(5, i) = m_zAxis[i][2];
	}
}

// ref::matlab ABB-Library;
void elfinKinematics::getJacobianDotWithToolMatrix(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, Eigen::MatrixXd& JacobianDot)
{
	double calcAcs[6];
	JacobianDot.resize(NUMJOINTS, NUMJOINTS);
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}

	Frame T = Frame::Identity();
	m_zAxis[0] = T.M.UnitZ();
	m_pAxis[0] = T.p;
	for (EcSizeT i = 1; i < NUMJOINTS + 1; i++)
	{
		T = T * Frame::DH(m_mk2DH.a[i - 1], m_mk2DH.alpha[i - 1], m_mk2DH.d[i - 1], calcAcs[i - 1]);
		if (i == NUMJOINTS)
			T = T * m_toolFrame;			// 叠加工具坐标系的偏置

		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;

		m_omega[i] = jointVel(i - 1) * m_zAxis[i - 1] + m_omega[i - 1];
		m_pdotAxis[i] = m_pdotAxis[i - 1] + vectorCrossOperator(m_omega[i], m_pAxis[i] - m_pAxis[i - 1]);
	}


	for (EcSizeT i = 0; i < NUMJOINTS; i++)
	{
		//JvDot(:,i) = [cross(cross(m_omega(:,i),m_zAxis(:,i)),(m_pAxis(:,7)-m_pAxis(:,i)))+cross(m_zAxis(:,i),m_pdotAxis(:,7)-m_pdotAxis(:,i)); 
		// cross(m_omega(:, i), m_zAxis(:, i))];
		m_velAxis[i] = T.p - m_pAxis[i];
		Vector z = vectorCrossOperator(vectorCrossOperator(m_omega[i], m_zAxis[i]), (m_pAxis[NUMJOINTS] - m_pAxis[i])) + vectorCrossOperator(m_zAxis[i], m_pdotAxis[NUMJOINTS] - m_pdotAxis[i]);
		Vector w = vectorCrossOperator(m_omega[i], m_zAxis[i]);

		JacobianDot(0, i) = z[0];
		JacobianDot(1, i) = z[1];
		JacobianDot(2, i) = z[2];

		JacobianDot(3, i) = w[0];
		JacobianDot(4, i) = w[1];
		JacobianDot(5, i) = w[2];
	}
}



void elfinKinematics::calcSpatialVel2JointVel(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, Eigen::VectorXd& jointVel)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	jointVel = J.inverse() * spatialVel;

}

void elfinKinematics::calcSpatialAcc2JointAcc(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, const Eigen::VectorXd& spatialAcc, Eigen::VectorXd& jointVel, Eigen::VectorXd& jointAcc)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS), Jdot(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	Eigen::MatrixXd Jinv = J.inverse();

	jointVel = Jinv * spatialVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	jointAcc = Jinv * (spatialAcc - Jdot * jointVel);
}

void elfinKinematics::calcSpatialJerk2JointJerk(const EcReal deltaT, const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& spatialVel, const Eigen::VectorXd& spatialAcc, const Eigen::VectorXd& spatialJerk, Eigen::VectorXd& jointVel, Eigen::VectorXd& jointAcc, Eigen::VectorXd& jointJerk)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS), Jdot(NUMJOINTS, NUMJOINTS), Jdotdot(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	Eigen::MatrixXd Jinv = J.inverse();
	jointVel = Jinv * spatialVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	jointAcc = Jinv * (spatialAcc - Jdot * jointVel);

	// 如果是样条曲线的控制参数u，那么 dt = |p'|du / v(u)
	Jdotdot = (Jdot - m_preJdot) / deltaT;				// 在第一次计算周期，m_preJdot是不准确的，可能会带来很大的误差；

	jointJerk = Jinv * (spatialJerk - Jdotdot * jointVel - 2 * Jdot * jointAcc);


	//EcReal diff = Jdotdot.sum();
	m_preJdot = Jdot;
}


void elfinKinematics::calcJointVel2SpatialVel(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel,
	Eigen::VectorXd& spatialVel)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

}

void elfinKinematics::calcJointAcc2SpatialAcc(const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc,
	Eigen::VectorXd& spatialVel, Eigen::VectorXd& spatialAcc)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS), Jdot(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	spatialAcc = Jdot * jointVel + J * jointAcc;
}

void elfinKinematics::calcJointJerk2SpatialJerk(const EcReal deltaT, const Eigen::VectorXd& jointPosition, const Eigen::VectorXd& jointVel, const Eigen::VectorXd& jointAcc, const Eigen::VectorXd& jointJerk,
	Eigen::VectorXd& spatialVel, Eigen::VectorXd& spatialAcc, Eigen::VectorXd& spatialJerk)
{
	Eigen::MatrixXd J(NUMJOINTS, NUMJOINTS), Jdot(NUMJOINTS, NUMJOINTS), Jdotdot(NUMJOINTS, NUMJOINTS);
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	spatialAcc = Jdot * jointVel + J * jointAcc;

	Jdotdot = (Jdot - m_preJdot) / deltaT;				// 在第一次计算周期，m_preJdot是不准确的，可能会带来很大的误差；
	spatialJerk = Jdotdot * spatialVel + 2 * Jdot * spatialAcc + J * spatialJerk;

	//EcReal diff = Jdotdot.sum();
	m_preJdot = Jdot;

}



