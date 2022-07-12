#pragma once
/*******************************************************************
 *	Copyright(c) 2020-2022 Company Name
 *  All rights reserved.
 *	
 *	FileName:HR_Pro.h
 *	Descriptio:C++SDK
 *  version:1.0.0.0
 *	
 *	Date:2022-02-25
 *	Author:chenpeng
 *	Descriptio:���Ӻ���˵��
 ******************************************************************/

#ifndef _HR_PRO_H_
#define _HR_PRO_H_


#include <string>
#include <vector>
using namespace std;

#define MaxBox 5
//#ifdef ONEDLL_EXPORTS
//#define ONEDLL_API __declspec(dllexport)
//#else
//#define ONEDLL_API __declspec(dllimport)
//#endif
#ifdef HANSROBOT_PRO_EXPORTS
#define HANSROBOT_PRO_API __declspec(dllexport)
#else
#define HANSROBOT_PRO_API __declspec(dllimport)
#endif
#ifdef __cplusplus
extern "C"
{
#endif
//************************************************************************/
//**    �ص��ຯ���ӿ�
//**    ���µĽӿڣ�boxID�Ǵ����ĸ����䣬����������5������    
//**    Ĭ��ʹ��ŷ���Ǳ�ʾ����λ�Ǻ��ף���                                                                  
//************************************************************************/

/**
 *	@brief:��־�ص�����
 *	@param logLevel : ��־�ȼ�
 *	@param strLog : ��־��Ϣ
 *	@param arg:
 *	
 *	@return : �ص����
 */
typedef void (*LogDbgCallback)(int logLevel, const string &strLog, void *arg);
HANSROBOT_PRO_API int HRIF_SetLogDbgCB(unsigned int boxID, LogDbgCallback ptr, void *arg);

/**
 *	@brief:�¼��ص�����
 *	@param nErrorCode : ������
 *	@param nState : ״̬
 *	@param strState: ������Ϣ
 *	@param arg:
 *	
 *	@return : �ص����
 */
typedef void (*EventCallback)(int nErrorCode, int nState, const string &strState, void *arg);
HANSROBOT_PRO_API int HRIF_SetEventCB(unsigned int boxID, EventCallback ptr, void *arg);


//************************************************************************/
//**    ��ʼ�������ӿ�                                                             
//************************************************************************/
/**
 *	@brief:���ӿ�����10003�˿�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param hostName : ������IP��ַ������ʵ�����õ�IP��ַ����
 *	@param nPort : �������˿ںţ�Ĭ��ֵ=10003
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_Connect(unsigned int boxID, const char *hostName, unsigned short nPort);

/**
 *	@brief:�Ͽ����ӿ�����10003�˿�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_DisConnect(unsigned int boxID);

/**
 *	@brief:�������Ƿ�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : false : ������δ����
 *            true : ������������
 */
HANSROBOT_PRO_API bool HRIF_IsConnected(unsigned int boxID);

/**
 *	@brief:�������ϵ�(�Ͽ������˹��磬ϵͳ�ػ�)
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ShutdownRobot(unsigned int boxID);

/**
 *	@brief:���ӿ���������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_Connect2Box(unsigned int boxID);

/**
 *	@brief:�������ϵ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_Electrify(unsigned int boxID);

/**
 *	@brief:�����˶ϵ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_Blackout(unsigned int boxID);

/**
 *	@brief:���ӹ����л�������վ����ʼ����վ�����ò�����������ã���ɺ���ת��ȥʹ��״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_Connect2Controller(unsigned int boxID);

/**
 *	@brief:�ж��Ƿ�Ϊģ�������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nSimulateRobot ���Ƿ�Ϊģ�������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_IsSimulateRobot(unsigned int boxID, int &nSimulateRobot);

/**
 *	@brief:�������Ƿ�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nSimulateRobot : 0 : ��ʵ������
 *                           1 : ģ�������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_IsControllerStarted(unsigned int boxID, int &nStarted);

/**
 *	@brief:��ȡ�������汾��
			�汾�ŵĸ�ʽ��.�ָ������ֵ���������
			CPSVer.controlVer.BoxVerMajor.BoxVerMid.BoxVerMin.AlgorithmVer.ElfinFirmwareVer
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param strVer : ����汾��
 *	@param nCPSVersion : CPS�汾
 *	@param nCodesysVersion : �������汾
 *	@param nBoxVerMajor : ����汾��:0��ģ����� 1-4������汾��
 *	@param nBoxVerMid : ���ư�̼��汾
 *	@param nBoxVerMin : ���ư�̼��汾
 *	@param nAlgorithmVer : �㷨�汾
 *	@param nElfinFirmwareVer : �̼��汾
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadVersion(unsigned int boxID, string &strVer, int &nCPSVersion, int &nCodesysVersion,
                     int &nBoxVerMajor, int &nBoxVerMid, int &nBoxVerMin,
                     int &nAlgorithmVer, int &nElfinFirmwareVer);

/**
 *	@brief:��ȡ����������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param strModel : ����������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadRobotModel(unsigned int boxID, string &strModel);

/**
 *	@brief:������ʹ������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpEnable(unsigned int boxID);

/**
 *	@brief:������ȥʹ������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpDisable(unsigned int boxID);

/**
 *	@brief:�����˸�λ����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpReset(unsigned int boxID);

/**
 *	@brief:ֹͣ�˶�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpStop(unsigned int boxID);

/**
 *	@brief:��ͣ�˶�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpInterrupt(unsigned int boxID);

/**
 *	@brief:�����˶�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpContinue(unsigned int boxID);

/**
 *	@brief:������������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpOpenFreeDriver(unsigned int boxID);

/**
 *	@brief:�ر���������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_GrpCloseFreeDriver(unsigned int boxID);

//************************************************************************/
//**    �ű������ӿ�                                                             
//************************************************************************/
/**
 *	@brief:����ָ���ű�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param strFuncName : ָ���ű��������ƣ���Ӧʾ��������ĺ���
 *	@param param : ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_RunFunc(unsigned int boxID, string strFuncName, vector<string> param);

/**
 *	@brief:ִ�нű�Main���������ú�ִ��ʾ����ҳ�����õĽű��ļ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_StartScript(unsigned int boxID);

/**
 *	@brief:ֹͣ�ű������ú�ֹͣʾ����ҳ������ִ�нű��ļ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_StopScript(unsigned int boxID);

/**
 *	@brief:��ͣ�ű������ú���ͣʾ����ҳ������ִ�нű��ļ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PauseScript(unsigned int boxID);

/**
 *	@brief:�����ű������ú��������ʾ����ҳ��������ͣ�Ľű��ļ�����������ͣ״̬�򷵻�20018����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ContinueScript(unsigned int boxID);


//************************************************************************/
//**    ������ƺ����ӿ�                                                             
//************************************************************************/
/**
 *	@brief:��ȡ������Ϣ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nConnected : ��������״̬
 *	@param n48V_ON : 48V��ѹֵ
 *	@param d48OUT_Voltag : 48V�����ѹֵ
 *	@param d48OUT_Current : 48V�������ֵ
 *	@param nRemoteBTN : Զ�̼�ͣ״̬
 *	@param nThreeStageBTN : ���ΰ�ť״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxInfo(unsigned int boxID,
                    int &nConnected, 
                    int &n48V_ON, 
                    double &d48OUT_Voltag, 
                    double &d48OUT_Current, 
                    int &nRemoteBTN, 
                    int &nThreeStageBTN);

/**
 *	@brief:��ȡ���������������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ������������λ
 *	@param nVal : �������������Ӧλ״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxCI(unsigned int boxID, int nBit, int &nVal);

/**
 *	@brief:��ȡ����ͨ����������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ͨ����������λ
 *	@param nVal : ͨ�����������Ӧλ״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxDI(unsigned int boxID, int nBit, int &nVal);

/**
 *	@brief:��ȡ����������������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : �������������λ
 *	@param nVal : �������������Ӧλ״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxCO(unsigned int boxID, int nBit, int &nVal);

/**
 *	@brief:��ȡ����ͨ���������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ͨ���������λ
 *	@param nVal : ͨ�����������Ӧλ״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxDO(unsigned int boxID, int nBit, int &nVal);

/**
 *	@brief:��ȡ����ģ��������ֵ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ģ��������ͨ��
 *	@param dbVal : ����ģʽ:4-20mA,��ѹģʽ:0-10V
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxAI(unsigned int boxID, int nBit, double &dbVal);

/**
 *	@brief:��ȡ����ģ�������ģʽ��ֵ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ģ��������ͨ��
 *	@param nMode : ��Ӧλģ����ͨ��ģʽ
 *	@param dbVal : ����ģʽ:4-20mA,��ѹģʽ:0-10V
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadBoxAO(unsigned int boxID, int nBit, int &nMode, double &dbVal);

/**
 *	@brief:���ÿ����������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : �����������λ
 *	@param nVal : �������������Ŀ��״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetBoxCO(unsigned int boxID, int nBit, int nVal);

/**
 *	@brief:���õ���ͨ���������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ͨ���������λ
 *	@param nVal : ͨ���������Ŀ��״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetBoxDO(unsigned int boxID, int nBit, int nVal);

/**
 *	@brief:���õ���ģ�������ģʽ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ģ�������ģʽͨ��
 *	@param nMode : ģ�������ģʽĿ��״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetBoxAOMode(unsigned int boxID, int nBit, int nMode);

/**
 *	@brief:���õ���ģ�������ֵ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ģ�������ͨ��
 *	@param nMode : ģ�������Ŀ��ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetBoxAOVal(unsigned int boxID, int nBit, double dbVal);

/**
 *	@brief:��ȡĩ����������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ĩ��������������
 *	@param nVal : ĩ����������״ֵ̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadEndDI(unsigned int boxID, int &nBit, int &nVal);

/**
 *	@brief:��ȡĩ���������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ĩ�������������
 *	@param nVal : ĩ���������״ֵ̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadEndDO(unsigned int boxID, int &nBit, int &nVal);

/**
 *	@brief:����ĩ���������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ĩ�������������
 *	@param nVal : ĩ���������״ֵ̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetEndDO(unsigned int boxID, int nBit, int nVal);

/**
 *	@brief:��ȡĩ��ģ��������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nBit : ĩ��ģ������������
 *	@param nVal : ĩ��ģ��������״ֵ̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadEndAI(unsigned int boxID, int &nBit, double &dVal);

//////////////////////////////////////////////////////////////////////////////////////
/**
 *	@brief:�����ٶȱ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dbOverride : ��Ҫ���õ��ٶȱ�(0.01-1) 
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetOverride(unsigned int boxID, double dOverride);

/**
 *	@brief:������ر�Tool����ϵ�˶�ģʽ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState : 0(�ر�)/1(����)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetTCPMotion(unsigned int boxID, int nState);

//�ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
/**
 *	@brief:���õ�ǰ���ز���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param Mass������
 *	@param dX������X����ƫ��
 *	@param dY������Y����ƫ��
 *	@param dZ������Z����ƫ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetPayload(unsigned int boxID, double dMass, double dX, double dY, double dZ);
/**
 *	@brief:���ùؽ�����ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : J1-J6������ٶȣ���λ[��/ s]
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetJointMaxVel(unsigned int boxID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6);

/**
 *	@brief:���ùؽ������ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : J1-J6���������ٶȣ���λ[��/ s2]
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetJointMaxAcc(unsigned int boxID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6);

/**
 *	@brief:����ֱ���˶�����ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxVel : ֱ���˶�����ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetLinearMaxVel(unsigned int boxID, double dMaxVel);

/**
 *	@brief:����ֱ���˶������ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxAcc : ֱ���˶������ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetLinearMaxAcc(unsigned int boxID, double dMaxAcc);

/**
 *	@brief:�������ؽ��˶���Χ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxJ1-dMaxJ6 : ���ؽڽǶ�
 *	@param dMinJ1-dMinJ6 : ��С�ؽڽǶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetMaxAcsRange(unsigned int boxID,
                        double dMaxJ1, double dMaxJ2, double dMaxJ3, double dMaxJ4, double dMaxJ5, double dMaxJ6,
                        double dMinJ1, double dMinJ2, double dMinJ3, double dMinJ4, double dMinJ5, double dMinJ6);

/**
 *	@brief:���ÿռ�����˶���Χ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxX-dMaxZ : XYZ���Χ
 *	@param dMinX-dMinZ : XYZ��С��Χ
 *	@param dUcs_X-dUcs_Rz : �����û�����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetMaxPcsRange(unsigned int boxID,
                        double dMaxX, double dMaxY, double dMaxZ, double dMinX, double dMinY, double dMinZ,
                        double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz);

/**
 *	@brief:���ùؽ�����˶���Χ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : J1-J6������˶���Χ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadJointMaxVel(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡ�ؽ������ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : J1-J6�������ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadJointMaxAcc(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡ�ؽ����Ӽ��ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : J1-J6�����Ӽ��ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadJointMaxJerk(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡֱ���˶��ٶȲ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxVel : ֱ���˶��ٶ�
 *	@param dMaxAcc : ֱ���˶����ٶ�
 *	@param dMaxJerk : ֱ���˶��Ӽ��ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadLinearMaxSpeed(unsigned int boxID, double &dMaxVel, double &dMaxAcc, double &dMaxJerk);

//////////////////////////////////////////////////////////////////////////////////////
/**
 *	@brief:��ȡ��ͣ��Ϣ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nESTO_Error:��ͣ��·����·������·�źŲ���ͬʱ������Ϊ��ͣ��·�д��������Ϊ1
 *	@param nESTO����ͣ�źţ�������ͣʱ�����48V���������Ĺ��磬���ǲ����220V��48V�Ĺ���
 *	@param nSafetyGuard_Error:��ȫ��Ļ��·����·������·�źŲ���ͬʱ������Ϊ��ȫ��Ļ��·�д��������Ϊ1
 *	@param nSafetyGuard����ȫ��Ļ�źţ�������ȫ��Ļʱ����ֹͣ�������˶������Ҳ��ٽ����˶�ָ�����ϱ��幩��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadEmergencyInfo(unsigned int boxID,
                            int &nESTO_Error, 
                            int &nESTO, 
                            int &nSafetyGuard_Error, 
                            int &nSafetyGuard);

/**
 *	@brief:��ȡ��ǰ������״̬��־
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nMovingState : �˶�״̬
 *	@param nEnableState : ʹ��״̬
 *	@param nErrorState : ����״̬
 *	@param nErrorCode : ������
 *	@param nErrorAxis : ������ID
 *	@param nBreaking : ��բ�Ƿ��״̬
 *	@param nPause : ��ͣ״̬
 *	@param nEmergencyStop : ��ͣ״̬
 *	@param nSaftyGuard : ��ȫ��Ļ״̬
 *	@param nElectrify : �ϵ�״̬
 *	@param nIsConnectToBox : ���ӵ���״̬
 *	@param nBlendingDone : WayPoint�˶����״̬
 *	@param nInPos : �˶�����λ����ʵ��λ���Ƿ�λ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadRobotState(unsigned int robotId,
						int &nMovingState, 
						int &nEnableState, 
						int &nErrorState, 
						int &nErrorCode, 
						int &nErrorAxis,
						int &nBreaking, 
						int &nPause,
						int &nEmergencyStop, 
						int &nSaftyGuard, 
						int &nElectrify, 
						int &nIsConnectToBox,
						int &nBlendingDone, 
						int &nInpos);

/**
 *	@brief:��ȡ��ǰ������״̬��־
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nMovingState : �˶�״̬
 *	@param nEnableState : ʹ��״̬
 *	@param nErrorState : ����״̬
 *	@param nErrorCode : ������
 *	@param nErrorAxis : ������ID
 *	@param nBreaking : ��բ�Ƿ��״̬
 *	@param nPause : ��ͣ״̬
 *	@param nEmergencyStop : ��ͣ״̬
 *	@param nSaftyGuard : ��ȫ��Ļ״̬
 *	@param nElectrify : �ϵ�״̬
 *	@param nIsConnectToBox : ���ӵ���״̬
 *	@param nBlendingDone : WayPoint�˶����״̬
 *	@param nPos : �˶�����λ����ʵ��λ���Ƿ�λ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadRobotFlags(unsigned int robotId,
						int &nMovingState, 
						int &nEnableState, 
						int &nErrorState, 
						int &nErrorCode, 
						int &nErrorAxis,
						int &nBreaking, 
						int &nPause,
						int &nBlendingDone);
/**
 *	@brief:��ȡWayPoint��ǰ�˶�ID��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param strCurWaypointID : ��ǰID��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCurWaypointID(unsigned int boxID, string &strCurWaypointID);

/**
 *	@brief:��ȡ������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nErrorCode : ������
 *	@param Params : ÿ����(J1~J6)�Ĵ����룬���û�д�����Ϊ0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadAxisErrorCode(unsigned int boxID, int &nErrorCode, int &nJ1, int &nJ2, int &nJ3, int &nJ4, int &nJ5, int &nJ6);

/**
 *	@brief:��ȡ��ǰ״̬��״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nCurFSM : ��ǰ״̬��״̬�������������ӿ�˵���ĵ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCurFSM(unsigned int boxID, int &nCurFSM, string &strCurFSM);

/**
 *	@brief:��ȡ״̬��״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nCurFSM : ��ǰ״̬��״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadCurFSMFromCPS(unsigned int boxID, int &nCurFSM);

/**
 *	@brief:��ȡ��ǰλ����Ϣ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : �Ͽ�������
 *	@param dJ1-dJ6 : �ؽ�����
 *	@param dTcp_X-dTcp_Rz : TCP����
 *	@param dUcs_X-dUcs_Rz : �û�����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActPos(unsigned int boxID,
					 double &dX, double &dY, double &dZ, double &dRx, double &dRy, double &dRz,
					 double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6,
					 double &dTcp_X, double &dTcp_Y, double &dTcp_Z, double &dTcp_Rx, double &dTcp_Ry, double &dTcp_Rz,
                    double &dUcs_X, double &dUcs_Y, double &dUcs_Z, double &dUcs_Rx, double &dUcs_Ry, double &dUcs_Rz);

/**
 *	@brief:��ȡ����ؽ�λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ����ؽ�λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCmdJointPos(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡʵ�ʹؽ�λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ʵ�ʹؽ�λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActJointPos(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡ����TCPλ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ����TCPλ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCmdTcpPos(unsigned int boxID, double &dX, double &dY, double &dZ, double &dRx, double &dRy, double &dRz);

/**
 *	@brief:��ȡʵ��TCPλ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ʵ��TCPλ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActTcpPos(unsigned int boxID, double &dX, double &dY, double &dZ, double &dRx, double &dRy, double &dRz);

/**
 *	@brief:��ȡ����ؽ��ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ����ؽ��ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCmdJointVel(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡʵ�ʹؽ��ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ʵ�ʹؽ��ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActJointVel(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡ����TCP�ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ����TCP�ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCmdTcpVel(unsigned int boxID, double &dX, double &dY, double &dZ, double &dRx, double &dRy, double &dRz);

/**
 *	@brief:��ȡʵ��TCP�ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ʵ��TCP�ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActTcpVel(unsigned int boxID, double &dX, double &dY, double &dZ, double &dRx, double &dRy, double &dRz);
 
/**
 *	@brief:��ȡ����ؽڵ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ����ؽڵ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadCmdJointCur(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);
 
/**
 *	@brief:��ȡʵ�ʹؽڵ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1-dJ6 : ʵ�ʹؽڵ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadActJointCur(unsigned int boxID, double &dJ1, double &dJ2, double &dJ3, double &dJ4, double &dJ5, double &dJ6);

/**
 *	@brief:��ȡĩ��TCP�ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dCmdVel : �����ٶ�mm/s
 *	@param dActVel : ʵ���ٶ�mm/s
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadTcpVelocity(unsigned int boxID, double &dCmdVel, double &dActVel);

//************************************************************************/
//**    �����ຯ���ӿ�                                                             
//************************************************************************/
/**
 *	@brief:��Ԫ��תŷ���ǣ�ŷ���ǵĵ�λ�Ƕ�
 *	@param dQuaW : ��Ԫ��W
 *	@param dQuaX : ��Ԫ��X
 *	@param dQuaY : ��Ԫ��Y
 *	@param dQuaZ : ��Ԫ��Z
 *	@param dRx: ŷ����rx
 *	@param rRy: ŷ����ry
 *	@param dRz: ŷ����rz
 *	
 *	@return : �Ƿ�ɹ�
 */
HANSROBOT_PRO_API int HRIF_Quaternion2RPY(unsigned int boxID, double dQuaW, double dQuaX, double dQuaY, double dQuaZ, double &dRx, double &dRy, double &dRz);

/**
 *	@brief:ŷ����ת��Ԫ�أ�ŷ���ǵĵ�λ�Ƕ�
 *	@param dRx: ŷ����rx
 *	@param dRy: ŷ����ry
 *	@param dRz: ŷ����rz
 *	@param dQuaW : ��Ԫ��W
 *	@param dQuaX : ��Ԫ��X
 *	@param dQuaY : ��Ԫ��Y
 *	@param dQuaZ : ��Ԫ��Z
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_RPY2Quaternion(unsigned int boxID, double dRx, double dRy, double dRz, double &dQuaW, double &dQuaX, double &dQuaY, double &dQuaZ);
/**
 *	@brief:��⣬��ָ���û�����ϵλ�ú͹�������ϵ�µĵϿ�����������Ӧ�Ĺؽ�����λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dCoord : ָ���û�����ϵ�͹�������ϵ�µĵϿ�������λ��(��Ҫ���ĵϿ�������λ��)
 *	@param dTcp : PCS����Ӧ�Ĺ�������
 *	@param dUcs : PCS����Ӧ���û�����
 *	@param dJ1 : �ο��ؽ����꣬���ڶ��������ѡȡ�ӽ��Ĺؽ�����λ��
 *	@param dTargetJ1 : ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_GetInverseKin(unsigned int boxID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx, double dCoord_Ry, double dCoord_Rz,
                 double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz,
                 double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz,
                 double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                 double &dTargetJ1, double &dTargetJ2, double &dTargetJ3, double &dTargetJ4, double &dTargetJ5, double &dTargetJ6);

/**
 *	@brief:���⣬�ɹؽ�����λ�ü���ָ���û�����ϵ�͹�������ϵ�µĵϿ�������λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dJ1 : ��Ҫ����Ĺؽ�����λ��
 *	@param dTcp : Target����Ӧ�Ĺ�������
 *	@param dUcs : Target����Ӧ���û�����
 *	@param dTarget : ָ���û�����ϵ�͹�������ϵ�µĵϿ�������λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */				 
HANSROBOT_PRO_API int HRIF_GetForwardKin(unsigned int boxID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                 double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz,
                 double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz,
                 double &dTarget_X, double &dTarget_Y, double &dTarget_Z, double &dTarget_Rx, double &dTarget_Ry, double &dTarget_Rz);

/**
 *	@brief:�ɻ�������ϵ�µ�����λ�ü���ָ���û�����ϵ�͹�������ϵ�µĵϿ�������λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dCoord : ��������ϵ�µĵϿ�������λ��
 *	@param dTcp : dTarget����Ӧ�Ĺ�������
 *	@param dUcs : dTarget����Ӧ���û�����
 *	@param dTarget : ָ���û�����ϵ�͹�������ϵ�µĵϿ�������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */				
HANSROBOT_PRO_API int HRIF_Base2UcsTcp(unsigned int boxID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx, double dCoord_Ry, double dCoord_Rz,
                     double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz,
                     double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz,
                     double &dTarget_X, double &dTarget_Y, double &dTarget_Z, double &dTarget_Rx, double &dTarget_Ry, double &dTarget_Rz);
					 
/**
 *	@brief:��ָ���û�����ϵ�͹�������ϵ�µĵϿ�������λ�ü����������ϵ�µ�����λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dCoord : ָ���û�����ϵ�͹�������ϵ�µĵϿ�������
 *	@param dTcp : dCoord����Ӧ�Ĺ�������
 *	@param dUcs : dCoord����Ӧ���û�����
 *	@param dTarget : ��������ϵ�µĵϿ�������λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */		
HANSROBOT_PRO_API int HRIF_UcsTcp2Base(unsigned int boxID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx, double dCoord_Ry, double dCoord_Rz,
                     double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz,
                     double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz,
                     double &dTarget_X, double &dTarget_Y, double &dTarget_Z, double &dTarget_Rx, double &dTarget_Ry, double &dTarget_Rz);

/**
 *	@brief: ��λ�ӷ�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose2_X-dPose2_Rz : ����2
 *	@param dPose3_Y : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PoseAdd(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double dPose2_X,double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
                    double &dPose3_X,double &dPose3_Y,double &dPose3_Z,double &dPose3_Rx,double &dPose3_Ry,double &dPose3_Rz);

/**
 *	@brief: ��λ��������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose2_X-dPose2_Rz : ����2
 *	@param dPose3_Y : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	                  
HANSROBOT_PRO_API int HRIF_PoseSub(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double dPose2_X,double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
                    double &dPose3_X,double &dPose3_Y,double &dPose3_Z,double &dPose3_Rx,double &dPose3_Ry,double &dPose3_Rz);

/**
 *	@brief: ����任,�������HRIF_PoseTrans(p1,HRIF_PoseInverse(p2))���õ��ľ��ǻ�����ϵ�µ�p1,���û�����ϵp2�µ�λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose2_X-dPose2_Rz : ����2
 *	@param dPose3_Y : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	                       
HANSROBOT_PRO_API int HRIF_PoseTrans(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double dPose2_X,double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
                    double &dPose3_X,double &dPose3_Y,double &dPose3_Z,double &dPose3_Rx,double &dPose3_Ry,double &dPose3_Rz);

/**
 *	@brief: ������任,�������HRIF_PoseTrans(p1,HRIF_PoseInverse(p2))���õ��ľ��ǻ�����ϵ�µ�p1,���û�����ϵp2�µ�λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose3_Y : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	                       
HANSROBOT_PRO_API int HRIF_PoseInverse(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double &dPose3_X,double &dPose3_Y,double &dPose3_Z,double &dPose3_Rx,double &dPose3_Ry,double &dPose3_Rz);

/**
 *	@brief: �����λ����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose2_X-dPose2_Rz : ����2
 *	@param dDistance : ��λ���룬��λ[mm]
 *	@param dAngle : ��̬���룬��λ[��]
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	                       
HANSROBOT_PRO_API int HRIF_PoseDist(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double dPose2_X,double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
                    double &dDistance,double &dAngle);


/**
 *	@brief: �ռ�λ��ֱ�߲岹����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Rz : ����1
 *	@param dPose2_X-dPose2_Rz : ����2
 *	@param dAlpha : �岹����
 *	@param dPose3_Y : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	                      
HANSROBOT_PRO_API int HRIF_PoseInterpolate(unsigned int boxID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
                    double dPose2_X,double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,double dAlpha,
                    double &dPose3_X,double &dPose3_Y,double &dPose3_Z,double &dPose3_Rx,double &dPose3_Ry,double &dPose3_Rz);

/**
 *	@brief: �켣������ת���㣬p1,p2,p3Ϊ��תǰ�켣�������㣬p4,p5,p6Ϊ��ת��Ĺ켣��������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dPose1_X-dPose1_Z : ����1
 *	@param dPose2_X-dPose2_Z : ����2
 *	@param dPose3_X-dPose3_Z : ����1
 *	@param dPose4_X-dPose4_Z : ����2
 *	@param dPose5_X-dPose5_Z : ����1
 *	@param dPose6_X-dPose6_Z : ����2
 *	@param dUcs_X-dUcs_Z : ������
 *
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	  
HANSROBOT_PRO_API int HRIF_PoseDefdFrame(unsigned int boxID, double dPose1_X,double dPose1_Y,double dPose1_Z,double dPose2_X,double dPose2_Y,double dPose2_Z,
                    double dPose3_X,double dPose3_Y,double dPose3_Z,double dPose4_X,double dPose4_Y,double dPose4_Z,
                    double dPose5_X,double dPose5_Y,double dPose5_Z,double dPose6_X,double dPose6_Y,double dPose6_Z,
                    double &dUcs_X,double &dUcs_Y,double &dUcs_Z,double &dUcs_Rx,double &dUcs_Ry,double &dUcs_Rz);

									 
//////////////////////////////////////////////////////////////////////////////////////
/**
 *	@brief:���õ�ǰ��������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dTcp_x-dTcp_Rz : �����õĹ�������ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetTCP(unsigned int boxID, double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz);

/**
 *	@brief:���õ�ǰ�û�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dUcs_x-dUcs_Rz : �����õ��û�����ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetUCS(unsigned int boxID, double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz);

/**
 *	@brief:��ȡ��ǰ��������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dTcp_x-dTco_Rz : ��ȡ�Ĺ�������ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadCurTCP(unsigned int boxID, double& dTcp_X, double &dTcp_Y, double &dTcp_Z, double &dTcp_Rx, double &dTcp_Ry, double &dTcp_Rz);

/**
 *	@brief:��ȡ��ǰ�û�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dUcs_x-dUcs_Rz : ��ȡ���û�����ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadCurUCS(unsigned int boxID, double &dUcs_X, double &dUcs_Y, double &dUcs_Z, double &dUcs_Rx, double &dUcs_Ry, double &dUcs_Rz);

/**
 *	@brief:ͨ����������TCP�б��е�ֵΪ��ǰTCP����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTcpName : TCP��������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetTCPByName(unsigned int boxID, string sTcpName);

/**
 *	@brief:ͨ�����������û������б��е�ֵΪ��ǰ�û�����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sUcsName : UCS��������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetUCSByName(unsigned int boxID, string sUcsName);

/**
 *	@brief:ͨ�����ƶ�ȡָ��TCP����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sName : TCP���ƣ���ʾ����ҳ���TCP��Ӧ
 *	@param dX-dRz : ��ȡ���Ķ�ӦTCPֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadTCPByName(unsigned int boxID,
                        string sName, 
                        double& dTcp_X, 
                        double& dTcp_Y, 
                        double& dTcp_Z, 
                        double& dTcp_Rx, 
                        double& dTcp_Ry, 
                        double& dTcp_Rz);

/**
 *	@brief:ͨ�����ƶ�ȡָ��UCS����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sName : UCS���ƣ���ʾ����ҳ���UCS��Ӧ
 *	@param dX-dRz : ��ȡ���Ķ�ӦUCSֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadUCSByName(unsigned int boxID,
                        string sName, 
                        double& dUcs_X, 
                        double& dUcs_Y, 
                        double& dUcs_Z, 
                        double& dUcs_Rx, 
                        double& dUcs_Ry, 
                        double& dUcs_Rz);

/**
 *	@brief:������ʾ�̵�ָ�����Ƶ�TCP����ֵ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTcpName : ��ʾ�̵�ָ����TCP����
 *	@param x-rz : ��Ҫ�޸ĵ�TCPֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
//int HRIF_ConfigTCP(unsigned int boxID, string sTcpName,double dX, double dY, double dZ, double dRx, double dRy, double dRz);



HANSROBOT_PRO_API int HRIF_GetForceParams(unsigned int boxID, double *dInertia, double *dDamping, double *dStiffness);

/**
 *	@brief:�������ز���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dInertia : ��������(size=6)
 *	@param dDamping : �������(size=6)
 *	@param dStiffness : �նȲ���(size=6)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceParams(unsigned int boxID, double *dInertia, double *dDamping, double *dStiffness);

/**
 *	@brief:��������̽Ѱ����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dForceLimit : ̽Ѱ���������(size=6)
 *	@param dDistLimit : ̽Ѱ����������(size=6)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceControlParams(unsigned int boxID, double *dForceLimit, double *dDistLimit);

/**
 *	@brief:���ÿ������߹ر�����ģʽ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dEnbaleFlag : ����(TRUE)/�ر�(FALSE)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceMode(unsigned int boxID, bool dEnbaleFlag);


 
//////////////////////////////////////////////////////////////////////////////////////
/**
 *	@brief:��������״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState : 0:�ر������˶�/1:���������˶�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceControlState(unsigned int boxID, int nState);
/**
 *	@brief:��ȡ����״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState : ����״̬ 0:�ر�״̬,1:����̽Ѱ״̬,2:����̽Ѱ���״̬,3:������������״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadForceControlState(unsigned int boxID, int &nState);
/**
 *	@brief:����������������Ϊtool���귽��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState : 0(������������Ϊtool���귽��)/1(������������Ϊtool���귽��)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceToolCoordinateMotion(unsigned int boxID, int nMode);
/**
 *	@brief:��ͣ�����˶�������ͣ���ع��ܣ�����ͣ�˶��ͽű�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ForceControlInterrupt(unsigned int boxID);

/**
 *	@brief:���������˶��������������˶����ܣ��������˶��ͽű�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ForceControlContinue(unsigned int boxID);
/**
 *	@brief:������������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceZero(unsigned int boxID);
/**
 *	@brief:��������̽Ѱ������ٶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMaxLinearVelocity : ֱ��̽Ѱ����ٶ�
 *	@param dMaxAngularVelocity : ��̬̽Ѱ����ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetMaxSearchVelocities(unsigned int boxID, double dMaxLinearVelocity, double dMaxAngularVelocity);

/**
 *	@brief:��������̽Ѱ���ɶ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nX-nRz : �������������ɶ�״̬ 0���ر�  1������
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetControlFreedom(unsigned int boxID, int nX, int nY, int nZ, int nRx, int nRy, int nRz);

/**
 *	@brief:���ÿ��Ʋ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState : ���Ʋ��� 0����˳ģʽ 1������ģʽ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceControlStrategy(unsigned int boxID, int nState);
/**
 *	@brief:��������������������ڷ����̵İ�װλ�ú���̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ����������������ڷ����̵İ�װλ�ú���̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetFreeDrivePositionAndOrientation(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
/**
 *	@brief:��������̽ѰPID����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dFp-dTd : PID����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetPIDControlParams(unsigned int boxID, double dFp, double dFi, double dFd, double dTp, double dTi, double dTd);
/**
 *	@brief:���ù������Ʋ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : �������Ʋ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetMassParams(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
/**
 *	@brief:��������(b)���Ʋ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ����(b)���Ʋ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetDampParams(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
/**
 *	@brief:���øն�(k)���Ʋ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : �ն�(k)���Ʋ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetStiffParams(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
/**
 *	@brief:�������ؿ���Ŀ��ֵ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param X-Rz: ��ӦĿ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceControlGoal(unsigned int boxID, double dX, double dY, double dZ, double dRX, double dRY, double dRZ);
/**
 *	@brief:��������̽ѰĿ������С
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dWrench(dX-dRz) : ������̽ѰĿ����
 *	@param dDistance(dX-dRz) : ������̽Ѱ����(��δ���ã���ȫ������Ϊ0)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetControlGoal(unsigned int boxID, double dWrench_X, double dWrench_Y, double dWrench_Z, double dWrench_Rx, double dWrench_Ry, double dWrench_Rz,
                        double dDistance_X,double dDistance_Y, double dDistance_Z,double dDistance_Rx,double dDistance_Ry, double dDistance_Rz);

/**
 *	@brief:�����������Ʒ�Χ-�������������˷�Χ��������ϵ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dMax(dX-dRz) : �����򴫸����������ֵ
 *	@param dMin(dX-dRz) : �����򴫸���������Сֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceDataLimit(unsigned int boxID, double dMax_X, double dMax_Y, double dMax_Z, double dMax_Rx, double dMax_Ry, double dMax_Rz,
                            double dMin_X,double dMin_Y, double dMin_Z,double dMin_Rx,double dMin_Ry, double dMin_Rz);
/**
 *	@brief:���������α䷶Χ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dAllowDistance : �����̽Ѱ����
 *	@param dStrengthLevel : λ����߽�����ƫ�������ݴ�����2���ͱ�ʾ������ƫ��߽��ƽ����ɱ��������3���ͱ�ʾ������ƫ��߽��������ɱ���
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceDistanceLimit(unsigned int boxID, double dAllowDistance, double dStrengthLevel);

/**
 *	@brief:���ÿ������߹ر�������������ģʽ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nEnableFlag : 0(�ر�)/1(����)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetForceFreeDriveMode(unsigned int boxID, bool bEnable);

/**
 *	@brief:��ȡĩ����������ֵ(�궨��)
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : ���ȡ���Ķ�Ӧ��������ֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadFTCabData(unsigned int boxID, double &dX, double &dY, double &dZ, double &dRX, double &dRY, double &dRZ);



HANSROBOT_PRO_API bool HRIF_ReadTCPInCorrectValue();

//************************************************************************/
//**    �˶��ຯ���ӿ�                                                             
//************************************************************************/
/**
 *	@brief:�ؽڶ̵㶯 �˶�����2�㣬����ٶ�<10��/s
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nAxisId : ��Ҫ�˶��Ĺؽ�����0-5
 *	@param nDerection : ��Ҫ�˶��Ĺؽ��˶�����0(������)/1(������)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ShortJogJ(unsigned int boxID, int nAxisId, int nDerection);

/**
 *	@brief:�Ͽ�������̵㶯 �˶�����2mm������ٶ�<10mm/s
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nAxisId : ��Ҫ�˶��Ĺؽ�����0-5
 *	@param nDerection : ��Ҫ�˶��Ĺؽ��˶�����0(������)/1(������)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ShortJogL(unsigned int boxID, int nAxisId, int nDerection);

/**
 *	@brief:�ؽڳ��㶯
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nAxisId : ָ���˶��Ĺؽ�����0-5
 *	@param nDerection : ָ���˶��Ĺؽڷ���0(������)/1(������)
 *	@param nState : 0(�ر�)/1(����)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_LongJogJ(unsigned int boxID, int nAxisId, int nDerection, int nState);

/**
 *	@brief:�Ͽ������곤�㶯
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nAxisId : ָ���˶��ĵϿ�����������0-5
 *	@param nDerection : ָ���˶��ĵϿ������귽��0(������)/1(������)
 *	@param nState : 0(�ر�)/1(����)
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_LongJogL(unsigned int boxID, int nAxisId, int nDerection, int nState);

/**
 *	@brief:���㶯����ָ�����ʼ���㶯֮��Ҫ�� 500 ��������ʱ��Ϊʱ�����ڷ���һ�θ�ָ����򳤵㶯��ֹͣ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_LongMoveEvent(unsigned int boxID);

//////////////////////////////////////////////////////////////////////////////////////

/**
 *	@brief:�ж��˶��Ƿ�ֹͣ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param bDone : true: ��ɣ�false: δ���
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ����룬��ʱbDone��ȡֵ������
 */	
HANSROBOT_PRO_API int HRIF_IsMotionDone(unsigned int boxID, bool& bDone);

/**
 *	@brief:�ж�·���Ƿ��������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param bDone : true: ��ɣ�false: δ��ɣ�
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ����룬��ʱbDone��ȡֵ������
 */	
HANSROBOT_PRO_API int HRIF_IsBlendingDone(unsigned int boxID, bool& bDone);

/**
 *	@brief:ִ��·���˶�-����ֱ��ʹ��HRIF_MoveJ��HRIF_MoveL
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nMoveType : �˶�����
			enum EN_MoveType
			{
				enMoveType_MoveJ=0,
				enMoveType_MoveL,
			};
 *	@param dX-dRz : Ŀ��Ͽ���λ��-��nIsUseJoint=1ʱ��Ч��nIsUseJoint=0ʱ���ô˵Ͽ���������ΪĿ��λ�ã�ͨ��������õ��ؽ�����ΪĿ��ؽ�����
 *	@param dJ1-dJ6 : Ŀ��ؽ�λ��-��nIsUseJoint=1ʱ��ʹ�ô˹ؽ�������ΪĿ��ؽ����꣬nIsUseJoint=0ʱ���˹ؽ��������Ϊ�������ʱѡ��Ĳο��ؽ�����
 *	@param dTcp_X-dTcp_Rz : Ŀ��Ͽ������������Ĺ�������ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ����nIsUseJoint=1ʱ��Ч��������Ϊ0
 *	@param dUcs_X-dUcs_Rz : Ŀ��Ͽ��������������û�����ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ-��nIsUseJoint=1ʱ��Ч��������Ϊ0��
 *	@param dVelocity : �˶��ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dAcc : �˶����ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dRadius : �ǹ��ɰ뾶����λ����
 *	@param nIsJoint : �Ƿ�ʹ�ùؽڽǶ���ΪĿ��㣬���nMoveType����enMoveType_MoveJ����nisJoint��������
 *	@param nIsSeek,nIOBit,nIOState��̽Ѱ��������nisSeekΪ1������̽Ѱ����ʱ�����DO nIOBitλΪnIOStateʱ����ֹͣ�˶��������˶���Ŀ�����ֹͣ
 *	@param strCmdID����ǰ·��ID�������Զ��壬Ҳ���԰�˳������Ϊ��1������2������3��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_WayPointEx(unsigned int boxID, int nMoveType,
                    double dX, double dY, double dZ, double dRx, double dRy, double dRz,                        
                    double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,                      
                    double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz,
                    double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz,
                    double dVelocity, double dAcc, double dRadius, int nIsJoint, int nIsSeek, int nIOBit, int nIOState, string strCmdID);
/**
 *	@brief:ִ��·���˶�-����ֱ��ʹ��HRIF_MoveJ��HRIF_MoveL
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nMoveType : �˶�����
			enum EN_MoveType
			{
				enMoveType_MoveJ=0,
				enMoveType_MoveL,
			};
 *	@param dX-dRz : Ŀ��Ͽ���λ��-��nIsUseJoint=1ʱ��Ч��nIsUseJoint=0ʱ���ô˵Ͽ���������ΪĿ��λ�ã�ͨ��������õ��ؽ�����ΪĿ��ؽ�����
 *	@param dJ1-dJ6 : Ŀ��ؽ�λ��-��nIsUseJoint=1ʱ��ʹ�ô˹ؽ�������ΪĿ��ؽ����꣬nIsUseJoint=0ʱ���˹ؽ��������Ϊ�������ʱѡ��Ĳο��ؽ�����
 *	@param sTcpName : Ŀ��Ͽ������������Ĺ�������ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ����nIsUseJoint=1ʱ��Ч����ʹ��Ĭ�����ơ�TCP��
 *	@param sUcsName : Ŀ��Ͽ��������������û�����ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ-��nIsUseJoint=1ʱ��Ч����ʹ��Ĭ�����ơ�Base��
 *	@param dVelocity : �˶��ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dAcc : �˶����ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dRadius : �ǹ��ɰ뾶����λ����
 *	@param nIsUseJoint : �Ƿ�ʹ�ùؽڽǶ���ΪĿ��㣬���nMoveType����enMoveType_MoveJ����nisJoint��������
 *	@param nIsSeek,��nisSeekΪ1������̽Ѱ����ʱ�����nIOBit������DI��״̬ΪnIOState����ʱ����ֹͣ�˶��������˶���Ŀ�����ֹͣ�˶�
 *	@param strCmdID����ǰ·��ID�������Զ��壬Ҳ���԰�˳������Ϊ��1������2������3��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_WayPoint(unsigned int boxID, int nMoveType,
                double dX, double dY, double dZ, double dRx, double dRy, double dRz,                         
                double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                string sTcpName, string sUcsName, double dVelocity, double dAcc, double dRadius, 
                int nIsUseJoint,int nIsSeek, int nIOBit, int nIOState, string strCmdID);

/**
 *	@brief:ִ��·���˶�-����ֱ��ʹ��HRIF_MoveJ��HRIF_MoveL
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nMoveType : �˶�����,0:
 *	@param dEndPos_X-dEndPos_Rz : Ŀ��Ͽ���λ��-��nIsUseJoint=1ʱ��Ч��nIsUseJoint=0ʱ���ô˵Ͽ���������ΪĿ��λ�ã�ͨ��������õ��ؽ�����ΪĿ��ؽ�����
*	@param dAuxPos_X-dAuxPos_Rz :                                  
 *	@param dJ1-dJ6 : Ŀ��ؽ�λ��-��nIsUseJoint=1ʱ��ʹ�ô˹ؽ�������ΪĿ��ؽ����꣬nIsUseJoint=0ʱ���˹ؽ��������Ϊ�������ʱѡ��Ĳο��ؽ�����
 *	@param sTcpName : Ŀ��Ͽ������������Ĺ�������ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ����nIsUseJoint=1ʱ��Ч����ʹ��Ĭ�����ơ�TCP��
 *	@param sUcsName : Ŀ��Ͽ��������������û�����ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ-��nIsUseJoint=1ʱ��Ч����ʹ��Ĭ�����ơ�Base��
 *	@param dVelocity : �˶��ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dAcc : �˶����ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dRadius : �ǹ��ɰ뾶����λ����
 *	@param nIsUseJoint : �Ƿ�ʹ�ùؽڽǶ���ΪĿ��㣬���nMoveType����enMoveType_MoveJ����nisJoint��������
 *	@param nIsSeek,��nisSeekΪ1������̽Ѱ����ʱ�����nIOBit������DI��״̬ΪnIOState����ʱ����ֹͣ�˶��������˶���Ŀ�����ֹͣ�˶�
 *	@param strCmdID����ǰ·��ID�������Զ��壬Ҳ���԰�˳������Ϊ��1������2������3��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_WayPoint2(unsigned int boxID, int nMoveType,
                double dEndPos_X, double dEndPos_Y, double dEndPos_Z, double dEndPos_Rx, double dEndPos_Ry, double dEndPos_Rz,   
                double dAuxPos_X, double dAuxPos_Y, double dAuxPos_Z, double dAuxPos_Rx, double dAuxPos_Ry, double dAuxPos_Rz,                       
                double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                string sTcpName, string sUcsName, double dVelocity, double dAcc, double dRadius, 
                int nIsUseJoint,int nIsSeek, int nIOBit, int nIOState, string strCmdID);


/**
 *	@brief:�������˶���ָ���ĽǶ�����λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dX-dRz : Ŀ��Ͽ���λ��-��nIsUseJoint=1ʱ��Ч��nIsUseJoint=0ʱ���ô˵Ͽ���������ΪĿ��λ�ã�ͨ��������õ��ؽ�����ΪĿ��ؽ�����
 *	@param dJ1-dJ6 : Ŀ��ؽ�λ��-��nIsUseJoint=1ʱ��ʹ�ô˹ؽ�������ΪĿ��ؽ����꣬nIsUseJoint=0ʱ���˹ؽ��������Ϊ�������ʱѡ��Ĳο��ؽ�����
 *	@param sTcpName : Ŀ��Ͽ������������Ĺ�������ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ-��nIsUseJoint=1ʱ��Ч
 *	@param sUcsName : Ŀ��Ͽ��������������û�����ϵ���ƣ���ʾ����ҳ������ƶ�Ӧ-��nIsUseJoint=1ʱ��Ч
 *	@param dVelocity : �˶��ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dAcc : �˶����ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dRadius : �ǹ��ɰ뾶����λ����
 *	@param nIsUseJoint : �Ƿ�ʹ�ùؽڽǶ���ΪĿ��㣬��nIsUseJoint��������
 *	@param nIsSeek,nIOBit,nIOState��̽Ѱ��������nisSeekΪ1������̽Ѱ����ʱ�����DO nIOBitλΪnIOStateʱ����ֹͣ�˶��������˶���Ŀ�����ֹͣ
 *	@param strCmdID����ǰ·��ID�������Զ��壬Ҳ���԰�˳������Ϊ��1������2������3��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MoveJ(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz,
                double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                string sTcpName, string sUcsName, double dVelocity, double dAcc, double dRadius, 
                int nIsUseJoint,int nIsSeek, int nIOBit, int nIOState, string strCmdID);

/**
 *	@brief:������ֱ���˶���ָ���Ŀռ�����λ��
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
*	@param dX-dRz : Ŀ��Ͽ���λ��
 *	@param dJ1-dJ6 : �ؽ�λ��-����ʹ��Ŀ��Ͽ������긽���Ĺؽ�����ֵ��
 *	@param sTcpName : Ŀ��Ͽ������������Ĺ�������ϵ����
 *	@param sUcsName : Ŀ��Ͽ��������������û�����ϵ����
 *	@param dVelocity : �˶��ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dAcc : �˶����ٶȣ��ٶȵ�λ�Ǻ���ÿ�룬��ÿ�룬���ٶȺ���ÿ��ƽ������ÿ��ƽ��
 *	@param dRadius : �ǹ��ɰ뾶����λ����
 *	@param nIsSeek,nIOBit,nIOState��̽Ѱ��������nisSeekΪ1������̽Ѱ����ʱ�����DO nIOBitλΪnIOStateʱ����ֹͣ�˶��������˶���Ŀ�����ֹͣ
 *	@param strCmdID����ǰ·��ID�������Զ��壬Ҳ���԰�˳������Ϊ��1������2������3��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MoveL(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz,
                double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
                string sTcpName, string sUcsName, double dVelocity, double dAcc, double dRadius,
                int nIsSeek, int nIOBit, int nIOState, string strCmdID);

/**
 *	@brief: Բ���켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dStartPos : Բ����ʼλ��
 *	@param dAuxPos : Բ������λ��
 *	@param dEndPos : Բ������λ��
 *	@param nFixedPosure : 0:��ʹ�ù̶���̬��1:ʹ�ù̶���̬
 *	@param nMoveCType : 0:Բ���켣��1:��Բ�켣
 *	@param dRadLen : ��nMoveCType=0ʱ������Ч����������ȷ��Բ���켣
 *			         ��nMoveCType=1ʱ������Ϊ��Բ��Ȧ��,ͨ��������λȷ��Բ��·������ʹ����Բ�˶�ʱ��ʾ��Բ��Ȧ����С��������Ч��
 *	@param dVelocity : �ٶ�
 *	@param dAcc : ���ٶ�
 *	@param nRadius : ���ɰ뾶
 *	@param sTcpName : ������������
 *	@param sUcsName : �û���������
 *	@param sCmdID : ����ID
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MoveC(unsigned int boxID,
                double dStartPos_X,double dStartPos_Y,double dStartPos_Z,double dStartPos_Rx,double dStartPos_Ry,double dStartPos_Rz,
                double dAuxPos_X,double dAuxPos_Y,double dAuxPos_Z,double dAuxPos_Rx,double dAuxPos_Ry,double dAuxPos_Rz,
                double dEndPos_X,double dEndPos_Y,double dEndPos_Z,double dEndPos_Rx,double dEndPos_Ry,double dEndPos_Rz,
                int nFixedPosure,int nMoveCType,double dRadLen,double dVelocity, double dAcc, 
                double ddRadius, string sTcpName, string sUcsName, string sCmdID);

/**
 *	@brief: Z�͹켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dStartPos : ��ʼλ��
 *	@param dEndPos : ����λ��
 *	@param dPlanePos : ȷ��ƽ���λ��
 *	@param dVelocity : �ٶ�
 *	@param dAcc : ���ٶ�
 *	@param dWIdth : ���
 *	@param dDensity : �ܶ�
 *	@param nEnableDensity : �Ƿ�ʹ���ܶ�(0:��ʹ�ã�1:ʹ��)
 *	@param nEnablePlane : �Ƿ�ʹ��ƽ��㣬��ʹ��ʱ����ѡ����û�����ȷ��XYZƽ��
 *	@param nEnableWaiTime : �Ƿ���ת�۵�ȴ�ʱ��(0:��ʹ�ã�1:ʹ��)
 *	@param nPosiTime : ����ת�۵�ȴ�ʱ��ms
 *	@param nNegaTime : ����ת�۵�ȴ�ʱ��ms
 *	@param dRadius : ���ɰ뾶
 *	@param sTcpName : ������������
 *	@param sUcsName : �û���������
 *	@param sCmdID : ����ID
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MoveZ(unsigned int boxID,
                double dStartPos_X,double dStartPos_Y,double dStartPos_Z,double dStartPos_Rx,double dStartPos_Ry,double dStartPos_Rz,
                double dEndPos_X,double dEndPos_Y,double dEndPos_Z,double dEndPos_Rx,double dEndPos_Ry,double dEndPos_Rz,
                double dPlanePos_X,double dPlanePos_Y,double dPlanePos_Z,double dPlanePos_Rx,double dPlanePos_Ry,double dPlanePos_Rz,
                double dVelocity, double dAcc, double dWIdth, double dDensity, int nEnableDensity, int nEnablePlane, int nEnableWaiTime, 
                int nPosiTime, int nNegaTime, double dRadius, string sTcpName, string sUcsName, string sCmdID);

/**
 *	@brief: ��Բ�켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dP1 : ʾ�̵�1,ʾ�̵�λ���밴�չ켣˳������
 *	@param dP2 : ʾ�̵�2
 *	@param dP3 : ʾ�̵�3
 *	@param dP4 : ʾ�̵�4
 *	@param dP5 : ʾ�̵�5
 *	@param dP6 : ʾ�̵�6
 *	@param nOrientMode : ���˶����ͣ�0��Բ����1����Բ
 *	@param nMoveType : 0:��ʹ�ù̶���̬��1:ʹ�ù̶���̬
 *	@param dArcLength : ����
 *	@param dVelocity : �ٶ�
 *	@param dAcc : ���ٶ�
 *	@param dRadius : ���ɰ뾶
 *	@param sTcpName : ������������
 *	@param sUcsName : �û���������
 *	@param sCmdID : ����ID
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	

HANSROBOT_PRO_API int HRIF_MoveE(unsigned int boxID,
                double dP1_X, double dP1_Y, double dP1_Z, double dP1_Rx, double dP1_Ry, double dP1_Rz, 
                double dP2_X, double dP2_Y, double dP2_Z, double dP2_Rx, double dP2_Ry, double dP2_Rz, 
                double dP3_X, double dP3_Y, double dP3_Z, double dP3_Rx, double dP3_Ry, double dP3_Rz, 
                double dP4_X, double dP4_Y, double dP4_Z, double dP4_Rx, double dP4_Ry, double dP4_Rz, 
                double dP5_X, double dP5_Y, double dP5_Z, double dP5_Rx, double dP5_Ry, double dP5_Rz, 
                int nOrientMode, int nMoveType, double dArcLength,
                double dVelocity, double dAcc, double dRadius, string sTcpName, string sUcsName, string sCmdID);

/**
 *	@brief: �����˶��켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dSpiralIncrement : �����˶�ÿȦ�����뾶
 *	@param dSpiralDiameter : �����˶������뾶
 *	@param dVelocity : �ٶ�
 *	@param dAcc : ���ٶ�
 *	@param dRadius : ���ɰ뾶
 *	@param sUcsName : �û���������
 *	@param sCmdID : ����ID
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MoveS(unsigned int boxID, double dSpiralIncrement, double dSpiralDiameter,
                double dVelocity, double dAcc, double dRadius, string sUcsName, string sCmdID);

//////////////////////////////////////////////////////////////////////////////////////
/**
 *	@brief:��ʼ���ؽ������켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣����
 *	@param dSpeedRatio : �켣�˶��ٶȱ�
 *	@param dRadius : ���ɰ뾶,��λ[mm]
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_StartPushMovePathJ(unsigned int boxID, string sTrackName, double dSpeedRatio, double dRadius);

/**
 *	@brief:�·��˶��켣
			����HRIF_StartPushMovePath���ɶ�ε��ô˺�����һ������µ�λ������Ҫ>4
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣����
 *	@param J1-J6 : �ؽ�λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushMovePathJ(unsigned int boxID, string sTrackName, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6);

/**
 *	@brief:�켣�·���ɣ���ʼ����켣
			����HRIF_StartPushMovePath���ɶ�ε��ô˺�����һ������µ�λ������Ҫ>4
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_EndPushMovePathJ(unsigned int boxID, string sTrackName);

/**
 *	@brief:ִ��ָ���Ĺ켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MovePathJ(unsigned int boxID, string sTrackName);

/**
 *	@brief:��ȡ��ǰ�Ĺ켣״̬
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣����
 *	@param nResult : ��ǰ�켣״̬
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadMovePathJState(unsigned int boxID, string sTrackName, int &nResult);

/**
 *	@brief:����ָ���켣������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣ԭ����
 *	@param sTrackNewName : ��Ҫ���µĹ켣����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_UpdateMovePathJName(unsigned int boxID, string trackName, string sTrackNewName);

/**
 *	@brief:ɾ��ָ���켣
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : ��Ҫɾ���Ĺ켣����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_DelMovePathJ(unsigned int boxID, string sTrackName);

/**
 *	@brief:��ȡ�켣����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dProcess : ��ǰ�˶�����(0-1),>0.999999��ʾ�˶����
 *	@param nIndex : ��ǰ�켣�˶����ĵ�λ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadTrackProcess(unsigned int boxID, double &dProcess, int &nIndex);


/**
 *	@brief:��ʼ���Ͽ�������켣�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sTrackName : �켣���ƣ�Ŀǰ�Ͽ��������˶��Ĺ켣����û�����ÿ������ⶨ�壬
						ͬһ�켣������Ҫִͬ��HRIF_InitMovePathL��HRIF_PushMovePaths��HRIF_EndMovePathL
						����HRIF_EndMovePathL��������켣��ֱ�ӿ�ʼ�˶�������ʱ��2-4s���ң�����ʵ�ʹ켣��Сȷ��
 *	@param dVelocity : �˶��ٶ�
 *	@param dAcceleration : �˶����ٶ�
 *	@param jerk : �˶��Ӽ��ٶ�
 *	@param sUcsName : ָ���켣���ڵ��û�����ϵ����
 *	@param sTcpName : ָ���켣���ڵĹ�������ֵ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_InitMovePathL(unsigned int boxID, string sTrackName, double dVelocity, double dAceleration, double djerk, string sUcsName, string sTcpName);

/**
 *	@brief:�·��켣��λ-����һ���·�һ��Ŀ���λ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param trackName : �켣���ƣ�Ŀǰ�Ͽ��������˶��Ĺ켣����û�����ÿ������ⶨ�壬
						ͬһ�켣������Ҫִͬ��HRIF_InitMovePathL��HRIF_PushMovePaths��HRIF_EndMovePathL
						����HRIF_EndMovePathL��������켣��ֱ�ӿ�ʼ�˶�������ʱ��2-4s���ң�����ʵ�ʹ켣��Сȷ��
 *	@param dX-dRz : Ŀ��Ͽ�������λ�� 
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushMovePathL(unsigned int boxID, string sTrackName, double dX, double dY, double dZ, double dRX, double dRY, double dRZ);

/**
 *	@brief:�����·��켣��λ-����һ�ο��·������λ���ݣ���,�ָ�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param trackName : �켣���ƣ�Ŀǰ�Ͽ��������˶��Ĺ켣����û�����ÿ������ⶨ�壬
						ͬһ�켣������Ҫִͬ��HRIF_InitMovePathL��HRIF_PushMovePaths��HRIF_EndMovePathL
						����HRIF_EndMovePathL��������켣��ֱ�ӿ�ʼ�˶�������ʱ��2-4s���ң�����ʵ�ʹ켣��Сȷ��
 *	@param nMoveType : �˶�����-MovePathJ���Թ��� 0(MovePathJ)/1(MovePathL)
 *	@param nPointsSize : �켣��λ����
 *	@param sPoints : �켣��λ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushMovePaths(unsigned int boxID, string sTrackName, int nMoveType, int nPointsSize, string sPoints);

/**
 *	@brief:ִ�еϿ�������켣�˶���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param trackName : �켣���ƣ�Ŀǰ�Ͽ��������˶��Ĺ켣����û�����ÿ������ⶨ�壬
						ͬһ�켣������Ҫִͬ��HRIF_InitMovePathL��HRIF_PushMovePaths��HRIF_EndMovePathL
						����HRIF_EndMovePathL��������켣��ֱ�ӿ�ʼ�˶�������ʱ��2-4s���ң�����ʵ�ʹ켣��Сȷ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_MovePathL(unsigned int boxID, string sTrackName);


/**
 *	@brief:�������������߿��ƣ�servoJ �� servoP��ʱ���趨λ�ù̶����µ����ں�ǰհʱ��
 *  * ע��HRIF_StartServo/HRIF_PushServoJ/HRIF_PushServoPΪһ�׽ӿڣ�����servoָ��Ϊһ�׽ӿڣ������á�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dServoTime : �̶����µ����� s
 *	@param dLookaheadTime : ǰհʱ�� s
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_StartServo(unsigned int boxID, double dServoTime, double dLookaheadTime);

/**
 *	@brief:���߹ؽ�λ��������ƣ��� StartServo �趨�Ĺ̶�����ʱ�䷢�͹ؽ�λ�ã������˽�ʵʱ�ĸ��ٹؽ�λ��ָ��
 * * ע��HRIF_StartServo/HRIF_PushServoJ/HRIF_PushServoPΪһ�׽ӿڣ�����servoָ��Ϊһ�׽ӿڣ������á�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param J1-J6 : ���µĹؽ�λ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushServoJ(unsigned int boxID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6);

/**
 *	@brief:����ĩ��TCPλ��������ƣ��� StartServo �趨�Ĺ̶�����ʱ�䷢�� TCP λ�ã������˽�ʵʱ�ĸ���Ŀ�� TCP λ��������ת����Ĺؽ�λ��ָ��
*   * ע��HRIF_StartServo/HRIF_PushServoJ/HRIF_PushServoPΪһ�׽ӿڣ�����servoָ��Ϊһ�׽ӿڣ������á�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dCoord : ���µ�Ŀ��Ͽ�������λ��
 *	@param vecUcs : Ŀ��λ�ö�Ӧ��UCS
 *	@param vecTcp : Ŀ��λ�ö�Ӧ��TCP
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushServoP(unsigned int boxID, vector<double> &dCoord, vector<double> &vecUcs, vector<double> &vecTcp);

/**
 *	@brief:��ʼ�����߿���ģʽ����ջ����λ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_InitServoEsJ(unsigned int boxID);

/**
 *	@brief:�������߿���ģʽ����ʼ�˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dServoTime : �̶����µ����� s
 *	@param dLookaheadTime : ǰհʱ�� s
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_StartServoEsJ(unsigned int boxID, double dServoTime, double dLookheadTime);

/**
 *	@brief:�����·����߿��Ƶ�λ
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nPointsSize: ��λ��������������·�500����λ
 *	@param sPoints: ��λ��Ϣ�����磺"0,0,0,0,0,0,1,1,1,1,1,1,1"
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_PushServoEsJ(unsigned int boxID, int nPointsSize, string sPoints);

/**
 *	@brief:��ȡ��ǰ�Ƿ���Լ����·���λ��Ϣ��ѭ����ȡ���>20ms
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState: 0�������·���λ
 *			        1���������·���λ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_ReadServoEsJState(unsigned int boxID, int& nState);

/**
 *	@brief:������Ը����˶����Ʋ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState: 0:�ر���Ը��٣�1:������Ը���
 *	@param dDistance: ��Ը����˶����ֵ���Ծ���
 *	@param dAwayVelocity: ��Ը��ٵ��˶���Զ���ٶ�
 *	@param dGobackVelocity: ��Ը��ٵ��˶��ķ����ٶ�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetMoveTraceParams(unsigned int boxID, int nState, double dDistance, double dAwayVelocity, double dGobackVelocity);

/**
 *	@brief:������Ը����˶���ʼ������
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param dK,dB: ���㹫ʽy = dK * x + dB
 *	@param dMaxLimit: ���⴫�������������ֵ
 *	@param dMinLinit: ���⴫������������Сֵ
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetMoveTraceInitParams(unsigned int boxID, double dK, double dB, double dMaxLimit, double dMinLinit);

/**
 *	@brief:������Ը����˶��ĸ���̽Ѱ����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param X-Rx: ����̽Ѱ����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_SetMoveTraceUcs(unsigned int boxID, double dX, double dY, double dZ, double dRx, double dRy, double dRz);

/**
 *	@brief:���ô��ʹ������˶�
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nState: 0:�رմ��ʹ����٣�1:�������ʹ�����
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_SetTrackingState(unsigned int boxID, int nState);

/**
 *	@brief:ִ�в��app����
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param sCmdName: ��������
 *	@param sParams: �����б�
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_HRAppCmd(unsigned int boxID, string sCmdName, string sParams);

/**
 *	@brief:��ĩ�����ӵ�modbus��վ�Ĵ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nSlaveID: Modbus��վID
 *	@param nFunction: ������
 *	@param nRegAddr: ��ʼ��ַ
 *	@param nRegCount: �Ĵ������������11��
 *	@param vecData: ��Ҫд�ļĴ���ֵ��vector�Ĵ�С��Ĵ�������һ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */
HANSROBOT_PRO_API int HRIF_WriteEndHoldingRegisters(unsigned int boxID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, vector<int> vecData);

/**
 *	@brief:��ĩ�����ӵ�modbus��վ�Ĵ���
 *	@param boxID : ����ID�ţ�Ĭ��ֵ=0
 *	@param nSlaveID: Modbus��վID
 *	@param nFunction: ������
 *	@param nRegAddr: ��ʼ��ַ
 *	@param nRegCount: �Ĵ������������12��
 *	@param vecData: ���ض�ȡ�ļĴ���ֵ��vector�Ĵ�С��Ĵ�������һ��
 *	
 *	@return : nRet=0 : ���غ������óɹ�
 *            nRet>0 : ���ص���ʧ�ܵĴ�����
 */	
HANSROBOT_PRO_API int HRIF_ReadEndHoldingRegisters(unsigned int boxID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, vector<int>& vecData);
#ifdef __cplusplus
}
#endif
#endif // !_HR_PRO_H_