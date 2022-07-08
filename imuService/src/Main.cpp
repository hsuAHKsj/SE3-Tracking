
#include "string.h"
#include "Math.h"
#include "windows.h"
#include "Com.h"
#include "udp.h"

#include<stdio.h>
#include<conio.h>
#include "wit_c_sdk.h"

#include <mutex>

std::mutex mtx;

static char s_cDataUpdate = 0;
int iComPort = 4;
int iBaud = 9600;
int iAddress = 0x50;

void ComRxCallBack(char *p_data, UINT32 uiSize)
{
	for(UINT32 i = 0; i < uiSize; i++)
	{
		WitSerialDataIn(p_data[i]);
	}
}
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void DelayMs(uint16_t ms);
void main(void)
{	
	// ��ɴ���ͨѶ
	float a[3],w[3],Angle[3],h[3];

	// ����ͨѶ����
	//printf("�����봮�ں�:");
	//scanf_s("%d",&iComPort);
	OpenCOMDevice(iComPort,iBaud);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	AutoScanSensor();

	int iter = 0; 
	while (iter++< 10)
    {
        Sleep(500);

		mtx.lock();
		for (int i = 0;i<3;i++)
		{
			a[i] = (float)sReg[AX+i]/32768.0f*16.0f;
			w[i] = (float)sReg[GX+i]/32768.0f*2000.0f;
			Angle[i] = (float)sReg[Roll+i]/32768.0f*180.0f;
			h[i] = (float)sReg[HX+i];
		}
		mtx.unlock();

		printf("a:%.2f %.2f %.2f\r\n",a[0],a[1],a[2]);
		printf("w:%.2f %.2f %.2f\r\n",w[0],w[1],w[2]);
		printf("Angle:%.1f %.1f %.1f\r\n",Angle[0],Angle[1],Angle[2]);
		printf("h:%.0f %.0f %.0f\r\n\r\n",h[0],h[1],h[2]);
	}

	char send_buf[BUFSIZ];
	char recv_buf[BUFSIZ];
	//���������׽��֣����������׽���
	SOCKET s_server;
	SOCKET s_accept;

	initialization(); // ��ʼ�������׽���
	s_server = createServeSocket("127.0.0.1");
	cout << "wait client connect..." << endl;
	// ����пͻ�����������
	s_accept = accept(s_server, NULL, NULL);
	if (s_accept == INVALID_SOCKET) {
		cout << "����ʧ�ܣ�" << endl;
		WSACleanup();
		return;
	}

	// ���ԺͿͻ��˽���ͨ����
	while (true) {
		// recv��ָ����socket������Ϣ
		if (recv(s_accept, recv_buf, BUFSIZ, 0) > 0) {
			//cout << "Message form Client:" << endl;
			// �ж��������ݣ���������� ��׼���ݣ�������Ϣ
			if (recv_buf[0] == 'q')
			{
				cout << "Receive a 'q' Commamnd, Server quit!" << endl;
				break;
			}

			// Ӧ���ʱ�����ݴ��������ݻش�
			if (recv_buf[0] != 'q')
			{
				mtx.lock();
				for (int i = 0;i<3;i++)
				{
					a[i] = (float)sReg[AX+i]/32768.0f*16.0f;
					w[i] = (float)sReg[GX+i]/32768.0f*2000.0f;
					Angle[i] = (float)sReg[Roll+i]/32768.0f*180.0f;
					h[i] = (float)sReg[HX+i];
				}
				mtx.unlock();

				float sS[12];
				//sS[0] = 1.00;
				//sS[1] = 1.00;
				//sS[2] = 1.00;

				//sS[3] = 1.00;
				//sS[4] = 1.00;
				//sS[5] = 1.00;

				//sS[6] = 1.00;
				//sS[7] = 1.00;
				//sS[8] = 1.00;

				//sS[9] = 1.00;
				//sS[10] = 1.00;
				//sS[11] = 1.00;

				sS[0] = a[0];
				sS[1] = a[1];
				sS[2] = a[2];

				sS[3] = w[0];
				sS[4] = w[1];
				sS[5] = w[2];

				sS[6] = Angle[0];
				sS[7] = Angle[1];
				sS[8] = Angle[2];

				sS[9] =  h[0];
				sS[10] = h[1];
				sS[11] = h[2];
				send(s_accept, (char*)&sS, sizeof(sS), NULL);
				//cout << Angle[0] << ", " << Angle[1] << "," << Angle[2] << endl;
			}
		}
		else {

			cout << "Connection Fail.. Waiting for Client.." << endl;
			closesocket(s_server);
			closesocket(s_accept);
			initialization(); // ��ʼ�������׽���
			s_server = createServeSocket("127.0.0.1");
			s_accept = accept(s_server, NULL, NULL);
			Sleep(1000);
		}
	}
	//�ر��׽���
	closesocket(s_server);
	closesocket(s_accept);
	//�ͷ�DLL��Դ
	WSACleanup();
	return;
}

static void DelayMs(uint16_t ms)
{
	Sleep(ms);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	SendUARTMessageLength((const char*)p_data, uiSize);
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	s_cDataUpdate = 1;
}

static void AutoScanSensor(void)
{
	const uint32_t c_uiBaud[7] = {9600, 19200, 38400, 57600, 115200, 230400};
	int i, iRetry;
	
	for(i = 0; i < 7; i++)
	{
		SetBaundrate(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Sleep(100);
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

