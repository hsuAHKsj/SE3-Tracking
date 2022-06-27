// client.cpp
#include<iostream>
#include<winsock.h>   // windowsƽ̨�������ͷ�ļ�
#pragma comment(lib,"ws2_32.lib")   // ���ļ�
using namespace std;

#define PORT 7890
#define BUFSIZ 512

void initialization() {
	//��ʼ���׽��ֿ�
	// WSA  windows socket async  windows�첽�׽���     WSAStartup�����׽���
	// parm1:�����socket�汾 2.2 2.1 1.0     parm2:��������    ������ʽ��WORD  WSADATA
	WORD w_req = MAKEWORD(2, 2);//�汾��  
	WSADATA wsadata;
	// �ɹ���WSAStartup����������
	if (WSAStartup(w_req, &wsadata) != 0) {
		cout << "��ʼ���׽��ֿ�ʧ�ܣ�" << endl;
	}
	else {
		cout << "��ʼ���׽��ֿ�ɹ���" << endl;
	}
}

SOCKET createClientSocket(const char* ip)
{
	//1.�����յ�Socket					
		//parm1:af ��ַЭ���� ipv4 ipv6
		//parm2:type ����Э������ ��ʽ�׽���(SOCK_STREAM) ���ݱ�
		//parm3��protocl ʹ�þ����ĳ������Э��
	SOCKET c_client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (c_client == INVALID_SOCKET)
	{
		cout << "�׽��ִ���ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ִ����ɹ���" << endl;
	}

	//2.���ӷ�����
	struct sockaddr_in addr;   // sockaddr_in, sockaddr  �ϰ汾���°������
	addr.sin_family = AF_INET;  // �ʹ���socketʱ����һ��
	addr.sin_port = htons(PORT);       // �˿ں�  ��ˣ���λ���洢(����)��С�ˣ���λ���洢(���磩�������洢˳���Ƿ��ŵ�  htons �������ֽ���תΪ�����ֽ���
	addr.sin_addr.S_un.S_addr = inet_addr(ip); //inet_addr�����ʮ���Ƶ�ip��ַתΪ������

	if (connect(c_client, (struct sockaddr*)&addr, sizeof(addr)) == INVALID_SOCKET)
	{
		cout << "����������ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "���������ӳɹ���" << endl;
	}
	return c_client;
}

struct imuDataPack
{
	double a[3];
	double w[3];
	double angle[3];
};

class elfin_com
{
public:
	// ��ʼ�� s_server
	elfin_com()
	{
		initialization(); // ��ʼ�������׽���

		s_server = createClientSocket("192.168.44.1");
		send_buf = new char[BUFSIZ];
		recv_buf = new char[BUFSIZ];
	};

	~elfin_com()
	{
		delete[] send_buf;
		delete[] recv_buf;

		closesocket(s_server);
		//�ͷ�DLL��Դ
		WSACleanup();
	};

	bool push_elfin_command(imuDataPack& iData)
	{
		while (true)
		{
			send_buf[0] = 's';
			if (send(s_server, send_buf, BUFSIZ, 0) < 0) {
				cout << "Send Fail" << endl;
				return false;
			}
			else
			{
				// ��������
				float sS[12];

				//sS[0] = a[0];
				//sS[1] = a[1];
				//sS[2] = a[2];

				//sS[3] = w[0];
				//sS[4] = w[1];
				//sS[5] = w[2];

				//sS[5] = Angle[0];
				//sS[6] = Angle[1];
				//sS[8] = Angle[2];

				//sS[9] = h[0];
				//sS[10] = h[1];
				//sS[11] = h[2];
				recv(s_server, (char*)&sS, sizeof(sS), NULL);
				// ��ӡ����������Ϣ

				// �洢��Ϣ
				for (int i = 0; i < 3; i++)
				{
					iData.a[i] = sS[i];
					iData.w[i] = sS[i + 3];
					iData.angle[i] = sS[i + 6];
				}

				// ���в��, ��������Ϣ���ӽӿڷ���
				return true;
			}
		}
	}

	SOCKET s_server;
	char* send_buf;
	char* recv_buf;
};
