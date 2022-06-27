// client.cpp
#include<iostream>
#include<winsock.h>   // windowsƽ̨�������ͷ�ļ�
#pragma comment(lib,"ws2_32.lib")   // ���ļ�
using namespace std;

#define PORT 5050
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

int main() {
	//���巢�ͻ������ͽ��ܻ���������
	char send_buf[BUFSIZ];
	char recv_buf[BUFSIZ];
	//����ͻ����׽��֣����������׽���
	SOCKET s_server;
	initialization(); // ��ʼ�������׽���
	s_server = createClientSocket("127.0.0.1");

	//����,��������
	while (true) {
		cout << "�����뷢����Ϣ:";
		cin >> send_buf;
		if (send(s_server, send_buf, BUFSIZ, 0) < 0) {
			cout << "����ʧ�ܣ�" << endl;
			break;
		}
	}
	//�ر��׽���
	closesocket(s_server);
	//�ͷ�DLL��Դ
	WSACleanup();
	return 0;
}
