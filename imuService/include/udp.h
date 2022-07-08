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

SOCKET createServeSocket(const char* ip)
{
	//1.�����յ�Socket					
	SOCKET s_server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s_server == INVALID_SOCKET)
	{
		cout << "�׽��ִ���ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ִ����ɹ���" << endl;
	}
	//2.��socket��ip��ַ�Ͷ˿ں�
	struct sockaddr_in server_addr;   // sockaddr_in, sockaddr  �ϰ汾���°������
	server_addr.sin_family = AF_INET;  // �ʹ���socketʱ����һ��
	server_addr.sin_port = htons(PORT);       // �˿ں�  ��ˣ���λ���洢(����)��С�ˣ���λ���洢(���磩�������洢˳���Ƿ��ŵ�  htons �������ֽ���תΪ�����ֽ���
	server_addr.sin_addr.S_un.S_addr = inet_addr(ip); //inet_addr�����ʮ���Ƶ�ip��ַתΪ������
	if (bind(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "�׽��ְ�ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ְ󶨳ɹ���" << endl;
	}

	//3.�����׽���Ϊ����״̬  SOMAXCONN �����Ķ˿��� �Ҽ�ת������Ϊ5
	if (listen(s_server, SOMAXCONN) < 0) {
		cout << "���ü���״̬ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "���ü���״̬�ɹ���" << endl;
	}
	return s_server;
}
