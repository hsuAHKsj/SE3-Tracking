// client.cpp
#include<iostream>
#include<winsock.h>   // windows平台的网络库头文件
#pragma comment(lib,"ws2_32.lib")   // 库文件
using namespace std;

#define PORT 5050
#define BUFSIZ 512

void initialization() {
	//初始化套接字库
	// WSA  windows socket async  windows异步套接字     WSAStartup启动套接字
	// parm1:请求的socket版本 2.2 2.1 1.0     parm2:传出参数    参数形式：WORD  WSADATA
	WORD w_req = MAKEWORD(2, 2);//版本号  
	WSADATA wsadata;
	// 成功：WSAStartup函数返回零
	if (WSAStartup(w_req, &wsadata) != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
}

SOCKET createClientSocket(const char* ip)
{
	//1.创建空的Socket					
		//parm1:af 地址协议族 ipv4 ipv6
		//parm2:type 传输协议类型 流式套接字(SOCK_STREAM) 数据报
		//parm3：protocl 使用具体的某个传输协议
	SOCKET c_client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (c_client == INVALID_SOCKET)
	{
		cout << "套接字创建失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字创建成功！" << endl;
	}

	//2.连接服务器
	struct sockaddr_in addr;   // sockaddr_in, sockaddr  老版本和新版的区别
	addr.sin_family = AF_INET;  // 和创建socket时必须一样
	addr.sin_port = htons(PORT);       // 端口号  大端（高位）存储(本地)和小端（低位）存储(网络），两个存储顺序是反着的  htons 将本地字节序转为网络字节序
	addr.sin_addr.S_un.S_addr = inet_addr(ip); //inet_addr将点分十进制的ip地址转为二进制

	if (connect(c_client, (struct sockaddr*)&addr, sizeof(addr)) == INVALID_SOCKET)
	{
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}
	return c_client;
}

int main() {
	//定义发送缓冲区和接受缓冲区长度
	char send_buf[BUFSIZ];
	char recv_buf[BUFSIZ];
	//定义客户端套接字，接受请求套接字
	SOCKET s_server;
	initialization(); // 初始化启动套接字
	s_server = createClientSocket("127.0.0.1");

	//发送,接收数据
	while (true) {
		cout << "请输入发送信息:";
		cin >> send_buf;
		if (send(s_server, send_buf, BUFSIZ, 0) < 0) {
			cout << "发送失败！" << endl;
			break;
		}
	}
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
	return 0;
}
