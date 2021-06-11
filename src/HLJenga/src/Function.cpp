/****************************************************************************
 *
 *   Copyright (C) 2021 Benxu Tang, Yueqian Liu. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 ****************************************************************************/

 /**
  * @file Function.cpp
  * For Windows platform, Vistual Studio 2019
  *
  * @author Benxu Tang, Yueqian Liu
  */


#include <winsock.h>
#include <conio.h>
#pragma comment(lib, "ws2_32.lib")
using namespace std;
#include "Function.h"
#include "RectDetect.h"
#include <iostream>
#include "MotionPlan.h"
#include "FtpControl.h"
#include <ctime>
#include <cstdlib>

//#define DEBUGMACRO

//定义长度变量
int send_len = 0;
int recv_len = 0;
//定义发送缓冲区和接受缓冲区
char send_buf[100] = {};
char recv_buf[200] = {};
string recvstr;
//定义服务端套接字，接受请求套接字
SOCKET s_server;
//服务端地址客户端地址
SOCKADDR_IN server_addr;

PosStruct cfgToPos(pointCCSCfg cfg)
{
	PosStruct pos;
	pos.x = cfg.x;
	pos.y = cfg.y;
	pos.z = cfg.z;
	pos.yaw = cfg.theta1;
	pos.pitch = cfg.theta2;
	pos.roll = cfg.theta3;
	return pos;
}

void RobotConnect()
{

	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR)
	{
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else
	{
		cout << "服务器连接成功！" << endl;
	}
}

void initialization()
{
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2); //版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0)
	{
		cout << "初始化套接字库失败！" << endl;
	}
	else
	{
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2)
	{
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else
	{
		cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息
}

void close()
{
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
}

void sysLogin()
{
	send_len = send(s_server, "[1# System.Login 0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void powerUp()
{
	send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void sysAbort()
{
	send_len = send(s_server, "[3# System.Abort 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void sysStart()
{
	send_len = send(s_server, "[4# System.Start]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void enterAutoMode()
{
	send_len = send(s_server, "[5# System.Auto 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "\t\t[AutoMode]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}


void setSpeed(string speed)
{
	string temp = "[6# System.Speed ";
	temp += (speed + ']');
	send_len = send(s_server, temp.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void home()
{
	send_len = send(s_server, "[7# Robot.Home 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void moveToJointConfig(pointJointCfg cfg)
{
	string configName = "CFG";
	srand(time(0));
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		switch (rand() % 3)
		{
		case 1:
			configName += 'A' + rand() % 26;
			break;
		case 2:
			configName += 'a' + rand() % 26;
			break;
		case 3:
			configName += '0' + rand() % 10;
			break;
		default:
			break;
		}
	}
	string config = "[8# LocationJ ";
	config += (configName + "=");
	config += (to_string(cfg.j1) + ',');
	config += (to_string(cfg.j2) + ',');
	config += (to_string(cfg.j3) + ',');
	config += (to_string(cfg.j4) + ',');
	config += (to_string(cfg.j5) + ',');
	config += (to_string(cfg.j6) + ']');
	string cmd = "[9# Move.Joint " + configName + ']';
	cout << config << endl;
	cout << cmd << endl;
#ifndef DEBUGMACRO
	send_len = send(s_server, config.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

	Sleep(100);
	send_len = send(s_server, cmd.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

#else
	cout << cfg.j1 << ',' << cfg.j2 << ',' << cfg.j3 << ',' << cfg.j4 << ',' << cfg.j5 << ',' << cfg.j6 << endl;
	cout << config << endl;
	cout << cmd << endl;
#endif
}

void moveToCCSConfig(pointCCSCfg cfg)
{
	string configName = "CFG1";
	srand(time(0));
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		switch (rand() % 3)
		{
		case 1:
			configName += 'A' + rand() % 26;
			break;
		case 2:
			configName += 'a' + rand() % 26;
			break;
		case 3:
			configName += '0' + rand() % 10;
			break;
		default:
			break;
		}
	}
	string config = "[10# Location ";
	config += (configName + "=");
	config += (to_string(cfg.x) + ',');
	config += (to_string(cfg.y) + ',');
	config += (to_string(cfg.z) + ',');
	config += (to_string(cfg.theta1) + ',');
	config += (to_string(cfg.theta2) + ',');
	config += (to_string(cfg.theta3) + ']');
	string cmd = "[11# Move.Line " + configName + ']';
#ifndef DEBUGMACRO
	send_len = send(s_server, config.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);

	send_len = send(s_server, cmd.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

#else
	cout << config << endl;
	cout << cmd << endl;
#endif
}

void enablePPB()
{
	send_len = send(s_server, "[12# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[PPB_Enable]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void disablePPB()
{
	send_len = send(s_server, "[13# PPB.Enable 1,0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[PPB_disable]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void setJointFrame()
{
	send_len = send(s_server, "[14# Robot.Frame 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Frame1]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void setWorldFrame()
{
	send_len = send(s_server, "[15# Robot.Frame 1,2]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Frame2]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void ppbToStartPoint()
{
	send_len = send(s_server, "[16# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void ppbLoadFile(string path)
{
	string cmd = "[17# PPB.ReadFile 1," + path + ']';
	cout << cmd << endl;
	send_len = send(s_server, cmd.data(), 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[LoadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

}

void ppbRun()
{
	send_len = send(s_server, "[18# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[PPBRun]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void printCCSConfig()
{
	send_len = send(s_server, "[19# Robot.Where 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[CCS]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void printJointConfig()
{
	send_len = send(s_server, "[20# Robot.WhereAngle 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Joint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void ppbStop()
{
	send_len = send(s_server, "[21# PPB.Stop 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[PPBStop]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void vacuumStart()
{
	send_len = send(s_server, "[22# IO.Set DOUT(20104),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void vacuumStop()
{
	send_len = send(s_server, "[23# IO.Set DOUT(20104),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void pressureStart()
{
	send_len = send(s_server, "[24# IO.Set DOUT(20103),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}

void pressureStop()
{
	send_len = send(s_server, "[25# IO.Set DOUT(20103),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
}