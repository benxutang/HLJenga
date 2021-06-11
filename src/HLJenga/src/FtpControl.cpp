#include "FtpControl.h"

#include "Poco/Net/FTPClientSession.h"
#include "Poco/Net/DialogSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/NetException.h"
//#include <iostream>
#include <string>
#include <fstream>

using namespace std;

bool FtpControl::Upload(std::string ip, std::string workingDirectory, std::string srcfile, std::string dstfile)
{
    try
    {
        //连接FTP服务器
        Poco::Net::FTPClientSession session(ip); //创建FTP会话对象
        if (!session.isOpen())
        {
            return false;
        }

        //登录(默认账户名与密码)
        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        //设置工作空间及文件类型
        session.setWorkingDirectory(workingDirectory);
        session.setFileType(Poco::Net::FTPClientSession::FileType::TYPE_TEXT);

        //打开文件
        fstream fp(srcfile, ios::in);
        if (!fp)
        {
            session.close();
            return false;
        }

        //删除服务器上原有的文件
        std::istream& ftpin = session.beginList("");
        string list;
        vector<string> filelist;
        while (ftpin >> list)
        {
            if (list == dstfile)
            {
                //删除服务器本地文件
                session.remove(dstfile);
            }
        }
        session.endList();

        //上传文件(移除空行)
        ostream& os = session.beginUpload(dstfile); //指定远程文件名开始上传
        string line;
        bool isFirst = true;
        while (getline(fp, line))
        {
            if (line != "")
            {
                if (!isFirst)
                {
                    os << "\r\n"
                        << line;
                }
                else
                {
                    os << line;
                    isFirst = false;
                }
            }
        }
        session.endUpload();

        //关闭退出
        fp.close();
        session.close();

        return true;
    }
    catch (const Poco::Net::FTPException& e)
    {
        std::cout << e.message() << std::endl;
    }

    return false;
}

bool FtpControl::CheckServer(std::string ip)
{
    try
    {
        //连接FTP服务器
        Poco::Net::FTPClientSession session(ip); //创建FTP会话对象
        if (!session.isOpen())
        {
            return false;
        }

        //登录(默认账户名与密码)
        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        session.close();
        return true;
    }
    catch (const std::exception&)
    {
    }

    return false;
}
