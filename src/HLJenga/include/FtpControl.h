#pragma once
#include <iostream>
class FtpControl
{
public:

    static bool Upload(std::string ip, std::string workingDirectory, std::string srcfile, std::string dstfile);

    static bool CheckServer(std::string ip);
};

