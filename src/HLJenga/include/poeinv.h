#pragma once
//#pragma once
#ifndef POEINV_H
#define POENIV_H
#include <Eigen/Dense>
using namespace Eigen;


/**
*
*逆运动学的三个子问题
*
*/
class poeinv
{
private:
    double mangle1;
    double mangle2;

public:
    poeinv(/* args */);

    double getsolu1();
    double getsolu2();
    void subproblem1(Vector3d omega, Vector3d p, Vector3d q, Vector3d r);
    void subproblem2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r);
    void subproblem2_2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r);
    void subproblem3(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double delta);


    ~poeinv();
};
#endif


