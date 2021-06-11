#include "poeinv.h"
#include <cmath>
#include <Eigen/Dense>
#define PI 3.1415926

using namespace Eigen;
using namespace std;

poeinv::poeinv(/* args */)
{
    mangle1 = 0;
    mangle2 = 0;
}

double poeinv::getsolu1()
{
    return mangle1;
}

double poeinv::getsolu2()
{
    return mangle2;
}

//求解subproblem1
void poeinv::subproblem1(Vector3d omega, Vector3d p, Vector3d q, Vector3d r)
{
    Vector3d u(0, 0, 0);
    Vector3d v(0, 0, 0);
    Vector3d u_dot(0, 0, 0);
    Vector3d v_dot(0, 0, 0);
    Matrix3d I;
    I.setIdentity();

    u = p - r;
    v = q - r;
    u_dot = (I - omega * omega.transpose()) * u;
    v_dot = (I - omega * omega.transpose()) * v;
    double tan1 = (omega.dot(u_dot.cross(v_dot))) / u_dot.dot(v_dot);
    //得到一个解
    mangle1 = atan2(omega.dot(u_dot.cross(v_dot)), u_dot.dot(v_dot));
}

//求解subproblem2
void poeinv::subproblem2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r)
{
    Vector3d u(0, 0, 0);
    Vector3d v(0, 0, 0);
    Vector3d z(0, 0, 0);
    Vector3d c(0, 0, 0);
    u = p - r;
    v = q - r;
    double alpha, beta, gamma;
    alpha = ((omega1.dot(omega2)) * omega2.dot(u) - omega1.dot(v)) / (omega1.dot(omega2) * omega1.dot(omega2) - 1);
    beta = ((omega1.dot(omega2)) * omega2.dot(v) - omega2.dot(u)) / (omega1.dot(omega2) * omega1.dot(omega2) - 1);
    gamma = sqrt((u.norm() * u.norm() - alpha * alpha - beta * beta - 2 * alpha * beta * omega1.dot(omega2)) / pow((omega1.cross(omega2)).norm(), 2));
    z = alpha * omega1 + beta * omega2 + gamma * (omega1.cross(omega2));
    c = r + z;
    poeinv angle1;
    poeinv angle2;
    angle1.subproblem1(omega1, c, q, r);
    angle2.subproblem1(omega2, p, c, r);
    mangle1 = angle1.getsolu1();
    mangle2 = angle2.getsolu1();

}

//求解subproblem2-2
void poeinv::subproblem2_2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r)
{
    Vector3d u(0, 0, 0);
    Vector3d v(0, 0, 0);
    Vector3d z(0, 0, 0);
    Vector3d c(0, 0, 0);
    u = p - r;
    v = q - r;
    double alpha, beta, gamma;
    alpha = ((omega1.dot(omega2)) * omega2.dot(u) - omega1.dot(v)) / (omega1.dot(omega2) * omega1.dot(omega2) - 1);
    beta = ((omega1.dot(omega2)) * omega2.dot(v) - omega2.dot(u)) / (omega1.dot(omega2) * omega1.dot(omega2) - 1);
    gamma = -sqrt((u.norm() * u.norm() - alpha * alpha - beta * beta - 2 * alpha * beta * omega1.dot(omega2)) / pow((omega1.cross(omega2)).norm(), 2));
    z = alpha * omega1 + beta * omega2 + gamma * (omega1.cross(omega2));
    c = r + z;
    poeinv angle1;
    poeinv angle2;
    angle1.subproblem1(omega1, c, q, r);
    angle2.subproblem1(omega2, p, c, r);
    mangle1 = angle1.getsolu1();
    mangle2 = angle2.getsolu1();

}


//求解subproblem3
void poeinv::subproblem3(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double delta)
{
    double theta0 = 0;
    Vector3d u(0, 0, 0);
    Vector3d v(0, 0, 0);
    Vector3d u_dot(0, 0, 0);
    Vector3d v_dot(0, 0, 0);
    Matrix3d I;
    I.setIdentity();
    double delta_dot_2;
    delta_dot_2 = delta * delta - ((omega.dot(p - q)) * (omega.dot(p - q)));

    u = p - r;
    v = q - r;
    u_dot = (I - omega * omega.transpose()) * u;
    v_dot = (I - omega * omega.transpose()) * v;
    theta0 = atan2((omega.dot((u_dot.cross(v_dot)))), u_dot.dot(v_dot));
    double cos1 = (u_dot.norm() * u_dot.norm() + v_dot.norm() * v_dot.norm() - delta_dot_2) / (2 * u_dot.norm() * v_dot.norm());
    //得到两个解
    mangle1 = (theta0 - acos(cos1));
    mangle2 = (theta0 + acos(cos1));
}

poeinv::~poeinv()
{
}