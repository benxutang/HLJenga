clc
clear
format long

% HECalib result
R = [-0.635577496393785, 0.7720030958578598, 0.007243346166531329, -0.1069818200305933;
    -0.7719826680497758, -0.6356161167748569, 0.005908665399324461, 0.07838359836716292;
    0.00916549554349339, -0.001836322937715745, 0.999956309850341, 0.01769080453963107;
     0, 0, 0, 1]
                     
% end effector x y z theta_z theta_y theta_z
p = [400.000 -0.000 500.000 0.000 180.000 39.000]

% calculate g
theta1 = deg2rad(0);
R_Z_1 = [  cos(theta1), -sin(theta1),           0;
           sin(theta1),  cos(theta1),           0;
                     0,            0,           1;];
                 
theta2 = deg2rad(180);       
R_Y_2 = [  cos(theta2),           0,  sin(theta2);
                     0,           1,            0;
          -sin(theta2),           0,  cos(theta2);];
           
theta3 = deg2rad(40);       
R_Z_3 = [  cos(theta3), -sin(theta3),           0;
           sin(theta3),  cos(theta3),           0;
                     0,            0,           1;];

R_ = R_Z_1 * R_Y_2 * R_Z_3;
g = [         R_, [p(1)/1000; p(2)/1000; p(3)/1000];
     zeros(1, 3),                                 1;]

% T
T = g * R