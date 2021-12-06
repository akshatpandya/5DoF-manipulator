% MAE 263A Project
% Simulation

clc;
clear;

% Main
% Parameter
a3 = 75; % mm
a5 = 25; %mm
d2 = 95; % mm
d5 = 84; %mm
de = 75; %mm

c = [a3 a5 d2 d5 de];
num_joints = 5;

% Trajectory Cartesian Space
N = 100;
t = linspace(0,pi,N);

T5e = [1 0 0 a5;
       0 1 0 0;
       0 0 1 de;
       0 0 0 1];

prev_joint = [0, 0, 0, 0, 0];
% 
% Tresting_pos = [-1 0 0 0;
%                0 0 -1 -d5-d2-a3;
%                0 -1 0 90;
%                0 0 0 1];
% T_rest = Tresting_pos*T5e;
% 
% theta = 0/180*pi;
% 
% x(1) = T_rest(1,4);
% y(1) = T_rest(2,4);
% z(1) = T_rest(3,4);
% [d1(1),theta2(1),theta3(1),theta4(1),theta5(1)] = IK(T_rest,c,prev_joint);
% prev_joint = [d1(1),theta2(1),theta3(1),theta4(1),theta5(1)];
% 
% x(2) = T0e(1,4);
% y(2) = T0e(2,4);
% z(2) = T0e(3,4);
% 
% path = [x;y;z];
% disp(path);
% [d1(2),theta2(2),theta3(2),theta4(2),theta5(2)] = IK(T0e,c,prev_joint);
% prev_joint = [d1(2),theta2(2),theta3(2),theta4(2),theta5(2)];

% Joint Space
% for i = 1:N
% %     T05 = [0 sin(t(i)) cos(t(i)) (a3+d5)*cos(t(i));
% %            -1 0 0 -d2;
% %            0 -cos(t(i)) sin(t(i)) 100+(a3+d5)*sin(t(i));
% %            0 0 0 1];
%     T05 = [-sin(t(i)) 0 cos(t(i)) (a3+d5)*cos(t(i));
%            -cos(t(i)) 0 -sin(t(i)) -(d5+a3)*sin(t(i))-d2;
%            0 -1 0 100;
%            0 0 0 1];
%     T0e = T05*T5e;
%     x(i) = T0e(1,4);
%     y(i) = T0e(2,4);
%     z(i) = T0e(3,4);
%     [d1(i),theta2(i),theta3(i),theta4(i),theta5(i)] = IK(T0e,c,prev_joint);
%     prev_joint = [d1(i),theta2(i),theta3(i),theta4(i),theta5(i)];
% end

[joint, path] = trajectory1(T5e,c);
% [joint, path] = dance1(T5e,c);

% joint = [d1;theta2;theta3;theta4;theta5];
% disp("joint");
% % disp(joint);
% path = [x;y;z];
% disp(path);

% motor_values = map_angle2motor(joint, num_joints);
% disp(motor_values);
% 
% track_traj(motor_values, num_joints);

% movie = 1; % create movie if 1
% speed = 1; % 1 to N
% 
% figure(1)
% for i = 1:1
%     animation(c,joint,path,movie,speed);
% end