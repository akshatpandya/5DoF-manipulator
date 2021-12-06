function [joint, path] = dance1(T5e, c)
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];

    a3 = c(1);
    a5 = c(2);
    d2 = c(3);
    d5 = c(4);
    de = c(5);

    x = [];
    y = [];
    z = [];

    prev_joint = [0, 0, 0, 0, 0];

    T05_rest = [-1 0 0 0;
               0 0 -1 -d5-d2-a3;
               0 -1 0 90;
               0 0 0 1];
    T0e_rest = T05_rest*T5e;

    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    N = 10;
    t2 = linspace(0,pi/2,N);
    t3 = linspace(0,pi/2,N);
    t4 = linspace(-pi/4,pi/4,N);

    for i = 1:N
%         T05 = [0 sin(t(i)) cos(t(i)) (a3+d5)*cos(t(i));
%                -1 0 0 -d2;
%                0 -cos(t(i)) sin(t(i)) 100+(a3+d5)*sin(t(i));
%                0 0 0 1];
%         T05*T5e
%         T05 = [-sin(t(i)) 0 cos(t(i)) (a3+d5)*cos(t(i));
%                -cos(t(i)) 0 -sin(t(i)) -(d5+a3)*sin(t(i))-d2;
%                0 -1 0 100;
%                0 0 0 1];
        T05 = [-cos(t2(i))*sin(t3(i)+t4(i)) sin(t2(i)) cos(t3(i)+t4(i))*cos(t2(i)) cos(t2(i))*(d5*cos(t3(i)+t4(i))+a3*cos(t3(i)));
               -cos(t3(i)+t4(i)) 0 -sin(t3(i)+t4(i)) -d5*sin(t3(i)+t4(i))-d2-a3*sin(t3(i));
               -sin(t2(i))*sin(t3(i)+t4(i)) -cos(t2(i)) cos(t3(i)+t4(i))*sin(t2(i)) 100+d5*sin(t2(i))*cos(t3(i)+t4(i))+a3*cos(t3(i))*sin(t2(i));
               0 0 0 1];
        T0e = T05*T5e;
        x(end+1) = T0e(1,4);
        y(end+1) = T0e(2,4);
        z(end+1) = T0e(3,4);
        [d1(i),theta2(i),theta3(i),theta4(i),theta5(i)] = IK(T0e,c,prev_joint);
        prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];
    end

    t2 = linspace(pi/2,pi,N);
    t3 = linspace(pi/2,0,N);
    t4 = linspace(pi/4,-pi/4,N);

    for i = 1:N
%         T05 = [0 sin(t(i)) cos(t(i)) (a3+d5)*cos(t(i));
%                -1 0 0 -d2;
%                0 -cos(t(i)) sin(t(i)) 100+(a3+d5)*sin(t(i));
%                0 0 0 1];
%         T05*T5e
%         T05 = [-sin(t(i)) 0 cos(t(i)) (a3+d5)*cos(t(i));
%                -cos(t(i)) 0 -sin(t(i)) -(d5+a3)*sin(t(i))-d2;
%                0 -1 0 100;
%                0 0 0 1];
        T05 = [-cos(t2(i))*sin(t3(i)+t4(i)) sin(t2(i)) cos(t3(i)+t4(i))*cos(t2(i)) cos(t2(i))*(d5*cos(t3(i)+t4(i))+a3*cos(t3(i)));
               -cos(t3(i)+t4(i)) 0 -sin(t3(i)+t4(i)) -d5*sin(t3(i)+t4(i))-d2-a3*sin(t3(i));
               -sin(t2(i))*sin(t3(i)+t4(i)) -cos(t2(i)) cos(t3(i)+t4(i))*sin(t2(i)) 100+d5*sin(t2(i))*cos(t3(i)+t4(i))+a3*cos(t3(i))*sin(t2(i));
               0 0 0 1];
        T0e = T05*T5e;
        x(end+1) = T0e(1,4);
        y(end+1) = T0e(2,4);
        z(end+1) = T0e(3,4);
        [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e,c,prev_joint);
        prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];
    end

    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];
end