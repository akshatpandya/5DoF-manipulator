function [joint, path] = trajectory1(T5e, c)
    syms d_1 t2 t3 t4 t5;
    num_joints = 5;
    gripper_motor_id = 6;

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

    T05_FK = [sin(t2)*sin(t5)-cos(t2)*cos(t5)*sin(t3+t4) sin(t2)*cos(t5)+cos(t2)*sin(t5)*sin(t3+t4) cos(t3+t4)*cos(t2) cos(t2)*(d5*cos(t3+t4)+a3*cos(t3));
              -cos(t3+t4)*cos(t5) cos(t3+t4)*sin(t5) -sin(t3+t4) -d5*sin(t3+t4)-d2-a3*sin(t3);
              -cos(t2)*sin(t5)-sin(t2)*cos(t5)*sin(t3+t4) -cos(t2)*cos(t5)+sin(t2)*sin(t5)*sin(t3+t4) cos(t3+t4)*sin(t2) d_1+d5*sin(t2)*cos(t3+t4)+a3*cos(t3)*sin(t2);
              0 0 0 1];

    prev_joint = [0, 0, 0, 0, 0];

%     T05_rest = [-1 0 0 0;
%                0 0 -1 -d5-d2-a3;
%                0 -1 0 90;
%                0 0 0 1];
    T05_rest = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,0,pi/2,0,0]);
    T0e_rest = T05_rest*T5e;

    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];
    
    % Pick object 1
    move_gripper(gripper_motor_id, 0);
    T05_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,pi/2,pi/2,0,0]);
    disp(T05_im1);
    T0e_obj1_im1 = T05_im1*T5e;
    x(end+1) = T0e_obj1_im1(1,4);
    y(end+1) = T0e_obj1_im1(2,4);
    z(end+1) = T0e_obj1_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_obj1_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    T05_obj1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,pi/2,22/180*pi,-22/180*pi,0]);
    T0e_obj1 = T05_obj1*T5e;
    x(end+1) = T0e_obj1(1,4);
    y(end+1) = T0e_obj1(2,4);
    z(end+1) = T0e_obj1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_obj1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    disp(joint);
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);
    move_gripper(gripper_motor_id, 1);
% 
%     % Go back to resting position
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    T05__rest_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,pi/2,pi/2,0,0]);
    T0e_rest_im1 = T05__rest_im1*T5e;
    x(end+1) = T0e_rest_im1(1,4);
    y(end+1) = T0e_rest_im1(2,4);
    z(end+1) = T0e_rest_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);
% 
%     % Place object 1
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    t3_ = linspace(pi/2, 30/180*pi,3);
    t4_ = linspace(0, -30/180*pi,3);
    for i=1:3
        T05_dock = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,0,t3_(i),t4_(i),0]);
        T0e_dock = T05_dock*T5e;
        x(end+1) = T0e_dock(1,4);
        y(end+1) = T0e_dock(2,4);
        z(end+1) = T0e_dock(3,4);
        [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_dock,c,prev_joint);
        prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];
    end
    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);
    move_gripper(gripper_motor_id, 0);

    % Go back to resting position
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    T05__rest_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,0,pi/2,0,0]);
    T0e_rest_im1 = T05__rest_im1*T5e;
    x(end+1) = T0e_rest_im1(1,4);
    y(end+1) = T0e_rest_im1(2,4);
    z(end+1) = T0e_rest_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    T05_rest = subs(T05_FK, [d_1,t2,t3,t4,t5], [90,0,pi/2,0,0]);
    T0e_rest = T05_rest*T5e;
    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);

    % Pick object 2
    move_gripper(gripper_motor_id, 0);
    T05_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [150,pi/2,pi/2,0,0]);
    disp(T05_im1);
    T0e_obj1_im1 = T05_im1*T5e;
    x(end+1) = T0e_obj1_im1(1,4);
    y(end+1) = T0e_obj1_im1(2,4);
    z(end+1) = T0e_obj1_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_obj1_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    T05_obj1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [150,pi/2,45/180*pi,-45/180*pi,0]);
    T0e_obj1 = T05_obj1*T5e;
    x(end+1) = T0e_obj1(1,4);
    y(end+1) = T0e_obj1(2,4);
    z(end+1) = T0e_obj1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_obj1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    disp(joint);
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);
    move_gripper(gripper_motor_id, 1);

    % Go back to resting position
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    T05__rest_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [150,pi/2,pi/2,0,0]);
    T0e_rest_im1 = T05__rest_im1*T5e;
    x(end+1) = T0e_rest_im1(1,4);
    y(end+1) = T0e_rest_im1(2,4);
    z(end+1) = T0e_rest_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    T05_rest = subs(T05_FK, [d_1,t2,t3,t4,t5], [150,0,pi/2,0,0]);
    T0e_rest = T05_rest*T5e;
    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);

    % Place object 2
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    t3_ = linspace(pi/2, 30/180*pi,3);
    t4_ = linspace(0, -30/180*pi,3);
    for i=1:3
        T05_dock = subs(T05_FK, [d_1,t2,t3,t4,t5], [148,0,t3_(i),t4_(i),0]);
        T0e_dock = T05_dock*T5e;
        x(end+1) = T0e_dock(1,4);
        y(end+1) = T0e_dock(2,4);
        z(end+1) = T0e_dock(3,4);
        [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_dock,c,prev_joint);
        prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];
    end
    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);
    move_gripper(gripper_motor_id, 0);


    % Go back to resting position
    d1 = [];
    theta2 = [];
    theta3 = [];
    theta4 = [];
    theta5 = [];
    T05__rest_im1 = subs(T05_FK, [d_1,t2,t3,t4,t5], [148,0,pi/2,0,0]);
    T0e_rest_im1 = T05__rest_im1*T5e;
    x(end+1) = T0e_rest_im1(1,4);
    y(end+1) = T0e_rest_im1(2,4);
    z(end+1) = T0e_rest_im1(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest_im1,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    T05_rest = subs(T05_FK, [d_1,t2,t3,t4,t5], [148,0,pi/2,0,0]);
    T0e_rest = T05_rest*T5e;
    x(end+1) = T0e_rest(1,4);
    y(end+1) = T0e_rest(2,4);
    z(end+1) = T0e_rest(3,4);
    [d1(end+1),theta2(end+1),theta3(end+1),theta4(end+1),theta5(end+1)] = IK(T0e_rest,c,prev_joint);
    prev_joint = [d1(end),theta2(end),theta3(end),theta4(end),theta5(end)];

    joint = [d1;theta2;theta3;theta4;theta5];
    path = [x;y;z];

    motor_values = map_angle2motor(joint, num_joints);
    disp(motor_values);

    track_traj(motor_values, num_joints);

    move_gripper(gripper_motor_id, 1);
    move_gripper(gripper_motor_id, 0);
    move_gripper(gripper_motor_id, 1);
    move_gripper(gripper_motor_id, 0);


end