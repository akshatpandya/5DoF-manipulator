function [] = track_traj(joints, num_joints)
    % Initialize motors
    initialize();
%     disp(joints);
  
    for i=1:size(joints,1)
        for j=1:num_joints
            write4ByteTxRx(port_num, PROTOCOL_VERSION, j, MX28_GOAL_POSITION, joints(i,j));
        end
        tic
        while 1
            motors_track = zeros(1,num_joints);
            % Read present position
            for k=1:num_joints
                dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, k, MX28_PRESENT_POSITION);
%                 fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', MX28_ID(1), joints(i,k), typecast(uint32(dxl_present_position), 'int32'));
                if ~(abs(joints(i,k) - typecast(uint32(dxl_present_position), 'int32')) > MOVING_STATUS_THRESHOLD)
                    motors_track(k) = 1;
                    continue;
                end
            end
            time_elapsed = toc;
%             disp(sum(motors_track));
            if sum(motors_track) == num_joints || time_elapsed > 1
                pause(0.5);
                break;
            end
        end
    end
end