function [] = move_gripper(motor_id, grip)
    grip_value = 2025;
    ungrip_value = 1850;
    initialize();
    if grip == 1    % grip
        write4ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, MX28_GOAL_POSITION, grip_value);
        tic
        while 1
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, MX28_PRESENT_POSITION);
            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', motor_id, grip_value, typecast(uint32(dxl_present_position), 'int32'));
            time_elapsed = toc;
            if ~(abs(grip_value - typecast(uint32(dxl_present_position), 'int32')) > MOVING_STATUS_THRESHOLD) || time_elapsed > 1
                break;
            else
                continue;
            end
        end
    else    % ungrip
        write4ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, MX28_GOAL_POSITION, ungrip_value);
        tic
        while 1
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, MX28_PRESENT_POSITION);
            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', motor_id, ungrip_value, typecast(uint32(dxl_present_position), 'int32'));
            time_elapsed = toc;
            if ~(abs(ungrip_value - typecast(uint32(dxl_present_position), 'int32')) > MOVING_STATUS_THRESHOLD) || time_elapsed > 1
                break;
            else
                continue;
            end
        end
    end
end