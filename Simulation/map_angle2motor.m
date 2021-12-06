function [motor_values] = map_angle2motor(joints, num_joints)
    angle_motor_map = [40.95 11.377 11.377 11.377 11.377;
                       -2252.25 308 716 1536 204];
    motor_values = zeros(size(joints,2), num_joints);
    for i = 1:size(joints,2)
        for j = 1:num_joints
            % Convert -pi to pi to 0 to 2pi
            if j ~= 1
                if joints(j,i)<0
                    angle_ = joints(j,i)+2*pi;
                else
                    angle_ = joints(j,i);
                end

                % Convert angle to degree
                angle_ = angle_*180/pi;
                % Find motor value
                motor_value = angle_motor_map(1,j)*angle_ + angle_motor_map(2,j);
                if motor_value > 4095
                    motor_value = motor_value - 4095;
                end
                motor_value = typecast(int32(motor_value), 'uint32');
                motor_values(i,j) = motor_value;
            else
                % Find motor value
                motor_value = angle_motor_map(1,j)*joints(j,i) + angle_motor_map(2,j);
                if motor_value > 4095
                    motor_value = motor_value - 4095;
                end
                motor_value = typecast(int32(motor_value), 'uint32');
                motor_values(i,j) = motor_value;
            end
        end
    end
end