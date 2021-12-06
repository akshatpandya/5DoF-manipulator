% MAE C163A/C263A Project
% Team X
clc;
clear;
% Initialize
initialize();
input('Press any key to continue!');


% Main
theta1 = pi; % DH joint angle unit rad

Theta1 = theta1/2/pi*4096; % Motor angle 0-4095; You may also need to consider the offset, i.e., when theta1 = 0, Theta1 ~= 0.

% Move MX28_ID(1) to Theta1 angle
% write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, typecast(int32(Theta1), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(7), MX28_GOAL_POSITION, 1900);
% while 1
%         % Read present position
%         dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_PRESENT_POSITION);
%         if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
%         elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 737
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
%         end
% 
%         fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', MX28_ID(1), 0, typecast(uint32(dxl_present_position), 'int32'));
% 
%         if ~(abs(737 - typecast(uint32(dxl_present_position), 'int32')) > MOVING_STATUS_THRESHOLD)
%             break;
%         end
% end
% write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, 4095);


% Terminate
input('Press any key to terminate!');
terminate();