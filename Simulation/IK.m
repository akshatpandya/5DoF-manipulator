function [d1,t2,t3,t4,t5] = IK(T0e,c,prev_joint)
a3 = c(1);
a5 = c(2);
d2 = c(3);
d5 = c(4);
de = c(5);

T5e = [1 0 0 a5;
       0 1 0 0;
       0 0 1 de;
       0 0 0 1];
T05 = T0e*inv(T5e);

x = T05(1,4);
y = T05(2,4);
z = T05(3,4);

% theta3+theta4
ct34_1 = sqrt(T05(2,1)^2 + T05(2,2)^2);
ct34_2 = -sqrt(T05(2,1)^2 + T05(2,2)^2);
st34 = -T05(2,3);
t34 = [atan2(st34,ct34_1) atan2(st34,ct34_2)];

%theta3
s3 = (-d5*sin(t34)-d2-y)/a3;
c3 = [sqrt(1-s3.^2) -sqrt(1-s3.^2)];
if imag(c3) == 0
    t3_ = [atan2(s3(1), c3(1)) atan2(s3(2), c3(2)) atan2(s3(1), c3(3)) atan2(s3(2), c3(4))];
else
    t3_ = [inf inf];
end

% disp("t3_");
% disp(t3_);

%theta4
t4_ = [t34(1)-t3_(1) t34(1)-t3_(3) t34(2)-t3_(2) t34(2)-t3_(2)];

% disp("t4_");
% disp(t4_);

%theta2 and theta5
cases = [1 3 2 4;
         1 2 3 4];
val_per_case = [0 0 0 0];
t5_ = [];
t2_ = [];
for i=1:length(cases)
    if i==1 || i==2
        k = 1;
    else
        k = 2;
    end
    if mod((t3_(cases(1,i))+t4_(cases(2,i)))*180/pi, 90) ~= 0 || mod((t3_(cases(1,i))+t4_(cases(2,i)))*180/pi,180) == 0
%         disp("here0");
        t5_(end+1,:) = atan2(T05(2,2), -T05(2,1));
        t2_(end+1,:) = atan2(T05(3,3), T05(1,3));
        val_per_case(i) = val_per_case(i)+1;
    else
        if mod(t3_(cases(1,i))*180/pi, 90) ~= 0 || mod(t3_(cases(1,i))*180/pi,180) == 0
%             disp("here1");
            c2 = T05(1,4)/(a3*cos(t3_(cases(1,i))));
            if imag(c2)~=0
                continue;
            end
            s2 = [sqrt(1-c2^2) -sqrt(1-c2^2)];
            if imag(s2)~=0
                val_per_case(i) = val_per_case(i)+2;
                t2_(end+1,end+1:end+2) = [inf inf];
                t5_(end+1,end+1:end+2) = [inf inf];
                continue;
            end
            val_per_case(i) = val_per_case(i)+2;
            t2_(end+1,end+1:end+2) = [atan2(s2(1),c2) atan2(s2(2),c2)];
            t5_(end+1,end+1:end+2) = [atan2(T05(1,1)*s2(1)-T05(3,1)*c2, T05(1,2)*s2(1)-T05(3,2)*c2) atan2(T05(1,1)*s2(2)-T05(3,1)*c2, T05(1,2)*s2(2)-T05(3,2)*c2)];
        else
            % Choose t5 = 0
            t5_(end+1,:) = 0;
            if sin(t34(k)) == 1
                t2_(end+1,:) = atan2(-T05(3,1), -T05(1,1));
%                 disp("here2");
            elseif sin(t34(k)) == -1
                t2_(end+1,:) = atan2(T05(3,1), T05(1,1));
            end
            val_per_case(i) = val_per_case(i)+1;
        end
    end
end

% disp("t2_");
% disp(t2_);

% d1
d1_ = [];
for i = 1:length(cases)
    if i==1 || i==2
        k = 1;
    else
        k = 2;
    end
    for j=1:val_per_case(i)
        d1_temp = T05(3,4)-d5*sin(t2_(i,j))*cos(t34(k))-a3*cos(t3_(cases(1,i)))*sin(t2_(i,j));
        if isnan(d1_temp)
            d1_(i,j) = inf;
            continue;
        end
        d1_(i,j) = d1_temp;
    end
end

% disp("d1");
% disp(d1_);

joints = zeros(sum(val_per_case), 5);
last_index = 1;
for i=1:length(cases)
    joints(last_index:last_index+val_per_case(i)-1,1) = d1_(i,1:val_per_case(i))';
    joints(last_index:last_index+val_per_case(i)-1,2) = t2_(i,1:val_per_case(i))';
    joints(last_index:last_index+val_per_case(i)-1,5) = t5_(i,1:val_per_case(i))';
    joints(last_index:last_index+val_per_case(i)-1,3) = t3_(cases(1,i));
    joints(last_index:last_index+val_per_case(i)-1,4) = t4_(cases(2,i));
    last_index = last_index+val_per_case(i);
end

% disp("joints");
% disp(joints);

row_to_remove = [];
for i=1:sum(val_per_case)
    if joints(i,1)<60 || joints(i,1)>155
        row_to_remove(end+1) = i;
        continue
    end
    if joints(i,3) < -10/180*pi && joints(i,3) > -pi
        row_to_remove(end+1) = i;
        continue
    end
    if joints(i,4) < -90/180*pi || joints(i,4) > 90/180*pi
        row_to_remove(end+1) = i;
        continue
    end
end

joints(row_to_remove,:) = [];

% disp("joints");
% disp(joints);
[m,index] = min(abs(joints(:,1)-prev_joint(1)));

min_index = index;
min_value = inf;
for i=1:size(joints,1)
    if abs((joints(i,1)-prev_joint(1))) == m
        if sum(abs(joints(i,2:end))) < min_value
            min_index = i;
            min_value = (joints(:,1)-prev_joint(1));
        end
    end
end

d1 = joints(min_index,1);
t2 = joints(min_index,2);
t3 = joints(min_index,3);
t4 = joints(min_index,4);
t5 = joints(min_index,5);
% 
% disp("joints");
% disp([d1,t2,t3,t4,t5]);

end