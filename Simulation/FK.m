function [fx,fy,fz,T] = FK(c,joint)

a3 = c(1); % mm
a5 = c(2); %mm
d2 = c(3); % mm
d5 = c(4); %mm
de = c(5); %mm

T5e = [1 0 0 a5;
       0 1 0 0;
       0 0 1 de;
       0 0 0 1];

d1 = joint(1);
t2 = joint(2);
t3 = joint(3);
t4 = joint(4);
t5 = joint(5);

% DH parameters
DH = [0 0 d1 0;
      pi/2 0 d2 t2;
      pi/2 0 0 t3;
      0 a3 0 t4+pi/2;
      pi/2 0 d5 t5];
alpha = DH(:,1); a = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4);
fx = 0; fy = 0; fz = 0;
T{1} = To;

for j = 1:5
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    fx = [fx To(1,4)]; % frame x coordiante
    fy = [fy To(2,4)]; % frame y coordiante
    fz = [fz To(3,4)]; % frame z coordiante
    T{j+1} = To;
    
end
To = T{6}*T5e;
fx = [fx To(1,4)]; % frame x coordiante
fy = [fy To(2,4)]; % frame y coordiante
fz = [fz To(3,4)]; % frame z coordiante
T{7} = To;

end