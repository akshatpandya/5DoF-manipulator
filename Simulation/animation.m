function animation(c,joint,path,movie,speed)

a3 = c(1); % mm
a5 = c(2); %mm
d2 = c(3); % mm
d5 = c(4); %mm
de = c(5); %mm
px = path(1,:);
py = path(2,:);
pz = path(3,:);

h = 0.2; % base height

% Create Movie
if movie == 1
    v = VideoWriter('ani.avi');
    open(v);
end

Rz = [0 -1 0;
      1 0 0
      0 0 1];
Ry = [0 0 -1;
      0 1 0;
      1 0 0];
R = Rz*Ry;
T_ = [transpose(R) zeros(3,1); 0 0 0 1];

for i = 1:speed:size(joint,2)
    
    [fx,fy,fz,T] = FK(c,joint(:,i));
    disp(".......");
%     disp([fx(6) fy(6) fz(6)]);
    disp([fx(end) fy(end) fz(end)]);
    disp(".......");
%     %fz = fz + 0.1;

% Link1 - J1-2
% Link2 - J2-3 P0J2 = T02*P2J2
%     disp(".........")
%     disp([fx(5) fy(5) fz(5) 1])
%     disp([fx(6) fy(6) fz(6) 1])
%     disp(".........")
    P0J2 = T_*T{3}*[0; 0; -d2; 1];
    P0J3 = T_*[fx(4); fy(4); fz(4); 1];
    P0J4 = T_*[fx(5); fy(5); fz(5); 1];
    P0J5 = T_*[fx(6); fy(6); fz(6); 1];
    P0JE = T_*[fx(7); fy(7); fz(7); 1];

% Link3 - J3-4
% Link4 - J4-5
% Link4 - J5-ee

    plot3([P0J2(1), P0J3(1)], [P0J2(2), P0J3(2)], [P0J2(3), P0J3(3)], 'k', 'LineWidth',8);
    hold on;
    plot3([P0J3(1), P0J4(1)], [P0J3(2), P0J4(2)], [P0J3(3), P0J4(3)], 'b', 'LineWidth',8);
    hold on;
    plot3([P0J4(1), P0J5(1)], [P0J4(2), P0J5(2)], [P0J4(3), P0J5(3)], 'g', 'LineWidth',8);
    hold on;
    plot3([P0J5(1), P0JE(1)], [P0J5(2), P0JE(2)], [P0J5(3), P0JE(3)], 'm', 'LineWidth',8);
    hold on;
%     
%     % Base
%     plot3([0 fx(1)],[0 fy(1)],[-h fz(1)],'k','linewidth',8);
%     hold on;
%     % Manipulator
%     plot3(fx(1:end-1),fy(1:end-1),fz(1:end-1),'k','linewidth',4);
%     % Tool
%     plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m','linewidth',3);
%     % Frames
%     for j = 1:8
%         Rj = T{j}(1:3,1:3);
%         mag = 0.025;
%         plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
%         plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
%         plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
%     end
    % Trajectory
    traj = T_*[px;py;pz;ones(1,length(px))];
    plot3(traj(1,:),traj(2,:),traj(3,:),'b');
    hold on;
%     % Ground
%     X = [1 -1;1 -1]*0.2;
%     Y = [1 1;-1 -1]*0.2;
%     Z = [1 1;1 1]*-h;
%     surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
%     % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    axis([-100 300 -350 350 -100 300]);
%     pbaspect([1 1 1]);
%     grid on;
% %     view(0,0);
%     view(40,30);
    hold off;
%     drawnow;
%     
    if movie == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
    end

end

if movie == 1
    close(v)
end

end