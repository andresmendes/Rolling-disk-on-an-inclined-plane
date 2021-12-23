%% Rolling disk on an inclined plane - Solid vs. Hollow
% Simulation and animation of two disks rolling down an inclined plane
% without slipping. One disk is solid and the other is hollow.
%
%%

clear ; close all ; clc

%% Parameters

% Disk
m       = 1;                    % Mass                          [kg]
R       = 1;                    % Radius                        [m]
Jz_1    = 1/2*m*R^2;            % Moment of inertia 1 (solid)   [kg.m2]
Jz_2    = m*R^2;                % Moment of inertia 2 (hollow)  [kg.m2]
th      = 25*pi/180;            % Inclination                   [rad]

% Struct 1
parameters_1.m      = m;
parameters_1.Jz     = Jz_1;
parameters_1.R      = R;
% Struct 2
parameters_2 = parameters_1;
parameters_2.Jz     = Jz_2;

L = 10;                         % Length of the ramp            [m]
H = L*sin(th) + R*cos(th);      % Height initial                [m] 

% Disk circumference
beta = 0:0.1:2*pi+0.1;          % Angle for disc shape          [rad]
disc_x = R*cos(beta);           % Disc circumference x coord.   [m]
disc_y = R*sin(beta);           % Disc circumference y coord.   [m]

% Video
playback_speed = 0.3;           % Speed of playback
tF      = 10;                   % Final time                    [s]
fR      = 30/playback_speed;    % Frame rate                    [fps]
dt      = 1/fR;                 % Time resolution               [s]
time    = linspace(0,tF,tF*fR); % Time                          [s]

%% Simulation

% Initial conditions
z0 = [0 0]; % [initial_position   initial_speed]

options = odeset('Events',@(t,z) terminate_rolling(t,z,L));
[time,zout_1] = ode45(@(t,z) disk_rolling(t,z,parameters_1,th),time,z0,options);
[time,zout_2] = ode45(@(t,z) disk_rolling(t,z,parameters_2,th),time,z0,options);

x_1 = zout_1(:,1);              % Longitdunial position disk 1  [m]
x_2 = zout_2(:,1);              % Longitdunial position disk 2  [m]

%% Animation

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

hold on ; grid on ; box on ; axis equal
set(gca,'Xlim',[-1 12],'Ylim',[-1 8])
set(gca,'FontName','Verdana','FontSize',18)

an = annotation('textbox', [0.22 0.03, 0.5, 0.1], 'string','The hollow always loses.','FitBoxToText','on');
an.FontName     = 'Verdana';
an.FontSize     = 18;
an.LineStyle    = 'none';
an.FontWeight   = 'Bold';

% Create and open video writer object
v = VideoWriter('Rolling_disk_inclined_comparison.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(time)
    
    % Disk 1
    disk_1_c_x = x_1(i)*cos(th) + R*sin(th);
    disk_1_c_y = H - x_1(i)*sin(th);
    
    % Disk 2
    disk_2_c_x = x_2(i)*cos(th) + R*sin(th);
    disk_2_c_y = H - x_2(i)*sin(th);
    
    % Arc 1
    th_arc_1 = 0:0.0001:disk_1_c_x/R;
    arc_x_1 = -R*sin(th_arc_1);
    arc_y_1 = -R*cos(th_arc_1);
    
    % Arc 2
    th_arc_2 = 0:0.0001:disk_2_c_x/R;
    arc_x_2 = -R*sin(th_arc_2);
    arc_y_2 = -R*cos(th_arc_2);
    
    cla
    
    title(["Rolling disk - Solid vs. Hollow",strcat('Time=',num2str(time(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')')])
    
    % Ramp
    plot([0 L*cos(th) 0],[L*sin(th) 0 0],'k','LineWidth',3)

    % Disk 1 circumference
    plot(disc_x+disk_1_c_x,disc_y+disk_1_c_y,'k','LineWidth',4)
    % Disk 2 circumference
    plot(disc_x+disk_2_c_x,disc_y+disk_2_c_y,'k','LineWidth',4)
    
    % Disk 1 diameter 
    plot([disk_1_c_x-arc_x_1(end) disk_1_c_x+arc_x_1(end)],[disk_1_c_y-arc_y_1(end) disk_1_c_y+arc_y_1(end)],'k--','LineWidth',2)
    % Disk 2 diameter 
    plot([disk_2_c_x-arc_x_2(end) disk_2_c_x+arc_x_2(end)],[disk_2_c_y-arc_y_2(end) disk_2_c_y+arc_y_2(end)],'k--','LineWidth',2)
    
    % Disk 1 center
    d1 = plot(disk_1_c_x,disk_1_c_y,'ko','MarkerFaceColor','r','MarkerSize',15,'Displayname','Solid');
    % Disk 2 center
    d2 = plot(disk_2_c_x,disk_2_c_y,'ko','MarkerFaceColor','b','MarkerSize',15,'Displayname','Hollow');
    
    % Contact point initial
    plot(0,L*sin(th),'ko','MarkerFaceColor','r','MarkerSize',10) % Disk 1 
    plot(0,L*sin(th),'ko','MarkerFaceColor','b','MarkerSize',10) % Disk 2
    
    % Disk 1 x distance
    plot([0 disk_1_c_x-R*sin(th)],[L*sin(th)  L*sin(th)-x_1(i)*sin(th)],'r','LineWidth',2)
    % Disk 2 x distance
    plot([0 disk_2_c_x-R*sin(th)],[L*sin(th)  L*sin(th)-x_2(i)*sin(th)],'b','LineWidth',2)
    
    % Disk 1 contact point 
    plot(disk_1_c_x-R*sin(th),L*sin(th)-x_1(i)*sin(th),'ko','MarkerFaceColor','r','MarkerSize',10)
    % Disk 2 contact point 
    plot(disk_2_c_x-R*sin(th),L*sin(th)-x_2(i)*sin(th),'ko','MarkerFaceColor','b','MarkerSize',10)
    
    xlabel('x [m]')
    ylabel('y [m]')

    legend([d1 d2])
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end

close(v);

function dz = disk_rolling(~,z,parameters,th)
    % Rolling disk longitudinal dynamics rolling down an inclined plane.

    m   = parameters.m;     % Mass              [kg]
    Jz  = parameters.Jz;    % Moment of inertia [kg.m2]
    R   = parameters.R;     % Radius            [m]
    g   = 9.81;             % Gravity           [m/s2]

    % Dynamics
    dz(1,1) = z(2);
    dz(2,1) = m*g*sin(th) / (m+Jz/R^2);

end

function [value,isterminal,direction] = terminate_rolling(~,z,L)
    value       = L-z(1);       % Final position at ramp
    isterminal  = 1;            % stop the integration
    direction   = 0;            % negative direction
end
