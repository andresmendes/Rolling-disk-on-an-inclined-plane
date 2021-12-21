%% Rolling disk on an inclined plane
% Simulation and animation of a disk rolling down an inclined plane without
% slipping.
%
%%

clear ; close all ; clc

%% Parameters

% Disk
m   = 1;                        % Mass                          [kg]
R   = 1;                        % Radius                        [m]
Jz  = m*R^2;                    % Moment of inertia             [kg.m2]
th  = 25*pi/180;                % Inclination                   [rad]

% Struct
parameters.m    = m;
parameters.Jz   = Jz;
parameters.R    = R;

L = 10;                         % Length of the ramp            [m]
H = L*sin(th) + R*cos(th);      % Height initial                [m] 

% Disk circumference
beta = 0:0.1:2*pi+0.1;          % Angle for disc shape          [rad]
disk_x = R*cos(beta);           % Disc circumference x coord.   [m]
disk_y = R*sin(beta);           % Disc circumference y coord.   [m]

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
[time,zout] = ode45(@(t,z) disk_rolling(t,z,parameters,th),time,z0,options);

x = zout(:,1);                  % Longitudinal position         [m]

%% Animation

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

hold on ; grid on ; box on ; axis equal
set(gca,'Xlim',[-1 12],'Ylim',[-1 8])
set(gca,'FontName','Verdana','FontSize',18)

% Create and open video writer object
v = VideoWriter('rolling_disk_inclined.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(time)
    
    % Disk
    disk_c_x = x(i)*cos(th) + R*sin(th);
    disk_c_y = H - x(i)*sin(th);
    
    % Arc
    th_arc = 0:0.0001:disk_c_x/R;
    arc_x = -R*sin(th_arc);
    arc_y = -R*cos(th_arc);
    
    cla
    
    title(["Rolling disk on an inclined plane",strcat('Time=',num2str(time(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')')])
    
    % Ramp
    plot([0 L*cos(th) 0],[L*sin(th) 0 0],'k','LineWidth',3)

    % Disk circumference
    plot(disk_x+disk_c_x,disk_y+disk_c_y,'k','LineWidth',4)
    
    % Disk diameter 
    plot([disk_c_x-arc_x(end) disk_c_x+arc_x(end)],[disk_c_y-arc_y(end) disk_c_y+arc_y(end)],'k--','LineWidth',2)

    % Disk center
    plot(disk_c_x,disk_c_y,'ko','MarkerFaceColor','k','MarkerSize',10)
    
    % Contact point initial
    plot(0,L*sin(th),'ko','MarkerFaceColor','m','MarkerSize',10)

    % Disk x distance
    plot([0 disk_c_x-R*sin(th)],[L*sin(th)  L*sin(th)-x(i)*sin(th)],'m','LineWidth',2)
    
    % Disk contact point 
    plot(disk_c_x-R*sin(th),L*sin(th)-x(i)*sin(th),'ko','MarkerFaceColor','m','MarkerSize',10)
    
    xlabel('x [m]')
    ylabel('y [m]')

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
