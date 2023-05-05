clear; clc;
%% Robot initialization
% Standard DH parameters 
th(1) = 0; d(1) = 89.16; a(1) = 0; alp(1) = pi/2;
th(2) = 0; d(2) = 0; a(2) =-425; alp(2) = 0;   
th(3) = 0; d(3) = 0; a(3) = -392.25; alp(3) = 0;
th(4) = 0; d(4) =109.15; a(4) = 0; alp(4) = pi/2;
th(5) = 0; d(5) = 94.65; a(5) = 0; alp(5) = -pi/2;
th(6) = 0; d(6) = 82.3; a(6) = 0; alp(6) = 0;

% Robot linkage set up: Link([theta, d, a, alpha, sigma],'standard')
L1 = Link([th(1), d(1), a(1), alp(1), 0], 'standard'); 
L2 = Link([th(2), d(2), a(2), alp(2), 0], 'standard');
L3 = Link([th(3), d(3), a(3), alp(3), 0], 'standard');
L4 = Link([th(4), d(4), a(4), alp(4), 0], 'standard');
L5 = Link([th(5), d(5), a(5), alp(5), 0], 'standard');  
L6 = Link([th(6), d(6), a(6), alp(6), 0], 'standard');
robot = SerialLink([L1, L2, L3, L4, L5, L6]); 
robot.name='UR5 Robot';

% Circle path
t = (0:0.2:15)'; count = length(t); center = [-402.2153, -294.5843, 335.9848]; radius = 100;
theta1 = t*(2*pi/t(end));
points = (center + radius*[cos(theta1) sin(theta1) zeros(size(theta1))])';


% Forward and Inverse kinematics
hold on
title("Cirular Trajectory Tracking");
xlabel('x/m','FontSize',12);
ylabel('y/m','FontSize',12);
zlabel('z/m','FontSize',12);

for i = 1:size(points,2)
    pause(0.001)
    bx = points(1,i); by = points(2,i); bz = points(3,i);
    plot3(bx,by,bz,'*','LineWidth',1);
    
    % Position vector of end-effector
    targetPos = [bx by bz]; 
    
  % Euler angular rotation matrix to transformation matrix
    tform = rpy2tr(0, pi, 0); % Rotate end-effector 180 degrees about the Y axis

    % Transformation matrix of end-effector
    TR = transl(targetPos) * tform;
    hold on
    grid on
    q = robot.ikine(TR);
    pause(0.001)
    
    % Animation
    robot.plot(q);
end