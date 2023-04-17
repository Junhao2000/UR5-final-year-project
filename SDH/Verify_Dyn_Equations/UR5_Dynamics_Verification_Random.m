% UR5 robot (SDH)--动力学验证
%1. 使用SDH参数与机器人动力学参数建立UR5机器人。
%3. 将六个关节的关节角度，关节速度，关节力矩分别带入Newton-Euler(toolbox), Lagrange获得关节加速度验证动力学模型的准确性。 
clear; clc;
%% Robot initialization
% Standard DH parameters 
th(1) = 0; d(1) = 0.08916; a(1) = 0; alp(1) = pi/2;
th(2) = 0; d(2) = 0; a(2) =-0.425; alp(2) = 0;   
th(3) = 0; d(3) = 0; a(3) = -0.39225; alp(3) = 0;
th(4) = 0; d(4) =0.10915; a(4) = 0; alp(4) = pi/2;
th(5) = 0; d(5) = 0.09465; a(5) = 0; alp(5) = -pi/2;
th(6) = 0; d(6) = 0.0823; a(6) = 0; alp(6) = 0;

% Robot linkage set up: Link([theta, d, a, alpha, sigma],'modified')
L1 = Link([th(1), d(1), a(1), alp(1), 0], 'standard'); 
L2 = Link([th(2), d(2), a(2), alp(2), 0], 'standard');
L3 = Link([th(3), d(3), a(3), alp(3), 0], 'standard');
L4 = Link([th(4), d(4), a(4), alp(4), 0], 'standard');
L5 = Link([th(5), d(5), a(5), alp(5), 0], 'standard');  
L6 = Link([th(6), d(6), a(6), alp(6), 0], 'standard');

% Mass property identification 
% L1.I = Link mass [m]
L1.m = 3.7; L2.m = 8.393; L3.m = 2.275;
L4.m = 1.219; L5.m = 1.219; L6.m = 0.1979;
% L1.r = Location of centre of mass [m]
L1.r = 10^-3*[0; -25.61; 1.93]; L2.r = 10^-3*[212.5; 0; 113.36]; L3.r = 10^-3*[119.93; 0; 26.5];
L4.r = 10^-3*[0; -1.8; 16.34]; L5.r = 10^-3*[0; 1.8; 16.34]; L6.r = 10^-3*[0; 0; -1.159];
% L1.I = Interia tensor [kg*m^2]
L1.I = [0.0067 0 0; 0 0.0064 0; 0 0 0.0067]; 
L2.I = [0.0149 0 0; 0 0.3564 0; 0 0 0.3553];
L3.I = [0.0025 0 0; 0 0.0551 0; 0 0 0.0546];
L4.I = [0.0012 0 0; 0 0.0012 0; 0 0 0.0009];
L5.I = [0.0012 0 0; 0 0.0012 0; 0 0 0.0009];
L6.I = [0.0001 0 0; 0 0.0001 0; 0 0 0.0001];
% L1.Jm = Motor inertia [kg*m^2]
L1.Jm = 0; L2.Jm = 0; L3.Jm = 0; 
L4.Jm = 0; L5.Jm = 0; L6.Jm = 0; 

%Robot build up: SerialLink([link_num])
robot = SerialLink([L1, L2, L3, L4, L5, L6]); 
robot.name='UR5 Robot';
robot.display() 

%% Dynamics verification
% Verify inverse dynamics equation with 50 random points (random rotation angle, velocity and acceralation)
ACC_Toolbox = zeros(6, 50);
ACC_Lagrange = zeros(6, 50);

for i = 1:50
% tau, q, qdd 最好是实验参数，随机参数会出现不合理的结果。
tau = rand(1,6); % random torque in N*m
q = rand(1,6)*pi/180; % -360 to 360 degrees
qd=rand(1,6)*pi/180; % -180 to 180 degree per second  
% Calculate joint inertia matrix
M = robot.inertia(q);
% Calculate joint Coriolis matrix
C = robot.coriolis(q, qd);
% Calculate joint gravity matrix
G = -1*robot.gravload(q);
% Solve for joint acceleration qdd
qdd = M \ (tau' - C * qd' - G');
% Store joint acceleration in ACC_Toolbox
ACC_Toolbox(:,i) = qdd;
ACC_Lagrange(:,i) = myLagrange_system(tau, q, qd)';
end

% Calculate the ratio of acceleration difference between the two methods to the acceleration
% obtained by the toolbox at 50 points per joint
diff_ratio = abs(ACC_Toolbox - ACC_Lagrange) ./ abs(ACC_Toolbox);
% Calculate the average ratio of the difference for each joint over all 50 points
avg_ratio = mean(diff_ratio, 2);
% Output the average ratio of torque difference for each joint to the command window
disp("Average ratio of acceleration difference for each joint:");
disp(avg_ratio');

% Plot the joint acceleration separately as a function of point
figure;
for j = 1:6
    subplot(2,3,j);
    plot(1:50, ACC_Toolbox(j,:), 'b--', 'MarkerSize', 8, 'LineWidth', 1);
    hold on;
    plot(1:50, ACC_Lagrange(j,:), 'r:', 'MarkerSize', 8, 'LineWidth', 1.5);
    % Set y limits to show only the overlapping part of the curves
    ylim([min(min(ACC_Toolbox(j,:), ACC_Lagrange(j,:))), max(max(ACC_Toolbox(j,:), ACC_Lagrange(j,:)))]);
    % Find overlapping region and make it thicker
    [~, idx1, idx2] = intersect(ACC_Toolbox(j,:), ACC_Lagrange(j,:));
    if ~isempty(idx1) && ~isempty(idx2)
        plot(idx1, ACC_Toolbox(j,idx1), 'b-', 'MarkerSize', 20, 'LineWidth', 1.5);
        plot(idx2, ACC_Lagrange(j,idx2), 'r-', 'MarkerSize', 20, 'LineWidth', 1.5);
    end
    hold off;
    xlabel('Point');
    ylabel('Acceleration (mm/s^2)');
    title(['Joint ', num2str(j)]);
end

% Create legend outside the subplots
lgd = legend('Exact', 'EOM', 'Location', 'EastOutside', 'Orientation', 'vertical');
lgd.FontSize = 11; % Set font size

% Adjust the spacing between subplots
set(gcf, 'Units', 'normalized', 'Position', [0.1,0.1,0.8,0.8]);
set(gcf, 'DefaultAxesPosition', [0.1,0.1,0.8,0.8]);
set(gcf, 'DefaultAxesFontSize', 12);
set(gcf, 'DefaultLineLineWidth', 1.5);
set(gcf, 'DefaultAxesLineWidth', 1.5);
set(gcf, 'DefaultAxesBox', 'on');