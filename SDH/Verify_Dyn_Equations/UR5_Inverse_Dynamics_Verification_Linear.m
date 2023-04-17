% UR5 robot (SDH)--逆动力学验证
%1. 使用SDH参数与机器人动力学参数建立UR5机器人。
%2. 机械臂末端执行器在空间中匀速画一条直线。选择直线上50个点，将他们对应的机械臂六个关节的转动角度，转动速度，转动加速度分别记录下来。
%3. 将六个关节的转动角度，转动速度，转动加速度分别使用Newton-Euler(toolbox), Lagrange获得逆动力学公式验证模型的准确性。 
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

%% Generate trajectory
% Define start and end points of the line in Cartesian coordinates
p_start = [3 4 5];
p_end = [4 5 6];
% Generate a linear trajectory
n_points = 50; % number of points on the trajectory
p_traj = zeros(n_points, 3); % initialize trajectory array
for i = 1:n_points
    p_traj(i,:) = p_start + (i-1)/(n_points-1)*(p_end-p_start);
end

% Calculate inverse kinematics for each point on the trajectory
q_traj = zeros(n_points, 6); % initialize joint angles array
qd_traj = zeros(n_points, 6); % initialize joint velocities array
qdd_traj = zeros(n_points, 6); % initialize joint accelerations array
for i = 1:n_points
    T = transl(p_traj(i,:)); % generate a homogenous transformation matrix for each point
    q_traj(i,:) = robot.ikcon(T); % calculate inverse kinematics for each point
end

% Calculate joint velocities and accelerations using numerical differentiation
dt = 0.01; % time step
for i = 2:n_points-1
    qd_traj(i,:) = (q_traj(i+1,:) - q_traj(i-1,:))/(2*dt);
    qdd_traj(i,:) = (q_traj(i+1,:) - 2*q_traj(i,:) + q_traj(i-1,:))/(dt^2);
end
qd_traj(1,:) = (q_traj(2,:) - q_traj(1,:))/dt; % forward difference for first point
qd_traj(n_points,:) = (q_traj(n_points,:) - q_traj(n_points-1,:))/dt; % backward difference for last point
qdd_traj(1,:) = (q_traj(3,:) - 2*q_traj(2,:) + q_traj(1,:))/(dt^2); % second-order central difference for first point
qdd_traj(n_points,:) = (q_traj(n_points,:) - 2*q_traj(n_points-1,:) + q_traj(n_points-2,:))/(dt^2); % second-order central difference for last point
%% Dynamics verification
% Verify inverse dynamics equation with 50 points on a linear trajectory
TAU_Toolbox = zeros(6, n_points);
TAU_Lagrange = zeros(6, n_points);
for i = 1:n_points
TAU_Toolbox(:,i) = robot.rne(q_traj(i,:), qd_traj(i,:), qdd_traj(i,:), [0, 0, -9.8])';
TAU_Lagrange(:,i) = myLagrange(q_traj(i,:), qd_traj(i,:), qdd_traj(i,:))';
end

% Calculate the ratio of torque difference between the two methods to the torque obtained by the toolbox at 50 points per joint
diff_ratio = abs(TAU_Toolbox - TAU_Lagrange) ./ abs(TAU_Toolbox);
% Calculate the average ratio of the difference for each joint over all 50 points
avg_ratio = mean(diff_ratio, 2);
% Output the average ratio of torque difference for each joint to the command window
disp("Average ratio of torque difference for each joint:");
disp(avg_ratio');

% Plot the moment of each joint separately as a function of point
figure;
for j = 1:6
    subplot(3,2,j);
    plot(1:n_points, TAU_Toolbox(j,:), 'b.--', 'MarkerSize', 8, 'LineWidth', 0.5);
    hold on;
    plot(1:n_points, TAU_Lagrange(j,:), 'r.--', 'MarkerSize', 8, 'LineWidth', 0.5);
    % Set y limits to show only the overlapping part of the curves
    ylim([min(min(TAU_Toolbox(j,:), TAU_Lagrange(j,:))), max(max(TAU_Toolbox(j,:), TAU_Lagrange(j,:)))]);
    % Find overlapping region and make it thicker
    [~, idx1, idx2] = intersect(TAU_Toolbox(j,:), TAU_Lagrange(j,:));
    if ~isempty(idx1) && ~isempty(idx2)
        plot(idx1, TAU_Toolbox(j,idx1), 'b.-', 'MarkerSize', 20, 'LineWidth', 1);
        plot(idx2, TAU_Lagrange(j,idx2), 'r.-', 'MarkerSize', 20, 'LineWidth', 1);
    end
    hold off;
    xlabel('Point');
    ylabel('Joint Torque (Nm)');
    title(['Joint ', num2str(j)]);
end

% Create legend outside the subplots
lgd = legend('Toolbox', 'Lagrange', 'Location', 'NorthOutside', 'Orientation', 'horizontal');
lgd.FontSize = 11; % Set font size
lgd.Position(1) = 0.5 - lgd.Position(3)/2; % Center the legend horizontally
lgd.Position(2) = 0.02; % Place the legend at the bottom of the figure

% Adjust the spacing between subplots
set(gcf, 'Units', 'normalized', 'Position', [0.1,0.1,0.8,0.8]);
set(gcf, 'DefaultAxesPosition', [0.1,0.1,0.8,0.8]);
set(gcf, 'DefaultAxesFontSize', 12);
set(gcf, 'DefaultLineLineWidth', 1.5);
set(gcf, 'DefaultAxesLineWidth', 1.5);
set(gcf, 'DefaultAxesBox', 'on');

%在这个代码中，最后一关节可能会出现误差相对较大的问题，因为机器人末端在直线轨迹上移动时，
% 最后一关节的运动相对于其他关节的运动可能更复杂和敏感。这可能会导致运动轨迹的微小变化，从而导致误差。
% 因此，可以在代码中检查末端执行器的轨迹，查看是否存在一些不平滑或者不连续的运动，或者可以尝试改变轨迹的生成方式，使其更加平滑，以减少误差。
% 另外，也可以检查最后一关节的质量参数或DH参数是否准确，或者使用其他数值微分方法来计算关节速度和加速度，以进一步提高计算的精度。