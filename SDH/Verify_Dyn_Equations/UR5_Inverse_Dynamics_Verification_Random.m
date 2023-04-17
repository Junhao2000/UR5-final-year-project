% UR5 robot (SDH)--逆动力学验证
%1. 使用SDH参数与机器人动力学参数建立UR5机器人。
%3. 将六个关节的随机转动角度，随机转动速度，随机转动加速度分别使用Newton-Euler(toolbox), Lagrange获得逆动力学公式验证模型的准确性。 
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
TAU_Toolbox = zeros(6, 50);
TAU_Lagrange = zeros(6, 50);

for i = 1:50
q = ((rand(1,6)*720) - 360)*pi/180; % -360 to 360 degrees
qd =((rand(1,6)*360) - 180)*pi/180; % -180 to 180 degree per second  
qdd = ((rand(1,6)*360) - 180)*pi/180; % -180 to 180 degree per second^2
TAU_Toolbox(:,i) = robot.rne(q, qd, qdd, [0, 0, -9.8])';
TAU_Lagrange(:,i) = myLagrange(q, qd, qdd)';
%TAU_NewtonEuler = myNewtonEuler(q, qd', qdd');
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
    plot(1:50, TAU_Toolbox(j,:), 'b.--', 'MarkerSize', 8, 'LineWidth', 0.5);
    hold on;
    plot(1:50, TAU_Lagrange(j,:), 'r.--', 'MarkerSize', 8, 'LineWidth', 0.5);
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


%% 第六个关节的力矩误差比其他关节大可能有以下原因：
%     机械结构方面：UR5机器人的第六个关节是一个旋转手腕，相对于其他关节来说机械结构比较复杂，惯量分布不均匀，可能导致在逆动力学计算中误差比较大。
%     控制方面：机器人的控制器可能会对第六个关节施加额外的力矩，这些力矩可能会干扰逆动力学计算的结果，导致误差比较大。
%     数值计算方面：机器人的逆动力学计算涉及到大量的数值计算，可能存在数值误差累积的问题，导致误差比较大。
%     第六个关节特性：第六个关节可能在某些角度、速度或加速度下具有较高的非线性特性，这可能导致计算误差较大。同时，第六个关节受到前五个关节运动的影响，可能会导致更复杂的力矩分布
 
% 针对这个问题，可以考虑采取以下措施：
%     重新检查机器人模型的参数设置是否准确，尤其是第六个关节的惯量和质心位置是否设置正确。
%     检查机器人的控制器设置，尤其是第六个关节是否被正确地控制。
%     优化逆动力学计算算法，减小数值误差的累积。例如可以使用更高精度的数值计算方法，或者使用符号计算等方法来避免数值误差的累积。
%     考虑第六个关节的特性，分析其可能的非线性影响，并在计算中加以考虑。
%% 部分随机点的力矩值非常大可能有以下原因：
%      随机产生的一组关节角度，速度，加速度不具有现实意义。
%      随机产生的一组关节角度，速度，加速度未排除奇异点情况。

% 针对这个问题，可以考虑采取以下措施：
%      使用实验数据或者基于运动学的模拟数据。


