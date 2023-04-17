clear; clc;
% 读取CSV文件
filename = 'circle_movement_150mms.csv';
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1; % 设定标题行是第一行
data = readtable(filename, opts);

% 提取数据
time = data{:,1};
theta = data{:,3:8};
theta_d = data{:,10:15};
current = data{:,17:22};

% 使用五点差分法计算加速度
theta_dd = zeros(size(theta_d));
dt = mean(diff(time));
for i = 3:length(time) - 2
    theta_dd(i,:) = (-1/12*theta_d(i-2,:) + 2/3*theta_d(i-1,:) - 2/3*theta_d(i+1,:) + 1/12*theta_d(i+2,:))/dt;
end

% 使用线性插值处理边界点
theta_dd(1,:) = 2*theta_dd(3,:) - theta_dd(5,:);
theta_dd(2,:) = 2*theta_dd(3,:) - theta_dd(4,:);
theta_dd(end-1,:) = 2*theta_dd(end-2,:) - theta_dd(end-3,:);
theta_dd(end,:) = 2*theta_dd(end-1,:) - theta_dd(end-2,:);

% 使用自定义函数计算关节力矩
tau = zeros(size(theta));
for i = 1:length(time)
    tau(i,:) = myLagrange(theta(i,:), theta_d(i,:), theta_dd(i,:));
end

% 绘制关节力矩和关节电流随时间的变化
figure;
for i = 1:6
    subplot(3, 2, i);
    yyaxis left;
    plot(time, tau(:,i));
    ylabel('Torque (Nm)');
    
    yyaxis right;
    plot(time, current(:,i));
    ylabel('Current (A)');
    
    title(['Joint ', num2str(i), ' Torque and Current']);
    xlabel('Time (s)');
    legend('Torque', 'Current');
end