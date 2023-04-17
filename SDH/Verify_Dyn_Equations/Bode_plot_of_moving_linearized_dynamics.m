clear; clc;
%% Input circular trajectory tracing data.
% Read CSV file
filename = 'circle_movement_100mms.csv';
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;
data = readtable(filename, opts);

% Extract data
time = data{:,1};
theta = data{:,3:8};
theta_d = data{:,10:15};

% Calculate acceleration using the five-point difference method
theta_dd = zeros(size(theta_d));
dt = mean(diff(time));
for i = 3:length(time) - 2
    theta_dd(i,:) = (-1/12*theta_d(i-2,:) + 2/3*theta_d(i-1,:) - 2/3*theta_d(i+1,:) + 1/12*theta_d(i+2,:))/dt;
end

% Use linear interpolation to handle boundary points
theta_dd(1,:) = 2*theta_dd(3,:) - theta_dd(5,:);
theta_dd(2,:) = 2*theta_dd(3,:) - theta_dd(4,:);
theta_dd(end-1,:) = 2*theta_dd(end-2,:) - theta_dd(end-3,:);
theta_dd(end,:) = 2*theta_dd(end-1,:) - theta_dd(end-2,:);

%% Moving linearization and make a bode plot of each joint at a certain point.
% Random chose a local point
p = 5000;
q = theta(p,:);
dq = theta_d(p,:);
ddq = theta_dd(p,:);
delta = 1e-6;

% Compute the matrices A and B of the linearized system
[A, B] = movingLinearization(q, dq, ddq, delta);

% Convert state space representation to transfer function representation
n = size(A, 1);
tf_sys = cell(n, 1);
for i = 1:n
    [num, den] = ss2tf(A, B, eye(n), zeros(n, n), i);
    tf_sys{i} = tf(num(i,:), den);
end

% Bode plot of the joint
figure;
freq_range = [0.1, 10];  % Specify the frequency range
options = bodeoptions;
options.FreqUnits = 'Hz';
options.FreqScale = 'log';
options.Xlim = freq_range;

for i = 1:n
    subplot(3, 2, i);
    bode(tf_sys{i}, options);
    title(['Joint ', num2str(i)]);
end
set(findall(gcf,'type','text'),'FontSize',11); % 修改字体大小
