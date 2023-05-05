clear; clc;

% Read CSV file
filename = 'circle_movement_150mms.csv';
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1; % Set the header row as the first row
data = readtable(filename, opts);

% Extract data
time = data{:,1};
theta = data{:,3:8};
theta_d = data{:,10:15};
current = data{:,17:22};

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

% Calculate joint torque using a custom function
tau = zeros(size(theta));
for i = 1:length(time)
    tau(i,:) = myLagrange(theta(i,:), theta_d(i,:), theta_dd(i,:));
end

% Apply a moving average filter to the joint torque data
window_size = 5; % Moving average window size
tau_filtered = smoothdata(tau, 1, 'movmean', window_size);

% Plot joint displacement and joint speeds over time
figure;
subplot(1, 2, 1);
plot(time, theta);
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angle vs Time');
xlim([time(1), time(end)]); % Limit the x-axis to the maximum time value

subplot(1, 2, 2);
plot(time, theta_d);
xlabel('Time (s)');
ylabel('Joint Velocity (rad/s)');
title('Joint Velocity vs Time');
xlim([time(1), time(end)]); % Limit the x-axis to the maximum time value

% Plot the filtered joint torque and joint current data for each joint over time
figure;
for i = 1:6
    subplot(3, 2, i);
    yyaxis left;
    plot(time, tau_filtered(:,i));
    ylabel('Torque (Nm)');
    xlim([time(1), time(end)]); % Limit the x-axis to the maximum time value

    yyaxis right;
    plot(time, current(:,i));
    ylabel('Current (A)');
    xlim([time(1), time(end)]); % Limit the x-axis to the maximum time value
    
    title(['Joint ', num2str(i)]);
    xlabel('Time (s)');
end