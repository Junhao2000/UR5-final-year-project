clear; clc;

% Read CSV file
filename = 'circle_movement_100mms.csv';
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1; % Set the header row as the first row
data = readtable(filename, opts);

% Extract data
time = data{:,1};
theta = data{:,3:8};

% Forward kinematics: function SDH=SDHTrans(alp, a, d, th)
P0 = zeros(length(theta), 3);
for i = 1:length(theta)
    theta1 = theta(i,3); theta2 = theta(i,2); theta3 = theta(i,1);
    theta4 = theta(i,4); theta5 = theta(i,5); theta6 = theta(i,6);

    SDH01 = SDHTrans(pi/2, 0, 89.16, theta1);
    SDH12 = SDHTrans(0, -425, 0, theta2);
    SDH23 = SDHTrans(0, -392.25, 0, theta3);
    SDH34 = SDHTrans(pi/2, 0, 109.15, theta4);
    SDH45 = SDHTrans(-pi/2, 0, 94.65, theta5);
    SDH56 = SDHTrans(0, 0, 82.3, theta6);

    SDH06 = SDH01*SDH12*SDH23*SDH34*SDH45*SDH56;
    P6 = [0; 0; 0; 1];

    P0_temp = SDH06*P6;
    P0(i,:) = P0_temp(1:3)'; % Store x, y, and z coordinates for each iteration
end

figure; % Create a new figure
plot3(P0(:,1), P0(:,2), P0(:,3), 'o-'); % Plot x, y, and z coordinates in 3D space
grid on; % Turn on the grid
xlabel('X'); ylabel('Y'); zlabel('Z'); % Label the axes
title('3D Circular Trajectory'); % Add a title to the plot