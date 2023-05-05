% Circle center and radius
circle_center = [-402.2153, -294.5843, 335.9848];
radius = 100;

% Generate circle angles
circle_angles = linspace(0, 2*pi, 100);

% Calculate points on the circle
circle_x = circle_center(1) + radius * cos(circle_angles);
circle_y = circle_center(2) + radius * sin(circle_angles);
circle_z = circle_center(3) * ones(size(circle_angles));

% Plot ideal trajectory (red dashed circle)
figure;
plot3(circle_x, circle_y, circle_z, 'r--', 'LineWidth', 2);
hold on;

% Generate simulated trajectory (blue dotted circle) - without noise
simulated_circle_x = circle_center(1) + radius * cos(circle_angles);
simulated_circle_y = circle_center(2) + radius * sin(circle_angles);
simulated_circle_z = circle_center(3) * ones(size(circle_angles));

% Plot simulated trajectory (blue dotted circle)
plot3(circle_x, circle_y, circle_z, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'none', 'LineWidth', 1);

% Add grid, labels, title, and legend
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Circular Trajectory: Ideal vs Simulated');
legend('Actual circle', 'Simulated circle');
hold off;