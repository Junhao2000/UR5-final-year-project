function [A, B] = movingLinearization(theta, theta_d, theta_dd, delta)
% movingLinearization: Calculate linearized system matrices A and B based on the provided
% robotic manipulator dynamics function, myLagrange.
% Inputs:
%   theta - joint angles
%   theta_d - joint velocities
%   theta_dd - joint accelerations
%   delta - a small perturbation value for numerical differentiation
% 确定一个合适的 delta 值通常需要根据实际问题进行尝试和调整。
% 在实践中，您可以从一个较小的常数值开始，然后根据计算结果的精度和稳定性来调整 delta 值。
% 
% Outputs:
%   A - linearized system matrix A
%   B - linearized system matrix B

n = length(theta);
A = zeros(n);
B = zeros(n);

tau = myLagrange(theta, theta_d, theta_dd);

for i = 1:n
    d_theta = zeros(n, 1);
    d_theta(i) = delta;
    
    % Numerical differentiation for A matrix
    A(:, i) = (myLagrange(theta + d_theta, theta_d, theta_dd) - tau) / delta;
    
    d_theta_d = zeros(1, 6);
    d_theta_d(i) = delta;
    
    % Numerical differentiation for B matrix
    B(:, i) = (myLagrange(theta, theta_d + d_theta_d, theta_dd) - tau)/ delta;
end

end