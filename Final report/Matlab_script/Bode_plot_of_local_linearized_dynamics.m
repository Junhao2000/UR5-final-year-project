% Randomly select joint positions and velocities
q0 = rand(6,1)*pi/180;
qd0 = rand(6,1)*pi/180;
tau0 = rand(6,1);

% Compute the Jacobian matrices A and B
[A, B] = num_linearize_dynamics(q0, qd0, tau0);

% State-space representation
C = eye(12); % Define based on the actual system
D = zeros(12,6); % Define g based on the actual system
sys = ss(A, B, C, D);

% Compute frequency response
omega = logspace(-1, 2, 200); % Define frequency range
H = freqresp(sys, omega);

% Plot the Bode plot
bode(sys, omega);

