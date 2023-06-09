% 六自由度机械臂瞬态运动的牛顿-欧拉递归动力学求解(考虑重力影响)：
% 参数：（运动指令）各关节运动角度， 关节速度， 关节加速度（6*1矩阵）
% 返回值：各关节力矩（6*1矩阵）
% 由于不清楚机械臂实际动力学参数，因此下面的惯性张量及质心位置均为假设杜撰
function tau = myNewtonEuler(theta, theta_d, theta_dd)
%% Robot initialization
% Standard DH parameters 
th(1) = theta(1); d(1) = 0.08916; a(1) = 0; alp(1) = pi/2;
th(2) = theta(2); d(2) = 0; a(2) =-0.425; alp(2) = 0;   
th(3) = theta(3); d(3) = 0; a(3) = -0.39225; alp(3) = 0;
th(4) = theta(4); d(4) =0.10915; a(4) = 0; alp(4) = pi/2;
th(5) = theta(5); d(5) = 0.09465; a(5) = 0; alp(5) = -pi/2;
th(6) = theta(6); d(6) = 0.0823; a(6) = 0; alp(6) = 0;
%Base_link initial velocity and acceleration
w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = [0; 0; -9.8];
%Revolute joint axial vector
z = [0; 0; 1]; 
% Transform function: MDH = MDHTrans(alpha, a, d, theta)
T01 = SDHTrans(alp(1), a(1), d(1), th(1));
T12 = SDHTrans(alp(2), a(2), d(2), th(2));
T23 = SDHTrans(alp(3), a(3), d(3), th(3));
T34 = SDHTrans(alp(4), a(4), d(4), th(4));
T45 = SDHTrans(alp(5), a(5), d(5), th(5));
T56 = SDHTrans(alp(6), a(6), d(6), th(6));
% Rotation matrix
R01 = T01(1:3, 1:3); R12 = T12(1:3, 1:3); R23 = T23(1:3, 1:3);
R34 = T34(1:3, 1:3); R45 = T45(1:3, 1:3); R56 = T56(1:3, 1:3);
R10 = R01'; R21 = R12'; R32 = R23';
R43 = R34'; R54 = R45'; R65 = R56';
R67 = [1 0 0; 0 1 0; 0 0 1]; R76 = R67';
% Position of center of mass pc
p10 = T01(1: 3, 4); p21 = T12(1: 3, 4); p32 = T23(1: 3, 4);
p43 = T34(1: 3, 4); p54 = T45(1: 3, 4); p65 = T56(1: 3, 4); p76 = [0, 0, 0]';

%% Robot mass parameters 
% Joint mass (provided)
m1 = 3.7; m2 = 8.393; m3 = 2.275;
m4 = 1.219; m5 = 1.219; m6 = 0.1979;
% Interia tensor (approximation by Kufita)
I1=  [0.0067 0 0; 0 0.0064 0; 0 0 0.0067]; I2 = [0.0149 0 0; 0 0.3564 0; 0 0 0.3553];
I3 = [0.0025 0 0; 0 0.0551 0; 0 0 0.0546]; I4 = [0.0012 0 0; 0 0.0012 0; 0 0 0.0009];
I5 = [0.0012 0 0; 0 0.0012 0; 0 0 0.0009]; I6 = [0.0001 0 0; 0 0.0001 0; 0 0 0.0001];
% Position of center of mass pc (provided)
pc11 = 10^-3*[0; -25.61; 1.93]; pc22 = 10^-3*[212.5; 0; 113.36]; pc33 = 10^-3*[119.93; 0; 26.5];
pc44 = 10^-3*[0; -1.8; 16.34]; pc55 = 10^-3*[0; 1.8; 16.34]; pc66 = 10^-3*[0; 0; -1.159];

%% Outward iterations: i: 0->5 (Iteration of linkage 1 to linkage 6 outward)
% i = 0
w11 = R10*w00 + theta_d(1)*z;
w11d = R10*w00d + cross(R10*w00, z*theta_d(1)) + theta_dd(1)*z;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d);
vc11d = cross(w11d, pc11) + cross(w11, cross(w11, pc11)) + v11d;
F11 = m1*vc11d;
N11 = I1*w11d + cross(w11, I1*w11);
% i = 1
w22 = R21*w11 + theta_d(2)*z;
w22d = R21*w11d + cross(R21*w11, z*theta_d(2)) + theta_dd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, pc22) + cross(w22, cross(w22, pc22)) + v22d;
F22 = m2*vc22d;
N22 = I2*w22d + cross(w22, I2*w22);
% i = 2
w33 = R32*w22 + theta_d(3)*z;
w33d = R32*w22d + cross(R32*w22, z*theta_d(3)) + theta_dd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, pc33) + cross(w33, cross(w33, pc33)) + v33d;
F33 = m3*vc33d;
N33 = I3*w33d + cross(w33, I3*w33);
% i= 3
w44 = R43*w33 + theta_d(4)*z;
w44d = R43*w33d + cross(R43*w33, z*theta_d(4)) + theta_dd(4)*z;
v44d = R43*(cross(w33d, p43) + cross(w33, cross(w33, p43)) + v33d);
vc44d = cross(w44d, pc44) + cross(w44, cross(w44, pc44)) + v44d;
F44 = m4*vc44d;
N44 = I4*w44d + cross(w44, I4*w44);
% i = 4
w55 = R54*w44 + theta_d(5)*z;
w55d = R54*w44d + cross(R54*w44, z*theta_d(5)) + theta_dd(5)*z;
v55d = R54*(cross(w44d, p54) + cross(w44, cross(w44, p54)) + v44d);
vc55d = cross(w55d, pc55) + cross(w55, cross(w55, pc55)) + v55d;
F55 = m5*vc55d;
N55 = I5*w55d + cross(w55, I5*w55);
% i = 5
w66 = R65*w55 + theta_d(6)*z;
w66d = R65*w55d + cross(R65*w55, z*theta_d(6)) + theta_dd(6)*z;
v66d = R65*(cross(w55d, p65) + cross(w55, cross(w55, p65)) + v55d);
vc66d = cross(w66d, pc66) + cross(w66, cross(w66, pc66)) + v66d;
F66 = m6*vc66d;
N66 = I6*w66d + cross(w66, I6*w66);

%% Inward iterations: i: 6->1 (Iteration inward from linkage 6 to linkage 1)
f77 = [0; 0; 0]; n77 = [0; 0; 0];
% i = 6
f66 = R67*f77 + F66;
n66 = N66 + R67*n77 + cross(pc66, F66) + cross(p76, R67*f77);
tau(6) = n66'*z;
% i = 5
f55 = R56*f66 + F55;
n55 = N55 + R56*n66 + cross(pc55, F55) + cross(p65, R56*f66);
tau(5) = n55'*z;
% i = 4
f44 = R45*f55 + F44;
n44 = N44 + R45*n55 + cross(pc44, F44) + cross(p54, R45*f55);
tau(4) = n44'*z;
% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(pc33, F33) + cross(p43, R34*f44);
tau(3) = n33'*z;
% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(pc22, F22) + cross(p32, R23*f33);
tau(2) = n22'*z;
% i =1
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(pc11, F11) + cross(p21, R12*f22);
tau(1) = n11'*z;

end
