close all; clear all; clc

% DH Parameters
syms t1 t2 t3 t4 t5 t6 a2 a3 d3 d4
alpha_1_mod = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
a2 = 0.4318;
a3 = 0.0191;
d3 = 0.1254;
d4 = 0.4318;
a_mod =       [0, 0, a2, a3, 0, 0, 0.1];
d_mod =       [0, 0, d3, d4, 0, 0, 0.08];
theta_mod =   [t1, t2, t3, t4, t5, t6, 0];

% Build Robot
L1 = Link('revolute', 'd', d_mod(1), 'a', a_mod(1), 'alpha', alpha_1_mod(1), 'modified')
L2 = Link('revolute', 'd', d_mod(2), 'a', a_mod(2), 'alpha', alpha_1_mod(2), 'modified')
L3 = Link('revolute', 'd', d_mod(3), 'a', a_mod(3), 'alpha', alpha_1_mod(3), 'modified')
L4 = Link('revolute', 'd', d_mod(4), 'a', a_mod(4), 'alpha', alpha_1_mod(4), 'modified')
L5 = Link('revolute', 'd', d_mod(5), 'a', a_mod(5), 'alpha', alpha_1_mod(5), 'modified')
L6 = Link('revolute', 'd', d_mod(6), 'a', a_mod(6), 'alpha', alpha_1_mod(6), 'modified')
LT = Link('revolute', 'd', d_mod(7), 'a', a_mod(7), 'alpha', alpha_1_mod(7), 'modified')
PUMA560= SerialLink([L1, L2, L3, L4, L5, L6, LT], 'name', 'PUMA560')

% Forward Kinematics
fprintf('Forward Kinematics')
PUMA560_FK = simplify(PUMA560.fkine(theta_mod))

% Homogeneous Transformation Matrices
fprintf('Homogeneous Transformation Matrices')
PUMA560.A([1 2 3], theta_mod)

% Visualize Robot
fprintf('Plot PUMA560')
close all; clear all; clc

% DH Parameters
a2 = 0.4318; a3 = 0.019; d3 = 0.1254; d4 = 0.4318;
alpha_1_mod = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
a_mod =       [0, 0, a2, a3, 0, 0, 0.1];
d_mod =       [0, 0, d3, d4, 0, 0, 0.08];
theta_mod =   [0, 0, 0, 0, 0, 0, 0];

L1 = Link('revolute', 'd', d_mod(1), 'a', a_mod(1), 'alpha', alpha_1_mod(1), 'modified')
L2 = Link('revolute', 'd', d_mod(2), 'a', a_mod(2), 'alpha', alpha_1_mod(2), 'modified')
L3 = Link('revolute', 'd', d_mod(3), 'a', a_mod(3), 'alpha', alpha_1_mod(3), 'modified')
L4 = Link('revolute', 'd', d_mod(4), 'a', a_mod(4), 'alpha', alpha_1_mod(4), 'modified')
L5 = Link('revolute', 'd', d_mod(5), 'a', a_mod(5), 'alpha', alpha_1_mod(5), 'modified')
L6 = Link('revolute', 'd', d_mod(6), 'a', a_mod(6), 'alpha', alpha_1_mod(6), 'modified')
LT = Link('revolute', 'd', d_mod(7), 'a', a_mod(7), 'alpha', alpha_1_mod(7), 'modified')
PUMA560= SerialLink([L1, L2, L3, L4, L5, L6, LT], 'name', 'PUMA560')

figure()
PUMA560.plot(theta_mod);