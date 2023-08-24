clear all; close all; clc
%% HW 4

%% joint space trajectory 
[traj, td, tdd] = lspb(0.1, 0.9, 50);
%%
t = linspace(0,4, 50);
figure(1)
plot(t, traj)
grid on;
axis([0,4,0.1,0.9])
xlabel('time (s)')
ylabel('x position (m)')
title('End Effector Position as a Function of Time')

%% find joint torques

% build robot
l1 = 0.5;
l2 = 0.5;
L(1) = Link('revolute','d', 0, 'a', 0, 'alpha', 0 ,'modified');
L(2) = Link('revolute','d', 0, 'a', l1, 'alpha', 0 ,'modified');
L(3) = Link('revolute','d', 0, 'a', l2, 'alpha', 0 ,'modified');

RR1 = SerialLink(L, 'name', '2D-1-RR');

x = traj';
y = zeros(1,50);
q = zeros(3,50);
qd = zeros(3,50);
qdd = zeros(3,50);
% solve ik for each step of traj
for i = 1:50
    q(:,i) = RR_robot_ik_xy(x(i),y(i), l1, l2);
    qd(:,i) = RR_robot_ik_xy(td(i)',y(i), l1, l2);
    qdd(:,i) = RR_robot_ik_xy(tdd(i)',y(i), l1, l2);
end
%%
% t = linspace(1,4,50);
% figure(2)
% plot(t,q(1,:), '.')
% 
% figure(3)
% plot(t,q(2,:), '.')
%%
% make start and end matrices
start = eye(4);
start(1,4) = 0.1;
fin = eye(4);
fin(1,4) = 0.9;
tf = 4;
% make relevant matrices
T1_0_1 = ctraj(start, fin, 50);
T1_1_2 = ctraj(start, fin, 50);
T1_2_T    = [1, 0, 0, l2;
             0, 1, 0,  0;
             0, 0, 1,  0;
             0, 0, 0,  1];
T1_0_T = ctraj(start, fin, 50);
R1_0_1 = zeros(3,3,50);
R1_1_0 = zeros(3,3,50);
R1_1_2 = zeros(3,3,50);
R1_2_T = zeros(3,3,50);
R1_2_1 = zeros(3,3,50);
R1_0_T = zeros(3,3,50);
R1_T_2 = zeros(3,3,50);
P1_0_1 = zeros(3,1,50);
P1_1_2 = zeros(3,1,50);
P1_2_T = zeros(3,1,50);
P1_0_T = zeros(3,1,50);
% fill these with proper matrices
for j = 1:50
% translations
theta_1 = q(1,j);
theta_2 = q(2,j);
T1_0_1(:,:,j) = [cos(theta_1), -sin(theta_1), 0, 0;
             sin(theta_1),  cos(theta_1), 0, 0;
             0,             0, 1, 0;
             0,             0, 0, 1];
T1_1_2(:,:,j) = [cos(theta_2), -sin(theta_2), 0, l1;
             sin(theta_2),  cos(theta_2), 0,  0;
             0,             0, 1,  0;
             0,             0, 0,  1];

T1_0_T(:,:,j) = [cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), - cos(theta_1)*sin(theta_2) - cos(theta_2)*sin(theta_1), 0, l2*(cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2)) + l1*cos(theta_1);
             cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1),   cos(theta_1)*cos(theta_2) - sin(theta_1)*sin(theta_2), 0, l2*(cos(theta_1)*sin(theta_2) + cos(theta_2)*sin(theta_1)) + l1*sin(theta_1);
                                                                 0,                                                       0, 1,                                                                            0;
                                                                 0,                                                       0, 0,                                                                            1];
% rotations
[R1_0_1(:,:,j), P1_0_1(:,:,j)] = tr2rt(T1_0_1(:,:,j)); R1_1_0(:,:,j) = transpose(R1_0_1(:,:,j));
[R1_1_2(:,:,j), P1_1_2(:,:,j)] = tr2rt(T1_1_2(:,:,j)); R1_2_1(:,:,j) = transpose(R1_1_2(:,:,j));
[R1_2_T(:,:,j), P1_2_T(:,:,j)] = tr2rt(T1_2_T); R1_T_2(:,:,j) = transpose(R1_2_T(:,:,j));
[R1_0_T(:,:,j), P1_0_T(:,:,j)] = tr2rt(T1_0_T(:,:,j));
end

% solve for theta dots
theta_dot_1 = zeros(1,50);
theta_dot_1(1,1) = 0; theta_dot_1(1,50) = 0;
theta_dot_2 = zeros(1,50);
theta_dot_1(1,1) = 0; theta_dot_1(1,50) = 0;
for i = 2:49
    theta_dot_1(:,i) = (q(1,i+1) - q(1,i-1))/(0.08*(i+1) - 0.08*(i-1));
    theta_dot_2(:,i) = (q(2,i+1) - q(2,i-1))/(0.08*(i+1) - 0.08*(i-1));
end
% solve for theta ddot
theta_ddot_1 = zeros(1,50);
theta_ddot_1(1,1) = 0; theta_ddot_1(1,50) = 0;
theta_ddot_2 = zeros(1,50);
theta_ddot_1(1,1) = 0; theta_ddot_1(1,50) = 0;
for i = 2:49
    theta_ddot_1(:,i) = (theta_dot_1(1,i+1) - theta_dot_1(1,i-1))/(0.08*(i+1) - 0.08*(i-1));
    theta_ddot_2(:,i) = (theta_dot_2(1,i+1) - theta_dot_2(1,i-1))/(0.08*(i+1) - 0.08*(i-1));
end

% positions of the center of mass of each link
PC1 = [l1/2; 0 ; 0];
PC2 = [l2/2; 0 ; 0];

m1 = 1;
% inertia of each link
IC1 = (1/12) * m1 * l1^2 * [0 0 0; 0 1 0; 0 0 1];
IC2 = (1/12) * m1 * l2^2 * [0 0 0; 0 1 0; 0 0 1];
g = 9.8;
% initial conditions
f3 = zeros(3,1); 
f3(1,1) = 10;
n3 = zeros(3,1); 
w0 = zeros(3,1);
wd0 = zeros(3,1); 
v0 = zeros(3,1); 
vd0 = [0 ; g;  0];

% Outward iteration
%initialize all variables
w1 = zeros(3,1,50);
wd1 = zeros(3,1,50);
vd1 = zeros(3,1,50);
vcd1 = zeros(3,1,50);
F1 = zeros(3,1,50);
N1 = zeros(3,1,50);
w2 = zeros(3,1,50);
wd2 = zeros(3,1,50);
vd2 = zeros(3,1,50);
vcd2 = zeros(3,1,50);
F2 = zeros(3,1,50);
N2 = zeros(3,1,50);
f2 = zeros(3,1,50);
n2 = zeros(3,1,50);
f1 = zeros(3,1,50);
n1 = zeros(3,1,50);
tau1 = zeros(1,50);
tau2 = zeros(1,50);
for i = 1:50
% i = 0
w1(:,:,i) = R1_1_0(:,:,i) * w0 + theta_dot_1(:,i)*R1_0_1(1:3,3,i);
wd1(:,:,i) = R1_1_0(:,:,i) * wd0 + R1_1_0(:,:,i) * cross(w0, theta_dot_1(:,i)*R1_0_1(1:3,3,i)) + theta_ddot_1(1,i)*R1_0_1(1:3,3,i);
vd1(:,:,i) = R1_1_0(:,:,i) * (cross(wd0, P1_0_1(:,:,i)) + cross(w0, cross(w0, P1_0_1(:,:,i))) + vd0);
vcd1(:,:,i) = cross(wd1(:,:,i),PC1) + cross(w1(:,:,i),cross(w1(:,:,i),PC1)) + vd1(:,:,i);
F1(:,:,i) = m1 * vcd1(:,:,i);
N1(:,:,i) = IC1 * wd1(:,:,i) + cross(w1(:,:,i),IC1*w1(:,:,i));
% i = 1
w2(:,:,i) = R1_2_1(:,:,i) * w1(:,:,i) + theta_dot_2(:,i)*R1_1_2(1:3,3,i);
wd2(:,:,i) = R1_2_1(:,:,i) * wd1(:,:,i) + R1_2_1(:,:,i) * cross(w1(:,:,i), theta_dot_2(:,i)*R1_1_2(1:3,3,i)) + theta_ddot_2(:,i)*R1_1_2(1:3,3,i);
vd2(:,:,i) = R1_2_1(:,:,i) * (cross(wd1(:,:,i), P1_1_2(:,:,i)) + cross(w1(:,:,i), cross(w1(:,:,i), P1_1_2(:,:,i))) + vd1(:,:,i));
vcd2(:,:,i) = cross(wd2(:,:,i),PC2) + cross(w2(:,:,i),cross(w2(:,:,i),PC2)) + vd2(:,:,i);
F2(:,:,i) = m1 * vcd2(:,:,i); 
N2(:,:,i) = IC2 * wd2(:,:,i) + cross(w2(:,:,i),IC2*w2(:,:,i));
% Inward iteration
% i = 2
f2(:,:,i) = R1_2_T(:,:,i) * f3 + F2(:,:,i);
n2(:,:,i) = N2(:,:,i) + R1_2_T(:,:,i)*n3 + cross(PC2, F2(:,:,i)) + cross(P1_2_T(:,:,i), R1_2_T(:,:,i)*f3);
% i = 1
f1(:,:,i) = R1_1_2(:,:,i) * f2(:,:,i) + F1(:,:,i);
n1 = N1 + R1_1_2(:,:,i)*n2(:,:,i) + cross(PC1, F1(:,:,i)) + cross(P1_1_2(:,:,i), R1_1_2(:,:,i)*f2(:,:,i));
tau1(:,i) = n1(:,:,i)'*[0 0 1]';
tau2(:,i) = n2(:,:,i)'*[0 0 1]';
end
tau = [tau1;tau2];
figure(4)
plot(t,tau1)
title('joint torque tau 1 vs. time')
xlabel('time (t)')
ylabel('torque (N*m)')

figure(5)
plot(t,tau2)
title('joint torque tau 2 vs. time')
xlabel('time (t)')
ylabel('torque (N*m)')

%% inertia tensor
syms m1 m2 m3
h = 0.1; w = 0.1; l = 0.1; d = 0.4;

bodyA1 = (m1/12)*[h^2 + l^2, 0, 0;
                  0, w^2 + h^2, 0;
                  0, 0, l^2 + h^2];
r = 0.01;
bodyA2 =        [(m1/12)*(3*r^2 + h^2), 0, 0;
                0, (m1/12)*(3*r^2 + h^2), 0;
                0, 0, (m1*r^2)/2];
bodyA = bodyA1 - bodyA2;
%translate
dmax = [d^2 0 0;
        0 0 0;
        0 0 0];
cmlinka = bodyA + m1*(transpose(P)*P*eye(3) - dmax);
r2 = 0.025;
h2 = 0.7;
cmlinkb = [(m2*r2^2)/2, 0, 0;
         0, (m2/12)*(3*r2^2+h2^2), 0;
         0, 0, (m2/12)*(3*r2^2+h2^2)];

bodyC = bodyA;
rotX = [1, 0, 0;
        0, cosd(-pi/4), -sind(-pi/4);
        0, sind(-pi/4),  cosd(-pi/4)];

bodyC = rotX*bodyC*transpose(rotX);
P = [-d; 0; 0];
Pc = [d; 0; 0];
cmlinkc= bodyC + m3*(transpose(Pc)*Pc*eye(3) - dmax);
cmlink = cmlinka + cmlinkb + cmlinkc

%% functions

function q = RR_robot_ik_xy(x,y, l1, l2)
c2 = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1-c2^2);
q2 = atan2(s2,c2);

q1 = atan2(y,x) - atan2(l2*s2, l1+l2*c2);
q = [q1 q2 0]';
end

