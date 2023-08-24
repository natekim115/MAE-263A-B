clear all; close all; clc;

syms l1 l2 d4 th1 th2 th3 pi
l1 = 0.325;
l2 = 0.225;
a = [0 l1 l2 0];
alp = [0 0 0 pi];
d = [0 0 0 d4];
th = [th1 th2 th3 0];

T01 = [  cosd(th(1))               -sind(th(1))              0               a(1);
        sind(th(1))*cosd(alp(1))    cosd(th(1))*cosd(alp(1))    -sind(alp(1))     -sind(alp(1))*d(1);
        sind(th(1))*sind(alp(1))    cosd(th(1))*sind(alp(1))    cosd(alp(1))      cosd(alp(1))*d(1);
        0                       0                       0               1 ];

T12 = [  cosd(th(2))               -sind(th(2))              0               a(2);
        sind(th(2))*cosd(alp(2))    cosd(th(2))*cosd(alp(2))    -sind(alp(2))     -sind(alp(2))*d(2);
        sind(th(2))*sind(alp(2))    cosd(th(2))*sind(alp(2))    cosd(alp(2))      cosd(alp(2))*d(2);
        0                       0                       0               1 ];

T23 = [  cosd(th(3))               -sind(th(3))              0               a(3);
        sind(th(3))*cosd(alp(3))    cosd(th(3))*cosd(alp(3))    -sind(alp(3))     -sind(alp(3))*d(3);
        sind(th(3))*sind(alp(3))    cosd(th(3))*sind(alp(3))    cosd(alp(3))      cosd(alp(3))*d(3);
        0                       0                       0               1 ];

T34 = [  cosd(th(4))               -sind(th(4))              0               a(4);
        sind(th(4))*cosd(alp(4))    cosd(th(4))*cosd(alp(4))    -sind(alp(4))     -sind(alp(4))*d(4);
        sind(th(4))*sind(alp(4))    cosd(th(4))*sind(alp(4))    cosd(alp(4))      cosd(alp(4))*d(4);
        0                       0                       0               1 ];


T04 = T01*T12*T23*T34;

l1 = 0.325;
l2 = 0.225;
q = [pi/3 pi/2, 0, 0.150];

% Modified DH parameters
alpha = [0,    0,    0,    pi];
a =     [0,    l1,   l2,   0];
d =     [0,    0,    0,    q(4)];
%th =    [th1,  th2,  th3,  0];
th = [q(1), q(2), q(3), 0]

feeder_R = eye(3,3);
feeder_T = [0.3953;
            0;
            0.083;]

feeder = eye(4,4);
feeder(1,4) = 0.3953;
feeder(2,4) = 0;
feeder(3,4) = 0.083;

c_one = [ 0 1 0 0.3653; 
         -1 0 0 0.045;
          0 0 1 0.083;
          0 0 0 1; ]

c_one_R = [ 0 1 0;
           -1 0 0;
            0 0 1;]

c_one_T = [ 0.3653;
            0.045;
            0.083;]

c_two = eye(4,4);
c_two(1,4) = 0.2753;
c_two(2,4) = 0.045;
c_two(3,4) = 0.083;;

c_two_R = eye(3,3);
c_two_T = [0.2753;
            0.045;
            0.083;]

c_three = [ 0 -1 0 0.2753;
              1 0 0 -0.045;
              0 0 1 0.083;
              0 0 0 1]

c_three_R = [ 0 -1 0;
              1 0 0;
              0 0 1;]

c_three_T = [0.2753;
             -0.045;
             0.083;]

c_four = [-1 0 0 0.3653;
             0 1 0 -0.045;
             0 0 1 0.083;
             0 0 0 1;]

c_four_R = [-1 0 0;
             0 1 0;
             0 0 1;]
            
c_four_T = [ 0.3653;
            -0.045;
             0.083;]

%%
% feeder 
[f_th1, f_th2, f_th3] = inverse(feeder, l1, l2, q)
%%
% corner 1
[c1_th1, c1_th2, c1_th3] = inverse(c_one, l1, l2, q)

%%
% corner 2
[c2_th1, c2_th2, c2_th3] = inverse(c_two, l1, l2, q)

%%
% corner 3
[c3_th1, c3_th2, c3_th3] = inverse(c_three, l1, l2, q)
%%
% corner 4
[c4_th1, c4_th2, c4_th3] = inverse(c_four, l1, l2, q)


%% build robot

i = 1; L(i) = Link('revolute',  'alpha', alp(i), 'a', a(i), 'd',     d(i),  'modified');
i = 2; L(i) = Link('revolute',  'alpha', alp(i), 'a', a(i), 'd',     d(i),  'modified');
i = 3; L(i) = Link('revolute',  'alpha', alp(i), 'a', a(i), 'd',     d(i),  'modified');
i = 4; L(i) = Link('prismatic', 'alpha', alp(i), 'a', a(i), 'theta', th(i), 'modified');
SCARA = SerialLink(L, 'name', 'SCARA');
% run forward kinematics
% SCARA_FK = SCARA.fkine(th);

%% trajectory for feeder to c1

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = feeder_T; quat1 = rotm2quat(feeder_R);
xyz2 = c_one_T; quat2 = rotm2quat(c_one_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(feeder_R, c_one_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = feeder_T; p2 = c_one_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end

%% Trajectory for c1 to feeder

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = c_one_T; quat1 = rotm2quat(c_one_R);
xyz2 = feeder_T; quat2 = rotm2quat(feeder_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(c_one_R, feeder_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = c_one_T; p2 = feeder_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end


%% Trajectory for feeder to c2

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = feeder_T; quat1 = rotm2quat(feeder_R);
xyz2 = c_two_T; quat2 = rotm2quat(c_two_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(feeder_R, c_two_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = feeder_T; p2 = c_two_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end


%% Trajectory for c2 to feeder

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = c_two_T; quat1 = rotm2quat(c_two_R);
xyz2 = feeder_T; quat2 = rotm2quat(feeder_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(c_two_R, feeder_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = c_two_T; p2 = feeder_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end
%% Trajectory for feeder to c3

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = feeder_T; quat1 = rotm2quat(feeder_R);
xyz2 = c_three_T; quat2 = rotm2quat(c_three_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(feeder_R, c_three_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = feeder_T; p2 = c_three_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end


%% Trajectory for c3 to feeder

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = c_three_T; quat1 = rotm2quat(c_three_R);
xyz2 = feeder_T; quat2 = rotm2quat(feeder_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(c_three_R, feeder_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = c_three_T; p2 = feeder_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end
%% Trajectory for feeder to  c4

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = feeder_T; quat1 = rotm2quat(feeder_R);
xyz2 = c_four_T; quat2 = rotm2quat(c_four_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(feeder_R, c_four_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = feeder_T; p2 = c_four_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end


%% Trajectory from c4 to feeder

t0 = 1;                 tf = 4;      % time schedules

tb = 0.75;

xyz1 = c_four_T; quat1 = rotm2quat(c_four_R);
xyz2 = feeder_T; quat2 = rotm2quat(feeder_R);
pose1 = [xyz1; quat1']; pose2 = [xyz2; quat2'];
    
n_intervals = 20;
% calculate linear parabolic blend for total transformation
% translation
[xyz, xyzd, xyzdd, t] = traj_linear_wpb_tb_vector(t0, tf, xyz1, xyz2, tb, n_intervals);
% rotation
rot_interpolation = rottraj(c_four_R, feeder_R, [t0 tf], t(1,:));


q = zeros(4,n_intervals);
for i = 1:n_intervals
    T_temp = rt2tr(rot_interpolation(:,:,i), xyz(:,i));
    q(:,i) = inverse(T_temp, l1, l2, q);
end

% Package viapoints
p1 = c_four_T; p2 = feeder_T;
viapoints = [p1 p2];

ws = [-1 1 -1 1 -1 1]/2;

% Animate trajectory
hold on;
for i = 1:n_intervals
    SCARA.plot(q(:,i)', 'workspace', ws);
    title(['Trajectory from feeder to corner 1'])
end

%% inverse

function [th1, th2, th3] = inverse(T04, l1, l2, q)
px = T04(1,4);
py = T04(2,4);

s2 = (l1^2 + l2^2 - px^2 - py^2)/(2*l1*l2);
c2 = sqrt(1-s2^2);
% find theta 2
th2 = atan2(-s2, c2);

c = l1 - l2*s2;
% find theta 1
th1 = atan2(-sqrt(px^2 + py^2 - c^2), c) + atan2(py, px);

r11 = T04(1,1);
r21 = T04(2,1);

% find theta 3
th3 = atan2(-r11, r21) - th1 - th2;

% find d
% d = highest point of robot with tool - inputted height
d = 270 - q(4);
end