close all; clear all; clc
%% build robot
syms t1 t2 t3 t4 t5 t6 %a2 a3 d3 d4
a2 = 0.4318;
a3 = 0.0191;
d3 = 0.1254;
d4 = 0.4318;
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
a_mod =       [0, 0, a2, a3, 0, 0, -0.1];
d_mod =       [0, 0, d3, d4, 0, 0, 0.13625];
q =   [t1, t2, t3, t4, t5, t6, 0];

for i = 1:7
    L(i) = Link('revolute',  'alpha', alpha(i), 'a', a_mod(i), 'd',     d_mod(i),  'modified');
end

PUMA560 = SerialLink(L, 'name', 'PUMA560');
%% velocity propagation
% Transformations, rotations, translations
T01 = PUMA560.A(1,q);
T12 = PUMA560.A(2,q);
T23 = PUMA560.A(3,q);
T34 = PUMA560.A(4,q);
T45 = PUMA560.A(5,q);
T56 = PUMA560.A(6,q);
T6T = PUMA560.A(7,q);
T0T = PUMA560.A([1 2 3 4 5 6 7],q);
T0T = simplify(T0T);
[R01, P01] = tr2rt(T01); R10 = transpose(R01);
[R12, P12] = tr2rt(T12); R21 = transpose(R12);
[R23, P23] = tr2rt(T23); R32 = transpose(R23);
[R34, P34] = tr2rt(T34); R43 = transpose(R34);
[R45, P45] = tr2rt(T45); R54 = transpose(R45);
[R56, P56] = tr2rt(T56); R65 = transpose(R56);
[R6T, P6T] = tr2rt(T6T); RT6 = transpose(R6T);
[R0T, P0T] = tr2rt(T0T);
% Joint velocities
syms dq1 dq2 dq3 dq4 dq5 dq6
dq = [dq1 dq2 dq3 dq4 dq5 dq6];
w00 = [0;0;0];
w11 = simplify(R10*w00 + [0;0;dq1]);
w22 = simplify(R21*w11 + [0;0;dq2]);
w33 = simplify(R32*w22 + [0;0;dq3]);
w44 = simplify(R43*w33 + [0;0;dq4]);
w55 = simplify(R54*w44 + [0;0;dq5]);
w66 = simplify(R65*w55 + [0;0;dq6]);
wtt = simplify(RT6*w66 + [0;0;0]);

v0 = [0;0;0];              % velocity of frame {0} expressed in frame {0}
v01 = cross(w00,P01)+v0;   % velocity of frame {1} expressed in frame {0}
v11 = collect(R10*v01,dq); % velocity of frame {1} expressed in frame {1}
v12 = cross(w11,P12)+v11;  % velocity of frame {2} expressed in frame {1}
v22 = collect(R21*v12,dq); % velocity of frame {2} expressed in frame {2}
v23 = cross(w22,P23)+v22;  % velocity of frame {3} expressed in frame {2}
v33 = collect(R32*v23,dq); % velocity of frame {3} expressed in frame {3}
v34 = cross(w33,P34)+v33;  % velocity of frame {4} expressed in frame {3}
v44 = collect(R43*v34,dq); % velocity of frame {4} expressed in frame {4}
v45 = cross(w44,P45)+v44;  % velocity of frame {5} expressed in frame {4}
v55 = collect(R54*v45,dq); % velocity of frame {5} expressed in frame {5}
v56 = cross(w55,P56)+v55;  % velocity of frame {6} expressed in frame {5}
v66 = collect(R65*v56,dq); % velocity of frame {6} expressed in frame {6}
v6t = cross(w66,P6T)+v66;
vtt = collect(R43*v34,dq); % velocity of frame {4} expressed in frame {4}
vtt = simplify(vtt);
% Jacobian expressed in frame {4}
[JV4] = equationsToMatrix([vtt],dq);
[JW4] = equationsToMatrix([wtt],dq);
J4_VP = simplify([JV4;JW4])
% Jacobian expressed in frame {0}
JT = [R0T, zeros(3,3);zeros(3,3), R0T];
J0_VP = JT * J4_VP;
J0_VP = simplify(J0_VP)

%% singularities
% pose definition
J_red = [J4_VP(1:3,:)
         J4_VP(6,:)];
   
find_singularity = simplify(det(J_red));
