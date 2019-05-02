%% System parameters
m_w     =   0.028;      % kg - wheel mass
m_b     =   1.114;      % kg - mass of the rest
J_w     =   2.8857e-5;  % kg m^2 - moment of inertia of the wheel
R       =   0.043;      % m - wheel radius
l       =   0.06721;    % m - center of mass distance
J_bz    =   8922223.33e-9 + m_b*l^2;     % kg m^2 - the full platform moment of inertia
g       =   9.81;       % m/s^2 - gravity acceleration

%% Sampling time
Ts      =   0.005;   %s

%% Constraints
MAX_ACCEL   =   inf;    % m/s^2
MAX_VELOCITY   =   2;    % m/s
MAX_RATE   =   3;    % m/s

%% Quantisation of stepper wheel angle - position x
X_k = (2 * pi / 200) * R;

%% System model linearisation
sys = linmod('nonlinear_model');
G = ss(sys.a,sys.b,sys.c,sys.d);
Gd = c2d(G,Ts);
%% LQR controller
% weight matrices
Qd = diag([100,2,5,5]);
Rd = 1;
% optimal controller gain
Kr = -lqrd(G.a,G.b,Qd,Rd,Ts);
% line of code to coppy to LQR controller function
disp(strcat('float Kr[4] = {',num2str(Kr(1)),',',num2str(Kr(2)),',',num2str(Kr(3)),',',num2str(Kr(4)),'};'))

%% PD stqbilisation controller 
G_PD = pid(-17.5,0,-2.5,0.003,Ts);
% linear system model
G_w = -tf([J_bz 0 -m_b*g*l],[1 0]);
% closed loop system model
G_cl = feedback(Gd,G_PD);