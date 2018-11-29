function [FS] = FilterState(S, M, P, C, TIME)
%% FilterState
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Filter measurements and generate a state estimate at current time
% Parameters:
%   S - Struct containing previous state estimate and covariance P
%   M - Struct containing sensor measurements at current time
%   P - Struct containing filter parameters at current time
%   C - Struct containing control input parameters at current time
%   TIME - Struct representing current time. Updates at 100Hz.
% Return Values:
%   ES - Struct containing current state estimate
%% Basic States/Matrices
dt = TIME.DT; % Update Rate

% Matrices from xdot = Ax + Bu
A = [zeros(3), eye(3); zeros(3,6)];
B = [zeros(3,6); zeros(3), eye(3)];

% Matrices from z = Cx + Du
Cul = [1,0,0; 1,-1,0; 0,1,-1];
Cm = [Cul, zeros(3); zeros(3,6)];
D = B;

% States and Controls
Xk = [S.x; S.v];
Uk = [zeros(3,1); C.a];

% Noises -- I'm gonna note that these noise values come from the sensor
% measurements, but the update rates aren't quite right... we simply assume
% this error at every point in time, for any measurement (which is a way,
% but not the only way, to do this)
x_std = 3; % 3m error from GPS
v_std = 5; % 10x the acceleration error, because we int. 10 accel samples

%% Kalman Filter Variables
Fk = expm(A*dt);
% Gk is given by: int(expm(A*s),s,0,dt)*B, but since it's a constant, I'm
% writing it in to save some silicon
Gk = [zeros(3), eye(3)./20000; zeros(3), eye(3)./100];
Qk = [x_std^2 .* eye(3), zeros(3); zeros(3); v_std^2 .* eye(3)];

% To get the measurement vectors, we stack the state with control inputs
% for a new input vector--else we can't accomodate acceleration without
% nonlinear integration of terms
Hk = [Cul, zeros(3,6); zeros(6,3), eye(3)]; 

% Z has a varied length depending on what timestep we are on, because of
% the different update rates of different components. We will propagate a
% posteriori updates only for each measurement we actually have, but
% we will propagate a priori updates for each step regardless.
if exist(M.x)
    Zk = [M.x; M.dx; M.a];
    Rk = diag([diag(Qk); 0.5; 0.5; 0.5]);
elseif exist(M.dx)
    Zk = [M.dx; M.a];
    Rk = Qk;
    Hk = Hk(2:end,:);
else
    Zk = M.a;
    Rk = x_std^2 .* eye(3);
    Hk = Hk(4:end,:);
end

%% Function
% A priori Measurements
xbar = Fk*Xk + Gk*Uk;
Pbar = Fk*S.P*Fk' + Q; % 6x6

% Measurement 
zbar = Hk*[xbar; C.a]; % State stacked on control for measurements
Hk = Hk(:,1:6); % Adjust back to correct dimensionality for real state

% A Posteriori Measurements
xhat = xbar + (Pbar*Hk') * inv(Hk*Pbar*Hk' + Rk) * (Zk - zbar);
P = Pbar - Pbar * Hk' * inv(Hk*Pbar*Hk' + R) * (Hk * Pbar);

ES.x = xhat(1:3);
ES.v = xhat(4:6);
ES.P = P;

return ES;