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

% States and Controls
Xk = S.x;
Uk = [zeros(3,1); C];

% Noises -- I'm gonna note that these noise values come from the sensor
% measurements, but the update rates aren't quite right... we simply assume
% this error at every point in time, for any measurement (which is a way,
% but not the only way, to do this)

%% Kalman Filter Variables
Fk = expm(A*dt);
% Gk is given by: int(expm(A*s),s,0,dt)*B, but since it's a constant, I'm
% writing it in to save some silicon
Gk = [zeros(3), eye(3)./200; zeros(3), eye(3)./10];
DTQM = [1/20 * dt^5, 1/8 * dt^4, 1/6 * dt^3; ...
        1/8 * dt^4,  1/3 * dt^3, 1/2 * dt^2; ...
        1/6 * dt^3,  1/2 * dt^2,         dt];
Qk = [DTQM./10, zeros(3); zeros(3), DTQM./20];

Hk = [Cul, zeros(3)];

% Z has a varied length depending on what timestep we are on, because of
% the different update rates of different components. We will propagate a
% posteriori updates only for each measurement we actually have, but
% we will propagate a priori updates for each step regardless.
if isfield(M,'x')
    Zk = [M.x; M.dx];
    Rk = diag([3; 1; 1]);
elseif isfield(M,'dx')
    % Attempt with inf variance
%     Zk = [0; M.dx];
%     Rk = diag([inf; 5; 5]);
    % Regular attempt
    Zk = [M.dx];
    Rk = diag([1; 1]);
    Hk = Hk(2:end,:);
end

%% Function
% A priori Measurements
xbar = Fk*Xk + Gk*Uk;
Pbar = Fk*S.P*Fk' + Qk;

% Measurement 
zbar = Hk*xbar; % State stacked on control for measurements=

% A Posteriori Measurements
Pzz = Hk*Pbar*Hk' + Rk;
W = Pbar*Hk'*inv(Pzz);
xhat = xbar + W * (Zk - zbar);
P = Pbar - W*Pzz*W';

FS.x = xhat;
FS.P = P;

end