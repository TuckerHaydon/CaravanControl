function [FS] = FilterState(S, M, P, C, t)
%% FilterState
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Filter measurements and generate a state estimate at current time
% Parameters:
%   S - Struct containing previous state estimate
%   M - Struct containing sensor measurements at current time
%   P - Struct containing filter parameters at current time
%   U - Struct containing control input parameters at current time
%   t - scalar representing current time. Updates at 100Hz.
% Return Values:
%   ES - Struct containing current state estimate
%% States and Matrices
A = [zeros(3), eye(3); zeros(3,6)];

B = [zeros(3,6); zeros(3), eye(3)];

Cul = [1,0,0; 1,-1,0; 0,1,-1];
Cm = [Cul, zeros(3); zeros(3,6)];

D = B;

X = [S.x; S.v];
U = [zeros(3,1); C.a];

% Z has a varied length depending on what timestep we are on
if exist(M.x)
    Z = [M.x; M.dx; M.a];
elseif exist(M.dx)
    Z = [M.dx; M.a];
else
    Z = M.a;
end

%% Function


 
 
 
 end