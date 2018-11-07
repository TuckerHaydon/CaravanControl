function [M] = GenerateSensorMeasurements(S, P, C, T)
%% GenerateSensorMeasurements
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Generate sensor measurements for the system.
% Parameters:
%   S - Struct containing true system state at current time
%   P - Struct containing sensor parameters at current time
%   C - Struct containing the zero-order hold on the last control input
%   T - Struct containing time information
% Return Values:
%   M - Struct containing system measurements at current time
% Notes:
%   1. S.x, S.v are to be the system components, where S.x has three
%      parts, and so does S.v. (one for each vehicle)
%   2. C.a is the acceleration control input on the vehicles
%   3. P.IMU, P.range, and P.GPS denote the standard deviations for each
%      component
%   4. M is the measurement matrix (also known as Z), and has M.a 
%      (3x1 accel), M.x1 (1x1 GPS measurement), and M.dx (2x1 range 
%      measurements). Measurements are only generated when a state is
%      needed.

%% Function

M.a = C.a + 0.5 .* randn(3,1); % 0.5 being the error every single sample

% Create range measurements
if mod(t,0.1) == 0
    
    M.dx = [S.x(1) - S.x(2); S.x(2) - S.x(3)];
    M.dx = M.dx + 10 .* randn(2,1);
    
end

% Create GPS measurements
if mod(t,1) == 0
    
    M.x = S.x(1) + 3.0 .* randn(1);

end

end