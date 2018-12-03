function [M] = GenerateSensorMeasurements(S, P, T)
%% GenerateSensorMeasurements
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Generate sensor measurements for the system.
% Parameters:
%   S - Struct containing true system state at current time
%   P - Struct containing sensor parameters at current time
%   T - Struct containing time information
% Return Values:
%   M - Struct containing system measurements at current time
% Notes:
%   1. S.x, S.v are to be the system components, where S.x has three
%      parts, and so does S.v. (one for each vehicle)
%   2. P.range, and P.GPS denote the standard deviations for each
%      component
%   4. M is the measurement matrix (also known as Z), and has M.x1 (1x1 GPS
%      measurement), and M.dx (2x1 range
%      measurements). Measurements are only generated when a state is
%      needed.

%% Function
t = T.now;
eps = 1e-4;

% Create range measurements
if mod(t,P.range_Update_Rate) <= eps
    
    M.dx = [S(1) - S(2); S(2) - S(3)];
    M.dx = M.dx + P.range_std .* randn(2,1);
    
end

% Create GPS measurements
if mod(t,P.GPS_Update_Rate) <= eps
    
    M.x = S(1) + P.GPS_std .* randn(1);

end

end