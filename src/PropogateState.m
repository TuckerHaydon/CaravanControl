function [NS] = PropogateState(S, U, P, TIME)
%% PropogateState
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Propogate the system forward with control inputs
% Parameters:
%   S - Vector containing nominal true state at current time
%   U - Struct containing control inputs at current time
%   P - Struct containing state propogation parameters at current time
%       - SYS - struct containing nominal system parameters
%   TIME - Struct containing time information
% Return Values:
%   NS - Struct containing next true state

%% Setup
persistent HAS_CONVERTED_SYSTEM DISCRETE_SYS
if isempty(HAS_CONVERTED_SYSTEM)
    SYS = ss(P.SYS.A, P.SYS.B, P.SYS.C, P.SYS.D);
    DISCRETE_SYS = c2d(SYS, TIME.DT);
    HAS_CONVERTED_SYSTEM = 1; 
end

%% Function

NS = DISCRETE_SYS.A * S + DISCRETE_SYS.B * U;

end