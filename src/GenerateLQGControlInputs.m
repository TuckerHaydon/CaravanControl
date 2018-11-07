function [U] = GenerateLQGControlInputs(S, P, TIME)
%% GenerateLQGControlInputs
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Generate contol inputs for the system using an LQG controller
% Parameters:
%   S - Struct containing estimated system state at current time
%   P - Struct containing control parameters at current time
%       - SYS - Struct containing control system parameters
%       - ref - 3x1 reference signal
%   T - Struct containing time information
% Return Values:
%   U - Struct containing control inputs at current time
%% Setup
persistent HAS_SOLVED_DARE P_DARE_MAT K_DARE_MAT R Q Qf
if isempty(HAS_SOLVED_DARE)
    % LQG input weighting matrix
    R = 10*eye(3);

    % LQG state weighting matrix
    Q = [
        zeros(1,0),      zeros(1,1),    zeros(1,8);
        zeros(2,1),    0.1 * eye(2),    zeros(2,6);
        zeros(1,3),    0.1 * eye(1),    zeros(1,5);
        zeros(5,9),      zeros(5,0),    zeros(5,0);
     ];

    % Final LQG state weighting matrix
    Qf = [
        zeros(1,0),      zeros(1,1),    zeros(1,8);
        zeros(2,1),    100 * eye(2),    zeros(2,6);
        zeros(1,3),    100 * eye(1),    zeros(1,5);
        zeros(5,9),      zeros(5,0),    zeros(5,0);
     ];
    
    % Solve the discrete-time differential riccati equation
    % https://stanford.edu/class/ee363/lectures/dlqr.pdf
    nx = size(P.SYS.A, 1);
    nu = size(P.SYS.B, 2);
    P_DARE_MAT = zeros(nx,nx,TIME.N);
    K_DARE_MAT = zeros(nu,nx,TIME.N);
    
    P_DARE_MAT(:,:,TIME.N) = Qf;
    for tdx = TIME.N:-1:2
       P_DARE_MAT(:,:,tdx-1) = Q + P.SYS.A' * P_DARE_MAT(:,:,tdx) * P.SYS.A ...
           - P.SYS.A' * P_DARE_MAT(:,:,tdx) * P.SYS.B ...
           * inv(R + P.SYS.B' * P_DARE_MAT(:,:,tdx) * P.SYS.B) * P.SYS.B' ...
           * P_DARE_MAT(:,:,tdx) * P.SYS.A;
    end
    
    for tdx = 1:1:TIME.N-1
        K_DARE_MAT(:,:,tdx) = -inv(R + P.SYS.B' * P_DARE_MAT(:,:,tdx+1) * P.SYS.B) ...
            * P.SYS.B' * P_DARE_MAT(:,:,tdx+1) * P.SYS.A;
    end
    
    HAS_SOLVED_DARE = 1;
end

%% Function
    U = 0;
%     U = K_DARE_MAT(:,:,TIME.idx) * S.x;

end