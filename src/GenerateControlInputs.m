function [U] = GenerateControlInputs(S, P, TIME)
%% GenerateControlInputs
% Author: 
%   Tucker Haydon, Connor Brashar
% Description:
%   Generate contol inputs for the system.
% Parameters:
%   S - Struct containing estimated system state at current time
%   P - Struct containing control parameters at current time
%       - SYS - Struct containing nominal system parameters
%       - ref - 3x1 reference signal
%       - control_law - function pointer to desired controller
%   TIME - struct containing time information
% Return Values:
%   U - Struct containing control inputs at current time

%% Setup
% Restructure system to conform to control definitions
persistent HAS_CONVERTED_SYSTEM A B C D T DISCRETE_SYS
if isempty(HAS_CONVERTED_SYSTEM)

    % Stacked system
    A_s = [
        P.SYS.A,    zeros(6,3);
        zeros(3,6), zeros(3,3);
        ];
    
    B_s = [
        P.SYS.B;
        zeros(3,3);
    ];

    C_s = [
        P.SYS.C,    zeros(6,3);
        zeros(3,6), eye(3);
    ];

    D_s = [
        P.SYS.D;
        zeros(3,3);
    ];

    T = [
      [
        1  0  0 0 0 0  0  0  0;
        1 -1  0 0 0 0 -1  0  0;
        0  1 -1 0 0 0  0 -1  0;
        0  0  0 1 0 0  0  0 -1;
      ];
      zeros(5,4), eye(5);  
    ];

    A = T * A_s * inv(T);
    B = T * B_s;
    C = C_s * inv(T);
    D = D_s;
    
    SYS = ss(A, B, C, D);
    DISCRETE_SYS = c2d(SYS, TIME.DT);
    
    HAS_CONVERTED_SYSTEM = 1; 
end

%% Function
% Use control definition of system
P.SYS.A = DISCRETE_SYS.A;
P.SYS.B = DISCRETE_SYS.B;
P.SYS.C = DISCRETE_SYS.C;
P.SYS.D = DISCRETE_SYS.D;

% Use the control definition of state
S_s = T * [
    S;
    P.ref;
];

U = P.control_law(S_s, P, TIME);

end