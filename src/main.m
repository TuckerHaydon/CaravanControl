%% Authors: Tucker Haydon, Connor Brashar

%% System initialization
% Define intial state, initial estimate, system propogation functions, 
% global variables, time, sensor params, control algorithm type, etc.

% Time information
% Max time is half an hour
% Clock progresses at 100 ticks/second
TIME.T_MAX_SECONDS = 0.5*60*60;
TIME.DT = 1/100;
TIME.N = T_MAX_SECONDS / DT;
TIME.t_vec = linspace(0,T_MAX_SECONDS, N);
TIME.idx = 1;
TIME.now = TIME.t_vec(TIME.idx);

% Initial state
% [x1, x2, x3, v1, v2, v3] in meters
x_0 = [2000, 500, 300, 30, 25, 20]';

% Nominal system description
SYS.A = [
  zeros(3,3), eye(3);
  zeros(3,3), zeros(3,3);
];

SYS.B = [
  zeros(3,3); 
  eye(3);
];

SYS.C = [
  [1, 0, 0; 1, -1, 0; 0, 1, -1], zeros(3,3);
  zeros(3,3),                    zeros(3,3);
];

SYS.D = [
  zeros(3,3);
  eye(3);
];

% Reference signal
% [Delta_x_12, Delta_x_23, v_1]
reference_signal = [5, 5, 30];

% Control law
control_law = @GenerateLQGControlInputs;


%% Main loop
for idx = 1:1:length(TIME.t_vec)
    %% Generate sensor measurements
    % Given the current true state, generate sensor measurements
%     [sensor_measurements] = ...
%         GenerateSensorMeasurements(true_state, sensor_params, TIME);


    %% Filter sensor measurements
    % Feed sensor measurements into kalman filter to generate a state estimate
%     [estimated_state] = ...
%         FilterState(estimated_state, sensor_measurements, filter_parameters, TIME);


    %% Generate control inputs
    % Given current filter estimate and specified state, generate control
    % inputs. Control input algorithm may change. Consider PID, input
    % saturation, LQG control.
%     control_parameters.SYS = SYS;
%     control_parameters.ref = reference_signal;
%     control_parameters.control_law = control_law;
%     estimated_state = 0;
%     [control_inputs] = ...
%         GenerateControlInputs(estimated_state, control_parameters, TIME);


    %% Propogate true state
    % Propogate the true state of the system with the generated control inputs.
%     [true_state] = ...
%         PropogateState(true_state, control_inputs, state_parameters, TIME);
    
    %% Update loop params
    TIME.idx = TIME.idx + 1;
    if TIME.idx < TIME.N
        TIME.now = TIME.t_vec(TIME.idx);
    end
end