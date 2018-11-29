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
% Note from Connor: We need to pass the control accelerations into this.
% It's the only way to get a linear estimate of the acceleration without
% differentiating velocity, which would not be a linear KF. I've got it in
% the FilterState function.
% Other notes:
%   1. The KF noise has been... sketchily made. We should discuss. Having
%   done it with working at MITRE, a true model for the measurement noises
%   for this model is less trivial than we may want it to be. I've made
%   some simplifying assumptions for our KF that prevent accel meas error
%   from affecting vel error, or pos, but since they're all technically
%   related, this is technically untrue.
%   2. In order to accomodate acceleration measurements, I create a
%   separate stacked state for the measurement update, which was a blast to
%   write, and I think should work. Looks hawt.
%   3. I can't actually test this code without generating measurements.
%   There shouldn't be much in the way of error *knocks on wood* but I'm
%   sure there's something that'll need debugging when we have the ability
%   to feed states in.


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