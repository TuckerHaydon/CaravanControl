%% Authors: Tucker Haydon, Connor Brashar
clc; clear all; close all;
tic
%% System initialization
% Define intial state, initial estimate, system propogation functions, 
% global variables, time, sensor params, control algorithm type, etc.

% Time information
% Clock progresses at 100 ticks/second
TIME.T_MAX_SECONDS = 5*60;
TIME.DT = 1/100;
TIME.N = TIME.T_MAX_SECONDS / TIME.DT;
TIME.t_vec = linspace(0,TIME.T_MAX_SECONDS, TIME.N);
TIME.idx = 1;
TIME.now = TIME.t_vec(TIME.idx);

% Initial state
% [x1, x2, x3, v1, v2, v3] in meters
x_0 = [200, 75, 0, 30, 27, 25]';

% History buffers for plotting
state_history = zeros(size(x_0, 1), TIME.N);
state_history(:, TIME.idx) = x_0;

input_history = zeros(3, TIME.N);

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
reference_signal = [5, 5, 30]';

% Control law
control_law = @GenerateLQGControlInputs;


%% Main loop
state = x_0;
for idx = 1:1:TIME.N
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
    control_parameters.SYS = SYS;
    control_parameters.ref = reference_signal;
    control_parameters.control_law = control_law;
    [control_inputs] = ...
        GenerateControlInputs(state, control_parameters, TIME);


    %% Propogate true state
    % Propogate the true state of the system with the generated control inputs.
    state_parameters.SYS = SYS;
    [state] = ...
        PropogateState(state, control_inputs, state_parameters, TIME);
    
    
    %% Update loop params
    TIME.idx = TIME.idx + 1;
    if TIME.idx <= TIME.N
        TIME.now = TIME.t_vec(TIME.idx);
        state_history(:, TIME.idx) = state;
        input_history(:, TIME.idx) = control_inputs;
    end
end

%% Plotting
% Plot states
figure();
hold on;

subplot(2, 3, 1);
plot(TIME.t_vec, state_history(1,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$x_{1}$', 'Interpreter','latex');
title('$x_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 2);
plot(TIME.t_vec, state_history(2,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$x_{2}$', 'Interpreter','latex');
title('$x_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 3);
plot(TIME.t_vec, state_history(3,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$x_{3}$', 'Interpreter','latex');
title('$x_{3}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 4);
plot(TIME.t_vec, state_history(4,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$v_{1}$', 'Interpreter','latex');
title('$v_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 5);
plot(TIME.t_vec, state_history(5,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$v_{2}$', 'Interpreter','latex');
title('$v_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 6);
plot(TIME.t_vec, state_history(6,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$v_{3}$', 'Interpreter','latex');
title('$v_{3}$ vs Time', 'Interpreter','latex');
grid on;

hold off;




% Plot state errors
figure();
hold on;

subplot(1,3,1)
plot(TIME.t_vec, state_history(1,:) - state_history(2,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{12}$', 'Interpreter','latex');
title('$\Delta x_{12}$ vs Time', 'Interpreter','latex');
ylim([-20, 20])
grid on;

subplot(1,3,2)
plot(TIME.t_vec, state_history(2,:) - state_history(3,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{23}$', 'Interpreter','latex');
title('$\Delta x_{23}$ vs Time', 'Interpreter','latex');
ylim([-20, 20])
grid on;

subplot(1,3,3)
plot(TIME.t_vec, state_history(4,:) - reference_signal(3));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{1}$', 'Interpreter','latex');
title('$\Delta v_{1}$ vs Time', 'Interpreter','latex');
ylim([-20, 20])
grid on;

hold off;


%% End Program
toc