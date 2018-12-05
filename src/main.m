%% Authors: Tucker Haydon, Connor Brashar
clc; clear all; close all;
tic;
%% System initialization
% Define intial state, initial estimate, system propogation functions, 
% global variables, time, sensor params, control algorithm type, etc.

% Time information
% Clock progresses at 100 ticks/second
TIME.T_MAX_SECONDS = 10*60;
TIME.DT = 1/10;
TIME.N = TIME.T_MAX_SECONDS / TIME.DT;
TIME.t_vec = TIME.DT:TIME.DT:TIME.T_MAX_SECONDS;
TIME.idx = 1;
TIME.now = TIME.t_vec(TIME.idx);

% sensor_params
sensor_params.range_std = 5;
sensor_params.GPS_std = 3;
% Update Rates (s)
sensor_params.range_Update_Rate = 1/10;
sensor_params.GPS_Update_Rate = 1;

% Initial state
% [x1, x2, x3, v1, v2, v3] in meters
x_0 = [200, 75, 0, 30, 27, 25]';

% History buffers for plotting
state_history = zeros(size(x_0, 1), TIME.N);
state_history(:, TIME.idx) = x_0;
est_error_history = state_history;
cov_history = zeros(size(x_0, 1), size(x_0,1), TIME.N);

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
    %% First step, no control
    if idx == 1
        control_inputs = zeros(3,1);
        state_estimate.x = zeros(6,1);
        state_estimate.P = 10*eye(6);
    end
    
    %% Generate sensor measurements
    % Given the current true state, generate sensor measurements
    [sensor_measurements] = ...
        GenerateSensorMeasurements(state, sensor_params, TIME);


    %% Filter sensor measurements
    % Feed sensor measurements into kalman filter to generate a state estimate
    state_estimate = ...
        FilterState(state_estimate, sensor_measurements, sensor_params, control_inputs, TIME);
    est_error_history(:,idx) = state_estimate.x - state;
    cov_history(:,:,idx) = state_estimate.P;


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
h=figure();
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
suptitle('States over Time');
saveas(h, 'states_over_time.jpg')

% Plot Estimator Error
h=figure();
hold on;

subplot(2, 3, 1);
plot(TIME.t_vec, est_error_history(1,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{1}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta x_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 2);
plot(TIME.t_vec, est_error_history(2,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{2}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta x_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 3);
plot(TIME.t_vec, est_error_history(3,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{3}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta x_{3}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 4);
plot(TIME.t_vec, est_error_history(4,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{1}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta v_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 5);
plot(TIME.t_vec, est_error_history(5,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{2}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta v_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 6);
plot(TIME.t_vec, est_error_history(6,:));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{3}$', 'Interpreter','latex');
ylim([-40,40]);
title('$\Delta v_{3}$ vs Time', 'Interpreter','latex');
grid on;

hold off;

suptitle('Estimation Error over time');
saveas(h, 'estimation_error_over_time.jpg')

% Plot Estimator Covariances
h = figure();
hold on;

subplot(2, 3, 1);
plot(TIME.t_vec, squeeze(cov_history(1,1,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{1}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta x_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 2);
plot(TIME.t_vec, squeeze(cov_history(2,2,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{2}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta x_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 3);
plot(TIME.t_vec, squeeze(cov_history(3,3,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{3}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta x_{3}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 4);
plot(TIME.t_vec, squeeze(cov_history(4,4,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{1}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta v_{1}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 5);
plot(TIME.t_vec, squeeze(cov_history(5,5,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{2}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta v_{2}$ vs Time', 'Interpreter','latex');
grid on;

subplot(2, 3, 6);
plot(TIME.t_vec, squeeze(cov_history(6,6,:)));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta v_{3}$', 'Interpreter','latex');
ylim([-10,10]);
title('$\Delta v_{3}$ vs Time', 'Interpreter','latex');
grid on;

hold off;

suptitle('Estimation Variance over time');
saveas(h, 'estimation_variance_over_time.jpg')

% Plot state errors
h = figure();
hold on;

subplot(1,3,1)
plot(TIME.t_vec, state_history(1,:) - state_history(2,:) - reference_signal(1));
xlabel('Time (s)', 'Interpreter','latex');
ylabel('$\Delta x_{12}$', 'Interpreter','latex');
title('$\Delta x_{12}$ vs Time', 'Interpreter','latex');
ylim([-20, 20])
grid on;

subplot(1,3,2)
plot(TIME.t_vec, state_history(2,:) - state_history(3,:) - reference_signal(2));
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

suptitle('Tracking Error Over Time');
saveas(h, 'tracking_error_over_time.jpg')


%% End Program
toc;