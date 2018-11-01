%% Authors: Tucker Haydon, Connor Brashar

%% System initialization
% Define intial state, initial estimate, system propogation functions, 
% global variables, time, sensor params, control algorithm type, etc.


%% Main loop
    %% Generate sensor measurements
    % Given the current true state, generate sensor measurements
    [sensor_measurements] = ...
        GenerateSensorMeasurements(true_state, sensor_params, time);


    %% Filter sensor measurements
    % Feed sensor measurements into kalman filter to generate a state estimate
    [estimated_state] = ...
        FilterState(estimated_state, sensor_measurements, filter_parameters, time);


    %% Generate control inputs
    % Given current filter estimate and specified state, generate control
    % inputs. Control input algorithm may change. Consider PID, input
    % saturation, LQG control.
    [control_inputs] = ...
        GenerateControlInputs(estimated_state, control_parameters, time);


    %% Propogate true state
    % Propogate the true state of the system with the generated control inputs.
    [true_state] = ...
        PropogateState(true_state, control_inputs, state_parameters, time);
    