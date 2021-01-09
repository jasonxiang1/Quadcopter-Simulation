%% Introduction
% Main function for 16-665 Air Mobility project
% Last updated: October 2019

% Usage: main takes in a question number and executes all necessary code to
% construct a trajectory, plan a path, simulate a quadrotor, and control
% the model. Possible arguments: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9. THE
% TAS WILL BE RUNNING YOUR CODE SO PLEASE KEEP THIS MAIN FUNCTION CALL 
% STRUCTURE AT A MINIMUM.

% Requirements: no additional packages are needed to run this program as
% is. 

% Version: this framework was made using MATLAB R2018b. Should function
% as expected with most recent versions of MATLAB.

function [] = main(question)

clc
close all;

%% Set up quadrotor physical parameters

params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',           0.1103, ...
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ...
    'moment_scale',           1.3719e-10, ...
    'motor_constant',        36.5, ...
    'rpm_min',             3000, ...
    'rpm_max',            20000, ...
    'inertia',            diag([0.0033 0.0033 0.005]),...
    'COM_vertical_offset',                0.05);

%% Get the waypoints for this specific question

[waypoints, waypoint_times] = lookup_waypoints(question,0,0,0);
% waypoints are of the form [x, y, z, yaw]
% waypoint_times are the seconds you should be at each respective waypoint,
% make sure the simulation parameters below allow you to get to all points

%% Set the simualation parameters

time_initial = 0; 
time_final = 10;
time_step = 0.005; % sec
% 0.005 sec is a reasonable time step for this system

time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

%% Create the state vector
state = zeros(16,1);

% Populate the state vector with the first waypoint (assumes that robot is
% at the first waypoint at the initial time.
state(1)  = waypoints(1,1); %x
state(2)  = waypoints(2,1); %y
state(3)  = waypoints(3,1); %z
state(4)  =  0;        %xdot
state(5)  =  0;        %ydot
state(6)  =  0;        %zdot
state(7) =   0;         %phi
state(8) =   0;         %theta
state(9) =   waypoints(4,1); %psi
state(10) =  0;         %phidot 
state(11) =  0;         %thetadot
state(12) =  0;         %psidot
state(13:16) =  0;      %rpm

%% Create a trajectory consisting of desired state at each time step

% Some aspects of this state we can plan in advance, some will be filled
% in during the loop
trajectory_matrix = trajectory_planner(question, waypoints, max_iter, waypoint_times, time_step);
% [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

%% Create a matrix to hold the actual state at each time step

actual_state_matrix = zeros(15, 1);  
% [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

%% Create a matrix to hold the actual desired state at each time step

% Need to store the actual desired state for acc, omega dot and omega as
% it will be updated by the controller
actual_desired_state_matrix = zeros(15, 1);  

%declare fsm state variable at zero to start
%this indicates takeoff start
fsm_state = 0;

%declare blank total time vector
%will append to the time as model runs through states
tot_time_vect = [];
takeoff_confirm = 0;
landing_confirm = 0;

if question == 4
    while fsm_state ~= 3
        switch fsm_state
            case 0 %takeoff case
                %initialize waypoints for takeoff
                [temp_waypoints,temp_waypoint_times] = lookup_waypoints(0,actual_desired_state_matrix(1,end),actual_desired_state_matrix(2,end),actual_desired_state_matrix(3,end));
                % Populate the state vector with the first waypoint (assumes that robot is
                % at the first waypoint at the initial time.
                state(1)  = temp_waypoints(1,1); %x
                state(2)  = temp_waypoints(2,1); %y
                state(3)  = temp_waypoints(3,1); %z
                state(4)  =  0;        %xdot
                state(5)  =  0;        %ydot
                state(6)  =  0;        %zdot
                state(7) =   0;         %phi
                state(8) =   0;         %theta
                state(9) =   temp_waypoints(4,1); %psi
                state(10) =  0;         %phidot 
                state(11) =  0;         %thetadot
                state(12) =  0;         %psidot
                state(13:16) =  0;      %rpm            
                %set times (for takeoff make the time between 0 and 1)
                temp_time_initial = 0+time_step;
                temp_time_iter = time_step;
                temp_time_final = 3.2;
                temp_time_vect = temp_time_initial:temp_time_iter:temp_time_final;
                temp_max_iter = length(temp_time_vect);
                %create trajectory matrix from values
                temp_trajectory_matrix = trajectory_planner(0,temp_waypoints,temp_max_iter,temp_waypoint_times,temp_time_iter);
                for iter = 1:temp_max_iter-1
                    %set current state
                    current_state.pos = state(1:3);
                    current_state.vel = state(4:6);
                    current_state.rot = state(7:9);
                    current_state.omega = state(10:12);
                    current_state.rpm = state(13:16);
                    %set desired state
                    desired_state.pos = temp_trajectory_matrix(1:3,iter);
                    desired_state.vel = temp_trajectory_matrix(4:6,iter);
                    desired_state.rot = temp_trajectory_matrix(7:9,iter);
                    desired_state.omega = temp_trajectory_matrix(10:12,iter);
                    desired_state.acc = temp_trajectory_matrix(13:15,iter);
                    %set time increment to pass into ode45 function
                    timeint = time_vec(iter:iter+1);
                    %run ode45 function
                    [tsave,xsave] = quadcopter_model(0);
                    state    = xsave(end, :)';
                    acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));
                    actual_desired_state_matrix = [actual_desired_state_matrix, [desired_state.pos;desired_state.vel;desired_state.rot;desired_state.omega;desired_state.acc] ];
                    actual_state_matrix = [actual_state_matrix, [state(1:12);acc]];
                    %convert fsm state to hover to stabilize

                end
                fsm_state = 9;
                tot_time_vect = [tot_time_vect temp_time_vect];

            case 9 %hover state -> wait for errors to be low at a certain point
                %initialize waypoints for takeoff
                [temp_waypoints,temp_waypoint_times] = lookup_waypoints(9,actual_desired_state_matrix(1,end),actual_desired_state_matrix(2,end),actual_desired_state_matrix(3,end));
                %assume that state is maintained from the previous case
                %set times (for takeoff make the time between 0 and 1)
                temp_time_initial = 0+time_step;
                temp_time_iter = time_step;
                temp_time_final = 1;
                temp_time_vect = temp_time_initial:temp_time_iter:temp_time_final;
                temp_max_iter = length(temp_time_vect);            
                %create trajectory matrix from values
                %use the same trajectory planner as 0
                temp_trajectory_matrix = trajectory_planner(0,temp_waypoints,temp_max_iter,temp_waypoint_times,temp_time_iter);
                %run for loop for model
                
                for iter = 1:temp_max_iter-1
                    %set current state
                    current_state.pos = state(1:3);
                    current_state.vel = state(4:6);
                    current_state.rot = state(7:9);
                    current_state.omega = state(10:12);
                    current_state.rpm = state(13:16);
                    %set desired state
                    desired_state.pos = temp_trajectory_matrix(1:3,iter);
                    desired_state.vel = temp_trajectory_matrix(4:6,iter);
                    desired_state.rot = temp_trajectory_matrix(7:9,iter);
                    desired_state.omega = temp_trajectory_matrix(10:12,iter);
                    desired_state.acc = temp_trajectory_matrix(13:15,iter);
                    %set time increment to pass into ode45 function
                    timeint = time_vec(iter:iter+1);
                    %run ode45 function
                    [tsave,xsave] = quadcopter_model(0);
                    state    = xsave(end, :)';
                    acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));
                    actual_desired_state_matrix = [actual_desired_state_matrix, [desired_state.pos;desired_state.vel;desired_state.rot;desired_state.omega;desired_state.acc] ];
                    actual_state_matrix = [actual_state_matrix, [state(1:12);acc]];
                    %convert fsm state to hover to stabilize

                end
                if takeoff_confirm == 0
                    fsm_state = 2;
                    takeoff_confirm = 1;
                elseif landing_confirm == 1
                    fsm_state = 3;
                else
                    fsm_state = 8;
                end
                tot_time_vect = [tot_time_vect temp_time_vect(1:end-1)];

            case 2
                %tracking state
                %use waypoint state 2 as an example
                %initialize waypoints for takeoff
                [temp_waypoints,temp_waypoint_times] = lookup_waypoints(4,actual_desired_state_matrix(1,end),actual_desired_state_matrix(2,end),actual_desired_state_matrix(3,end));            
                %assume that state is maintained from the previous case
                %set times (for takeoff make the time between 0 and 1)
                temp_time_initial = 0+time_step;
                temp_time_iter = time_step;
                temp_time_final = 8;
                temp_time_vect = temp_time_initial:temp_time_iter:temp_time_final;
                temp_max_iter = length(temp_time_vect);             
                %create trajectory matrix from values
                temp_trajectory_matrix = trajectory_planner(0,temp_waypoints,temp_max_iter,temp_waypoint_times,temp_time_iter);
                %run for loop for model            
                for iter = 1:temp_max_iter-1
                    %set current state
                    current_state.pos = state(1:3);
                    current_state.vel = state(4:6);
                    current_state.rot = state(7:9);
                    current_state.omega = state(10:12);
                    current_state.rpm = state(13:16);
                    %set desired state
                    desired_state.pos = temp_trajectory_matrix(1:3,iter);
                    desired_state.vel = temp_trajectory_matrix(4:6,iter);
                    desired_state.rot = temp_trajectory_matrix(7:9,iter);
                    desired_state.omega = temp_trajectory_matrix(10:12,iter);
                    desired_state.acc = temp_trajectory_matrix(13:15,iter);
                    %set time increment to pass into ode45 function
                    timeint = time_vec(iter:iter+1);
                    %run ode45 function
                    [tsave,xsave] = quadcopter_model(2);
                    state    = xsave(end, :)';
                    acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));
                    actual_desired_state_matrix = [actual_desired_state_matrix, [desired_state.pos;desired_state.vel;desired_state.rot;desired_state.omega;desired_state.acc] ];
                    actual_state_matrix = [actual_state_matrix, [state(1:12);acc]];
                    %convert fsm state to hover to stabilize

                end
                fsm_state = 9;
                tot_time_vect = [tot_time_vect temp_time_vect(1:end-1)];

            case 8 %landing case
                %tracking state
                %use waypoint state 2 as an example
                %initialize waypoints for takeoff
                [temp_waypoints,temp_waypoint_times] = lookup_waypoints(8,actual_desired_state_matrix(1,end),actual_desired_state_matrix(2,end),actual_desired_state_matrix(3,end)); 
                %assume that state is maintained from the previous case
                %set times (for takeoff make the time between 0 and 1)
                temp_time_initial = 5+time_step;
                temp_time_iter = time_step;
                temp_time_final = 8;
                temp_time_vect = temp_time_initial:temp_time_iter:temp_time_final;
                temp_max_iter = length(temp_time_vect);             
                %create trajectory matrix from values
                %use the same trajectory planner as 0            
                temp_trajectory_matrix = trajectory_planner(0,temp_waypoints,temp_max_iter,temp_waypoint_times,temp_time_iter);
                %run for loop for model            
                for iter = 1:temp_max_iter-1
                    %set current state
                    current_state.pos = state(1:3);
                    current_state.vel = state(4:6);
                    current_state.rot = state(7:9);
                    current_state.omega = state(10:12);
                    current_state.rpm = state(13:16);
                    %set desired state
                    desired_state.pos = temp_trajectory_matrix(1:3,iter);
                    desired_state.vel = temp_trajectory_matrix(4:6,iter);
                    desired_state.rot = temp_trajectory_matrix(7:9,iter);
                    desired_state.omega = temp_trajectory_matrix(10:12,iter);
                    desired_state.acc = temp_trajectory_matrix(13:15,iter);
                    %set time increment to pass into ode45 function
                    timeint = time_vec(iter:iter+1);
                    %run ode45 function
                    [tsave,xsave] = quadcopter_model(0);
                    state    = xsave(end, :)';
                    acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));
                    actual_desired_state_matrix = [actual_desired_state_matrix, [desired_state.pos;desired_state.vel;desired_state.rot;desired_state.omega;desired_state.acc] ];
                    actual_state_matrix = [actual_state_matrix, [state(1:12);acc]];
                    %convert fsm state to hover to stabilize

                end
                fsm_state = 9;
                tot_time_vect = [tot_time_vect temp_time_vect(1:end-1)];
                landing_confirm = 1;
        end
    %     tot_time_vect = [tot_time_vect temp_time_vect];
    %     fsm_state = 3;
    end

    %tot_time_vect = 0:time_step:length(tot_time_vect);
    tot_time_vect = linspace(0,length(tot_time_vect)*time_step,length(tot_time_vect));
    
else
    %% Loop through the timesteps and update quadrotor
    for iter = 1:max_iter-1

        % convert current state to stuct for control functions
        current_state.pos = state(1:3);
        current_state.vel = state(4:6);
        current_state.rot = state(7:9);
        current_state.omega = state(10:12);
        current_state.rpm = state(13:16);

        % Get desired state from matrix, put into struct for control functions
        desired_state.pos = trajectory_matrix(1:3,iter);
        desired_state.vel = trajectory_matrix(4:6,iter);
        desired_state.rot = trajectory_matrix(7:9,iter);
        desired_state.omega = trajectory_matrix(10:12,iter);
        desired_state.acc = trajectory_matrix(13:15,iter);

        timeint = time_vec(iter:iter+1);
        [tsave,xsave] = quadcopter_model(question);
        state    = xsave(end, :)';
        acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));

        actual_desired_state_matrix = [actual_desired_state_matrix, [desired_state.pos;desired_state.vel;desired_state.rot;desired_state.omega;desired_state.acc] ];

    %     % Update desired state matrix
    %     actual_desired_state_matrix(1:3,iter+1) =  desired_state.pos;
    %     actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
    %     actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
    %     actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
    %     actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;

        actual_state_matrix = [actual_state_matrix, [state(1:12);acc]];

    %     % Update actual state matrix
    %     actual_state_matrix(1:12, iter+1) = state(1:12);
    %     actual_state_matrix(13:15, iter+1) = acc;  
    end    
    tot_time_vect = time_vec;

end
    
plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, tot_time_vect)



    function [tsave_out,xsave_out] = quadcopter_model(question_input)
        % Get desired acceleration from position controller
        [F, desired_state.acc] = position_controller(current_state, desired_state, params, question_input);

        % Computes desired pitch and roll angles
        [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);

        % Get torques from attitude controller
        M = attitude_controller(current_state, desired_state, params, question_input);

        % Motor model
        %i.e. mixing matrix
        [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);

        % Get the change in state from the quadrotor dynamics
        [tsave_out, xsave_out] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
        
        
    end
%define variables to visualize code
x = actual_state_matrix(1,:);
y = actual_state_matrix(2,:);
z = actual_state_matrix(3,:);
phi = actual_state_matrix(7,:);
theta = actual_state_matrix(8,:);
psi = actual_state_matrix(9,:);
vx = actual_state_matrix(4,:);
vy = actual_state_matrix(5,:);
vz = actual_state_matrix(6,:);
vphi = actual_state_matrix(10,:);
vtheta = actual_state_matrix(11,:);
vpsi = actual_state_matrix(12,:);

%define xyHis
%define desired x,y,z
xyHis_desired = actual_desired_state_matrix(1:3,:);
xyHis_desired = reshape(xyHis_desired,3,1,length(xyHis_desired));
xyHis_actual = actual_state_matrix(1:3,:);
xyHis_actual = reshape(xyHis_actual,3,1,length(xyHis_actual));
xyHis = [xyHis_desired xyHis_actual];

%change this variable to your total time values
time_vector = tot_time_vect;

data = struct('x',[x ; y ;  z],'theta',[phi ; theta ; psi],'vel',[vx ; vy ; vz],'angvel',[vphi ; vtheta ; vpsi],'t',time_vector,'dt',time_step,'input',0);

%uncomment this line for animation of quadcopter based on current and
%desired states
% visualize(data, xyHis);

end

