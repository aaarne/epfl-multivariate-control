%==========================================================================
%   TP :            Case study: Exercse 1
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
%% Clear the workspace and close figures
clear all, close all, clc;

%% Declare script parameters
animate = false;
sampling_time = 0.1;
simulation_time = 10;
simulate_flag = true;

%% Load utilities class and solutions
ut = utilities;
sol1 = ut.load_solution_class(1);

%% Get parameters
parameters = sol1.getSystemParameters;
simulate_flag = ut.solutionImplemented(parameters, 'getSystemParameters', simulate_flag);

if ~isempty(parameters)
    k = parameters(1);
else
    k = inf;
end
radius = 1/k;
theta_vector = (-pi/2+(0:0.0001:2*pi))';
path_to_track.coordinates = radius * [cos(theta_vector), sin(theta_vector)] + [zeros(size(theta_vector,1),1), ones(size(theta_vector,1),1)*radius];
path_to_track.s_coordinate = (theta_vector-theta_vector(1))*radius;
path_to_track.orientation = theta_vector+ pi/2;
path_to_track.curvature = path_to_track.orientation *0 + k;

%% Get arrays defining linear model (calling your solutions). 
[A,B,C,D] = sol1.getLinealModelArrays(parameters);
simulate_flag = ut.solutionImplemented(A, 'getLinealModelArrays', simulate_flag);

% Printing models
[Phi_euler1, Gam_euler] = sol1.getDiscreteLinearModel(A,B,C,D,sampling_time,'Euler');
[Phi_psi,Gam_psi] = sol1.getDiscreteLinearModel(A,B,C,D,sampling_time,'Psi');
[Phi,Gam] = sol1.getDiscreteLinearModel(A,B,C,D,sampling_time,'c2d');
simulate_flag = ut.solutionImplemented([Phi_euler1,Phi_psi,Phi], 'getDiscreteLinearModel', simulate_flag);

if simulate_flag
    %Print Matrices
    ut.printLabeledAB(A, B, 'Continuous system','description','[A | B]');
    ut.printLabeledAB(Phi_euler1, Gam_euler, 'Discrete description obtained applying Euler approximation');
    ut.printLabeledAB(Phi_psi, Gam_psi, 'Discrete description obtained programmatically');
    ut.printLabeledAB(Phi, Gam, 'Discrete description obtained using Matlab functions');

    % Comparison Euler vs Matlab
    ut.printRelativeError(Phi, Phi_euler1, Gam, Gam_euler, 'Comparing Euler vs Matlab')
    ut.printRelativeError(Phi, Phi_psi, Gam, Gam_psi, 'Comparing Programmatically vs Matlab')
end
% time vector (required to generate nominal trajectories)
nominal_trajectory_sampling_time = sampling_time/100;


% generate nominal trajectories for simulation
[nominal_trajectory_x, nominal_trajectory_u] = sol1.getWorkingTrajectory(nominal_trajectory_sampling_time, simulation_time, parameters);
simulate_flag = ut.solutionImplemented(nominal_trajectory_x, 'getWorkingTrajectory', simulate_flag);

%% simulations
% Considering the nominal trajectory, get initial state. 
[x0_experiments, x0Tild_experiments] = sol1.getInitialState(nominal_trajectory_x);
simulate_flag = ut.solutionImplemented(x0_experiments, 'getInitialState', simulate_flag);

%Generate open loop control reference 
[input_control_sequence_exps] = sol1.getOpenLoopControlSignal(nominal_trajectory_sampling_time,simulation_time);
simulate_flag = ut.solutionImplemented(input_control_sequence_exps, 'getOpenLoopControlSignal', simulate_flag);

if simulate_flag
    for exp_id = 1:length(input_control_sequence_exps)
        fprintf('\nExperiment %02d out of %02d\n', exp_id, length(input_control_sequence_exps));

        x0 = x0_experiments{exp_id};
        x0_tild = x0Tild_experiments{exp_id};
        input_control_sequence = input_control_sequence_exps{exp_id};

        %run simulate 
        sim(ut.complete_simulink_model_name('open_loop_experiments'));

        % plot results
        ax_cells = ut.plotResults(time, discrete_time, continuous_non_linear_model_global_position, open_loop_u, continuous_non_linear_model_x, path_to_track, parameters, 'figure_name', sprintf('open Loop - exp%02d: state',exp_id),'plot_label','non-linear continuous model');
        ut.plotResults(time, discrete_time, continuous_linear_model_global_position, open_loop_u, continuous_linear_model_x, path_to_track, parameters, 'plot_label','continuous linear model','ax_cells',ax_cells,'animate',animate);
        %ut.plotResults(time, discrete_time, continuous_non_linear_model_global_position, open_loop_u, continuous_non_linear_model_x, path_to_track, parameters, 'plot_label','continuous non-linear model','ax_cells',ax_cells,'animate',animate);
        ut.plotResults(time, discrete_time, discrete_linear_model_global_position, open_loop_u, discrete_linear_model_x, path_to_track, parameters, 'plot_label','discrete linear model','ax_cells',ax_cells,'animate',false);
    end
end