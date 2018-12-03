%==========================================================================
%   TP :            Case study: Exercse 2
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
clear all;close all;

%% script inputs
    animate_flag = true;
    simulation_time = 200;
    sampling_time = 0.005;
    sampling_time_nominal_trajectory = sampling_time/2;
    simulate_flag = true;    
    
    ut = utilities;
    sol2 = ut.load_solution_class(2);
    
%% load the linearized discretized model (inherited from previous exercise)
    parameters = sol2.getSystemParameters;
    simulate_flag = ut.solutionImplemented(parameters, 'getSystemParameters', simulate_flag);
    
    [A,B,C,D] = sol2.getLinealModelArrays(parameters);
    simulate_flag = ut.solutionImplemented(A, 'getLinealModelArrays', simulate_flag);
    
    [Phi,Gam] = sol2.getDiscreteLinearModel(A,B,C,D,sampling_time,'c2d');
    simulate_flag = ut.solutionImplemented(Phi, 'getDiscreteLinearModel', simulate_flag);

%% Optimal Controller design

    % Get Q1 and Q2 matrices
    [Q1,Q2] = sol2.getCostFunctArrays(size(Phi,1),size(Gam,2));
    simulate_flag = ut.solutionImplemented(Q1, 'getCostFunctArrays', simulate_flag);
    
    %According to Q1 and Q2, calculate LQR gain iteratively. 
    [lqrRiccati, SInf, S_vs]  = sol2.calculateLQRGain(Phi,Gam,Q1,Q2,'Riccati');
    simulate_flag = ut.solutionImplemented(lqrRiccati, 'calculateLQRGain - Riccati method', simulate_flag);
    
    %Calculate LQR gain using Matlab command. 
    [KDlqr,SDlqr] = sol2.calculateLQRGain(Phi,Gam,Q1,Q2,'Matlab');
    simulate_flag = ut.solutionImplemented(KDlqr, 'calculateLQRGain - Matlab method', simulate_flag);

    %Compare solutions
    if simulate_flag
        ut.printRelativeError(KDlqr, lqrRiccati, SDlqr, SInf, 'Comparing Programmatically vs Matlab','description','[relative error K_lqr | relative error S_inf ]')
        %Draw singular values of S over the iterations. 
        ut.drawSingularValEvol(S_vs);
    end

%% Control implementation and simulation.
    % time vector (required to generate nominal trajectories)    
    chosen_path = sol2.select_reference_path;
    simulate_flag = ut.solutionImplemented(chosen_path, 'select_reference_path', simulate_flag);
    
    if ~isempty(chosen_path)
        load(chosen_path);
    end
    

    % Load nominal state and control trajectory 
    [nominal_trajectory_x, nominal_trajectory_u] = sol2.getWorkingTrajectory(sampling_time_nominal_trajectory, simulation_time, parameters);
    simulate_flag = ut.solutionImplemented(nominal_trajectory_x, 'getWorkingTrajectory', simulate_flag);
    
    %Get initial state
    [x0,x0Tild,x0Obs] = sol2.getInitialState(nominal_trajectory_x);
    simulate_flag = ut.solutionImplemented(x0, 'getInitialState', simulate_flag);
    
%%  Run simulation with clean and fully measurable states. 
    % Initial value of noise, Cmes, simulation time, time vector.
    noiseMod = ut.defaultNoiseModule;  Cprime = eye(5);
    
    %Simulate and represent results
    if simulate_flag
        sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
        ax_cells = ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - non linear - clean and fully measurable state','animate',animate_flag);
    end
    
%% Run simulation with noisy and fully measurable states. 
    %Load your noise module
    noiseMod = sol2.getNoiseModule;
    simulate_flag = ut.solutionImplemented(noiseMod, 'getNoiseModule', simulate_flag);
    
    %Simulate and represent results
    if simulate_flag
        sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
        ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - noisy measurable states','noisy_states', noisy_system_state,'animate',animate_flag);
    end
%% Run simulation with noisy and partially measurable states. 
    Corig   = C; Cprime  = sol2.getCprime;
    simulate_flag = ut.solutionImplemented(Corig, 'getCprime', simulate_flag);

    if simulate_flag
        sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
        ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - noisy partially measurable states','animate',animate_flag);
    end
%% Observer design and simulation
    % obtain CMes, 
    [CMes,CObs,DObs] = sol2.getCArrayConsideringMeasurableStates(C);
    simulate_flag = ut.solutionImplemented(CMes, 'getCArrayConsideringMeasurableStates', simulate_flag);

    % Check if the system is observable with the new outputs
    unobStates = sol2.checkObservability(Phi,CMes);
    simulate_flag = ut.solutionImplemented(unobStates, 'checkObservability', simulate_flag);
    
    % Design an observer using pole placement
    [L_options,poles_options] = sol2.getObserverGain(Phi,Gam,KDlqr,CMes);
    simulate_flag = ut.solutionImplemented(L_options, 'getObserverGain', simulate_flag);

    if ~iscell(L_options)
        L_options = {L_options}; 
        poles_options = {poles_options};
    end

    %eNow run simulations for all set up experiments
    if simulate_flag
        for exp_id = 1:length(L_options) 

            L = L_options{exp_id};
            ut.printComplexNumbers(poles_options{exp_id}, 'label', 'Observer poles')

            sim(ut.complete_simulink_model_name('LQR_observer'));
            ut.compareObservedVsRealStates(time, discrete_time, observed_x,noisy_x,x, 'figure_name', 'comparison observed states')
            ut.plotResults(time, discrete_time, global_position, u, x, path_to_track, parameters, 'figure_name', 'LQR + observer', 'animate', animate_flag);
        end
    end