%==========================================================================
%   TP :            Case study: Exercse 1
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef ex1
    %Class gathering the solutions of exercise 1. 
    methods
        %
        function varargout = getSystemParameters(~)
            % PARAMETERS = getSystemParameters() returns a 5-elements
            % column vector PARAMETERS containing the parameters value
            % characterizing the system and the linearization point.
            % Specifically it should contain (in the presented order):
            %   - k : value of the reference curvature. 
            %   - car_length : length of the car in meters.  
            %   - sigma_a : sigma_a characterizing the dynamic of the
            %   actuator tracking the inputed acceleration. 
            %   - sigma_S : sigma_s characterizing the dynamic of the
            %   actuator tracking the inputed reference speed. 
            %   - spReg : value of the speed reference. 

            %%- set up the parameters values 
            
            %%- set up the output of the function 
            varargout = {[1e-10, 4, 5, 1, 3]'};               
        end
        %
        function varargout = getLinealModelArrays(~,parameters)
            % [A,B,C,D] = getLinealModelArrays(PARAMETERS) outputs the
            % matrices A,B,C,D characterizing the continuous_time linear
            % version of the system given the set of parameters values as
            % they where defined in function 'getSystemParameters'.
            %
            k = parameters(1);
            L = parameters(2);
            sigma_a = parameters(3);
            sigma_s = parameters(4);
            v0 = parameters(5);
            
            A = [0, k*v0, 0, 1, 0;
                 0, 0, v0, 0, 0;
                 0, -k^2*v0, 0, 0, v0/(L*(cos(atan(L*k)))^2);
                 0, 0, 0, -sigma_a, 0;
                 0, 0, 0, 0, -sigma_s];
             
            B = [0, 0, 0, sigma_a, 0;
                 0, 0, 0, 0, sigma_s]';
             
            C = eye(5);
            D = zeros(5, 2);
            varargout = {A, B, C, D};
        end        
        %
        function varargout = getDiscreteLinearModel(~,A,B,C,D,sampling_time,method)
            % [PHI,GAM] =
            % getDiscreteLinearModel(A, B, C, D, SAMPLING_TIME, METHOD)
            % outputs the PHI and GAMMA matrices characterizing the
            % discrete linear model of the system given the matrices
            % A,B,C,D of the continuous time linear description of the
            % system and the desired SAMPLING_TIME.
            %
            % Additionally, the input METHOD will contain a string
            % indicating the way the matrices PHI and GAMMA are wanted to
            % be calculated. Such an input can take values 
            % - Euler : Euler approximation as discretization method. 
            % - Psi : Use the progrmmatically algorithm presented in the
            % note course which makes use of the intermediary matrix Psi.
            % - c2d : use the matlab command c2d. 
            %
            
            if strcmp(method,'Euler')
                % Use here Euler approximation to approximate the
                % discrete-time linear model of the system.
                
                %%-set up output of the function 
                dim = 5;
                Phi = eye(dim) + sampling_time * A;
                Gamma = sampling_time * B;
                varargout = {Phi, Gamma};
            elseif strcmp(method,'Psi')
                % Dimension of Psi and Psi initialization
                
                psi_dimension = 5; 
                Psi = zeros(psi_dimension);
                
                number_of_iterations = 10;
                for k=0:number_of_iterations
                    %update Psi iteratively. 
                    Psi = Psi + sampling_time^k * A^k / (factorial(k+1));
                end
                
                % Calculate matrices Phi and Gam
                Phi = eye(psi_dimension) + sampling_time*A*Psi;
                Gamma = sampling_time*Psi*B;
                
                
                %%-set up output of the function 
                varargout = {Phi, Gamma};                
                
            elseif strcmp(method,'c2d')
                %%- Continuous representation of the system with 'ss'
                
                Mc = ss(A,B,C,D);
                
                %%- Calculate the discrete-time linear model of the system. of discretized system using 'c2d'
                
                Md = c2d(Mc, sampling_time, 'zoh'); 
                
                %%- Extract from Md, the Phi and Gamma matrices. 
                
                Phi = Md.A;
                Gamma = Md.B;
                
                %%-set up output of the function 
                varargout = {Phi, Gamma};                
            end
        end                
        %
        function varargout = getWorkingTrajectory(~,sampling_time, simulation_time, parameters)
            % [NOMINAL_TRAJECTORY_X, NOMINAL_TRAJECTORY_U] =
            % getWorkingTrajectory(SAMPLING_TIME, SIMULTAION_TIME,
            % PARAMETERS)  
            % outputs the NOMINAL_TRAJECTORY_X and NOMINAL_TRAJECTORY U
            % given the SAMPLING_TIME between data points, the
            % SIMULATION_TIME up to which the trajectory has to be created,
            % and the vector PARAMETERS with the value sof tha system's
            % parameters.
            %
            % Outputs NOMINAL_TRAJECTORY_X, and NOMINAL_TRAJECTORY_U must
            % be arrays whose first column correspond to the time span of
            % the data point, and successive columns store the information
            % of the states and inputs, correspondingly.
            %
            % The defined output trajectories are meant to be used in
            % Simulink with "From Workspace" importing module. If any
            % additional doubt regarding how the data should be structured,
            % read the information provuded by the mentioned simulink block.
            %
            % todo
            % - create time vector. 
            % - create the nominal states trajectory output
            % - create the control inputs nominal trajectory output
            
            %%- create time vector
            time_vector = (0:sampling_time:simulation_time)';
            
            %%-create nominal state trajectory. 
            dim = size(time_vector,1);
            k = parameters(1);
            L = parameters(2);
            v0 = parameters(5);
            nominal_trajectory_x = [time_vector, ...
                time_vector.*v0, ...
                zeros(dim,1), ...
                zeros(dim,1), ...
                ones(dim,1).*v0, ...
                ones(dim,1).*atan(L*k)];
            
            %%-create nominal control input trajectory. 
            nominal_trajectory_u = [time_vector,  ...
                ones(dim,1).*v0, ...
                ones(dim,1).*atan(L*k)];
            varargout = {nominal_trajectory_x, nominal_trajectory_u};
        end
        %
        function varargout = getInitialState(~, nominal_trajectory_x)
            %[X0, X0TILDE] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the non linear system as the
            % initial state X0TILDE of the linearized model of the system,
            % given the information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x}.
            
            
            %%- define the value of x0 for experiment 1
            x0_experiment_1 = zeros(5,1);
            
            %%- define the value of x0Tilde for experiment 1
            x0Tilde_experiment_1 = x0_experiment_1 - ...
                nominal_trajectory_x(1,2:end)';
            
            %including the different values for different experiments as a
            %cell
            x0 = {x0_experiment_1};
            x0Tilde = {x0Tilde_experiment_1};
            
            %set outputs of the function 
            varargout = {x0, x0Tilde};
        end
        %
        function varargout = getOpenLoopControlSignal(~,sampling_time, simulation_time)
            %[INPUT_CONTROL_ACTIONS_OPEN_LOOP] = getOpenLoopControlSignal(SAMPLING_TIME, SIMULATION_TIME)
            % outputs a sequence of control signals to be applied in open
            % loop to the system, which will be then simulated. In order to
            % do that, the desired SAMPLING_TIME between data points as
            % well as the SIMULTION_TIME is provided. 
            %
            % As in the case of GETWORKINGTRAJECTORY function, the outputs
            % are meant to be used in Simulink with "From Workspace"
            % importing module. If any additional doubt regarding how the
            % data should be structured, read the information provuded by
            % the mentioned simulink block. 
            %
            % todo:
            % - Declare en appropriate time span vector. 
            % - Create the input_control_actions_open_loop array with the
            % sequence of control inputs to be applied in open loop. 
            %
            %
            % Notice: alternatively, this function can output a cell with
            % several arrays showing different control sequences to be
            % applied. This would make the provided script to run the
            % simulink mdel as many times as different control sequences
            % are gathered within the cell. Meaning that several
            % experiments can be set at once. 
            %
            
            %%- Create a time vector.
            time_vector = (0:sampling_time:simulation_time)';
            
            %%- set the control sequence to be applied in open loop for the
            %%1st experiment. 
            dim = size(time_vector, 1);
            uOpenLoop_experiment_1 = [time_vector, ...
                ones(dim,1).*3.0, ...
                ones(dim,1).*pi/6];
            
            %Include different values for the different experiments in a
            %cell.
            input_control_actions_open_loop = {uOpenLoop_experiment_1};
            
            %set output of the function
            varargout = {input_control_actions_open_loop};
        end
    end
end

