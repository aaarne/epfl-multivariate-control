%==========================================================================
%   TP :            Case study: Exercse 2
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef ex2
    %Class gathering the solutions of exercise 2. 
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
            
            %%- Calculate arrays A,B,C,D
            %A = 
            %B = 
            %C = 
            %D = 
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
            % - Psi : Use the programmatically algorithm presented in the
            % note course which makes use of the intermediary matrix Psi.
            % - c2d : use the matlab command c2d. 
            %
            % 3.4 - Continuous representation of the system with 'ss'
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
        function [varargout] = getCostFunctArrays(~,nStates,nInputs)
            % [Q1,Q2] = getCostFunctArrays(NSTATES,NINPUTS) returns the
            % matrices Q1 and Q2 defining the cost function leading to the
            % design of a LQR control gain, given the number of states
            % NSTATES and inputs NINPUTS. 
            %
            % The information regarding the values to be given to the
            % output matrices can be found in the exercise handout. 

            Q1 = [1e-10, 0, 0, 0, 0;
                  0, 5, 0, 0, 0;
                  0, 0, 1, 0, 0;
                  0, 0, 0, 1, 0;
                  0, 0, 0, 0, 1];
            Q2 = [1, 0;
                  0, 0.5];

            varargout = {Q1, Q2};
        end
        %        
        function varargout = calculateLQRGain(~,Phi,Gamma,Q1,Q2, method)
            % [LQR_K,SInf,SINGULAR_VALUES] =
            % solveLQRthroughRiccati(PHI,GAMMA,Q1,Q2,'Riccati') returns the
            % LQR control gain, the matrix SINF as well as the history of
            % the singular values of the S matrix over the iterations,
            % given the PHI and GAMMA metrices describing the dicrete
            % lineal model of the systme as well as the matrices Q1 and Q2
            % defining the cost function of the control problem. The last
            % input is a string the specifies the method to be used. 
            %
            % [LQR_K,SInf] =
            % solveLQRthroughRiccati(PHI,GAMMA,Q1,Q2,'Matlab') returns the 
            % value of the LQR control gain and SINF when calculated with
            % the matlab command 'dlqr'. For more details regarding how to
            % use 'dlqr' command, please check the matlab 'help' or 'doc'
            % information.
            %
            %TODO: implement algorithm in Section 7.1.7. 
            %
            % Hint: regarding the number of iterations for the calculation
            % of the control gain following Riccati approach, you should
            % find (by trial and error) a number of iterations that is
            % sufficiently large so that the SINF matrix converges to an
            % unique value. This condition can be observed by keeping track
            % of the elements of the matrix or any other characteristics.
            % In this case, we are to use the singular values of S as it is
            % updated over the iterations.
            %
            % Note: as the number of ouputs changes depending on the
            % requested method, the output will be defined as a cell
            % containing the appropriate matrices according to the case. 
            
            if strcmp(method,'Riccati')
                %%- Implement the algorithm to calculate the LQR control
                %%gain using riccati equation. 
                
                %%- Initialize S
                S = Q1; 
                
                %%- number of interations
                n_iterations = 2500;

                % Vector of singular values (to control the convergence of the method
                %singular_values = zeros(n_iterations,size(S,1));

                %Implementation of Ricatti method 
                for i=1:n_iterations
                    
                    %- Ricatti Equation 
                    S=Phi' * (S - S*Gamma*inv(Q2 + Gamma' * S * Gamma) * Gamma' * S) * Phi + Q1;
                    
                    %- store singular values (use svd function) 
                    singular_values(i,:) = svd(S);
                end
                
                %%-calculate the LQR control gain. 
                KLqrRiccati = inv(Q2 + Gamma'*S*Gamma) * Gamma' * S * Phi; 
                
                %%-set output of the function.
                varargout = {KLqrRiccati, S, singular_values};
                
            elseif strcmp(method,'Matlab')
                
                %%- Use dlqr to calculate the LQR control gain and S array
                
                %%- set the output of the function 

                [K, S, ~] = dlqr(Phi, Gamma, Q1, Q2);
                varargout = {K, S};
            end
        end
        %        
        function varargout = select_reference_path(~)
            % OUTPUT = select_reference_path returns a string with the name
            % of the mat file containing the path to be tracked that is
            % wanted to be used. 
            options = {'circle', 'path_1', 'path_2', 'path_3'};
            
            varargout = {options{4}};
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
            % read the information provided by the mentioned simulink block.
            %
            % todo
            % - create time vector. 
            % - create the nominal states trajectory output
            % - create the control inputs nominal trajectory output
            
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
        function varargout = getInitialState(~,ntX)
            %[X0, X0TILDE, XOBS] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the non linear system, the
            % initial state X0TILDE of the linearized model of the system,
            % as well as the initial value of the state for the observer,
            % given the,  information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x} and
            % that the observer is designed considering the linear model of
            % the system.
            
            
            %%- define the value of x0 for experiment 1
            x0_experiment_1 = [0, 0, 0, 0, 0];
            
            %%- define the value of x0Tilde for experiment 1
            x0Tilde_experiment_1 = zeros(5,1);
            
            %set outputs of the function 
            varargout = {x0_experiment_1, x0Tilde_experiment_1, x0_experiment_1};
        end        
        %        
        function varargout = getNoiseModule(~)
            % [NOISE] = getNoiseModule
            % generates a structure with fields 
            % - mean
            % - std_deviation
            % - seed
            % defining the noise to be introduced in the corresponding
            % simulations. The name of the structure fields are
            % self-explanatory, nontheless, for further information
            % regarding their meaning, it is used in the 'noise' block in
            % the 'LQR_observer' and 'LQR_closed_loop' simulink models.
            % Check the information of such a module out for more details.
            %
            % Notice 1: Notice that every field of the structure is a
            % vector with as many elements as states.
            %
            % Notice 2: The values that are proposed in the exercise
            % handout are just a recomendation. Feel free to play with
            % these values, observe the effect they have, and reason why.
            % Take into account the range of values that the state affected
            % by each term of the noise fields can take when playing around
            % with the values.
            % 
            %
            %%- prepare noise structure
            noise.mean = [0, 0, 0, 0, 0];
            noise.std_deviation = [1, 1, 0.1745, 2, 0];
            %noise.std_deviation = [0.1, 0.1, 0.01745, 0.2, 0.1];
            noise.seed = 42*[1, 1, 1, 1, 1];
            
            %%- set up the output of the function
            varargout = {noise};
        end        
        %        
        function varargout = getCprime(~)
            % CPRIME = getCprime(obj)
            % returns a third version of the matric C (called CPRIME) which
            % will multiply to the state vector before feeding the control
            % gain. In this way, we would easily simulate the consequences
            % of, after having designed a LQR which works with the full
            % state vector, not having the information regarding some of
            % the states. 
            %
            % Notice that CPRIME must have the same dimensions than C and
            % the elements on the row corresponding to the state that is
            % wanted to consider missing equal to zero.
            %
            %
            %%- set the desired value for C'
            Cprime = diag([1, 1, 0, 1, 1]);
            
            %%- set up the output of the function 
            varargout = {Cprime};
        end
        %        
        function varargout = getCArrayConsideringMeasurableStates(~,C)
            % [CMES,COBS,DOBS] = getCArrayConsideringMeasurableStates(C)
            % returns matrices CMES, COBS, and DOBS, which are needed to
            % implement the observer and have to be calculated from C. 
            %
            % Output CMES is a version of the C matrix with some rows
            % ommited, which represents the assumption that the states 
            % corresponding to the ommited rows are not measurable. 
            %
            % The output matrices COBS and DOBS are used in the 'Observer'
            % block in the simulink model 'LQR_observer'. Go and explore
            % such a module for some hints regarding the dimensions of the
            % matrices.  
            %
            % Moreover, CObs and DObs should be constructed following the
            % approach presented in the solution of exercise 6.6.1 - 3
            %
            
            %%- set up the value of CMes 
            
            %%- set up arrays CObs and DObs que the appropriate size. 
            Cmes = [
                1, 0, 0, 0, 0;
                0, 1, 0, 0, 0;
                %0, 0, 1, 0, 0;
                0, 0, 0, 1, 0;
                0, 0, 0, 0, 1
                ];

            n_outputs = size(Cmes, 1);
            Cobs = eye(5);
            Dobs = zeros(5, 2+n_outputs);
            
            %%- set up the outputs of the function
            varargout = {Cmes, Cobs, Dobs};
        end
        %
        function varargout = checkObservability(~,Phi,CMes)
            % [N_STATES_NOT_OBSERVABLE] = checkObservability(PHI,CMES)
            % checks whether the system described by Phi, 
            % CMes is observable and returns the number of states that
            % cannot be observed. 
            %
            % Remember that the number of the states that cannot be
            % observed will be given by the number of states and the rank
            % of the observability matric. 
            %
            % HINT: To calculate the observability matrix, you can use the
            % matlab command 'obsv'. 
            %
            % NOTICE: CMes is a version of the C matrix where some rows
            % have been ommited, which would be connected with the
            % assumption that the states corresponding to the ommited rows
            % are not measurable. 
            
            
            %%- calculate the observability matrix
            Observability_matrix = obsv(Phi, CMes);
            
            %%- Compute the number of non-observable states of the system
            n_states_not_observable = length(Phi) - rank(Observability_matrix);            
            
            fprintf('\nObservability check: %d states cannot be observed with this configuration\n',n_states_not_observable);
            
            varargout = {n_states_not_observable};
        end
        %
        function varargout = getObserverGain(~,Phi,Gamma,lqr_K,c_meassurable)
            % [L, SELECTED_POLES] =
            % getObserverGain(obj,PHI,GAMMA,LQR_K,c_messurable) 
            % returns the observer matrix L as well as the poles
            % SELECTED_POLES characterizing the closed observation loop
            % given the discrete linear model defined by PHI and GAMMA, the
            % LQR control gain LQR_K and the matrix C_meassurable, that is
            % the version of the C matrix where some states are ommited. 
            %
            % 
            % Hint1: as a rule of thumb, the poles of the observer are 
            % calculated as some percentage of the poles that characterize 
            % the close loop dynamic. 
            %
            % Hint2: remember that the eigenvalues of the control closed
            % loop are given by eig(\Phi - \Gamma * lqr_K)
            %
            % Hint3: to calculate the observer gain, use the matlab command
            %'place'. Read the help information and notice that, the
            %function is meant to be used to calculate control gains, that
            %is it returns gains to impose certain poles assuming the
            %closed loop dynamics is given by (A-B*K). However, we are
            %using it to place the poles of the observer. Check and compare
            %the observer close loop, build a paralelism with the control
            %case, and introduce the needed modifications in the matrices
            %given to 'places'. 
            %
            % HINT4: Think about what happens when you transpose the closed
            % loop dynamic of the observer.
            % 
            % Notice 1: The outputs of this function can be defined as
            % cells gathering as many instances of the observer gain as
            % experiments are wanted to be run.
            %
            
            %%- calculate the place where the close loop observation poles
            %%are wanted to be. 

            phi_cl = Phi - Gamma * lqr_K;
            
            percentage = 0.75;
           
            selected_poles = percentage * eig(phi_cl); 
            
            %%- calculate the observability matrix. 
            
            L = place(Phi', c_meassurable', selected_poles)';
            
            %%- set up the output of the function. 
            varargout = {L, selected_poles};
        end
    end
end

