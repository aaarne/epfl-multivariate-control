%==========================================================================
%   TP :            Case study: Exercse 3
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef ex3
    %Class gathering the solutions of exercise 3. 
    methods
        function varargout = getInitialState(~)
            %[X0] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the non linear system given
            % the information on the exercise handout and the 
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            
            %%- set up initial state
            
            %%- set up the output of the function 
            x0 = [0, 0, 0, 0, 0]';
            varargout = {x0};            
        end        
        %        
        function varargout = getSystemParameters(~)
            % PARAMETERS = getSystemParameters() returns a 5-elements
            % column vector PARAMETERS containing the parameters value
            % characterizing the system and the linearization point.
            % Specifically it should contain (in the presented order):
            %   - k : vlue of the reference curvature. 
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
        function varargout = getAllowedControlValues(~)
            % [U] = getAllowedControlValues
            % returns a two-columns array U gathering all the combinations
            % of control inputs that are considered to build the graph
            % defining the control problem to be solved through dynamic
            % programming. 
            %
            
            %%- set up the values that the control inputs are allowed to
            %%take 
            u_1 = 3;
            u_2 = [-pi/4, 0, pi/4];
            
            %%- Calculate all possible combinations of those two states 
            [X, Y] = meshgrid(u_1, u_2);
            u  = [X, Y];
            
            %%- set up the output of the function 
            varargout = {u};                        
        end        
        %
        function varargout = select_reference_path(~)
            % OUTPUT = select_reference_path returns a string with the name
            % of the mat file containing the path to be tracked that is
            % wanted to be used. 
            %
            options = {'circle', 'path_1', 'path_2', 'path_3'};
            
            %%- set up the output of the function 
            varargout = {options{4}};
        end        
        %
        function varargout = getObstacle(~)
            % [obstacles] = getObstacle
            % returns two-column array with as many rows as obstacles are
            % wanted to be considered in the problem, showing the position
            % [x|y] of the obstacles. 
            %
            % The ouput can be defined as a cell of as many different
            % versions of the obstacle array as experiments are wanted to
            % be run. 
            
            %%-set up the obstacle(s) position
            obstacles01 = [
                57, 40
                57.25, 40
                57.5, 40
                57.75, 40
                58, 40
                58.25, 40
                58.5, 40
                58.75, 40
                59, 40
                59.25, 40
                59.5, 40
                59.75, 40
                60, 40
                60, 40
                62, 40
                62.25, 40
                62.5, 40
                62.75, 40
                63, 40
                63.25, 40
                63.5, 40
                62.75, 40
                63, 40
            ];

            obstacles02 = obstacles01 + [2 0];

            obstacles03 = obstacles01 - [2 0];

            obstacles04 = obstacles01 - [4 0];

            obstacles05 = [];

            obstacles06 = [
                %54.87, 19.53
                %59.57, 39.73
                %58.28, 57.36
                %59.49, 48.77
                %20.1, 1.317
                %42.01, 56.62
                %2.56, 77.7
                %16.77, 53.9
                %-11.59, 29.38
                %-30.04, 29.27
                %-20.9, 0.1812
                %0, 0
            ];
        
            obstacles07 = [
                3.42, 0
                6, 0
                9, 0
                12, 0
            ];
        
            %%- set up the output of the function 
            varargout = {{obstacles01, obstacles02, obstacles03, obstacles04}}; 
            %varargout = {{obstacles06}}; 
        end        
        %
        function varargout = assignCostsToNodes(~,nodes, edges, obstacles)
            % [NODE_COSTS] = assignCostsToNodes(NODES, EDGES, OBSTACLES) 
            % must return an array NODE_COSTS containing the cost assigned
            % to the nodes represented by NODES considering the set of
            % considered EDGES connecting them and OBSTACLES. 
            %
            % NODES input contains information of all the nodes in the
            % graph. In particular, it is an array of as many rows as nodes
            % in the graph and 11 columns containing, in order, the following elements:
            %  - ID: node ID
            %  - t:  time associated to the node
            %  - flag: 
            %       - 0 : the node has not been visited. This is the case
            %       of the last nodes in the branches of the tree. 
            %       - 1 : the node was visited. Corresponds to the nodes
            %       that do have a successor and a predecessor. 
            %       - 2 : the node is a dead end, that is the node has a
            %       predecessor but does not have a successor because its
            %       state was considered to violate the constraints imposed
            %       when building the graph.
            %  - [px, py, yaw] : triple showing the absolute position of
            %  the vehicle the node represents. 
            %  - [x1 x2 x3 x4 x5] : state of the vehicle that node
            %  represents.
            % 
            % EDGES: The input array EDGES contains the information of which nodes
            % are connected. Specifically it will be a 4-columns array
            % containing, in order, 
            %  - predecessor node id
            %  - sucessor node id. 
            %  - [u1,u2] control input applied between nodes. 
            %
            % OBSTACLE contain the absolute position of the obstacles to be
            % considered. Its format matches the one that was used in
            % function getObstacle. 
            %
            % The cost c_j of a given node j, will be calculated as 
            % c_j = c_i + J(u_{i-j}) + J(x_j)
            %
            % - c_i is the cost of the predecessor node. 
            %
            % - J(u_{i-j}) if a cost term associated to the control input
            % that was applied to go from node-i to node-j. In our case it
            % will be assumed to be J(u_{i-j}) = 0;
            %
            % - J(x_j) is a cost associated to the node being evaluated and
            % will be composed of two terms 
            % J(x_j) = J_{tracking}(x_j) + J_{obstacle avoidance}(x_j) + J_{dead_end}(x_j).
            % which includes
            %
            %   - J_{tracking}(x_j) = w_{track d} x_2 + w_{track heading}
            %   x_3  representing the tracking error. You can freely modify
            %   the values of w_{track d} and w_{track heading} to
            %   observe how they influence the solution. As first vlues
            %   you can try w_{track d} = 10 and w_{track heading} = 20.
            %
            %   - J_{obstacle avoidance}(x_j) = \sum_{i = 1}^{n_obstacles}
            %   w_{collision avoidance}/(dist([px_j, py_j], obstacle_i)^3) penalizing the
            %   inverse of the distance to the considered obstacles. You
            %   can freely play with the value of w_{collision avoidance},
            %   as a initial value you could consider w_{collision
            %   avoidance} = 100.
            %
            %   - J_{dead_end}(x_j) = \inf if node_j is a dead end and
            %   J_{dead_end}(x_j) = 0 otherwise. 
            %
            % Summarizing the cost of a node_j, whose predecessor is the node_i, will be calculated as 
            % c_j = c_i + w_{track d} x_2 + w_{track heading} x_3 + \sum_{i
            % = 1}^{n_obstacles} w_{collision avoidance}/(dist([px_j, py_j], obstacle_i)^3) +
            % J_{dead_end}(x_j) 
            %
            %
            %
            % Hint: you could first calculate J(u_{i-j}) + J(x_j) for every
            % node in a parallel way (no loops needed), and then, iterate
            % over the edges updating the cost of the predecessor nodes
            % using the successor nodes information. 
            %

            % Weights:
            w_track_d = 10;
            w_track_heading = 20;
            w_obs_avoidance = 1000;

            fprintf('\nAssigning costs to nodes...');
            ut = utilities;
            
            %%- Calculate the path tracking cost for each node
            path_tracking_cost = abs(w_track_d * nodes(:,8)) + abs(w_track_heading * nodes(:,9));
            
            %%- Calculate the distance of every node to every considered
            %obstacle. You could iterate over the obstacles, but avoid
            %iterating over the nodes. 
            n_nodes = size(nodes, 1);
            n_obstacles = size(obstacles.position, 1);
            dists = zeros(n_nodes, n_obstacles);
            for i = 1:n_obstacles
                d = vecnorm((nodes(:,4:5) - obstacles.position(1,:))')'.^3;
                dists(:,i) = 1./d;
            end
            unit_obstacle_avoidance_cost = sum(dists, 2);
            
            %%- calculate the obstacle_avoidance cost term. 
            if n_obstacles > 0
                obstacle_avoidance_cost = w_obs_avoidance * unit_obstacle_avoidance_cost;
            else 
                obstacle_avoidance_cost = 0;
            end

            %%- calculate the dead_end_cost of every node
            dead_end_cost = zeros(n_nodes, 1);
            dead_end_cost(nodes(:,3) == 2) = inf;
            
            %%- calculate the resulting J(x_i) cost and store it in
            %%node_costs 
            node_costs = path_tracking_cost + obstacle_avoidance_cost + dead_end_cost;
            
            %Iterate over the edges updating the cost of the successor node
            %considering the cost of its predecessor.
            
            %iteration_counter = 0;
            for i = 1:size(edges,1)
                ut.printProgression(i, size(edges,1), 50);
                node_costs(edges(i,2)) = node_costs(edges(i,2)) + node_costs(edges(i,1));
            end
            
            %%- set up the output of the function 
            varargout = {node_costs};                                    
        end      
        %
        function varargout = backtrackNodeValues(~, nodes, edges, node_cost, sampling_time)
            % [OPTIMAL_NODE_SEQUENCE] = backtrackNodeValues(NODES, EDGES,
            % NODE_COST, SAMPLING_TIME) 
            % returns a column vector OPTIMAL_NODE_SEQUENCE containing the
            % id of the sequence of nodes representing the optimal
            % trajectory of the system. For that the NODES, EDGES and
            % NODE_COSTS information is provided along with the
            % SAMPLING_TIME used to build the graph representing the
            % control problem. 
            %
            % NODES, EDGES, and NODE_COST contain the same information than
            % it was presented in function. You are kindly referred to the
            % description of such a function for more details regarding the
            % information that those arrays contain. 
            %
            % The SAMPLING_TIME is uniquely provided as a means to
            % pre-calculate the number of nodes that are going to be part
            % of the optimal node sequence, and to avoid the change of its
            % dimension over the iterations. 
            %
            % Hint 1: to do this you first need to identify which are those
            % nodes representing the end of the branches in the graph. 
            % From those, you select the one with minimum cost which,
            % considering the way you propagated the cost un the previous
            % function, should correspond to the optimal trajectory. 
            % Once that node has been identidied it only remains using the
            % edges array to move through the graph backwards while storing
            % the nodes that are visited. 
            %
            % Notice: Remember that those nodes that are at the end of the
            % built branches, have a flag = 2, which is stored in the third
            % columnd of NODES array. 
            
            %%- identify the terminal nodes
            terminal_nodes_id = nodes(nodes(:,3) == 0, 1);

            [e, idx] = min(node_cost(terminal_nodes_id));
            
            %%- find the terminal node with the lowest cost. 
            terminal_node_lowest_cost_id = terminal_nodes_id(idx);
            fprintf("Lowest cost %f at node %d.\n", e, terminal_node_lowest_cost_id);
            
            %%- set up the selected terminal node as the 'current node'
            current_node = terminal_node_lowest_cost_id;
            
            %%- Initialize the optimal_node_sequence. 
            n_nodes_in_optimal_sequence = round(nodes(current_node, 2) / sampling_time);
            optimal_node_sequence = zeros(n_nodes_in_optimal_sequence, 1);
            
            %%- set up the last point of the optimal node sequence. 
            optimal_node_sequence(n_nodes_in_optimal_sequence) = current_node;
            
            %%- explore the graph from the last point backwards by saving
            %%the indexes of the explored nodes 
            
            for i =(n_nodes_in_optimal_sequence-1):-1:1
            
                %- find the predecessor node
                predecessor_node_id = edges(edges(:,2) == current_node, 1);
                
                %- store the node id in the sequence array
                optimal_node_sequence(i) = predecessor_node_id;
                
                %- update the current node for the next iteration
                current_node = predecessor_node_id;
            end

            %%- set up the output of the function 
            varargout = {optimal_node_sequence};                                    
        end
    end  
end

