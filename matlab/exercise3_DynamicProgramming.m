%==========================================================================
%   TP :            Case study: Exercse 3
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
close all;clear all;clc;

%% script parameters
    sampling_time = 0.24;
    max_n_nodes_graph = 40000;
    simulate_flag = true;
    %contraints
    max_lateral_deviation = 3.0; %[m]
    max_heading_deviation = pi/4; %[rad]
    time_to_loose_it = 1; %[s] time from which the tree is simplified
    threshold_number_of_nodes = 150; %average number of new nodes that have to be created at each iteration. 
    
%% load the linearized discretized model
    ut = utilities;
    sol3 = ut.load_solution_class(3);

    % initial state
    x0 = sol3.getInitialState;
    simulate_flag = ut.solutionImplemented(x0, 'getInitialState', simulate_flag);
    
    parameters = sol3.getSystemParameters;
    simulate_flag = ut.solutionImplemented(parameters, 'getSystemParameters', simulate_flag);
    
    [allowed_u] = sol3.getAllowedControlValues;
    simulate_flag = ut.solutionImplemented(allowed_u, 'getAllowedControlValues', simulate_flag);

    %load path of reference
    chosen_path = sol3.select_reference_path;
    simulate_flag = ut.solutionImplemented(chosen_path, 'select_reference_path', simulate_flag);
    
    if ~isempty(chosen_path)
        load(chosen_path);
    else
        path_to_track = [];
    end

%% 1 - Built graph for dynamic programming application
    % Generate graph
    if simulate_flag
        graph_args = {sampling_time, ...
            x0, ...
            path_to_track, ...
            allowed_u, ...
            parameters,...
            'plot_nodes',false,...
            'max_n_nodes',max_n_nodes_graph,...
            'max_lateral_deviation',max_lateral_deviation,...
            'max_heading_deviation',max_heading_deviation,...
            'time_to_loose_it',time_to_loose_it,...
            'threshold_number_of_nodes',threshold_number_of_nodes};
        f = fopen('./graph.mat');
        update_graph = f==-1;
        if f>=0
            fclose(f);
            load('./graph.mat');
            %compare the cells
            cells_are_equal = true;
            for i = 1:length(graph_args)
                if ~isstruct(graph_args{i})
                    field_1 = graph_args{i};
                    field_2 = saved_graph_args{i};
                    cells_are_equal = cells_are_equal & ~any(field_1(:) ~= field_2(:));
                else
                    fields_names = fields(graph_args{i});
                    for field_id = 1:length(fields_names)
                        field_1 = getfield(graph_args{i},fields_names{field_id});
                        field_2 = getfield(saved_graph_args{i},fields_names{field_id});
                        cells_are_equal = cells_are_equal & ~any( field_1(:) ~= field_2(:)); 
                    end
                end
            end
            update_graph = update_graph || ~cells_are_equal;
        end
        if update_graph
            [nodes, edges] = ut.buildDynProgGraph(graph_args{:});
            saved_graph_args = graph_args;
            save('./graph.mat','nodes','edges','saved_graph_args')
        else
            fprintf('\nUsing an existing graph\n')
        end
    end

%% set obstacles and solve the 'optimization' using dynamic programming
    %considered obstacles
    obstacles_options = sol3.getObstacle;
    simulate_flag = ut.solutionImplemented(obstacles_options, 'getObstacle', simulate_flag);
    
    %for every set of obstacles, obtain the solution
    for obstacle_id = 1:length(obstacles_options)
        fprintf('\nExperiment %02d', obstacle_id);

        %retrieve obstacle position at this iteration
        obstacles.position = obstacles_options{obstacle_id};

        %explore the graph forward, by calculating the costs. 
        node_cost = sol3.assignCostsToNodes(nodes, edges, obstacles);
        simulate_flag = ut.solutionImplemented(node_cost, 'assignCostsToNodes', simulate_flag);
        
        %plot the graph
        if simulate_flag
        [ax] = ut.plotGraph(nodes, edges, path_to_track, 'plot_nodes',true, 'plot_terminal_nodes', true, 'plot_dead_ends', true, 'obstacles', obstacles);
        end
        
        %Obtain the optimal path using bellman's principle of optimality.
        [optimal_node_sequence] = sol3.backtrackNodeValues(nodes, edges, node_cost, sampling_time);
        simulate_flag = ut.solutionImplemented(optimal_node_sequence, 'backtrackNodeValues', simulate_flag);

        %plot optimal trajectory
        if simulate_flag
        ut.plotDynProgTraj(nodes, optimal_node_sequence, ax);
        end
    end