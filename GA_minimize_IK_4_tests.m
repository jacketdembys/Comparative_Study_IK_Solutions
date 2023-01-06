%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/coop_coev_genetic_algorithm/")
addpath("scripts/particle_swarm_optimization/")
rng default                                 % For reproducibility
plot_mode = "No";                               % For plots: Yes, No
run_mode = "debug";                             % debug, run
save_mode = "No";                             % Yes, No
show_iter_info = false;

%% load robot data
data = table2array(readtable("cec_data_points_RRRRRR.csv"));
str_unit_chosen = "m";
my_objective = "poserpyrr"; %poserpys, poserpyr, poserpyrr, posehtm, position, posequaternion

%% choose the unit to investigate
if(str_unit_chosen == "m")                     % choose the m
    unit_chosen = 1; 
    unit_applied = 1000;
elseif (str_unit_chosen == "dm")               % choose the dm
    unit_chosen = 10;
    unit_applied = 100;
elseif (str_unit_chosen == "cm")               % choose the cm
    unit_chosen = 100;
    unit_applied = 10;
elseif (str_unit_chosen == "mm")               % choose the mm
    unit_chosen = 1000;
    unit_applied = 1;
end  


%% set the initial settings
% set the run mode
if run_mode == "debug"
    samples = 1;
    %crossoverType = ["gsbx"];
    crossoverType = ["singlepoint", "doublepoint", "uniform", "sbx", "gsbx"];
    pops = [400];
else
    samples = length(data);
    crossoverType = ["singlepoint", "doublepoint", "uniform", "sbx", "gsbx"];
    pops = linspace(200,1000,10);
end



for p=1:length(pops)
    

    
    npop = pops(p);                                         % Population size 300 used for position objective
    max_steps = 1000;                                    % maximum number of iterations for all the algorithms
    n = 6;                                              % number of variables or dimensions
    rate_mut = 0.8;     %0.8                              % rate of mutation for genetic algorithms
    rate_cross = 0.4;   %1                                  % rate of cross-over for genetic algorithms: 1
    lb = deg2rad([-90,  50,  19, -360, -360, -360]);                                % Lower bound of the decision variables 
    ub = deg2rad([ 90, 310, 341,  360,  360,  360]);                        % Upper bound of the decision variables    
    epsilon = unifrnd(-0.001,0.001,n,1);                % incremental steps for numerical gradient approximation 
    pose_tolerance = [(1/unit_applied),deg2rad(1)];     % expressed in mm


 

    for c=1:length(crossoverType)


        %% initialize the summary table
        if my_objective == "position"
            summaryTable_gsbx_ga = zeros(samples, 4+n+3+max_steps);
        else
            summaryTable_gsbx_ga = zeros(samples, 5+n+6+max_steps);
        end


        for i=1:samples


            % Problem definition
            problem.CostFunction = @(Q,D,U,d,ot) IK_cost_b(Q,D,U,d,ot);                    % Cost function
            problem.nVar = n;                               % Number of unknowns or decision variables
            problem.VarMin = lb;                            % Lower Bound of decision variables
            problem.VarMax = ub;                            % Upper Bound of decision variables
            problem.CrossFunctionType = "uniform";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
            problem.ParentSelectionType = "random";         % random, ranking, roulette (roulette wheel selection)
            problem.UnitChosen = unit_chosen;
            problem.UnitApplied = unit_applied;
            problem.DesiredTolerance = pose_tolerance;
            problem.ObjectiveType = my_objective;             % poserpys, poserpyr, posehtm, position, posequaternion
            if problem.ObjectiveType == "position"
                problem.Dimension = 3;    
                problem.DesiredPose = unit_chosen*data(i,1:3);
            else    
                problem.Dimension = 6;   
                data(i,1:3) = unit_chosen*data(i,1:3);
                problem.DesiredPose = data(i,1:6);
            end
            rng default                                     % For reproducibility


            % Parameters of GA
            params.MaxIt = max_steps;                            % Maximum number of iterations
            params.nPop = npop;                                  % Population size or Swarm Size
            params.rateMutation = rate_mut;
            params.rateCross = rate_cross;
            params.mu = 1;
            params.sigma = 0.1;
            params.gamma = 0;
            params.epsilon = epsilon;
            params.ShowIterInfo = show_iter_info;                     % Flag for showing iteration info: true,false

            fprintf("\n---> Sample [%d]", i)

            % Minimize with D GA 
            problem.CrossFunctionType = crossoverType(c);      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
            fprintf("\nMinimizing the function with %s Crossover Genetic Algorithm with npop = %d\n", crossoverType(c), npop)
            tsart_gsbx_ga = tic;
            out_gsbx_ga = IK_minimized_with_traditional_genetic_algorithm_4(problem, params);
            tend_gsbx_ga = toc(tsart_gsbx_ga); 
            recap_gsbx_ga = out_gsbx_ga.BestCosts;
            fval_gsbx_ga = out_gsbx_ga.BestSolution.Cost;

            % check if the solution is found or not
            if out_gsbx_ga.Iterations < max_steps
                solutionFound_gsbx_ga = 1;
            else
                solutionFound_gsbx_ga = 0;
            end

            
            if problem.ObjectiveType == "position"
                summaryTable_gsbx_ga(i,:) = [solutionFound_gsbx_ga, tend_gsbx_ga, out_gsbx_ga.PositionError, ...
                                             out_gsbx_ga.Iterations, recap_gsbx_ga', out_gsbx_ga.BestSolution.Position, problem.DesiredPose];
            else                                     
                summaryTable_gsbx_ga(i,:) = [solutionFound_gsbx_ga, tend_gsbx_ga, out_gsbx_ga.PositionError, out_gsbx_ga.OrientationError, ...
                                             out_gsbx_ga.Iterations, recap_gsbx_ga', out_gsbx_ga.BestSolution.Position, problem.DesiredPose];
            end


        end

        % save summaryTable
        if save_mode == "Yes"
            filename = char("summaryTable_" + crossoverType(c) + "_" + problem.ObjectiveType  + "_" + num2str(npop) + ".mat");
            save(filename, 'summaryTable_gsbx_ga')
        end
    end
end


