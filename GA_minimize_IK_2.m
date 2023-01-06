%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/coop_coev_genetic_algorithm/")
addpath("scripts/particle_swarm_optimization/")
rng default                                 % For reproducibility
plot_mode = "No";                               % For plots: Yes, No

%% load robot data
data = table2array(readtable("cec_data_points_RRRRRR.csv"));
str_unit_chosen = "m";

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
npop = 300;                                         % Population size
max_steps = 500;                                    % maximum number of iterations for all the algorithms
n = 6;                                              % number of variables or dimensions
rate_mut = 0.8;     %0.8                              % rate of mutation for genetic algorithms
rate_cross = 0.4;   %1                                  % rate of cross-over for genetic algorithms: 1
lb = deg2rad([-360,  50,  19, -360, -360, -360]);                                % Lower bound of the decision variables 
ub = deg2rad([ 360, 310, 341,  360,  360,  360]);                        % Upper bound of the decision variables    
epsilon = unifrnd(-0.001,0.001,n,1);                % incremental steps for numerical gradient approximation 
pose_tolerance = [(5/unit_applied),deg2rad(5)];     % expressed in mm

% Problem definition
problem.CostFunction = @(Q,D,U,d,ot) IK_cost(Q,D,U,d,ot);                    % Cost function
problem.nVar = n;                               % Number of unknowns or decision variables
problem.VarMin = lb;                            % Lower Bound of decision variables
problem.VarMax = ub;                            % Upper Bound of decision variables
problem.CrossFunctionType = "uniform";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)
problem.UnitChosen = unit_chosen;
problem.UnitApplied = unit_applied;
problem.DesiredTolerance = pose_tolerance;
problem.ObjectiveType = "poserpyr";             % poserpys, poserpyr, posehtm, position, posequaternion
if problem.ObjectiveType == "position"
    problem.Dimension = 3;    
    problem.DesiredPose = unit_chosen*data(1,1:3);
else    
    problem.Dimension = 6;   
    data(1,1:3) = unit_chosen*data(1,1:3);
    problem.DesiredPose = data(1,1:6);
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
params.ShowIterInfo = true;                     % Flag for showing iteration info: true,false



% Minimize with D GA 
fprintf("\nMinimizing the function with gSBX Crossover Cooperative Coevolutionary Genetic Algorithm\n")
problem.CrossFunctionType = "gsbx";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
tic
out = IK_minimized_with_traditional_genetic_algorithm_2(problem, params);
toc 
recap_gsbx_ga = out.BestCosts;
fval_gsbx_ga = out.BestSolution.Cost;


if plot_mode == "Yes"
    figure(1)
    hold on
    plot(recap_singlepoint_ga, 'LineWidth', 2)
    plot(recap_doublepoint_ga, 'LineWidth', 2)
    plot(recap_uniform_ga, 'LineWidth', 2)
    plot(recap_combined_sdu_ga, 'LineWidth', 2)
    plot(recap_sbx_ga, 'LineWidth', 2)
    plot(recap_gsbx_ga, 'LineWidth', 2)
    hold off
    title("Objective Function")
    ylabel("Minimum Objective Value")
    xlabel("Steps")
    lgd = legend("GA-S", "GA-D", "GA-U", "GA-SDU", "GA-SBX", "GA-gSBX");
end





