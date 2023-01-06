%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/coop_coev_genetic_algorithm/")
addpath("scripts/particle_swarm_optimization/")
rng default                                 % For reproducibility

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
npop = 100;                                         % Population size
max_steps = 500;                                    % maximum number of iterations for all the algorithms
n = 6;                                              % number of variables or dimensions
rate_mut=0.5;     %0.8                              % rate of mutation for genetic algorithms
rate_cross=1;   %1                                  % rate of cross-over for genetic algorithms: 1
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
params.beta = 1;
params.pC = rate_cross;                                       % 1
params.pM = rate_mut;                                         % 1
params.mu = 1;
params.sigma = 0.1;
params.gamma = 0;
params.epsilon = epsilon;
params.ShowIterInfo = true;                     % Flag for showing iteration info: true,false

D = data(1,1:6);
U = unit_chosen;
d = 6;
ot = "poserpyr";

% Minimize with GA 
FitnessFunction = @(x) IK_cost_b(x,D,U,d,ot);
%{
[x_val, f_val] = ga(FitnessFunction,n,[],[],[],[],lb,ub);
%}


options = optimoptions('ga', ...,
                        'PlotFcn', (@gaplotbestf), ...
                        'Display', 'final', ...
                        'PopulationSize', 100, ...
                        'CreationFcn', @gacreationuniform, ...,
                        'Selection', @selectionstochunif, ...
                        'FitnessScalingFcn', @fitscalingrank, ...,
                        'EliteCount', 5, ...
                        'CrossoverFraction', 0.8, ...
                        'CrossoverFcn', @crossoverscattered, ...
                        'MutationFcn', @mutationadaptfeasible, ...
                        'MaxStallGenerations', 50, ...
                        'MaxGeneration', 200);
%{
options = optimoptions('ga', ...,
                        'PlotFcn', (@gaplotbestf), ...
                        'Display', 'final', ...
                        'PopulationSize', 200, ...
                        'CreationFcn', @gacreationuniform, ...,
                        'Selection', @selectionstochunif, ...
                        'FitnessScalingFcn', @fitscalingrank, ...,
                        'EliteCount', 3, ...
                        'CrossoverFraction', 0.8, ...
                        'CrossoverFcn', @crossoverscattered, ...
                        'Mutation', @mutationadaptfeasible, ...
                        'MaxStallGenerations', 100, ...
                        'MaxGeneration', 1000);
                        %}
[x_val, f_val, exitflag, output, population, scores] = ga(FitnessFunction,n,[],[],[],[],lb,ub,[],options);

robot = 'RRRRRR';
summarize(robot, x_val, unit_chosen, unit_applied, d, D)

%{
figure(1)
hold on
plot(recap_singlepoint_ga, 'LineWidth', 2)
hold off
title("Objective Function")
ylabel("Minimum Objective Value")
xlabel("Steps")
lgd = legend("GA");
%}






