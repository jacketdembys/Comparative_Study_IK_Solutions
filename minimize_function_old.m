%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/genetic_algorithm/")
addpath("scripts/particle_swarm_algorithm/")
test_fminsearch = "No";                     % test the minimizaqtion with the "fminsearch" function
fmin_choice = "ml";                         % ml (mixed landscape (gemonn))
rng default                                 % For reproducibility

%% set the initial settings
npop = 100;
%number_of_generations = 100;
learning_rate = 0.1;
max_steps = 1000;
n = 60;                     % number of variables
rate_mut=0.8;               % rate of mutation
rate_cross=0.4;             % rate of cross-over

%% set the initial values (initial population)   
%x0 = randi([-500,500],n,1);
lb = -500; ub = 500; 
x0 = (ub-lb).*rand(n,1) + lb;
%x0 = unifrnd(a,b,n,1);

%% set the epsilon for the gradient descent
a = -0.001; b = 0.001;    
%epsilon = randi([-10,10],n,1);
epsilon = (b-a).*rand(n,1) + a;
%epsilon = unifrnd(a,b,n,1);

%% set the function to minimize
if strcmp (fmin_choice , "ml")
    fmin = @(x) sum(x(1:40).^2) - (1/2)*sum(x(41:60).*sin(sqrt(abs(x(41:60)))));
end
    
if strcmp(test_fminsearch, "Yes")
    %% use fminsearch in-built optimizer (Nelder-Mead simplex direct search search algorithm)
    options = optimset('Display','iter', 'PlotFcns',@optimplotfval, 'MaxIter', max_steps);
    [xmin,fval,exitflag,output] = fminsearch(fmin, x0, options);
end

%% minimize with gradient descent
fprintf("\nMinimizing the function with Gradient Descent\n")
x = x0;
tic
[recap_obj_gd, fval_gd] = minimize_with_gradient_descent_old(fmin, x, max_steps, learning_rate, epsilon);
toc

%% minimize with vanilla genetic algorithm
%n=60;                   % number of variables
%npop=100;               % number of indviduals in the population
%rate_mut=0.8;           % rate of mutation
%rate_cross=0.4;         % rate of cross-over
%max_steps=100;            % maximum number of iteration / of generations

fprintf("\nMinimizing the function with Vanilla Genetic Algorithm\n")
tic
[recap_obj_ga, fval_ga] = minimize_with_vanilla_genetic_algorithm(fmin, n, npop, rate_mut, rate_cross, max_steps, lb, ub);
toc 

%% minimize with SBX genetic algorithm
%n=60;                   % number of variables
%npop=100;               % number of indviduals in the population
%rate_mut=0.8;           % rate of mutation
%rate_cross=0.4;         % rate of cross-over
%max_steps=100;            % maximum number of iteration / of generations

fprintf("\nMinimizing the function with SBX Genetic Algorithm\n")
tic
[recap_obj_sbx_ga, fval_sbx_ga] = minimize_with_sbx_genetic_algorithm(fmin, n, npop, rate_mut, rate_cross, max_steps, lb, ub);
toc 

%% minimize with gSBX genetic algorithm

fprintf("\nMinimizing the function with gSBX Genetic Algorithm\n")
tic
[recap_obj_gsbx_ga, fval_gsbx_ga] = minimize_with_gsbx_genetic_algorithm(fmin, n, npop, rate_mut, rate_cross, max_steps, lb, ub, epsilon);
toc 

%% minimize with Vanilla Particle Swarm Optimization
fprintf("\nMinimizing the function with Particle Swarm Optimizarion\n")

problem.CostFunction = fmin;                     % Cost function
problem.nVar = n;                               % Number of unknowns or decision variables
problem.VarMin = lb;                           % Lower Bound of decision variables
problem.VarMax = ub;                            % Upper Bound of decision variables

use_constrictions = false;
kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = (2*kappa)/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt = max_steps;                            % Maximum number of iterations
params.nPop = npop;                               % Population size or Swarm Size
params.ShowIterInfo = false;                     % Flag for showing iteration info

if use_constrictions
    params.w = chi;                                 % Inertia coefficient (w = 1)
    params.wdamp = 1;                               % damping ratio of inertia coefficient (wdamp = 0.99 w/o constrictions)
    params.c1 = chi*phi1;                           % Personal acceleration coefficient (personal learning)   
    params.c2 = chi*phi2;                           % Global or social acceleration coefficient (global learning)       
else
    params.w = 1;                                   % Inertia coefficient (w = 1)
    params.wdamp = 0.99;                            % damping ratio of inertia coefficient (wdamp = 0.99 w/o constrictions)
    params.c1 = 2;                                  % Personal acceleration coefficient (personal learning)   
    params.c2 = 2;                                  % Global or social acceleration coefficient (global learning)       
end

%% Minimize with PSO
tic
out = minimize_with_particle_swarm(problem, params);
fval_pso = out.BestSolution.Cost;
recap_obj_pso = out.BestCosts;
toc




%% visualizations
figure(1)
hold on
plot(recap_obj_gd)
plot(recap_obj_ga)
plot(recap_obj_sbx_ga)
plot(recap_obj_gsbx_ga)
plot(recap_obj_pso)
hold off
title("Objective Function")
ylabel("Minimum Objective Value")
xlabel("Steps")
lgd = legend("Gradient Descent", "Vanilla Genetic Algorithm", "SBX Genetic Algorithm", "gSBX Genetic Algorithm", "Vanilla Particle Swarm Optimization");



figure(2)
hold on
plot(recap_obj_gd)
plot(recap_obj_gsbx_ga)
hold off
title("Objective Function")
ylabel("Minimum Objective Value")
xlabel("Steps")
lgd = legend("Gradient Descent", "gSBX Genetic Algorithm");

