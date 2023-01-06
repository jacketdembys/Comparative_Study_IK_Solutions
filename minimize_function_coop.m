%% set paths and running modes
clc, clear, close all
addpath("scripts/gradient_descent/")
addpath("scripts/coop_coev_genetic_algorithm/")
addpath("scripts/particle_swarm_optimization/")
fmin_choice = "ml";                         % ml (mixed landscape (gemonn))
run_mode = "debug";                         % debug, test
rng default                                 % For reproducibility

%% set the initial settings
npop = 100;                             % Population size
learning_rate = 0.1;                    % Learning rate for gradient descent
max_steps = 1000;                       % maximum number of iterations for all the algorithms
n = 60;                                 % number of variables or dimensions
rate_mut=0.8;                           % rate of mutation for genetic algorithms
rate_cross=1;                           % rate of cross-over for genetic algorithms: 1
lb = -500;                              % Lower bound of the decision variables 
ub = 500;                               % Upper bound of the decision variables    
epsilon = unifrnd(-0.001,0.001,n,1);    % incremental steps for numerical gradient approximation

%% set the function to minimize
if strcmp (fmin_choice , "ml")
    fmin = @(x) sum(x(1:40).^2) - (1/2)*sum(x(41:60).*sin(sqrt(abs(x(41:60)))));
end
    

if run_mode == "debug"
    
    
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "singlepoint";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                                  % Population size or Swarm Size
    params.beta = 1;
    params.pC = rate_cross;                                       % 1
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = true;                     % Flag for showing iteration info: true,false

    %{
    % Minimize with S GA
    fprintf("\nMinimizing the function with Single Point Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_s_ga = out.BestCosts;
    fval_s_ga = out.BestSolution.Cost;
    
    
    % Minimize with D GA    
    fprintf("\nMinimizing the function with Double Point Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    problem.CrossFunctionType = "doublepoint";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_d_ga = out.BestCosts;
    fval_d_ga = out.BestSolution.Cost;
    %}
    % Minimize with D GA 
    fprintf("\nMinimizing the function with Uniform Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    problem.CrossFunctionType = "uniform";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_u_ga = out.BestCosts;
    fval_u_ga = out.BestSolution.Cost;
    
    % Minimize with Comb GA    
    fprintf("\nMinimizing the function with Combined SDU Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    problem.CrossFunctionType = "combined_sdu";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_csdu_ga = out.BestCosts;
    fval_csdu_ga = out.BestSolution.Cost;
    
    % Minimize with Comb GA 
    fprintf("\nMinimizing the function with SBX Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    problem.CrossFunctionType = "sbx";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_sbx_ga = out.BestCosts;
    fval_sbx_ga = out.BestSolution.Cost;
    
    % Minimize with Comb GA  
    %{
    fprintf("\nMinimizing the function with gSBX Crossover Cooperative Coevolutionary Genetic Algorithm\n")
    problem.CrossFunctionType = "gsbx";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    tic
    out = minimize_with_coop_coev_genetic_algorithm(problem, params);
    toc 
    recap_obj_gsbx_ga = out.BestCosts;
    fval_gsbx_ga = out.BestSolution.Cost;
    %}
    
    
    
    figure(1)
    hold on
    plot(recap_obj_s_ga, 'LineWidth', 2)
    plot(recap_obj_d_ga, 'LineWidth', 2)
    plot(recap_obj_u_ga, 'LineWidth', 2)
    plot(recap_obj_csdu_ga, 'LineWidth', 2)
    plot(recap_obj_sbx_ga, 'LineWidth', 2)
    %plot(recap_obj_gsbx_ga)
    hold off
    title("Objective Function")
    ylabel("Minimum Objective Value")
    xlabel("Steps")
    lgd = legend("S", "D", "U", "CSDU", "SBX");
    %lgd = legend("S", "D", "U", "CSDU", "SBX", "gSBX");
    
else 
    %% minimize with gradient descent
    fprintf("\nMinimizing the function with Gradient Descent\n")

    problem.CostFunction = fmin;
    problem.nVar = n; 

    params.max_steps = max_steps;
    params.learning_rate = learning_rate;
    params.epsilon = epsilon;    
    params.lb = lb;
    params.ub = ub;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    tic
    [out] = minimize_with_gradient_descent(problem, params);
    toc
    recap_obj_gd = out.BestCosts;
    fval_gd = out.BestSolution.Cost;


    %% minimize with single point genetic algorithm
    fprintf("\nMinimizing the function with Single Point Crossover Genetic Algorithm\n")
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "singlepoint";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                                  % Population size or Swarm Size
    params.beta = 1;
    params.pC = rate_cross;                                       % 1
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    % Minimize with GA
    tic
    out = minimize_with_traditional_genetic_algorithm(problem, params);
    toc 
    recap_obj_s_ga = out.BestCosts;
    fval_s_ga = out.BestSolution.Cost;

    %% minimize with double point genetic algorithm
    fprintf("\nMinimizing the function with Double Point Crossover Genetic Algorithm\n")
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "doublepoint";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                               % Population size or Swarm Size
    params.beta = 1;
    params.pC = rate_cross;
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    % Minimize with GA
    tic
    out = minimize_with_traditional_genetic_algorithm(problem, params);
    toc 
    recap_obj_d_ga = out.BestCosts;
    fval_d_ga = out.BestSolution.Cost;


    %% minimize with uniform genetic algorithm
    fprintf("\nMinimizing the function with Uniform Crossover Genetic Algorithm\n")
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "uniform";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                               % Population size or Swarm Size
    params.beta = 1;
    params.pC = rate_cross;
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    % Minimize with GA
    tic
    out = minimize_with_traditional_genetic_algorithm(problem, params);
    toc 
    recap_obj_u_ga = out.BestCosts;
    fval_u_ga = out.BestSolution.Cost;




    %% minimize with SBX genetic algorithm
    fprintf("\nMinimizing the function with SBX Crossover Genetic Algorithm\n")
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "sbx";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                               % Population size or Swarm Size
    params.beta = 1;
    params.pC = rate_cross;
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    % Minimize with GA
    tic
    out = minimize_with_traditional_genetic_algorithm(problem, params);
    toc 
    recap_obj_sbx_ga = out.BestCosts;
    fval_sbx_ga = out.BestSolution.Cost;

    %% minimize with gSBX genetic algorithm
    fprintf("\nMinimizing the function with gSBX Crossover Genetic Algorithm\n")
    % Problem definition
    problem.CostFunction = fmin;                    % Cost function
    problem.nVar = n;                               % Number of unknowns or decision variables
    problem.VarMin = lb;                            % Lower Bound of decision variables
    problem.VarMax = ub;                            % Upper Bound of decision variables
    problem.CrossFunctionType = "gsbx";      % singlepoint, doublepoint, uniform, combined_sdu, sbx, gsbx
    problem.ParentSelectionType = "random";         % random, roulette (roulette wheel selection)  
    rng default                                     % For reproducibility


    % Parameters of GA
    params.MaxIt = max_steps;                            % Maximum number of iterations
    params.nPop = npop;                                  % Population size or Swarm Size
    params.beta = 1;                                     %    
    params.pC = rate_cross;
    params.pM = rate_mut;                                       % 1
    params.mu = 1;
    params.sigma = 0.1;
    params.gamma = 0;
    params.epsilon = epsilon;
    params.ShowIterInfo = false;                     % Flag for showing iteration info

    % Minimize with GA
    tic
    out = minimize_with_traditional_genetic_algorithm(problem, params);
    toc 
    recap_obj_gsbx_ga = out.BestCosts;
    fval_gsbx_ga = out.BestSolution.Cost;


    %% minimize with Vanilla Particle Swarm Optimization
    fprintf("\nMinimizing the function with Particle Swarm Optimizarion\n")

    problem.CostFunction = fmin;                        % Cost function
    problem.nVar = n;                                   % Number of unknowns or decision variables
    problem.VarMin = lb;                                % Lower Bound of decision variables
    problem.VarMax = ub;                                % Upper Bound of decision variables

    use_constrictions = false;
    kappa = 1;
    phi1 = 2.05;
    phi2 = 2.05;
    phi = phi1 + phi2;
    chi = (2*kappa)/abs(2-phi-sqrt(phi^2-4*phi));

    params.MaxIt = max_steps;                           % Maximum number of iterations
    params.nPop = npop;                                 % Population size or Swarm Size
    params.ShowIterInfo = false;                        % Flag for showing iteration info

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

    % Minimize with PSO
    tic
    out = minimize_with_particle_swarm(problem, params);
    fval_pso = out.BestSolution.Cost;
    recap_obj_pso = out.BestCosts;
    toc




    %% visualizations
    figure(1)
    hold on
    plot(recap_obj_gd)
    plot(recap_obj_s_ga)
    plot(recap_obj_d_ga)
    plot(recap_obj_u_ga)
    plot(recap_obj_sbx_ga)
    plot(recap_obj_gsbx_ga)
    plot(recap_obj_pso)
    hold off
    title("Objective Function")
    ylabel("Minimum Objective Value")
    xlabel("Steps")
    lgd = legend("Gradient Descent", "Single Point Genetic Algorithm", "Double Point Genetic Algorithm", ...
         "Uniform Genetic Algorithm", "SBX Genetic Algorithm", "gSBX Genetic Algorithm", "Vanilla Particle Swarm Optimization");
end




