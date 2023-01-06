function [out] = minimize_with_traditional_genetic_algorithm(problem, params)

        
        % The higher mutation rate increases the exploration ability of the
        % GA but can result to a slow convergence of the algorithm

        % Problem 
        CostFunction = problem.CostFunction;
        nVar = problem.nVar;
        VarSize = [1, nVar];
        VarMin = problem.VarMin;
        VarMax = problem.VarMax;
        crossover_type = problem.CrossFunctionType;
        selection_type = problem.ParentSelectionType;
        
        % Params
        MaxIt = params.MaxIt;
        nPop = params.nPop;
        beta = params.beta;
        pC = params.pC;                             % Percentage of children 
        nC = round(pC*nPop/2)*2;                    % Number of offsprings (to make sure it is an even number)
        pM = params.pM;                             % Percentage of children 
        nM = round(pM*nPop/2)*2;                    % Number of offsprings (to make sure it is an even number)
        mu = params.mu;                             % Mutation rate 
        sigma = params.sigma;  
        gamma = params.gamma;
        epsilon = params.epsilon;
        ShowIterInfo = params.ShowIterInfo;
        
        
        % Template for Empty Individuals
        empty_individual.Position = [];
        empty_individual.Cost = [];
        
        % Best solution ever found
        BestSolution.Cost = inf;
        
        % initialization
        pop = repmat(empty_individual, nPop, 1);
        for i=1:nPop
            
            % Generate random solution
            pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
            
            % Evaluate solution
            pop(i).Cost = CostFunction(pop(i).Position);
            
            % Compare solution to the best solution ever found
            if pop(i).Cost < BestSolution.Cost
                BestSolution = pop(i);
            end
                        
        end
        
        % Best cost of Iterations
        BestCosts = zeros(MaxIt, 1);
        
        % Main loop
        for it=1:MaxIt
            
            % Selection probabilities
            c = [pop.Cost];
            avgC = mean(c);
            if avgC ~= 0
                c = c/avgC;
            end
            probs = exp(-beta*c);
            
            % Initialize offsprings population
            popc = repmat(empty_individual, nC/2, 2);
            
            % Crossover
            for k = 1:nC/2
                
                % Select parents
                if selection_type == "random"
                    q = randperm(nPop);
                    p1 = pop(q(1));
                    p2 = pop(q(2));
                elseif selection_type == "roulette"
                    p1 = pop(RouletteWheelSelection(probs));
                    p2 = pop(RouletteWheelSelection(probs));
                end
                
                % Perform crossover
                if crossover_type == "singlepoint"
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        SinglePointCrossover(p1.Position, p2.Position);   
                elseif crossover_type == "doublepoint"
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        DoublePointCrossover(p1.Position, p2.Position); 
                elseif crossover_type == "uniform"
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        UniformCrossover(p1.Position, p2.Position, gamma);   
                elseif crossover_type == "combined_sdu"
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        CombinedCrossover(p1.Position, p2.Position, gamma);    
                elseif crossover_type == "sbx"
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        SBXCrossover(p1.Position, p2.Position);       
                elseif crossover_type == "gsbx"
                    grads_p1 = find_grads(CostFunction, p1.Position, epsilon);
                    grads_p2 = find_grads(CostFunction, p2.Position, epsilon);
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        gSBXCrossover(p1.Position, p2.Position, grads_p1, grads_p2);        
                end
            end
            
            
            % Convert popc to single column matrix
            popcc = popc(:);
            popc = popc(:);
            
            % Mutation
            for l = 1:nC
                
                % Perform Mutation
                popc(l).Position = Mutate(popc(l).Position, mu, sigma);
                
                % Checks for the bounds
                popc(l).Position = max(popc(l).Position, VarMin);
                popc(l).Position = min(popc(l).Position, VarMax);
                
                % Evaluation
                popc(l).Cost = CostFunction(popc(l).Position);
                
                % Compare solution to the best solution ever found
                if popc(l).Cost < BestSolution.Cost
                    BestSolution = popc(l);
                end
            end
            
            % Merge and Sort Populations
            pop = SortPopulation([pop; popc; popcc]);
            
            % Remove Extra Individuals
            pop = pop(1:nPop);
            
            % Update Best Cost of Iterations
            BestCosts(it) = BestSolution.Cost;
            
            % Display Iteration Information
            if ShowIterInfo
                disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
            end
            
        end
        
        % Results
        out.pop = pop;
        out.BestSolution = BestSolution;
        out.BestCosts = BestCosts;
        
        
end
