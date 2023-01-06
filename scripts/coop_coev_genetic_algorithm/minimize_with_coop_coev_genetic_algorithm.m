function [out] = minimize_with_coop_coev_genetic_algorithm(problem, params)

        
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
        VarSizePop = [1, nPop];
        beta = params.beta;
        pC = params.pC;                             % Percentage of children 
        nC = nVar; %round(pC*nPop/2)*2;                    % Number of offsprings (to make sure it is an even number)
        pM = params.pM;                             % Percentage of children 
        nM = round(pM*nPop/2)*2;                    % Number of offsprings (to make sure it is an even number)
        mu = params.mu;                             % Mutation rate 
        sigma = params.sigma;  
        gamma = params.gamma;
        epsilon = params.epsilon;
        ShowIterInfo = params.ShowIterInfo;
        
        %% Step 1: Problem Decomposition
        % Template for Empty Individuals
        empty_individual.Position = [];
        empty_individual.Cost = [];
        
        % Best solution ever found
        BestSolution.Position = unifrnd(VarMin, VarMax, VarSize);
        BestSolution.Cost = CostFunction(BestSolution.Position);
        %BestSolution.Cost = inf;
        
        %% Step 2: Random Initialization, Subpopulation and Context Vector Evaluations
        % initialization
        pop = repmat(empty_individual, nVar, 1);
        %popm = repmat(pop,[1 nM]);
        %popc = repmat(pop,[1 nC]);
        
        %bc = BestSolution;
        easySortArray = zeros(nVar*nPop*2, nVar+1);
        b = BestSolution;
        c = 1;
        for i=1:nVar
            
            % Generate random solution
            pop(i).Position = unifrnd(VarMin, VarMax, VarSizePop);
            
            % Evaluate solution
            for j=1:nPop
                %b = bc;
                b.Position(i) = pop(i).Position(j);
                pop(i).Cost(j) = CostFunction(b.Position);
                
                % store the formed individuals
                easySortArray(c, :) = [b.Position pop(i).Cost(j)];
                c = c + 1;
                
                % Compare solution to the best solution ever found
                if pop(i).Cost(j) < BestSolution.Cost
                    BestSolution.Position = b.Position;
                    BestSolution.Cost = pop(i).Cost(j);
                end                
            end                                    
        end
        
        % Best cost of Iterations
        BestCosts = zeros(MaxIt, 1);
        
        %% Evolution
        % Main loop
        for it=1:MaxIt            

            % Initialize offsprings population
            popc = repmat(empty_individual, nC/2, 2);

            for i=1:nC/2

                % Select parents to generate offsprings                
                q = randperm(nVar);
                p1 = pop(q(1));
                p2 = pop(q(2));
                %{
                [popc(i,1).Position, popc(i,2).Position] = ...
                        SinglePointCrossover(p1.Position, p2.Position); 
                %}
                    
                % Perform crossover
                if crossover_type == "singlepoint"
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        SinglePointCrossover(p1.Position, p2.Position);   
                elseif crossover_type == "doublepoint"
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        DoublePointCrossover(p1.Position, p2.Position); 
                elseif crossover_type == "uniform"
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        UniformCrossover(p1.Position, p2.Position, gamma);   
                elseif crossover_type == "combined_sdu"
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        CombinedCrossover(p1.Position, p2.Position, gamma);    
                elseif crossover_type == "sbx"
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        SBXCrossover(p1.Position, p2.Position);       
                elseif crossover_type == "gsbx"
                    grads_p1 = find_grads_coop(CostFunction, p1.Position, epsilon);
                    grads_p2 = find_grads_coop(CostFunction, p2.Position, epsilon);
                    [popc(i,1).Position, popc(i,2).Position] = ...
                        gSBXCrossover(p1.Position, p2.Position, grads_p1, grads_p2);        
                end

            end

            popc = popc(:);
            popcc = popc;

            % Mutation
            for l = 1:nC

                % Perform Mutation
                popc(l).Position = Mutate(popc(l).Position, mu, sigma);

                % Checks for the bounds
                popc(l).Position = max(popc(l).Position, VarMin);
                popc(l).Position = min(popc(l).Position, VarMax);

                % Evaluation
                %b = bc;
                for j=1:nPop
                    b.Position(l) = popc(l).Position(j);
                    popc(l).Cost(j) = CostFunction(b.Position);
                    
                    % store the formed individuals
                    easySortArray(c, :) = [b.Position popc(l).Cost(j)];
                    c = c + 1;                                   
                    
                    % Compare solution to the best solution ever found
                    % or update the context vector b = bc =
                    % BestSolution
                    if popc(l).Cost(j) < BestSolution.Cost
                        BestSolution.Position = b.Position;
                        BestSolution.Cost = popc(l).Cost(j);
                    end                
                end
            end

            % Merge and Sort Populations
            [pop, BestSolution] = SortPopulationCoop([pop; popc], easySortArray, nVar, nPop, CostFunction, BestSolution);

            % Remove Extra Individuals
            pop = pop(1:nVar);
            
            % Update Best Cost of Iterations
            BestCosts(it) = BestSolution.Cost;
            
            % Display Iteration Information
            if ShowIterInfo
                disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
            end
            
            % Update the context vector
            b = BestSolution;
            
        end
        
        % Results
        out.pop = pop;
        out.BestSolution = BestSolution;
        out.BestCosts = BestCosts;
        
        
end
