function [out] = IK_minimized_with_traditional_genetic_algorithm(problem, params)

        
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
        DesiredPose = problem.DesiredPose; 
        DesiredTolerance = problem.DesiredTolerance;
        UnitChosen = problem.UnitChosen;
        UnitApplied = problem.UnitApplied;
        Dimension = problem.Dimension;
        ObjectiveType = problem.ObjectiveType;
        
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
            %pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
            pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
            %pop(i).Position = [3.2363    2.7397    0.4123   -4.1514   -4.9490   -1.9607];

            
            % Evaluate solution
            pop(i).Cost = CostFunction(pop(i).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
            
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
                    grads_p1 = IK_find_grads(CostFunction, p1.Position, epsilon, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                    grads_p2 = IK_find_grads(CostFunction, p2.Position, epsilon, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                    [popc(k,1).Position, popc(k,2).Position] = ...
                        gSBXCrossover(p1.Position, p2.Position, grads_p1, grads_p2);        
                end
                
                
                popc(k,1).Position = max(popc(k,1).Position, VarMin);
                popc(k,1).Position = min(popc(k,1).Position, VarMax);
                
                popc(k,2).Position = max(popc(k,2).Position, VarMin);
                popc(k,2).Position = min(popc(k,2).Position, VarMax);
                
                % Evaluate the cost of the crossed individuals
                popc(k,1).Cost = CostFunction(popc(k,1).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                popc(k,2).Cost = CostFunction(popc(k,2).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                 
                % Compare solution to the best solution ever found
                if popc(k,1).Cost < BestSolution.Cost
                    BestSolution = popc(k,1);
                end
                
                if popc(k,2).Cost < BestSolution.Cost
                    BestSolution = popc(k,2);
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
                popc(l).Cost = CostFunction(popc(l).Position, DesiredPose, UnitChosen, Dimension, ObjectiveType);
                
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
            
            % stock the search with desired tolerance is reached
            if BestCosts(it) <= DesiredTolerance
                robot = 'RRRRRR';
                DH = getDH_rad_cec(robot, BestSolution.Position, UnitChosen);
                T_eval_current = forwardKinematics_rad_cec(DH); 
                CurrentPose = getPose_rad_cec(T_eval_current, Dimension)'; 
                if ObjectiveType == "position"
                    if abs(DesiredPose(1)-CurrentPose(1)) < DesiredTolerance(1) && abs(DesiredPose(2)-CurrentPose(2)) < DesiredTolerance(1) && abs(DesiredPose(3)-CurrentPose(3)) < DesiredTolerance(1)
                        disp(['Solution found within ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
                        fprintf("Desired Pose Vector:")
                        DesiredPose  %#ok<NOPRT>                        
                        fprintf("Estimated Pose Vector:")
                        CurrentPose  %#ok<NOPRT>                                  
                        fprintf("Estimated Joint Vector:")
                        CurrentJoints = BestSolution.Position  %#ok<NOPRT>                                     
                        fprintf("Position Error Joint Vector (mm):")              
                        position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT> 
                        break
                    end
                else                
                    if abs(DesiredPose(1)-CurrentPose(1)) < DesiredTolerance(1) && abs(DesiredPose(2)-CurrentPose(2)) < DesiredTolerance(1) && abs(DesiredPose(3)-CurrentPose(3)) < DesiredTolerance(1) && abs(DesiredPose(4)-CurrentPose(4)) < DesiredTolerance(2) && abs(DesiredPose(5)-CurrentPose(5)) < DesiredTolerance(2) && abs(DesiredPose(6)-CurrentPose(6)) < DesiredTolerance(2)
                        disp(['Solution found within ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
                        fprintf("Desired Pose Vector:")
                        DesiredPose  %#ok<NOPRT>                        
                        fprintf("Estimated Pose Vector:")
                        CurrentPose  %#ok<NOPRT>                                  
                        fprintf("Estimated Joint Vector:")
                        CurrentJoints = BestSolution.Position %#ok<NOPRT>                                      
                        fprintf("Position Error Joint Vector (mm):")              
                        position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT>  
                        fprintf("Orientation Error Joint Vector (degree):")         
                        orientation_error = rad2deg(sum(abs(DesiredPose(4:6)-CurrentPose(4:6)))/3) %#ok<NOPRT>
                        break
                    end
                end
            end
            
            
        end
        
        % Results
        out.pop = pop;
        out.BestSolution = BestSolution;
        out.BestCosts = BestCosts;
        
        if it >= MaxIt
            robot = 'RRRRRR';
            DH = getDH_rad_cec(robot, BestSolution.Position, UnitChosen);
            T_eval_current = forwardKinematics_rad_cec(DH); 
            CurrentPose = getPose_rad_cec(T_eval_current, Dimension)'; 
            disp(['Maximum Number of iterations reached ' num2str(it) ' iterations: Best Cost = ' num2str(BestCosts(it))]);
            fprintf("Desired Pose Vector:")
            DesiredPose  %#ok<NOPRT>                        
            fprintf("Estimated Pose Vector:")
            CurrentPose  %#ok<NOPRT>                                   
            fprintf("Estimated Joint Vector:")
            CurrentJoints = BestSolution.Position  %#ok<NOPRT>                                     
            fprintf("Position Error Joint Vector (mm):")              
            position_error = UnitApplied*sqrt(sum((DesiredPose(1:3)-CurrentPose(1:3)).^2)) %#ok<NOPRT>  
            if ObjectiveType ~= "position"
                fprintf("Orientation Error Joint Vector (degree):")         
                orientation_error = rad2deg(sum(abs(DesiredPose(4:6)-CurrentPose(4:6)))/3) %#ok<NOPRT>
            end
        end
        
        
end
