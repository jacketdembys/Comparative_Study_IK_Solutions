function [out] = minimize_with_genetic_algorithm(problem, params)

        
        fmin = problem.CostFunction;            % Cost Function
        cross = problem.CrossFunction;          % Crossover operator
        cross_type = problem.CrossFunctionType;
        n = problem.nVar;                       % Number of unknowns or decision variables
               
        npop = params.npop;                     % Population size or Swarm Size
        rate_mut = params.rate_mut;             % rate of mutation
        rate_cross = params.rate_cross;         % rate of crossover
        maxiter = params.max_steps;             % Maximum number of iterations
        lb = params.lb;                         % Lower Bound of decision variables
        ub = params.ub;                         % Upper Bound of decision variables
        epsilon = params.epsilon;

        %% initialize a struct to old the population cost and number
        pop.pos=[];
        pop.cost=[];
        popall=repmat(pop,[1 npop]);

        %% set the number of mutation and cross-over based on the chosen rate
        number_mut=round(npop*rate_mut);
        number_cross=round(npop*rate_cross);

        %% prepare the number of mutation and cross-over in the entire population
        popallm=repmat(pop,[1 number_mut]);
        popallc=repmat(pop,[1 number_cross*2]);


        %% randomly intialize the populations and evalate each individual in the population
        for i=1:npop
            %popall(i).pos=randi(100,[1 n]);
            %popall(i).pos=unifrnd(1,100,1,n);
            popall(i).pos=unifrnd(lb,ub,1,n);
            popall(i).cost=cost(fmin, popall(i).pos);
        end


        %% main iteration based on the maximum number of generations
        zcost=zeros(maxiter,1);
        for j=1:maxiter
           for m=1:number_mut
               r=randi(npop);
               popallm(m).pos=mutate(popall(r).pos);
               popallm(m).cost=cost(fmin, popallm(m).pos);
           end

           index=1;
           for c=1:number_cross
               t=randperm(npop);
               r1=t(1);
               r2=t(2);
               if cross_type == "gsbx"
                   grads_r1 = find_grads(fmin, popall(r1).pos, epsilon);
                   grads_r2 = find_grads(fmin, popall(r2).pos, epsilon);
                   ww=cross(popall(r1).pos,popall(r2).pos, grads_r1', grads_r2');
               else                  
                   ww=cross(popall(r1).pos,popall(r2).pos);
               end


               popallc(index).pos=ww(1,:);
               popallc(index).cost=cost(fmin, popallc(index).pos);

               index=index+1;

               popallc(index).pos=ww(2,:);
               popallc(index).cost=cost(fmin, popallc(index).pos);
               index=index+1;

           end

           poptemp=[popall popallm popallc];

           co=[poptemp.cost];
           [a, b]=sort(co);

           popall=poptemp(b(1:npop));

           zcost(j)=popall(1).cost;
           zcost_pos=popall(1).pos;

           %figure(1);
           %plot(zcost(1:j));
           %hold off;
           %pause(0.3);
        end

        %zcost_pos
        recap_results = zcost;
        fval = popall(1).cost;
        
        out.BestCosts = recap_results;
        out.BestSolution.Cost = fval;
        out.BestSolution.Position = zcost_pos;
        


end
