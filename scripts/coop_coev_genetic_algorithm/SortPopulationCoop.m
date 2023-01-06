function [popr,BestSolution]  = SortPopulationCoop(pop, array, nVar, nPop, CostFunction, BestSolution)

    [~,so] = sort([array(:,end)]);
    pops = array(so,:);
    popr = pop;
    b = BestSolution;
    for i=1:nVar
        popr(i).Position = pops(1:nPop,i)';
        for j=1:nPop
            b.Position(i) = pop(i).Position(j);
            popr(i).Cost(j) = CostFunction(b.Position);
            
            if popr(i).Cost(j) < BestSolution.Cost
                BestSolution.Position = b.Position;
                BestSolution.Cost = popr(i).Cost(j);
            end 

        end
    end

end