function [y1,y2] = gSBXCrossover(x1,x2,g1,g2)
  
    % gradient-based sbx crossover

    n = numel(x1);
    c = zeros(n, 2);
    for i = 1:n
              
        %% find eta
        eta = 20;

        %% sample and find mu1
        if g1(i)*(x1(i)-x2(i)) > 0
          mu1 = unifrnd(0,0.5);
        elseif g1(i)*(x1(i)-x2(i)) < 0
          mu1 = unifrnd(0.5+eps,1);
        else
          mu1 = unifrnd(0,1);
        end


        %% sample and find mu2
        if g2(i)*(x1(i)-x2(i)) < 0
          mu2 = unifrnd(0,0.5);
        elseif g2(i)*(x1(i)-x2(i)) > 0
          mu2 = unifrnd(0.5+eps,1);
        else
          mu2 = unifrnd(0,1);
        end

        %% calculate beta1
        if mu1 <= 0.5
          beta1 = (2*mu1)^(1/(eta+1));
        else
          beta1 = (2*(1-mu1))^(-1/(eta+1));
        end      

        %% calculate beta2      
        if mu2 <= 0.5
          beta2 = (2*mu2)^(1/(eta+1));
        else
          beta2 = (2*(1-mu2))^(-1/(eta+1));
        end

        %% calculate alpha1
        lambda = unifrnd(0,1);
        if lambda <= 0.5 && g1(i)*(x1(i)-x2(i)) > 0 && g2(i)*(x1(i)-x2(i)) < 0
          alpha1 = -beta2 - 1;
        else
          alpha1 = beta1 - 1;
        end

        %% calculate alpha2
        if lambda <= 0.5 && g1(i)*(x1(i)-x2(i)) > 0 && g2(i)*(x1(i)-x2(i)) < 0
          alpha2 = -beta1 - 1;
        else
          alpha2 = beta2 - 1;
        end

        %% repair alpha1
        if x1(i) == 0
          alpha1 = 0;
        end

        %% repair alpha2
        if x2(i) == 0
          alpha2 = 0;
        end         

        %% 2 children generated by SBX cross-over operator
        c(i,1) = x1(i) + alpha1*(x1(i)-x2(i))/2;
        c(i,2) = x2(i) - alpha2*(x1(i)-x2(i))/2;
      
    end
  
    y1 = c(:,1)';
    y2 = c(:,2)';
    %z = [y1,y2]';
end

