function y = Mutate(x,mu,sigma)

    
    flag = (rand(size(x)) < mu);
    y = x;
    r = randn(size(x));
    step = sigma*r(flag);
    y(flag) = x(flag) + step;   
    

    %{
    n=numel(x);
    r=randperm(n);
    t=round(n*0.3);
    select_index=r(1:t);

    x(select_index)=1-x(select_index);
    % x(select_index)=x(select_index)+sigma*randn(1,t);
    y=x;
    %}

end