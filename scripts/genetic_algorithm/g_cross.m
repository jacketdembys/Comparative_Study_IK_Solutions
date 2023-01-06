function z = g_cross(x1,x2)

    % single point crossover

    n=numel(x1);
    r=randi(n-1);
    y1=[x1(1:r) x2(r+1:end)];
    y2=[x2(1:r) x1(r+1:end)];

    z=[y1;y2];
  
end

