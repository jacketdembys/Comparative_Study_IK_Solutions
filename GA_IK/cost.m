function z = cost(x)
   
  n=numel(x);
  alpha=[ 23 45 6 7 2 3 45 2 3 4];
  c=[ 2 3 1 5 3 4 5 6 1 9];
  
  sum1=sum(x.*c);
  w=40;
  
  net1=sum(x.*alpha);

  z=1/(sum1+abs(min(0,w-net1)));
end

