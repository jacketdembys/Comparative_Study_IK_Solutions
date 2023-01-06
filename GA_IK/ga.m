%% Prepare worskpace
clc; close all; clear all;

%% set important variables
n=3;                % number of variables
npop=200;           % number of indviduals in the population
rate_mut=0.8;       % rate of mutation
rate_cross=0.4;     % rate of cross-over
maxiter=200;        % maximum number of iteration / of generations

%% initialize a struct to old the popultation cost and number
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
    popall(i).pos=randi(100,[1 n]);
    popall(i).cost=cost(popall(i).pos);
end


%% main iteration based on the maximum number of generations
zcost=zeros(maxiter,1);
for j=1:maxiter
   for m=1:number_mut
       r=randi(npop);
       popallm(m).pos=mutate(popall(r).pos);
       popallm(m).cost=cost(popallm(m).pos);
   end
   
   index=1;
   for c=1:number_cross
       t=randperm(npop);
       r1=t(1);
       r2=t(2);
       ww=cross(popall(r1).pos,popall(r2).pos);
       
       
       popallc(index).pos=ww(1,:);
       popallc(index).cost=cost(popallc(index).pos);
       
       index=index+1;
       
       popallc(index).pos=ww(2,:);
       popallc(index).cost=cost(popallc(index).pos);
       index=index+1;
       
   end
   
   poptemp=[popall popallm popallc];
    
   co=[poptemp.cost];
   [a b]=sort(co);
   
   popall=poptemp(b(1:npop));
   
   zcost(j)=popall(1).cost;
   zcost_pos=popall(1).pos;
    
   figure(1);
   plot(zcost(1:j));
   hold off;
   pause(0.3);
end

zcost_pos
popall(1).cost
