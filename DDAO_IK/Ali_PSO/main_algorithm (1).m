clc
clear all
close all
%% Problem Definition

% Cost Function
CostFunction = @(x) Sphere(x);

nVar = 5;           % Number of Unknown Variables
VarSize = [1 nVar]; % Unknown Variables Matrix Size

cons_sel = 1;
VarMin = 0;       % Unknown Variables Lower Bound
VarMax =10;       % Unknown Variables Upper Bound
%% runner Parameters

MaxIt = 200;        % Maximum Number of Iterations

nPop = 100;          % Population Size
%% Initialization

% Empty Structure for runners
runners.position = [];
runners.Cost = [];
runners.v=[];

% Initialize Population Array
pop = repmat(runners, nPop, 1);

% Initialize Best Solution
BestSol.Cost = inf;
X = zeros(MaxIt,3);
% Initialize Population Members
for i=1:nPop
    pop(i).position = unifrnd(VarMin, VarMax, VarSize);
    pop(i).v=0;
    pop(i).Cost = CostFunction(pop(i).position);
    
    if pop(i).Cost < BestSol.Cost
        BestSol = pop(i);
    end
end

% Initialize Best Cost Record
BestCosts = repmat(runners, MaxIt, 1);
diffrence = zeros(nPop-1,1); % diffrence between runner by  lower rank runner
c1=1;
c2=1;
newsol=runners;

co = [pop.Cost];
[a, b] = sort(abs(co),'ascend');
pop = pop(b(1:nPop));

Global_best.pos = [];
Global_best.cost = [];
Global_best.pos = pop(1).position;
Global_best.cost = pop(1).Cost;


%% runner main loop
for it=1:MaxIt
    co = [pop.Cost];
    [a ,b] = sort(abs(co),'ascend');
    pop = pop(b(1:nPop));
    
    if pop(1).Cost<= Global_best.cost
        Global_best.pos = pop(1).position;
        Global_best.cost = pop(1).Cost;
    end
    
    
    for k=2:nPop-1
        
        
        
        dX_upper = zeros(k-1,nVar);
        w=0.8;
        pop(k).v=w.*pop(k).v;
        for ii=1:k-1
            dX_upper(ii,:)= (pop(k-ii).position-pop(k).position);
            r1=rand(VarSize)*.01;
            pop(k).v=pop(k).v+c1*r1.*dX_upper(ii,:);
        end
        r3=rand(VarSize)*2;
        pop(k).v=w.*pop(k).v+r3.*(+Global_best.pos-pop(k).position);
        
        
        
        
        pop(k).position= pop(k).position+pop(k).v;
        if cons_sel == 1
            pop(k).position=min(500, pop(k).position);
            pop(k).position=max(-500, pop(k).position);
        end
        
        
        
    end
    
    
    r3 = rand(VarSize);
    pop(1).v=w.*pop(1).v+r3.*(+Global_best.pos-pop(1).position);
    pop(1).position=pop(1).position+pop(1).v;
    
        if cons_sel == 1
            pop(1).position=min(500, pop(1).position);
            pop(1).position=max(-500, pop(1).position);
        end
%     
    
    
    pop(nPop).v=w.*pop(nPop).v;
    for ii=1:nPop-1
        dX_upper(ii,:)= (1/(nPop-ii)).*upper_dif(pop(nPop-ii).position,pop(k).position);
        r1=rand(VarSize)*.01;
        pop(nPop).v=pop(nPop).v+c1*r1.*dX_upper(ii,:);
    end
    r3=rand(VarSize)*2;
    pop(nPop).v=pop(nPop).v+r3.*(+Global_best.pos-pop(nPop).position);
    
    pop(nPop).position= pop(nPop).position+pop(nPop).v;
        if cons_sel == 1
            pop(nPop).position=min(500, pop(nPop).position);
            pop(nPop).position=max(-500, pop(nPop).position);
        end
    
    
    
    
    for i=1:nPop
        pop(i).Cost=abs(CostFunction(pop(i).position));
        if pop(i).Cost < BestSol.Cost
            BestSol = pop(i);
        end
    end
    
    
    
    
    
    BestCosts(it)=pop(1);
    

    
    Error(it) = CostFunction(pop(1).position);
    Iter(it) = it;
     disp(['Iteration ' num2str(it) ' Cost =' num2str(BestCosts(it).Cost) ' Pos= ' num2str(BestCosts(it).position)]);
end

co = [pop.Cost];
[a ,b] = sort(abs(co),'ascend');
pop = pop(b(1:nPop));
% plot(Iter(1:end),Error(1:end),'-o')
% title('F3 function')
% ylabel('Cost')
% xlabel('Iteration')
% s = tf('s');
% Kp = X(1);
% Ks = X(2);
% Kd = X(3);
% PID = Kp*(1 + Ks/s + Kd*s);
% G = 1/(s^2 + 3*s + 10);
% G_c = PID*G;
% G_C = G_c/(1 + G_c);
% step(G_C)


