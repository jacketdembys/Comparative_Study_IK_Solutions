clc
close all
clear
n=3;
npop=100;

pop.pos=[];
pop.cost=[];
popall=repmat(pop,[1 npop]);

for i=1:npop
    popall(i).pos=unifrnd(-10,10,[1 3]);
    popall(i).cost=cost(popall(i).pos);
end
maxiter = 10;
correction_factor = 2.05;
inertia = 1.0;
zcost=zeros(maxiter,1);
Global = zeros(n,npop);
personal = zeros(n,npop);
V = zeros(n,npop);
co = zeros(maxiter,npop);

for k=1:maxiter
    
    co(k,:) = [popall.cost];
    [a1 b1]=sort(co(k,:));
    Global_best = popall(b1(end)).pos;
    Global = repmat(Global_best,[npop 1]);
    
%     for m=1:npop
%         [a2 b2]=sort(co(:,m),'descend');
%         if personal_best(1)==0
% %             personal = 
%         end
%     end
    
    
    for i=1:npop
        for j=1:n
            
            V(i,j) = inertia*unifrnd(0,1,[1 1])*V(i,j);
            V(i,j) = V(i,j) + correction_factor*unifrnd(0,1,[1 1])*personal(i,j);
            V(i,j) = V(i,j) + correction_factor*unifrnd(0,1,[1 1])*Global(i,j);
            
            
        end
        
    end
    
end






