clc;
close all;
clear all;
n=3;
npop=200;
rate_mut=0.8;
rate_cross=0.4;

pop.pos=[];
pop.cost=[];
popall=repmat(pop,[1 npop]);

number_mut=round(npop*rate_mut);
number_cross=round(npop*rate_cross);

popallm=repmat(pop,[1 number_mut]);
popallc=repmat(pop,[1 number_cross*2]);

maxiter=200;

for i=1:npop
    popall(i).pos=randi(100,[1 n]);
    popall(i).cost=cost(popall(i).pos);
end

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
