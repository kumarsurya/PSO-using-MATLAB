clc;
clear;
close all;
%% problem defination
CostFunction= @(x) sphere(x);
nVar=3;  %% number of unknown (decision variable)
VarSize=[1,nVar]; %%matrix size of decision variable
VarMin=-5.12; %% lower bound of decision variable
VarMax=5.12;  %% upper bound of decision variable
%% pso parameter
MaxItr=100; %% maximum nuber of iteration
nPop=30;    %%population size(swarmsize)
w=0.4;        %% interia coefficient
c1=2;
c2=2;       %% learnuing coefficient
%% pso intialization
empty_paricle.position=[];
empty_particle.velocity=[];
empty_particle.cost=[];
empty_particle.best.position=[];
empty_particle.best.cost=[];
% create population array
particle=repmat(empty_particle,nPop,1);

% intialization of global best
globalbest.cost=inf;
 % intialization population members
 for i=1:nPop
    % generate random solution 
    particle(i).position=unifrnd(VarMin,VarMax,VarSize);
    
    % intialization velocity
    particle(i).velocity=zeros(VarSize); 
    % Evaluation
    particle(i).cost=CostFunction(particle(i).position);
    
    % update the personal best
    particle(i).best.position=particle(i).position;
    particle(i).best.cost=particle(i).cost;
    
    % update globalbest
    if particle(i).best.cost < globalbest.cost
        globalbest = particle(i).best;
    end
     
    
 end  
% array to hold best cost value on each iteration
bestcost=zeros(MaxItr,1);
%% main loop of pso 
for it=1:MaxItr
    for i=1:nPop
        
        % update velocity
        particle(i).velocity=w*particle(i).velocity...
            +c1*rand(VarSize).*(particle(i).best.position - particle(i).position)+c2*rand(VarSize).*(globalbest.position-particle(i).position);
        
        % update position
        particle(i).position=particle(i).position + particle(i).velocity;
        
        %evaluation
        particle(i).cost=CostFunction(particle(i).position);
        % update personal best
        if particle(i).cost < particle(i).best.cost
            particle(i).best.position=particle(i).position;
            particle(i).best.cost=particle(i).cost;

             % update globalbest
            if particle(i).best.cost<globalbest.cost
                globalbest=particle(i).best
            end
            
        end   
    end
    
    %store the best cost value
    bestcost(it)=globalbest.cost;
    
     %display iteration information
     disp(['itertation ' num2str(it) ' : bestcost= ' num2str(bestcost(it))]);
end   
%% result
figure;
% plot(bestcosts,'LineWidth' 2);
semilogy(bestcost,'LineWidth',2);
xlabel('Iteration');
ylabel('bestcost');
grid on;