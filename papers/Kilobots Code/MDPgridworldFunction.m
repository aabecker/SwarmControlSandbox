function [V_hat_prob, DX,DY]  = MDPgridworldExample(map, goalX, goalY)
%  Applies value iteration to learn a policy for a Markov Decision Process
%  (MDP) -- a robot in a grid world.
% 
% The world is freespaces (0) or obstacles (1). Each turn the robot can
% move in 8 directions, or stay in place. A reward function gives one
% freespace, the goal location, a high reward. All other freespaces have a
% small penalty, and obstacles have a large negative reward. Value
% iteration is used to learn an optimal 'policy', a function that assigns a
% control input to every possible location.
% 
% This function compares a deterministic robot, one that always executes
% movements perfectly, with a stochastic robot, that has a small
% probability of moving +/-45degrees from the commanded move.  The optimal
% policy for a stochastic robot avoids narrow passages and tries to move to
% the center of corridors.
% 
% From Chapter 14 in 'Probabilistic Robotics', ISBN-13: 978-0262201629,
% http://www.probabilistic-robotics.org
% 
%  Aaron Becker, March 11, 2015, Shiva Shahrokhi Modified Summer 2016.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'DefaultAxesFontSize',18)
format compact
pauseOn = false;  %setting this to 'true' is useful for teaching, because it pauses between each graph
World = map;


colormap(gray)
R = -100*ones(size(World)); %-100 reward for obstacles
R(World==0) = -1; %small negative reward for not being at the goal
R(goalX,goalY) = 100; %goal state has big reward


[V_hat_prob, DX, DY] = MDP_discrete_value_iteration(R,World,true);

    function[V_hat, DX,DY] = MDP_discrete_value_iteration(R,World,prob)
        % iterates on the value function approximation V_hat until the V_hat converges.
        V_hat_prev = zeros(size(World));
        V_hat = -100*ones(size(World));
        V_hat(World==0) = R(World==0);
        colormap(jet)
        gamma = 0.97;
        
        xIND = find(World == 0);
        iteration = 0;
        
        [X,Y] = meshgrid(1:size(World,2),1:size(World,1));
        DX = zeros(size(X));
        DY = zeros(size(Y));
        figure(10);
        hImageV =   imagesc(V_hat);
        axis equal
        axis tight
        set(gca,'Xtick',[], 'Ytick',[])
        iteration_limit = 200; %value function needs ~600 iterations to converge, but the policy converges after ~100 iterations
        while ~isequal(V_hat,V_hat_prev) && iteration < iteration_limit 
            V_hat_prev = V_hat;
            for i = 1:numel(xIND)  
                [bestMove,bestPayoff] = policy_MDP(xIND(i),V_hat,prob);
                rowL = size(V_hat,1);
                rowV = xIND(i)/rowL;
                Ix = ceil(rowV);
                Iy =  xIND(i)- rowL* floor(rowV);
                DX(Iy,Ix) = bestMove(1);
                DY(Iy,Ix) = bestMove(2);
                V_hat(xIND(i)) = gamma*( R(xIND(i)) + bestPayoff );
            end
            iteration = iteration+1;
            set(hImageV,'cdata',V_hat);
            drawnow       
        end
        hold on; hq=quiver(X,Y,DY,DX,0.5,'color',[0,0,0]); hold off
        set(hq,'linewidth',2);
    end

    function [bestMove,bestPayoff] = policy_MDP(index,V_hat,prob)
        %computes the best control action, the (move) that generates the
        %most (payoff) according to the current value function V_hat
        [Iy,Ix] = ind2sub(size(V_hat),index);
        moves = [1,0; 1,1; 0,1; -1,1; -1,0; -1,-1; 0,-1; 1,-1; 0,0]; 
        bestPayoff = -200; %negative infinity
        probStraight = 0.1;
        for k = [1,3,5,7,2,4,6,8,9]% This order tries straight moves before diagonals %1:size(moves,1) %
            move = [moves(k,1),moves(k,2)];
            if ~prob 
                payoff = V_hat(Iy+move(1),Ix+move(2));
            else    
                if k < 8 %move +45deg of command
                    moveR = [moves(k+1,1),moves(k+1,2)];
                else
                    moveR = [moves(1,1),  moves(1,2)];
                end
                if k>1%move -45deg of command
                    moveL = [moves(k-1,1),moves(k-1,2)]; 
                else
                    moveL = [moves(8,1),  moves(8,2)];
                end
                if isequal(move,[0,0])
                    moveR = [0,0];
                    moveL = [0,0];
                end
                payoff =  probStraight*V_hat(Iy+move(1), Ix+move(2) )+...
                    (1-probStraight)/2*V_hat(Iy+moveR(1),Ix+moveR(2))+...
                    (1-probStraight)/2*V_hat(Iy+moveL(1),Ix+moveL(2));
            end
            
            if payoff > bestPayoff
                bestPayoff = payoff;
                bestMove = move;
            end
        end
    end
end

