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
%  Aaron Becker, March 11, 2015

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0,'DefaultAxesFontSize',18)
format compact
pauseOn = false;  %setting this to 'true' is useful for teaching, because it pauses between each graph
World = map;
% World = [  %this world is asymmetrical, so it can be used to talk about POMDPS
%     1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
%     1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
%     1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
%     1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1
%     1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1
%     1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1
%     1 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%     1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];

f1 = figure(1); clf
set(f1,'units','normalized','outerposition',[0 0 1 1])
colormap(gray)
%colormap(jet)
R = -500*ones(size(World)); %-50 reward for obstacles
R(World==0) = -1; %small penalty for not being at the goal
R(goalX,goalY) = 100; %goal state has big reward

% %  DRAW THE WORLD, REWARD, ANIMATE VALUE ITERATION, DISPLAY POLICY
% subplot(2,2,1)
% imagesc(~World);
% set(gca,'Xtick',[], 'Ytick',[])
% axis equal
% axis tight
% text(25,-1,'World','HorizontalAlignment','center','FontSize',18)
% drawnow
% if pauseOn; pause(); end %#ok<*UNRCH>
% 
% subplot(2,2,2)
% imagesc(R);
% axis equal
% axis tight
% set(gca, 'Xtick',[], 'Ytick',[])
% text(25,-1,'Reward function','HorizontalAlignment','center','FontSize',18)
% drawnow
%if pauseOn; pause(); end


% V_hat = MDP_discrete_value_iteration(R,World,false);
% if pauseOn; pause(); end
% 
% DrawPolicy(V_hat,World,false);
% if pauseOn; pause(); end

[V_hat_prob, DX, DY] = MDP_discrete_value_iteration(R,World,true);
%if pauseOn; pause(); end
%Gradient = V_hat_prob;

%DrawPolicy(V_hat_prob,World,true);
%if pauseOn; pause(); end

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
        if ~prob
            figure(10)
            %subplot(2,2,3)
        else
            figure(10)
            %subplot(2,2,4)
        end
        hImageV =   imagesc(V_hat);
        axis equal
        axis tight
        set(gca,'Xtick',[], 'Ytick',[])
        %htext = text(25,-1,'Vhat','HorizontalAlignment','center','FontSize',18);
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
                %[~,bestPayoff] = policy_MDP(xIND(i),V_hat,prob);
                V_hat(xIND(i)) = gamma*( R(xIND(i)) + bestPayoff );
            end
            iteration = iteration+1;
            set(hImageV,'cdata',V_hat);
            %set(htext,'String',['Vhat, iteration ',num2str(iteration)])
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

    function [DX,DY] = DrawPolicy(V_hat,World,prob)
        % uses arrows to draw the optimal policy according to the Value
        % Funtion approximation V_hat
        xIND = find(World == 0);
        %subplot(3,2,4)
        fh = figure(); clf
        %colormap(gray)
        set(fh,'units','normalized','outerposition',[0 0 1 1])
        imagesc(V_hat);
        axis equal
        axis tight
        set(gca,'Xtick',[], 'Ytick',[])
        if prob
            str = 'Policy under probabilistic motion model';
        else
            str = 'Policy under deterministic motion model';
        end
        text(25,-1,str,'HorizontalAlignment','center','FontSize',18);
        [X,Y] = meshgrid(1:size(World,2),1:size(World,1));
        DX = zeros(size(X));
        DY = zeros(size(Y));
        
        for i = 1:numel(xIND)
            [Iy,Ix] = ind2sub(size(V_hat),xIND(i));
            [bestMove,~] = policy_MDP(xIND(i),V_hat,prob);
            DX(Iy,Ix) = bestMove(1);
            DY(Iy,Ix) = bestMove(2);
        end
        hold on; hq=quiver(X,Y,DY,DX,0.5,'color',[0,0,0]); hold off
        set(hq,'linewidth',2);
        drawnow
    end

end

