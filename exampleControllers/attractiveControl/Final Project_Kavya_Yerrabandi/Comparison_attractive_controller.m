clear all;

%....Results taken from 15 simulations...%
MDP_Kavya = [32;27;21;27;24;59;27;28;25;27;39;26;26;25;28];
BFS_Janani = [35;32;45;45;35;46;38;45;32;33;92;33;37;40;44];

Time = [MDP_Kavya; BFS_Janani];

group = ['MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'MDP_Kavya ';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani';'BFS_Janani'; ];

boxplot(Time,group);
title('Time taken by the Robots for each gradient model');
xlabel('Gradient Model used for the attractive controller');
ylabel('Time in Seconds');