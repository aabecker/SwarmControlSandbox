clear all;

%....Results taken from 15 simulations...%
MDP = [121; 104; 116; 185; 118; 90; 93; 93; 123; 97; 128; 118; 139; 129; 132]; %... Time taken with my gradient model

BFS = [98; 105; 154; 244; 154; 123; 123; 116; 140; 211; 249; 425; 490; 263; 217];%...Time taken with Shiva's gradient model

%...to plot the boxplot...
Time = [MDP; BFS];


group = ['MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'MDP';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS';'BFS'; ];

boxplot(Time,group);
title('Comparison of BFS and VI');
xlabel('Algorithm');
ylabel('Time (s)');