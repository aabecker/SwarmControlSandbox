clear all;

%....Results taken from 15 simulations...%
result050 = [93.733;128.257;90.424;82.174;104.578;160.626;148.221;82.222;129.733;99.704]; 

result075 = [122.759;77.254;112.602;97.494;81.712;89.443;87.103;83.894;143.415;125.442];
result100 = [77.492;75.84;88.577;106.791;105.187;121.288;107.253;84.696;74.841;154.204];
%result100 = [76.868;93.446;68.473;87.333;84.375;116.218;108.54;106.594;81.217;74.186];
result125 = [137.918;76.143;89.689;130.424;100.966;119.111;83.886;72.423;182.625;109.818];
result150 = [96.93;64.471;92.351;114.401;100.407;75.588;101.824;89.568;143.857;112.561];

%...to plot the boxplot...
Time = [result050; result075; result100; result125; result150];


 group = ['result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';
     %'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';
     'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';
      %'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';
    'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';
    %'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';
    'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';
    %'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';
    %'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';
     'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';];

boxplot(Time, group);
title('How Object Density will affect time');
xlabel('Algorithm');
ylabel('Time (s)');