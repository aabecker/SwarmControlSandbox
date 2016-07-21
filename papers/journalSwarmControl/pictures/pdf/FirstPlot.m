clear all;

%....Results taken from 15 simulations...%
result050 = [124.307; 103.166; 436.978;242.923;93.456;201.714;458.818;159.175;89.401;103.069;
    77.416;213.125;119.876;145.832;268.768;231.005;94.334;191.861;104.442;294.406]; 

result075 = [136.713;73.953;92.391;84.117;74.035;126.548;149.497;140.038;136.118;223.616;
    98.687;142.678;107.452;108.786;145.359;111.328;94.657;85.641;99.769;158.786];
result100 = [136.713;73.953;92.391;84.117;74.035;126.548;149.497;140.038;136.118;223.616;
    118.71;88.957;126.567;77.769;75.929;76.603;140.051;101.785;69.3;105.819];
%result100 = [76.868;93.446;68.473;87.333;84.375;116.218;108.54;106.594;81.217;74.186];
result125 = [92.251;66.214;66.464;111.814;69.95;76.919;80.384;75.293;76.832;106.273;
    81.884;92.547;129.438;129.397;95.039;108.602;78.384;104.817;139.525;85.677];
result150 = [55.473;70.327;85.823;83.939;128.285;84.928;94.669;72.741;140.879;51.772;
    85.115;102.33;52.541;208.345;110.582;66.21;96.083;121.967;103.819;63.374];

%...to plot the boxplot...
Time = [result050; result075; result100; result125; result150];


 group = ['result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';
     'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';'result050';
     'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';
      'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';'result075';
    'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';
    'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';'result100';
    'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';
    'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';'result125';
    'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';
     'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';'result150';];

boxplot(Time, group);
title('How #robots will affect time');
xlabel('Algorithm');
ylabel('Time (s)');