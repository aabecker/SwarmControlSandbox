% load TSC10D.txt;
% x = TSC10D(:,1);
% y = TSC10D(:,2);
% plot(x,y)
% hold on
% load TSC10S.txt;
% x2 = TSC10S(:,1);
% y2 = TSC10S(:,2);
% plot(x2,y2)
% 
% load TSC10Up.txt;
% x3 = TSC10Up(:,1);
% y3 = TSC10Up(:,2);
% plot(x3,y3)
% 
% 
% % load Straight4.txt;
% % x4 = Straight4(:,1);
% % y4 = Straight4(:,2);
% % plot(x4,y4)
% % 
% % load Straight5.txt;
% % x5 = Straight5(:,1);
% % y5 = Straight5(:,2);
% % plot(x5,y5)
% line([11 30] , [0 0]);
% hold off
% load OrientationCont2.txt;
% x = OrientationCont2(:,1);
% y = OrientationCont2(:,2);
% z = OrientationCont2(:,3);
% plot(x,y)
% hold on
% plot (x,z)
% 
% load OrientationCont3.txt;
% x2 = OrientationCont3(:,1);
% y2 = OrientationCont3(:,2);
% z2 = OrientationCont3(:,3);
% plot(x2,y2)
% 
% plot (x2,z2)
% load OrientationCont4.txt;
% x3 = OrientationCont4(:,1);
% y3 = OrientationCont4(:,2);
% z3 = OrientationCont4(:,3);
% plot(x3,y3)
% 
% plot (x3,z3)
% hold off


% load PivotFull.txt;
% x3 = PivotFull(:,1);
% y3 = PivotFull(:,2);
% plot(x3,y3, 'color', 'red')
% hold on
% load PivotHalf.txt;
% x2 = PivotHalf(:,1);
% y2 = PivotHalf(:,2);
% plot(x2,y2, 'color', 'blue')
% 
% 
% 
% load PivotQuart.txt;
% x = PivotQuart(:,1);
% y = PivotQuart(:,2);
% 
% plot(x,y, 'color','green')
% 
% load PivotZero.txt;
% x4 = PivotZero(:,1);
% y4 = PivotZero(:,2);
% 
% plot(x4,y4, 'color', 'cyan')
% hold off


% load Straight30C7.txt;
% x4 = Straight30C7(:,1);
% y4 = Straight30C7(:,2)* 180/ pi;
% plot(x4,y4)
% hold on
% line([10 35] , [0 0]);
% 
% load StraightN30C7.txt;
% x5 = StraightN30C7(:,1);
% y5 = StraightN30C7(:,2)* 180/ pi;
% plot(x5,y5)
% load Straight15C7.txt;
% x3 = Straight15C7(:,1);
% y3 = Straight15C7(:,2)* 180/ pi;
% plot(x3,y3)
% 
% 
% load StraightN15C7.txt;
% x2 = StraightN15C7(:,1);
% y2 = StraightN15C7(:,2)* 180/ pi;
% plot(x2,y2)
% load Straight0C7.txt;
% x = Straight0C7(:,1);
% y = Straight0C7(:,2);
% plot(x,y)

% load High.txt;
% x1 = 17- High(:,1);
% y1 = High(:,2);
% t = High(:,3);
% plot(t, atan2(y1,x1)*180/ pi);
% 
% load TSC10S.txt;
% x2 = TSC10S(:,1);
% y2 = TSC10S(:,2)* 180 / pi;
% plot(x2,y2)
% hold off

% load New1.txt;
% t = New1(:,1);
% goalx = New1(:,2);
% X = New1(:,3);
% goaly = New1(:,4);
% Y = New1(:,5);
% goalAng = New1(:,6);
% Ang = New1(:,7);
% 
% subplot(3,1,1)
% plot(t,goalx);
% hold on
% plot(t,X);
% hold off
% 
% subplot(3,1,2)
% plot(t,goaly);
% hold on
% plot(t,Y);
% hold off
% 
% subplot(3,1,3)
% plot(t,goalAng);
% hold on
% plot(t,Ang);
% hold off


load Linear.txt;
t = Linear(:,1);
goalx = Linear(:,2);
X = Linear(:,3);
goaly = Linear(:,4);
Y = Linear(:,5);
goalAng = Linear(:,6);
Ang = Linear(:,7);

subplot(3,1,1)
plot(t,goalx);
hold on
plot(t,X);
hold off

subplot(3,1,2)
plot(t,goaly);
hold on
plot(t,Y);
hold off

subplot(3,1,3)
plot(t,goalAng);
hold on
plot(t,Ang);
hold off



