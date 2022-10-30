function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;

%theta2 = acos(sqrt((X*X+Y*Y)/4));
%theta1 = atan(Y/X)-theta2/2;
S = solve([theta1>0,theta1<pi/2,theta2>0,theta2<2*pi],[cos(theta1)+cos(theta1+theta2)==X,sin(theta1)+sin(theta1+theta2)==Y],[theta1,theta2]);

rads1 = double(S.theta1);
rads2 = double(S.theta2);
