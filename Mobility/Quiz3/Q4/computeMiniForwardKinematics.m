function [endeff] = computeMiniForwardKinematics(rads1,rads2)

alpha = pi + (rads1 + rads2) / 2;

beta = rads1 - rads2;

L_longer = 2.0;

L_shorter = 1.0;

syms x;
S = solve(cos(pi-beta/2) == (x^2 + L_shorter^2 - L_longer^2) / (2 * L_shorter * x));

L = double(S);

X = L * cos(alpha);

Y = L * sin(alpha);
endeff = [X, Y];