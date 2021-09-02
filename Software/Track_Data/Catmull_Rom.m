% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% Catmull_Rom
% Version : 1.0
% Date : 13/06/2021
% ======================================================================= %
% https://lucidar.me/fr/mathematics/catmull-rom-splines/

function [P_X,P_Y] = Catmull_Rom(X,Y)

X=[X(end),X,X(1:2)];
Y=[Y(end),Y,Y(1:2)];

P_X=[]; 
P_Y=[]; 

for i=2:length(X)-2
    
    T=1/2; % 2 % Tau
    A=[0 1 0 0; -T 0 T 0; 2*T T-3 3-2*T -T; -T 2-T T-2 T];
    
    t=linspace(0,1,2*ceil(sqrt((X(i+1)-X(i))^2+(Y(i+1)-Y(i))^2)));
    
    for tt=t
        P_X=[P_X,[1,tt,tt^2,tt^3]*(A*X(i-1:i+2)')];
        P_Y=[P_Y,[1,tt,tt^2,tt^3]*(A*Y(i-1:i+2)')];
    end

end

