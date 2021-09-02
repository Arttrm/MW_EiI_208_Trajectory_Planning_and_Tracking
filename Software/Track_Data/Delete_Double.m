% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% Delete_Double
% Version : 1.0
% Date : 09/07/2021
% ======================================================================= %

function [X_2,Y_2] = Delete_Double(X,Y)

X_2=X(1);
Y_2=Y(1);

for i=2:length(X)
    if X(i)==X_2(end) && Y(i)==Y_2(end)
        [X_2(end);Y_2(end)];
    else
        X_2=[X_2,X(i)];
        Y_2=[Y_2,Y(i)];
    end
end

end

