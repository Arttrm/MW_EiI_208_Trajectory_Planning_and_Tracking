% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% Performance_Calculator
% Version : 2.2
% Date : 25/08/2021
% ======================================================================= %

function [Performance_Info] = Performance_Calculator(xTrack,yTrack,out)

Data1=out.CarPosition.Data;
Data2=out.LapInfo.Data;

NCarPosition=1;
NLap=Data2(NCarPosition,1);

sError=[];

while NLap<4 % NCarPosition<25830
    
    % Look for closest point on trajectory
    NTrack1=dsearchn([xTrack;yTrack]',[Data1(NCarPosition,1);Data1(NCarPosition,2)]');
    
    % Look for second closest point on trajectory
    NTrack2=NTrack1-1;NTrack2=mod(NTrack2-1,length(xTrack)-1)+1;
    NTrack3=NTrack1+1;NTrack3=mod(NTrack3-1,length(xTrack)-1)+1;
    
    if norm([xTrack(NTrack2);yTrack(NTrack2)]-[Data1(NCarPosition,1);Data1(NCarPosition,2)])>...
       norm([xTrack(NTrack3);yTrack(NTrack3)]-[Data1(NCarPosition,1);Data1(NCarPosition,3)])
        NTrack2=NTrack3;
    end
    
    % Compute orthogonal distance between the car and the trajectory
    % https://fr.mathworks.com/matlabcentral/answers/95608-is-there-a-function-in-matlab-that-calculates-the-shortest-distance-from-a-point-to-a-line
    a=[xTrack(NTrack1);yTrack(NTrack1);0]-[xTrack(NTrack2);yTrack(NTrack2);0];
    b=[Data1(NCarPosition,1);Data1(NCarPosition,2);0]-[xTrack(NTrack2);yTrack(NTrack2);0];
    d=norm(cross(a,b))/norm(a);
    sError=[sError,d];
    
    % Next Car Position
    NCarPosition=NCarPosition+1;
    NLap=Data2(NCarPosition,1);
end

Performance_Info.sError=sError;

Performance_Info.sErrorPeak=max(sError);
Performance_Info.sErrorMean=mean(sError);

end

