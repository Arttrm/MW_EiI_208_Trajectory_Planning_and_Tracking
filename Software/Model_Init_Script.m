% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% Model Init Script
% Version : 4.0
% Date : 01/06/2021
% ======================================================================= %

clc; clear all; close all;

addpath(genpath(pwd));

% ======================================================================= %
%% Physics Paramters

Physics_Param.gGravity        = 9.81;           % [m/s2]
Physics_Param.DAir            = 1.2;            % [kg/m3]

% ======================================================================= %
%% Tire Paramters

Tire_Param.muTire             = 1.75;           % [-]

Tire_Param.KTireLong          = 500;            % [rad^-1]
Tire_Param.KTireLat           = 1000;           % [rad^-1]

Tire_Param.CRollResistance    = 0.01;           % [-]

Tire_Param;

% ======================================================================= %
%% Motor Paramters

Motor_Param.PMotorMax         = 7.6*100;        % [W]

Motor_Param.nLimit            = 1000;           % [RPM]

Motor_Param.eDrivetrain       = 0.8*0.8*0.3;    % [-] %  \eta_i*\eta_d*correction_term

Motor_Param.rGearRatio1       = 3.3250;         % 84/48*38/20 
Motor_Param.rGearRatio2       = 2.90;         
  
% ======================================================================= %
%% Car Paramters

Car_Param.lCarWidth           = 0.2;            % [m]
Car_Param.hCarHeight          = 0.115;          % [m]
Car_Param.CCarDrag            = 0.3;            % [m]

Car_Param.lTrackWidth         = 0.165;          % [m]
Car_Param.lWheelbase          = 0.26;           % [m]

Car_Param.mWheel              = 0.03;           % [kg]
Car_Param.rWheel              = 0.03;           % [m]
Car_Param.IWheel              = 2.0760e-05;     % [kgm2] % 1/2*mWheel*(rWheel^2+0.022^2)
Car_Param.rWheelEffective     = 0.1885;         % [m]    % rWheel*2*pi

Car_Param.mCar                = 1.32;           % [kg]   % car mass
Car_Param.mCarSprung          = 1.198;          % [kg]   % mCar-4*mWheel
Car_Param.IzCar               = 0.0104;         % [kgm2] % 1/12*mCar*(lTrackWidth^2+lWheelbase^2)

Car_Param.hCenterGravity      = 0.02;           % [m]

Car_Param.hRollCenterF        = 0.01;           % [m]
Car_Param.hRollCenterR        = 0.01;           % [m]

Car_Param.KRollF              = 0.5;            % [N/m]
Car_Param.KRollR              = 0.5;            % [N/m]         

Car_Param;

% ======================================================================= %
%% Track Select

% Track_Param.NTrack = 1;  % O-shape
% Track_Param.NTrack = 2;  % Oval-shape

Track_Param.NTrack = 5;  % Test track
% Track_Param.NTrack = 6;  % Test track

% Track_Param.NTrack = 10; % MCT
% Track_Param.NTrack = 11; % LMCC

% Track_Param.NTrack = 20; % AWS

run Track_Data\Track_Data.m
Track_Param.rTrajOpt = 0.009 %0.005;

% ======================================================================= %
%% Offline Trajectory Optimization

% run Trajectory_Optimizer.m
% Track_Param.xTrack=xTraj;
% Track_Param.yTrack=yTraj;

clearvars -except Physics_Param Tire_Param Motor_Param Car_Param Track_Param out

% ======================================================================= %
%% Open Model

open Simulink_Model_v12;

return

% ======================================================================= %
%% Plot Figures

close all;

Data1=out.CarPosition.Data;

figure(1)
cmap = colormap(jet);
s=scatter(Data1(:,1),Data1(:,2),0.1,Data1(:,4),'.'); 
c=colorbar;
ylabel(c,'v_{Car} (m/s)')

hold on
plot(Track_Param.xTrack,Track_Param.yTrack,'k')
plot(Track_Param.xRightSide',Track_Param.yRightSide','k--')
k=plot(Track_Param.xLeftSide' ,Track_Param.yLeftSide' ,'k--');
sf=plot(Track_Param.xStartLine,Track_Param.yStartLine,'k-.');

s.Annotation.LegendInformation.IconDisplayStyle = 'off';
k.Annotation.LegendInformation.IconDisplayStyle = 'off';
sf.Annotation.LegendInformation.IconDisplayStyle = 'off';
legend('Track centerline','Track limits')
axis off
text(0.5,-5,'S/F\rightarrow','HorizontalAlignment','center')
% title('Car position and velocity')

% ======================================================================= %
%% Performance Calculation

% If you change the number of lap (NLap) in the simulink, please change it
% also in the performance calculator (line 25)
Performance_Info=Performance_Calculator(Track_Param.xTrack,Track_Param.yTrack,out)

run Trajectory_Optimizer;
% Performance_Info=Performance_Calculator(xTraj,yTraj,out)

% Speed
% mean(Data1(:,4))
% max(Data1(:,4))
