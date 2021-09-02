% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% Traj_Opti
% Version : 1.0
% Date : 13/07/2021
% ======================================================================= %

%% Init

xyRightSide=[Track_Param.xRightSide;Track_Param.yRightSide];
xyLeftSide =[Track_Param.xLeftSide ;Track_Param.yLeftSide ];

xyRightSide=[xyRightSide,xyRightSide,xyRightSide,xyRightSide,xyRightSide];
xyLeftSide =[xyLeftSide ,xyLeftSide ,xyLeftSide ,xyLeftSide ,xyLeftSide];

N=length(xyRightSide);

rTrajOpt=Track_Param.rTrajOpt; % 0.001; % 1=Shortest

%% Shortest trajectory

H_S=zeros(N,N);

B_S=zeros(1,N);

for i=1:N-1

    % H
    dx  =(xyLeftSide(1,i)  -xyRightSide(1,i)  );
    dxp1=(xyLeftSide(1,i+1)-xyRightSide(1,i+1));

    h_x=[dx^2,-dx*dxp1;...
         -dx*dxp1,dxp1^2];

    dy  =(xyLeftSide(2,i)  -xyRightSide(2,i)  );
    dyp1=(xyLeftSide(2,i+1)-xyRightSide(2,i+1));

    h_y=[dy^2,-dy*dyp1;...
         -dy*dyp1,dyp1^2];

    H_S(i:i+1,i:i+1)=H_S(i:i+1,i:i+1)+h_x+h_y;

    % B
    b_x=2*(xyRightSide(1,i+1)-xyRightSide(1,i))*[-dx,dxp1];

    b_y=2*(xyRightSide(2,i+1)-xyRightSide(2,i))*[-dy,dyp1];

    B_S(i:i+1)=B_S(i:i+1)+b_x+b_y;
end

%% Minimum Curvature

H_C=zeros(N,N);

B_C=zeros(1,N);

for i=2:N-1

    % H
    dxm1=(xyLeftSide(1,i-1)-xyRightSide(1,i-1));
    dx  =(xyLeftSide(1,i)  -xyRightSide(1,i)  );
    dxp1=(xyLeftSide(1,i+1)-xyRightSide(1,i+1));

    h_x=[dxm1^2, -2*dxm1*dx, dxm1*dxp1;...
         -2*dx*dxm1, 4*dx^2, -2*dx*dxp1;...
         dxp1*dxm1, -2*dxp1*dx,dxp1^2];

    dym1=(xyLeftSide(2,i-1)-xyRightSide(2,i-1));
    dy  =(xyLeftSide(2,i)  -xyRightSide(2,i)  );
    dyp1=(xyLeftSide(2,i+1)-xyRightSide(2,i+1));

    h_y=[dym1^2, -2*dym1*dy, dym1*dyp1;...
         -2*dy*dym1, 4*dy^2, -2*dy*dyp1;...
         dyp1*dym1, -2*dyp1*dy,dyp1^2];

    H_C(i-1:i+1,i-1:i+1)=H_C(i-1:i+1,i-1:i+1)+h_x+h_y;

    % B
    b_x=2*(xyRightSide(1,i+1)-2*xyRightSide(1,i)+xyRightSide(1,i-1))*[dxm1, -2*dx, dxp1];

    b_y=2*(xyRightSide(2,i+1)-2*xyRightSide(2,i)+xyRightSide(2,i-1))*[dym1, -2*dy, dyp1];

    B_C(i-1:i+1)=B_C(i-1:i+1)+b_x+b_y;
end

%% Problem Formulation

H=rTrajOpt*H_S+(1-rTrajOpt)*H_C;

B=rTrajOpt*B_S+(1-rTrajOpt)*B_C;

%% Find current alpha

alpha_0=0.5;

%% quadprog

A=[eye(N);-eye(N)];
b=[0.95*ones(N,1);-0.05*ones(N,1)];

Aeq=zeros(N,N);
Aeq(1,1)=1;
beq=[alpha_0;zeros(N-1,1)];

% lb=zeros(N,1);
% ub=ones(N,1);
x0=0.5*ones(N,1);

coder.extrinsic('quadprog');
options=optimset('Display', 'off'); 
% options=optimoptions('quadprog','Algorithm','active-set');
alpha=0.5*ones(N,1);

alpha=quadprog(2*H,B',A,b,Aeq,beq,[],[],x0,options); % isfinite ?

%% Solution handling

xTraj=zeros(1,N);
yTraj=zeros(1,N);

for i=1:N

    xTraj(i)=xyRightSide(1,i)+alpha(i)*(xyLeftSide(1,i)-xyRightSide(1,i));
    yTraj(i)=xyRightSide(2,i)+alpha(i)*(xyLeftSide(2,i)-xyRightSide(2,i));
    
end

%% Plot 

figure(11)
hold on
% plot(xTraj,yTraj,'b')
% plot(xTraj,yTraj,'g')
plot(xTraj,yTraj,'r')
plot(Track_Param.xTrack,Track_Param.yTrack,'k')
plot(Track_Param.xRightSide',Track_Param.yRightSide','k--')
k=plot(Track_Param.xLeftSide' ,Track_Param.yLeftSide' ,'k--');
sf=plot(Track_Param.xStartLine,Track_Param.yStartLine,'k-.');

k.Annotation.LegendInformation.IconDisplayStyle = 'off';
sf.Annotation.LegendInformation.IconDisplayStyle = 'off';
% legend('Minimum curvature path','Track centerline','Track limits')
text(0.5,-5,'S/F\rightarrow','HorizontalAlignment','center')
% legend('Shortest Path (\epsilon=1)','Intermediate path (\epsilon=0.01)','Minimum Curvature path (\epsilon=0)','Track centerline','Track limits')
axis off
