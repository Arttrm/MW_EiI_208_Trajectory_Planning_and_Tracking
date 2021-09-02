% ======================================================================= %
% Arthur RODRIGUEZ
% Student ID: 32448139
% Mail: ar4n20@soton.ac.uk
% ======================================================================= %
% ELEC6259
% Project
% RC Car Modelling and Trajectory Tracking Control
% ======================================================================= %
% GIF_Creator
% Version : 1.0
% Date : 01/09/2021
% ======================================================================= %

% Before running this script, you need to run the simulink model.

Data1=out.CarPosition.Data;
Time1=out.CarPosition.Time;
Data3=out.TrajInfo.Data;
Time3=out.TrajInfo.Time;

KMax=0.2;

filename = 'Simulation_Animation.gif';

KList2=[];

figure(7)
set(gcf, 'color', 'white');

for i=1:20:length(Data3)
    XY=Data3(:,:,i);
    X=XY(1:100);
    Y=XY(101:end);
    
    K=Curv(X,Y);
    v_xt=0.8*sqrt(1.75*9.81*0.8./K);
    
    KList2=[KList2,maxk(K,4)'];
 
    h=figure(7)
    % plot(X,Y,'r')
    cmap = colormap(jet); % colormap(flipud(jet));
    % scatter(X(2:end-1),Y(2:end-1),1,K,'.');
    s=surf([X(2:end-1);X(2:end-1)],[Y(2:end-1);Y(2:end-1)],[v_xt;v_xt],'FaceColor', 'none','EdgeColor', 'interp','LineWidth', 1.5);
    view(2);caxis([0,25]);
    
    c=colorbar;
    ylabel(c,'v_{x,t} (m/s)')
    
    hold on
    % Car
    hg1 = hgtransform;
    rectangle('Position',[-1,-0.5,2,1],'Curvature', [0.5 0.5],'parent',hg1,'FaceColor','k');
    a=Data1(find(Time1==Time3(i)),3)
    hg1.Matrix = makehgtform('translate',[X(1),Y(1),0],'zrotate',a);
        
    % Track
    plot(Track_Param.xTrack,Track_Param.yTrack,'k')
    plot(Track_Param.xRightSide',Track_Param.yRightSide','k--')
    k=plot(Track_Param.xLeftSide' ,Track_Param.yLeftSide' ,'k--');
    sf=plot(Track_Param.xStartLine,Track_Param.yStartLine,'k-.');
    text(0.5,-5,'S/F\rightarrow','HorizontalAlignment','center')
    
    axis([-40,80,-70,10]);%([min(Track_Param.xTrack)-10,max(Track_Param.xTrack)+10,min(Track_Param.yTrack)-10,max(Track_Param.yTrack)+10])
    axis off
    hold off
    
    % GIF
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    if i == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1); 
    end 

    % pause(0.1)
end



function K = Curv(X,Y)
K=[];

for i=2:length(X')-1


    A=[X(i-1);Y(i-1);0];
    B=[X(i);Y(i);0];
    C=[X(i+1);Y(i+1);0];
    
    D = cross(B-A,C-A);
    b = norm(A-C);
    c = norm(A-B);
    a = norm(B-C);
    R = a*b*c/2/norm(D);
    
    k=(1/(R))^1/2;
    if isnan(k)
        k=0;
    end
    K=[K,k];
    
end
end