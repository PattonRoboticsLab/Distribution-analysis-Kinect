%% Draw reference man figure

function plotMan2(right_shoulder_filt)

%if ~exist('right_shoulder_filt');  right_shoulder_filt=[0.68 0.825 0.6];  end 
% right_shoulder_L=mean(right_shoulder(:,1));   %1.05
% left_shoulder_L=right_shoulder_L-0.2;   %0.75
% right_shoulder_R=right_shoulder_L+0.07; %1.12
% left_shoulder_R=left_shoulder_L-0.7;    %0.68
% 
% shoulder_U=mean(right_shoulder(:,2));   %0.65
% shoulder_B=shoulder_U-0.05;             %0.60
% shoulder_I=mean(right_shoulder(:,3));   %0.6
% shoulder_O=shoulder_I-0.5;              %0.1
% 
% xCalc = [left_shoulder_L left_shoulder_L right_shoulder_L right_shoulder_L right_shoulder_R right_shoulder_R left_shoulder_R left_shoulder_R];      % trapezoid x dimensions (bottom then top)
% yCalc = [shoulder_B shoulder_U shoulder_U shoulder_B shoulder_B shoulder_U shoulder_U shoulder_B];          % trapezoid y dimensions
% zCalc = [shoulder_O shoulder_O shoulder_O shoulder_O shoulder_I shoulder_I shoulder_I shoulder_I];              % trapezoid z dimensions


xCalc = [0.07 0.07 0.37 0.37 0.44 0.44 0 0];      % trapezoid x dimensions (bottom then top)
yCalc = [-0.1 -0.05 -0.05 -0.1 -0.1 -0.05 -0.05 -0.1];          % trapezoid y dimensions
zCalc = [-0.4 -0.4 -0.4 -0.4 0.1 0.1 0.1 0.1];              % trapezoid z dimensions

%bottom
fill3(xCalc([1 2 3 4]), yCalc([1 2 3 4]), zCalc([1 2 3 4]),1,'FaceAlpha',0.1); hold on
%up
fill3(xCalc([5 6 7 8]), yCalc([5 6 7 8]), zCalc([5 6 7 8]),1, 'FaceAlpha',0.1); hold on
%right
fill3(xCalc([3 4 5 6]), yCalc([3 4 5 6]), zCalc([3 4 5 6]),1, 'FaceAlpha',0.1); hold on
%left
fill3(xCalc([1 2 7 8]), yCalc([1 2 7 8]), zCalc([1 2 7 8]),1, 'FaceAlpha',0.1); hold on
%back
fill3(xCalc([1 4 5 8]), yCalc([1 4 5 8]), zCalc([1 4 5 8]),1, 'FaceAlpha',0.1); hold on
%front
fill3(xCalc([2 3 6 7]), yCalc([2 3 6 7]), zCalc([2 3 6 7]),1,'FaceAlpha',0.1); hold on

% head  
head=[0.22,-0.075,0.25]; 
headRadii=[.09 .09 .11];
ellipsoid(head(1),head(2),head(3), headRadii(1),headRadii(2),headRadii(3)); 

% right arm
shoulderR=[0 -0.075 0.1]; 
elbowR=[-0.03 -0.075 -0.2];
upperarmR=vertcat(shoulderR,elbowR);
fill3(upperarmR(:,1),upperarmR(:,2),upperarmR(:,3),'-','Linewidth',6)
wristR=[-0.08 -0.075 -0.34];
rightArm=vertcat(elbowR,wristR);
fill3(rightArm(:,1),rightArm(:,2),rightArm(:,3),'-','Linewidth',6)

% left arm
shoulderL=[0.44 -0.075 0.1]; 
elbowL=[0.47 -0.075 -0.2];
upperarmL=vertcat(shoulderL,elbowL);
fill3(upperarmL(:,1),upperarmL(:,2),upperarmL(:,3),'-','Linewidth',6)
handL=[0.52 -0.075 -0.34];
leftArm=vertcat(elbowL,handL);
fill3(leftArm(:,1),leftArm(:,2),leftArm(:,3),'-','Linewidth',6)

% right leg
hipR=[0.07 -0.075 -0.4]; 
ankleR=[0.05 -0.075 -1];
rightLeg=vertcat(hipR,ankleR);
fill3(rightLeg(:,1),rightLeg(:,2),rightLeg(:,3),'-','Linewidth',8)

% left leg
hipL=[0.37 -0.075 -0.4]; 
ankleL=[0.4 -0.075 -1];
leftLeg=vertcat(hipL,ankleL);
fill3(leftLeg(:,1),leftLeg(:,2),leftLeg(:,3),'-','Linewidth',8)

axis equal
grid on
