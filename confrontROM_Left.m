%% see only coverage Left

clear all
close all
clc

noExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession1/P02_FreeExp1.1.csv', 'NumHeaderLines',1);
withExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession1/P02_FreeExp1.2.csv', 'NumHeaderLines',1);
withExopost = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession1/P02_FreeExp1.3.csv', 'NumHeaderLines',1);
%pt1=readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession2/P02_FreeExp2.3pt1.csv', 'NumHeaderLines',1);
%pt2=readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession2/P02_FreeExp2.3pt2.csv', 'NumHeaderLines',1);
%withExopost=vertcat(pt1,pt2);

%% Kinect data acquisition accuracy

accuracy1=data_acquisition_accuracy(noExo);
accuracy2=data_acquisition_accuracy(withExo);
accuracy3=data_acquisition_accuracy(withExopost);

%% Change reference system, filter and get time reference in seconds

% rotation of 90º around X axis 
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(5,fc/(fs/2), 'low');     % 5th order

[noExo_wrist,noExo_elbow,noExo_shoulder,noExo_hip,noExo_spine,left_wrist,left_wrist_filt,left_elbow,left_elbow_filt,left_shoulder,left_shoulder_filt,L1,time1,tot_time1]=newref_filt_Left(noExo,b,a,theta1,theta2);
[withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine,left_wrist2,left_wrist2_filt,left_elbow2,left_elbow2_filt,left_shoulder2,left_shoulder2_filt,L2,time2,tot_time2]=newref_filt_Left(withExo, b, a, theta1, theta2);
[withExopost_wrist,withExopost_elbow,withExopost_shoulder,withExopost_hip,withExopost_spine,left_wrist3,left_wrist3_filt,left_elbow3,left_elbow3_filt,left_shoulder3,left_shoulder3_filt,L3,time3,tot_time3]=newref_filt_Left(withExopost, b, a, theta1, theta2);

clear a; clear b; clear theta1; clear theta2; clear fc; clear fs;

tot_time_max=max(tot_time1,tot_time2);

clear tot1; clear tot2;

%% Plot movement distribution in 3D 

figure
plotManL(left_shoulder_filt); 
hold on
p1=scatter3(noExo_wrist(:,1), noExo_wrist(:,2),noExo_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','b');
hold on
p2=scatter3(withExo_wrist(:,1), withExo_wrist(:,2),withExo_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
hold on
p3=scatter3(withExopost_wrist(:,1), withExopost_wrist(:,2),withExopost_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','g'); 
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
legend([p1 p2 p3],'No ExoNET','With ExoNET pre-therapy','With ExoNET post-therapy')
title('Free exploration - wrist motion in 3D space')
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPARISON
%% Histogram

figure
nBins=4;
plotDust(noExo_wrist,nBins)   
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('No ExoNET')

figure
nBins=4;  
plotDust(withExo_wrist,nBins)
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('With ExoNET pre-therapy')

figure
nBins=4;  
plotDust(withExopost_wrist,nBins)
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('With ExoNET post-therapy')

%% Volume and Area to define range of motion

% percentile=95;
% group1='No exo';
% volume1_noexo=calc_coverage(noExo_wrist,percentile,group1);
% group2='Exo Pre-therapy';
% volume2_exo=calc_coverage(withExo_wrist,percentile,group2);
% group3='Exo Post-therapy';
% volume3_post=calc_coverage(withExopost_wrist,percentile,group3);
% 
% clear group1; clear group2; clear group3;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Calculate velocity, acceleration and jerk
% 
% [vel_wrist1,acc_wrist1,jerk_wrist1]=calc_vel_acc_jerk(noExo_wrist,time1);
% [vel_wrist2,acc_wrist2,jerk_wrist2]=calc_vel_acc_jerk(withExo_wrist,time2);
% [vel_wrist3,acc_wrist3,jerk_wrist3]=calc_vel_acc_jerk(withExopost_wrist,time3);
% 
% % Plot velocity in 3D
% figure
% p1=scatter3(vel_wrist1(:,1),vel_wrist1(:,2),vel_wrist1(:,3),1,'.');
% hold on 
% p2=scatter3(vel_wrist2(:,1),vel_wrist2(:,2),vel_wrist2(:,3),1,'.');
% hold on 
% p3=scatter3(vel_wrist3(:,1),vel_wrist3(:,2),vel_wrist3(:,3),1,'.');
% hold off
% grid on
% axis equal
% grid off
% axis off
% xlabel('Frontal plane [m/s]');
% ylabel('Sagittal plane [m/s]');
% zlabel('Trasverse plane [m/s]');
% legend([p1 p2 p3],'No ExoNET','With ExoNET pre-therapy','With ExoNET post-therapy')
% title('Wrist movement velocity (m/s)');
% 
% %% Volume and Area to define velocity coverage
% 
% percentile=95;
% group1='No ExoNET';
% volume_vel1=calc_coverage(vel_wrist1,percentile,group1)
% group2='ExoNET Pre-therapy';
% volume_vel2=calc_coverage(vel_wrist2,percentile,group2)
% group3='ExoNET Post-therapy';
% volume_vel3=calc_coverage(vel_wrist3,percentile,group3)
% 
% clear group1; clear group2; clear group3;
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% JOINT ANGLES
% %% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles
% 
% [joint_angles1]=calc_jointangles(noExo_wrist,noExo_elbow,noExo_shoulder,noExo_hip,noExo_spine);
% [joint_angles2]=calc_jointangles(withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine);
% [joint_angles3]=calc_jointangles(withExopost_wrist,withExopost_elbow,withExopost_shoulder,withExopost_hip,withExopost_spine);
% 
% % Find joint angles velocity, acceleration and jerk
% [ja_vel1, ja_acc1,ja_jerk1]=calc_jointangles_vel_acc_jerk(joint_angles1,noExo_shoulder,time1);
% [ja_vel2, ja_acc2,ja_jerk2]=calc_jointangles_vel_acc_jerk(joint_angles2,withExo_shoulder,time2);
% [ja_vel3, ja_acc3,ja_jerk3]=calc_jointangles_vel_acc_jerk(joint_angles3,withExopost_shoulder,time3);
% 
% %% Plot histograms of joint angle 
% 
% figure
% nBins=4;
% plotDust(joint_angles1,nBins)   
% axis equal
% xlabel('Shoulder adduction [deg]');
% ylabel('Shoulder flexion [deg]');
% zlabel('Elbow flexion [deg]');
% title('Joint angles - no ExoNET')
% 
% figure
% nBins=4;
% plotDust(joint_angles2,nBins)  
% axis equal
% xlabel('Shoulder adduction [deg]');
% ylabel('Shoulder flexion[deg]');
% zlabel('Elbow flexion [deg]');
% title('Joint angles - ExoNET pre-therapy')
% 
% figure
% nBins=4;
% plotDust(joint_angles3,nBins)  
% axis equal
% xlabel('Shoulder adduction [deg]');
% ylabel('Shoulder flexion[deg]');
% zlabel('Elbow flexion [deg]');
% title('Joint angles - ExoNET post-therapy')
% % 
% %% Volume and Area to define range of motion
% 
% percentile=95;
% group1='No ExoNET';
% volume_ja1=calc_coverage(joint_angles1,percentile,group1)
% group2='ExoNET pre-therapy';
% volume_ja2=calc_coverage(joint_angles2,percentile,group2)
% group3='ExoNET post-therapy';
% volume_ja3=calc_coverage(joint_angles3,percentile,group3)
% 
% clear group1; clear group2; clear group3;
% 
% %% shoulder flexion extension only
% 
% nBins=4;
% horFE1=joint_angles1(:,2);
% horFE2=joint_angles2(:,2);
% horFE3=joint_angles3(:,2);
% [ja_Rsquared_sfe,ja_Rsquared_adj_sfe,ja_correlation_sfe,ja1_sfe,ja2_sfe]=find_Pcorr_and_COD_1D(horFE1,horFE2,nBins, L1, L2);
% fprintf('\n Pearson correlation is %d', ja_correlation_sfe);
% fprintf('\n Coefficient of determination is %d', ja_Rsquared_sfe);
% 
% [ja_Rsquared_sfe,ja_Rsquared_adj_sfe,ja_correlation_sfe,ja1_sfe,ja2_sfe]=find_Pcorr_and_COD_1D(horFE3,horFE2,nBins, L3, L2);
% fprintf('\n Pearson correlation is %d', ja_correlation_sfe);
% fprintf('\n Coefficient of determination is %d', ja_Rsquared_sfe);
