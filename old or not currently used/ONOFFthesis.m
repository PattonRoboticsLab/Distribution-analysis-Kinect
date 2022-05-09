%% Exo on off

%% Compare with and without ExoNET - right wrist in 3D


clear all
close all
clc

noExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P1/TreatmentSession2/P01_FreeExp2.1.csv', 'NumHeaderLines',1);
withExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P1/TreatmentSession2/P01_FreeExp2.2.csv', 'NumHeaderLines',1);

%% Kinect data acquisition accuracy

accuracy1=data_acquisition_accuracy(noExo);
accuracy2=data_acquisition_accuracy(withExo);

%% Change reference system, filter and get time reference in seconds

% rotation of 90º around X axis 
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(5,fc/(fs/2), 'low');     % 5th order

[noExo_wrist,noExo_elbow,noExo_shoulder,noExo_hip,noExo_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L1,time1,tot_time1]=newref_filt_Right(noExo,b,a,theta1,theta2);
[withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine,right_wrist2,right_wrist2_filt,right_elbow2,right_elbow2_filt,right_shoulder2,right_shoulder2_filt,L2,time2,tot_time2]=newref_filt_Right(withExo, b, a, theta1, theta2);

clear a; clear b; clear theta1; clear theta2; clear fc; clear fs;

tot_time_max=max(tot_time1,tot_time2);

clear tot1; clear tot2;

%% Plot movement distribution in 3D 

figure
plotManR(right_shoulder_filt); 
hold on
p1=scatter3(noExo_wrist(:,1), noExo_wrist(:,2),noExo_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','b');
hold on
p2=scatter3(withExo_wrist(:,1), withExo_wrist(:,2),withExo_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
legend([p1 p2],'No ExoNET','With ExoNET pre-therapy')
title('Free exploration - wrist motion in 3D space with respect to Kinect reference system')
hold off

%% Calculate velocity, acceleration and jerk

[vel_wrist1,acc_wrist1,jerk_wrist1]=calc_vel_acc_jerk(noExo_wrist,time1);
[vel_wrist2,acc_wrist2,jerk_wrist2]=calc_vel_acc_jerk(withExo_wrist,time2);

% Plot velocity in 3D
figure
p1=scatter3(vel_wrist1(:,1),vel_wrist1(:,2),vel_wrist1(:,3),1,'.');
hold on 
p2=scatter3(vel_wrist2(:,1),vel_wrist2(:,2),vel_wrist2(:,3),1,'.');
hold off
grid on
axis equal
xlabel('Frontal plane [m/s]');
ylabel('Sagittal plane [m/s]');
zlabel('Trasverse plane [m/s]');
legend([p1 p2],'No ExoNET','With ExoNET pre-therapy')
title('Wrist movement velocity (m/s)');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPARISON
%% Plot histograms of motion distribution

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

%% Coverage - position

percentile=95;
group1='No ExoNET';
volume_noexo=calc_coverage(noExo_wrist,percentile,group1);
group2='ExoNET';
volume_exo=calc_coverage(withExo_wrist,percentile,group2);

clear group1; clear group2;


%% Coverage - velocities

percentile=95;
group1='No exo';
volume1_vel=calc_coverage(vel_wrist1,percentile,group1);
group2='Exo Pre-therapy';
volume2_vel=calc_coverage(vel_wrist2,percentile,group2);

clear group1; clear group2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% JOINT ANGLES
%% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles

[joint_angles1]=calc_jointangles(noExo_wrist,noExo_elbow,noExo_shoulder,noExo_hip,noExo_spine);
[joint_angles2]=calc_jointangles(withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine);

% Find joint angles velocity, acceleration and jerk
[ja_vel1, ja_acc1,ja_jerk1]=calc_jointangles_vel_acc_jerk(joint_angles1,noExo_shoulder,time1);
[ja_vel2, ja_acc2,ja_jerk2]=calc_jointangles_vel_acc_jerk(joint_angles2,withExo_shoulder,time2);

%% Plot histograms of joint angle 

figure
nBins=4;
plotDust(joint_angles1,nBins)   
axis equal
xlabel('Shoulder adduction [deg]');
ylabel('Shoulder flexion [deg]');
zlabel('Elbow flexion [deg]');
title('Joint angles - no ExoNET')

figure
nBins=4;
plotDust(joint_angles2,nBins)  
axis equal
xlabel('Shoulder adduction [deg]');
ylabel('Shoulder flexion[deg]');
zlabel('Elbow flexion [deg]');
title('Joint angles - with ExoNET pre-therapy')

%% Volume and Area to define range of motion

percentile=95;
group1='No exo';
volume_ja1=calc_coverage(joint_angles1,percentile,group1);
group2='Exo';
volume_ja2=calc_coverage(joint_angles2,percentile,group2);

clear group1; clear group2;

%% shoulder flexion extension only
horFE1=joint_angles1(:,2);
horFE2=joint_angles2(:,2);
[ja_Rsquared_sfe,ja_Rsquared_adj_sfe,ja_correlation_sfe,ja1_sfe,ja2_sfe]=find_Pcorr_and_COD_1D(horFE1,horFE2,nBins, L1, L2);
fprintf('\n Pearson correlation is %d', ja_correlation_sfe);
fprintf('\n Coefficient of determination is %d', ja_Rsquared_sfe);
