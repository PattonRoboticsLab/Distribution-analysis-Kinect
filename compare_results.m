%% Compare 2 or more activities - wrist in 3D


clear all
close all
clc

exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P_circle.csv', 'NumHeaderLines',1);
exercise_sub2 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P_exocircle', 'NumHeaderLines',1);
% exercise_sub3 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/RaisingS', 'NumHeaderLines',1);

%% Change reference system, filter and get time reference in seconds

% rotation of 90º around X axis 
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(4,fc/(fs/2), 'low');     % 4th order

[exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L1,time]=newref_and_filter(exercise_sub1,b,a,theta1,theta2);
[exercise_sub2_wrist,exercise_sub2_elbow,exercise_sub2_shoulder,exercise_sub2_hip,exercise_sub2_spine,right_wrist2,right_wrist2_filt,right_elbow2,right_elbow2_filt,right_shoulder2,right_shoulder2_filt,L2,time2]=newref_and_filter(exercise_sub2, b, a, theta1, theta2);
%[exercise_sub3_wrist,exercise_sub3_elbow,exercise_sub3_shoulder,exercise_sub3_hip,exercise_sub3_spine,right_wrist3,right_wrist3_filt,right_elbow3,right_elbow3_filt,right_shoulder3,right_shoulder3_filt,L3,time3]=newref_and_filter(exercise_sub3, b, a, theta1, theta2);

% plot movement distribution in 3D 
figure
plotMan(right_shoulder_filt); 
hold on
p1=scatter3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
hold on
p2=scatter3(exercise_sub2_wrist(:,1), exercise_sub2_wrist(:,2),exercise_sub2_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','r'); 
% hold on
% p3=scatter3(exercise_sub3_wrist(:,1), exercise_sub3_wrist(:,2),exercise_sub3_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','g');
axis equal
xlabel('x')
ylabel('y')
% zlabel('z')
legend([p1 p2],'With ExoNET','No ExoNET')
% legend([p1 p2 p3],'Subject 1','Subject 2','Subject 3')   for 3 subjects
title('Raising movement - wrist motion in 3D space with respect to Kinect reference system')
hold off


%% Plot histograms and see Pearson correlation

nBins=4;
[pearson_corr,mov1,mov2]=find_Pearson_correlation(exercise_sub1_wrist, exercise_sub2_wrist, nBins, L1, L2);

%% Kullback-Leibler Divergence
% 
% temp_zeros = zeros(size(mov1,1),1);
% %mov1_Label = [mov1, temp_zeros];
% temp_ones = ones(size(mov2,1),1);
% %mov2_Label = [mov2, temp_ones];
% movForEntropy = cat(1, mov1, mov2);
% labelsForEntropy = cat(1, temp_zeros, temp_ones);
% labelsForEntropy_log=logical(labelsForEntropy);
% %writematrix(mov1,'mov1.csv')
% %writematrix(mov2,'mov2.csv')
% Z = relativeEntropy(movForEntropy,labelsForEntropy_log)


%% Coefficient of determination

%fit linear regression model
nBins=4;
[Rsquared,Rsquared_adj,mov1,mov2]= find_coef_determination(exercise_sub1_wrist, exercise_sub2_wrist, nBins, L1, L2);


%% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles

[joint_angles1]=calc_jointangles(exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine);
[joint_angles2]=calc_jointangles(exercise_sub2_wrist,exercise_sub2_elbow,exercise_sub2_shoulder,exercise_sub2_hip,exercise_sub2_spine);
% [joint_angles3]=calc_jointangles(exercise_sub3_wrist,exercise_sub3_elbow,exercise_sub3_shoulder,exercise_sub3_hip,exercise_sub3_spine);

nBins=4;
[ja_correlation,ja1,ja2]=find_Pearson_correlation(joint_angles1,joint_angles2, nBins, L1, L2);

[ja_Rsquared,ja_Rsquared_adj]= find_coef_determination(joint_angles1,joint_angles2, nBins, L1, L2);


