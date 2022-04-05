%% Compare pre/post therapy with ExoNET - wrist in 3D


clear all
close all
clc

withExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P1/Session2/FreeExploration2.csv', 'NumHeaderLines',1);
withExopost = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P1/Session2/FreeExploration3.csv', 'NumHeaderLines',1);

%% Kinect data acquisition accuracy

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

[withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine,right_wrist2,right_wrist2_filt,right_elbow2,right_elbow2_filt,right_shoulder2,right_shoulder2_filt,L2,time2,tot_time2]=newref_and_filter(withExo, b, a, theta1, theta2);
[withExopost_wrist,withExopost_elbow,withExopost_shoulder,withExopost_hip,withExopost_spine,right_wrist3,right_wrist3_filt,right_elbow3,right_elbow3_filt,right_shoulder3,right_shoulder3_filt,L3,time3,tot_time3]=newref_and_filter(withExopost, b, a, theta1, theta2);

clear a; clear b; clear theta1; clear theta2; clear fc; clear fs;

tot_time_max=max(tot_time2,tot_time3);

%% Plot movement distribution in 3D 

figure
plotMan2(right_shoulder2_filt); 
hold on
p1=scatter3(withExo_wrist(:,1), withExo_wrist(:,2),withExo_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
hold on
p2=scatter3(withExopost_wrist(:,1), withExopost_wrist(:,2),withExopost_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','g'); 
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
legend([p1 p2],'Pre-therapy','Post-therapy')
title('Free exploration with ExoNET - wrist motion in 3D space')
hold off


%% Calculate velocity, acceleration and jerk

[vel_wrist2,acc_wrist2,jerk_wrist2]=calc_vel_acc_jerk(withExo_wrist,time2);
[vel_wrist3,acc_wrist3,jerk_wrist3]=calc_vel_acc_jerk(withExopost_wrist,time3);

%% Plot position, velocity, acceleration, jerk in frontal plane in time (2D)

figure
subplot(2,1,1);
plot(time2(1:L2),withExo_wrist(:,3))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Position [m]');
title('Wrist position trasverse plane in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3),withExopost_wrist(:,3))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Position [m]');
title('Wrist position trasverse plane in time - post therapy')

figure
subplot(2,1,1);
plot(time2(1:L2-1),vel_wrist2(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Wrist velocity frontal plane in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-1),vel_wrist3(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Wrist velocity frontal plane in time - post therapy')

figure
subplot(2,1,1);
plot(time2(1:L2-2),acc_wrist2(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Wrist movement acceleration frontal plane in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-2),acc_wrist3(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Wrist movement acceleration frontal plane in time - post therapy')

figure
subplot(2,1,1);
plot(time2(1:L2-3),jerk_wrist2(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
title('Wrist jerk frontal plane in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-3),jerk_wrist3(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
title('Wrist jerk frontal plane in time - post therapy')

%% Plot position, velocity, acceleration, jerk in 3D

figure
subplot(1,4,1);
scatter3(withExo_wrist(:,1),withExo_wrist(:,2),withExo_wrist(:,3),1,'.','r');
hold on 
scatter3(withExopost_wrist(:,1),withExopost_wrist(:,2),withExopost_wrist(:,3),1,'.','c');
hold off
grid on
axis equal
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('Wrist movement position (m)');

%figure
subplot(1,4,2);
scatter3(vel_wrist2(:,1),vel_wrist2(:,2),vel_wrist2(:,3),1,'.','r');
hold on 
scatter3(vel_wrist3(:,1),vel_wrist3(:,2),vel_wrist3(:,3),1,'.','c');
hold off
grid on
axis equal
xlabel('Frontal plane [m/s]');
ylabel('Sagittal plane [m/s]');
zlabel('Trasverse plane [m/s]');
%legend([p1 p2],'No ExoNET','With ExoNET')
title('Wrist movement velocity (m/s)');

subplot(1,4,3);
scatter3(acc_wrist2(:,1),acc_wrist2(:,2),acc_wrist2(:,3),1,'.', 'r');
hold on 
scatter3(acc_wrist3(:,1),acc_wrist3(:,2),acc_wrist3(:,3),1,'.','c');
hold off
grid on
axis equal
xlabel('Frontal plane [m/s^2]');
ylabel('Sagittal plane [m/s^2]');
zlabel('Trasverse plane [m/s^2]');
title('Wrist movement acceleration (m/s^2)');

subplot(1,4,4);
scatter3(jerk_wrist2(:,1),jerk_wrist2(:,2),jerk_wrist2(:,3),1,'.','r');
hold on 
scatter3(jerk_wrist3(:,1),jerk_wrist3(:,2),jerk_wrist3(:,3),1,'.','c');
hold off
grid on
axis equal
xlabel('Frontal plane [m/s^3]');
ylabel('Sagittal plane [m/s^3]');
zlabel('Trasverse plane [m/s^3]');
legend('Pre-therapy','Post-therapy')
title('Wrist movement jerk (m/s^3)');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPARISON

%% Volume and Area to define range of motion

percentile=90;
group1='No exo';
volume_ctrl=calc_coverage(withExo_wrist,percentile,group1);
group2='Exo';
volume_exo=calc_coverage(withExopost_wrist,percentile,group2);

clear group1; clear group2;

%% See Pearson correlation and coefficient of determination between 2 activities

nBins=4;
[Rsquared,Rsquared_adj,pearson_corr,mov1,mov2]=find_Pcorr_and_COD(vel_wrist3, vel_wrist2, nBins, L3-1, L2-1);
fprintf('\n Pearson correlation is %d', pearson_corr);
fprintf('\n Coefficient of determination is %d', Rsquared);

%% See number of counts using CDF (cummulative density function) --> forse inutile

% nBins=4;
% counts1=plotDust(noExo_wrist,nBins)
% nBins=4;
% counts2=plotDust(withExo_wrist,nBins)
% 
% cdf1 = cumsum(counts1); 
% cdf1 = cdf1 / cdf1(end); 
% data1GT90_ = find(cdf1>= 0.9, 1, 'first')
% 
% cdf2 = cumsum(counts2); 
% cdf2 = cdf2 / cdf2(end); 
% data2GT90 = find(cdf2>= 0.8, 1, 'first')
% 

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% JOINT ANGLES
%% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles

[joint_angles2]=calc_jointangles(withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine);
[joint_angles3]=calc_jointangles(withExopost_wrist,withExopost_elbow,withExopost_shoulder,withExopost_hip,withExopost_spine);

%% Plot 3 joint angles in time - 2D

figure
subplot(2,1,1);
plot(time2,joint_angles2(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Adduction/abduction [degree]');
title('Shoulder adduction/abduction angles in time - pre therapy')
subplot(2,1,2);
plot(time3,joint_angles3(:,1))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Adduction/abduction [degree]');
title('Shoulder adduction/abduction angles in time - post therapy')

figure
subplot(2,1,1);
plot(time2,joint_angles2(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Flexion/extension [degree]');
title('Shoulder flexion/extension angles in time - pre therapy')
subplot(2,1,2);
plot(time3,joint_angles3(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Flexion/extension [degree]');
title('Shoulder flexion/extension angles in time - post therapy')

figure
subplot(2,1,1);
plot(time2,joint_angles2(:,3))
xlim([0 tot_time_max])
xlabel('Cycle [%]');
ylabel('Flexion/extension [degree]');
title('Elbow flexion/extension angles in time - pre therapy')
subplot(2,1,2);
plot(time3,joint_angles3(:,3))
xlim([0 tot_time_max])
xlabel('Cycle [%]');
ylabel('Flexion/extension [degree]');
title('Elbow flexion/extension angles in time - post therapy')

%% Find joint angles velocity, acceleration and jerk

[ja_vel2, ja_acc2,ja_jerk2]=calc_jointangles_vel_acc_jerk(joint_angles2,withExo_shoulder,time2);
[ja_vel3, ja_acc3,ja_jerk3]=calc_jointangles_vel_acc_jerk(joint_angles3,withExopost_shoulder,time3);

figure
subplot(2,1,1);
plot(time2(1:L2-1),ja_vel2(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Shoulder hor flexion/extension velocity in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-1),ja_vel3(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Shoulder hor flexion/extension angular velocity in time - post therapy')

figure
subplot(2,1,1);
plot(time2(1:L2-2),ja_acc2(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Shoulder hor flexion/extension acceleration in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-2),ja_acc3(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Shoulder hor flexion/extension angular acceleration in time - post therapy')

figure
subplot(2,1,1);
plot(time2(1:L2-3),ja_jerk2(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
title('Shoulder hor flexion/extension jerk in time - pre therapy')
subplot(2,1,2);
plot(time3(1:L3-3),ja_jerk3(:,2))
xlim([0 tot_time_max])
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
title('Shoulder hor flexion/extension angular jerk in time - post therapy')

%% Correlation

nBins=4;
[ja_Rsquared,ja_Rsquared_adj,ja_correlation,ja2,ja3]=find_Pcorr_and_COD(joint_angles2,joint_angles3, nBins, L2, L3);
fprintf('\n Pearson correlation is %d', ja_correlation);
fprintf('\n Coefficient of determination is %d', ja_Rsquared);

% shoulder flexion extension only
[ja_Rsquared_sfe,ja_Rsquared_adj_sfe,ja_correlation_sfe,ja2_sfe,ja3_sfe]=find_Pcorr_and_COD(joint_angles2,joint_angles3, nBins, L2, L3);
fprintf('\n Pearson correlation is %d', ja_correlation_sfe);
fprintf('\n Coefficient of determination is %d', ja_Rsquared_sfe);

%% Cycle
% precision=0.1;
% ja1= cycle(joint_angles1,time1,L1,precision);
% ja2 = cycle(joint_angles2,time2,L2,precision);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HOW MUCH DOES NOEXO MOVEMENT PREDICT EXO MOVEMENT? 
%% Number of samples vs coef of determination

% nBins=4;
% nFolds=10;
% [cod,cod_adj,mean_cod,max_cod,min_cod]=predict(exercise_sub1_wrist,exercise_sub2_wrist,nBins,nFolds);
% 
% %% Plot of COD with mean line
% 
% figure
% scatter(cod(:,3),cod(:,1))
% hold on
% plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
% hold on 
% plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth',1 )
% hold on 
% plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth',1 )
% grid on
% hold off
% xlabel('Time (s)');
% ylabel('Coefficient of determination (0-100%)');
% legend('Data','Mean line','Min/max lines')
% title('COD of random samples vs full set of data')
% 
% clear fill;
% 
% %% Plot shaded area 
% 
% figure
% plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
% hold on
% plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth', 1)
% hold on 
% plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth', 1)
% hold on 
% x_axis = [mean_cod(:,1); flipud(mean_cod(:,1))];
% inBetween = [min_cod(:,2); flipud(max_cod(:,2))];
% fill=fill(x_axis, inBetween, 'r');
% set(fill,'facealpha',.1);
% set(fill,'linewidth',0.1)
% hold on
% yline(0.99,'-','99%','LineWidth',2,'Color','r')
% grid on
% hold off
% xlabel('Time (s)');
% ylabel('Coefficient of determination (0-100%)');
% legend('Mean values','Min/max values')
% title('COD of random samples vs full set of data')
% 
% clear fill; clear x_axis; clear inBetween; clear theta2;
