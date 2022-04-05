%% Load csv file manually

clear all
close all
clc

exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P1/Session2/FreeExploration1.csv', 'NumHeaderLines',1);

%% Kinect data acquisition accuracy

accuracy=data_acquisition_accuracy(exercise_sub1);

%% Change reference system, filter and get time reference in seconds

% rotation of 90º around X axis 
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(5,fc/(fs/2), 'low');     % 5th order

[exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L,time,tot_time]=newref_and_filter(exercise_sub1,b,a,theta1,theta2);

clear a; clear b; clear theta1; clear theta2; clear fc; clear fs;

%% check if data are normally distributed
% 
% figure
% normplot(exercise_sub1_wrist)
% legend('x','y','z')

%% Plot activity wrist motion in 3D after filtering
figure
plotMan2(right_shoulder_filt); 
hold on
% plot3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b'); 
scatter3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
axis equal
%axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('Free exploration - wrist motion in 3D space')
hold off

% h=figure(1);
% saveas(h,'plotMan','png')

%% Plot velocity, acceleration, jerk in 3D

[vel_wrist,acc_wrist,jerk_wrist]=calc_vel_acc_jerk(exercise_sub1_wrist,time);

figure
subplot(1,3,1);
scatter3(vel_wrist(:,1),vel_wrist(:,2),vel_wrist(:,3),1,'.');
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement velocity (m/s)');

subplot(1,3,2);
scatter3(acc_wrist(:,1),acc_wrist(:,2),acc_wrist(:,3),1,'.');
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement acceleration (m/s^3)');

subplot(1,3,3);
scatter3(jerk_wrist(:,1),jerk_wrist(:,2),jerk_wrist(:,3),1,'.');
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement jerk (m/s^3)');

%% Plot velocity, acceleration, jerk in frontal plane in time

figure
subplot(3,1,1);
plot(time(1:L-1),vel_wrist(:,1))
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Wrist movement velocity frontal plane in time')

subplot(3,1,2);
plot(time(1:L-2),acc_wrist(:,1))
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Wrist movement acceleration frontal plane in time')

subplot(3,1,3);
plot(time(1:L-3),jerk_wrist(:,1))
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
title('Wrist movement jerk frontal plane in time')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DISTRIBUTION OF MOVEMENTS IN SPACE
%% Plot histograms of motion distribution, velocity, acceleration and jerk

figure
nBins=4;
plotDust(exercise_sub2_wrist,nBins)   % plotDust(x,nBins)
                                      % X = rows of multidimensional values, cols are dimension
                                      % nBins = # equally-spaced bins, for counting
axis equal
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
title('Wrist movement distribution histogram')


figure
nBins=4;
plotDust(vel_wrist,nBins)   
axis equal
xlabel('Motion in frontal plane [m/s]');
ylabel('Motion in sagittal plane [m/s]');
zlabel('Motion in trasverse plane [m/s]');
title('Wrist movement velocity distribution histogram')

figure
nBins=4;
plotDust(acc_wrist,nBins)   
axis equal
xlabel('Motion in frontal plane [m/s^2]');
ylabel('Motion in sagittal plane [m/s^2]');
zlabel('Motion in trasverse plane [m/s^2]');
title('Wrist movement acceleration distribution histogram')

figure
nBins=4;
plotDust(jerk_wrist,nBins)   
axis equal
xlabel('Motion in frontal plane [m/s^3]');
ylabel('Motion in sagittal plane [m/s^3]');
zlabel('Motion in trasverse plane [m/s^3]');
title('Wrist movement jerk distribution histogram')

%% Calculate coverage

percentile=90;
group='Free exploration distribution'; % name the activity considered
volume_distribution=calc_coverage(exercise_sub1_wrist,percentile,group);

percentile=95;
group='Free exploration'; % name the activity considered
volume_velocity=calc_coverage(vel_wrist,percentile,group);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COEFFICIENT OF DETERMINATION
%% check coef determination dividing data in half and odd rows

% L_half=round(L/2);
% 
% nBins=4;
% [Rsquared_half,Rsquared_adj_half,pearson_corr_half,mov1,mov2]= find_Pcorr_and_COD(exercise_sub1_wrist(1:(L_half-1),:), exercise_sub1_wrist(L_half:L,:), nBins, (L_half-1), L_half);
% [Rsquared_evenodd,Rsquared_adj_evenodd,pearson_corr_evenodd,mov1,mov2]= find_Pcorr_and_COD(exercise_sub1_wrist(1:2:L,:), exercise_sub1_wrist(2:2:(L-1),:), nBins, (L_half-1), L_half);
% 
% clear mov1; clear mov2;

%%  Number of samples vs coef of determination

nBins=4;
nFolds=100;
[cod,cod_adj,mean_cod,max_cod,min_cod]=random(exercise_sub1_wrist,nBins,nFolds);

%% Plot of COD with mean line

figure
scatter(cod(:,3),cod(:,1))
hold on
% normplot(cod(:,1))
% yline(mean(cod(:,1)),'g')
plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
hold on 
plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth',1 )
hold on 
plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth',1 )
% hold on 
% x_axis = [mean_cod(:,1); flipud(mean_cod(:,1))];
% inBetween = [min_cod(:,2); flipud(max_cod(:,2))];
% fill=fill(x_axis, inBetween, 'r');
% set(fill,'facealpha',0.3);
% set(fill,'linewidth',0.1)
grid on
hold off
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
legend('Data','Mean line','Min/max lines')
title('COD of random samples vs full set of data')

clear fill;

%% Plot shaded area 

figure
plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
hold on
plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth', 1)
hold on 
plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth', 1)
hold on 
x_axis = [mean_cod(:,1); flipud(mean_cod(:,1))];
inBetween = [min_cod(:,2); flipud(max_cod(:,2))];
fill=fill(x_axis, inBetween, 'r');
set(fill,'facealpha',.1);
set(fill,'linewidth',0.1)
hold on
yline(0.99,'-','99%','LineWidth',2,'Color','r')
grid on
hold off
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
legend('Mean values','Min/max values')
title('COD of random samples vs full set of data')

clear fill; clear x_axis; clear inBetween; clear theta2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% JOINT ANGLES
%% Calculate shoulder (A/A and F/E) elbow (F/E) angles

joint_angles=calc_jointangles(exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine);
[ja_vel, ja_acc,ja_jerk]=calc_jointangles_vel_acc_jerk(joint_angles,exercise_sub1_shoulder,time);

angleAA_shoulder=joint_angles(:,1);        % shoulder abduction/adduction
angleFE_shoulder=joint_angles(:,2);        % shoulder flexion/extension
angleFE_elbow=joint_angles(:,3);           % elbow flexion/extension

figure
subplot(3,1,1);
plot(time,angleAA_shoulder)
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Adduction/abduction [degree]');
title('Shoulder adduction/abduction angles in time ')

subplot(3,1,2);
plot(time,angleFE_shoulder)
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Flexion/extension [degree]');
title('Shoulder flexion/extension angles in time ')

subplot(3,1,3);
plot(time,angleFE_elbow)
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Flexion/extension [degree]');
title('Elbow flexion/extension angles in time ')

%% Plot velocity, acceleration and jerk of an angle (ex. FE shoulder)

figure
subplot(3,1,1);
plot(time(1:L-1),ja_vel(:,2))
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Adduction/abduction [degree]');
title('Shoulder adduction/abduction angles in time ')

subplot(3,1,2);
plot(time(1:L-2),ja_acc(:,2))
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Flexion/extension [degree]');
title('Shoulder flexion/extension angles in time ')

subplot(3,1,3);
plot(time(1:L-3),ja_jerk(:,2))
xlim([0 tot_time])
xlabel('Time[s]');
ylabel('Flexion/extension [degree]');
title('Elbow flexion/extension angles in time ')

%% Plot joint angles' distribution, velocity, acceleration, jerk in 3D

figure
plot3(ja_vel(:,1),ja_vel(:,2),ja_vel(:,3))
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement velocity')

figure
plot3(ja_acc(:,1),ja_acc(:,2),ja_acc(:,3))
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement acceleration')

figure
plot3(ja_jerk(:,1),ja_jerk(:,2),ja_jerk(:,3))
grid on
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
title('Wrist movement jerk')

%% Number of samples vs coef of determination - shoulder flexion/extension angle

nBins=4;
nFolds=10;
[cod_ja,cod_adj_ja,mean_cod,max_cod,min_cod]=random1D(angleFE_shoulder,nBins,nFolds);

%% Plot of COD with mean line

figure
scatter(cod_ja(:,3),cod_ja(:,1))
hold on
% normplot(cod(:,1))
% yline(mean(cod(:,1)),'g')
plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
hold on 
plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth',1 )
hold on 
plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth',1 )
% hold on 
% x_axis = [mean_cod(:,1); flipud(mean_cod(:,1))];
% inBetween = [min_cod(:,2); flipud(max_cod(:,2))];
% fill=fill(x_axis, inBetween, 'r');
% set(fill,'facealpha',0.3);
% set(fill,'linewidth',0.1)
grid on
hold off
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
legend('Data','Mean line','Min/max lines')
title('COD of random samples vs full set of data')

clear fill;

%% Plot shaded area 

figure
plot(mean_cod(:,1),mean_cod(:,2),'.-','Color','g')
hold on
plot(mean_cod(:,1),min_cod(:,2),'.-','Color','k','LineWidth', 1)
hold on 
plot(mean_cod(:,1),max_cod(:,2),'.-','Color','k','LineWidth', 1)
hold on 
x_axis = [mean_cod(:,1); flipud(mean_cod(:,1))];
inBetween = [min_cod(:,2); flipud(max_cod(:,2))];
fill=fill(x_axis, inBetween, 'r');
set(fill,'facealpha',.1);
set(fill,'linewidth',0.1)
hold on
yline(0.99,'-','99%','LineWidth',2,'Color','r')
grid on
hold off
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
legend('Mean values','Min/max values')
title('COD of random samples vs full set of data')

clear fill; clear x_axis; clear inBetween;


