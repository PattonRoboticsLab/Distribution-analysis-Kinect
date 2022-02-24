%% Load csv file manually

clear all
close all
clc

exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/jialin_FE_4min.csv', 'NumHeaderLines',1);

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
[b,a] = butter(4,fc/(fs/2), 'low');     % 4th order

[exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L,time]=newref_and_filter(exercise_sub1,b,a,theta1,theta2);

%% Plot activity wrist motion in 3D after filtering
figure
plotMan(right_shoulder_filt); 
hold on
% plot3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b'); 
scatter3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b'); 
axis equal
xlabel('Motion in x [m]');
ylabel('Motion in y [m]');
zlabel('Motion in z [m]');
title('Free exploration - wrist motion in 3D space')
hold off

%% Plot histograms of motion distribution

figure
nBins=4;
plotDust(exercise_sub1_wrist,nBins)   % plotDust(x,nBins)
                                      % X = rows of multidimensional values, cols are dimension
                                      % nBins = # equally-spaced bins, for counting
axis equal
xlabel('Motion in x [m]');
ylabel('Motion in y [m]');
zlabel('Motion in z [m]');
title('Wrist movement distribution histogram')

%% Plot 2D histograms of motion distribution - yz axis
% 
% figure
% nBins=4;
% results=plotDust(exercise_sub1_wrist(:,2:3),nBins)   % plotDust(x,nBins)
%                                       % X = rows of multidimensional values, cols are dimension
%                                       % nBins = # equally-spaced bins, for counting
% axis equal
% xlabel('Motion in x [m]');
% ylabel('Motion in y [m]');
% zlabel('Motion in z [m]');
% title('Wrist movement distribution histogram')


%% Plot histograms of velocity and acceleration

[vel_act,acc_act,smooth_act]=calc_vel_acc_smooth(exercise_sub1_wrist,time)

figure
nBins=4;
plotDust(vel_act,nBins)   
axis equal
xlabel('Motion in x [m/s]');
ylabel('Motion in y [m/s]');
zlabel('Motion in z [m/s]');
title('Wrist movement velocity distribution histogram')

figure
nBins=4;
plotDust(acc_act,nBins)   
axis equal
xlabel('Motion in x [m/s^2]');
ylabel('Motion in y [m/s^2]');
zlabel('Motion in z [m/s^2]');
title('Wrist movement acceleration distribution histogram')

figure
nBins=4;
plotDust(smooth_act,nBins)   
axis equal
xlabel('Motion in x [m/s^3]');
ylabel('Motion in y [m/s^3]');
zlabel('Motion in z [m/s^3]');
title('Wrist movement smoothness distribution histogram')

