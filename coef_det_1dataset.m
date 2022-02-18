%% See correlation of same dataset

clear all
clc

% Input data - change in base of need
exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/FE_ele.csv', 'NumHeaderLines',1);

% Filtering and homogeneous transformation
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(4,fc/(fs/2), 'low');     % 4th order

[exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L,time]=newref_and_filter(exercise_sub1,b,a,theta1,theta2);

%% check coef determination dividing data in half

L_half=round(L/2);

nBins=4;
[Rsquared,Rsquared_adj,mov1,mov2]= find_coef_determination(exercise_sub1_wrist(1:(L_half-1),:), exercise_sub1_wrist(L_half:L,:), nBins, (L_half-1), L_half);

%% check coef determination even and odd numbers

nBins=4;
[Rsquared,Rsquared_adj,mov1,mov2]= find_coef_determination(exercise_sub1_wrist(1:2:L,:), exercise_sub1_wrist(2:2:(L-1),:), nBins, (L_half-1), L_half);



