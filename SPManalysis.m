%% Compare using SPM the trajectory of the movements not based on time

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

%% SPM 

figure
F=spm1d.stats.ttest_paired(exercise_sub1_wrist,exercise_sub1_elbow);
Fi=F.inference(0.05,'two_tailed',true); %inference
Fi.plot()