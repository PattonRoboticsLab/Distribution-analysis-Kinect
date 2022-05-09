clear all
close all
clc


noExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession1/P02_FreeExp1.3.csv', 'NumHeaderLines',1);
%withExo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession2/P02_FreeExp2.3.csv', 'NumHeaderLines',1);
withExopost = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession3/P02_FreeExp3.3.csv', 'NumHeaderLines',1);
w4 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession4/P02_FreeExp4.3.csv', 'NumHeaderLines',1);
w5 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession5/P02_FreeExp5.3.csv', 'NumHeaderLines',1);
w6 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession6/P02_FreeExp6.3.csv', 'NumHeaderLines',1);

pt1=readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession2/P02_FreeExp2.3pt1.csv', 'NumHeaderLines',1);
pt2=readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/P2/TreatmentSession2/P02_FreeExp2.3pt2.csv', 'NumHeaderLines',1);
withExo=vertcat(pt1,pt2);


%% Change reference system, filter and get time reference in seconds

% rotation of 90º around X axis 
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(5,fc/(fs/2), 'low');     % 5th order

[noExo_wrist,noExo_elbow,noExo_shoulder,noExo_hip,noExo_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L1,time1,tot_time1]=newref_filt_Left(noExo,b,a,theta1,theta2);
[withExo_wrist,withExo_elbow,withExo_shoulder,withExo_hip,withExo_spine,right_wrist2,right_wrist2_filt,right_elbow2,right_elbow2_filt,right_shoulder2,right_shoulder2_filt,L2,time2,tot_time2]=newref_filt_Left(withExo, b, a, theta1, theta2);
[withExopost_wrist,withExopost_elbow,withExopost_shoulder,withExopost_hip,withExopost_spine,right_wrist3,right_wrist3_filt,right_elbow3,right_elbow3_filt,right_shoulder3,right_shoulder3_filt,L3,time3,tot_time3]=newref_filt_Left(withExopost, b, a, theta1, theta2);
[w4_wrist,w4_elbow,w4_shoulder,w4_hip,w4_spine,right_wrist4,right_wrist4_filt,right_elbow4,right_elbow4_filt,right_shoulder4,right_shoulder4_filt,L4,time4,tot_time4]=newref_filt_Left(w4, b, a, theta1, theta2);
[w5_wrist,w5_elbow,w5_shoulder,w5_hip,w5_spine,right_wrist5,right_wrist5_filt,right_elbow5,right_elbow5_filt,right_shoulder5,right_shoulder5_filt,L5,time5,tot_time5]=newref_filt_Left(w5, b, a, theta1, theta2);
[w6_wrist,w6_elbow,w6_shoulder,w6_hip,w6_spine,right_wrist6,right_wrist6_filt,right_elbow6,right_elbow6_filt,right_shoulder6,right_shoulder6_filt,L6,time6,tot_time6]=newref_filt_Left(w6, b, a, theta1, theta2);

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
hold on
p3=scatter3(withExopost_wrist(:,1), withExopost_wrist(:,2),withExopost_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','g'); 
hold on
p4=scatter3(w4_wrist(:,1), w4_wrist(:,2),w4_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','c'); 
hold on
p5=scatter3(w5_wrist(:,1), w5_wrist(:,2),w5_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','y'); 
hold on
p6=scatter3(w6_wrist(:,1), w6_wrist(:,2),w6_wrist(:,3),2,'MarkerEdgeColor','k','MarkerFaceColor','m'); 
axis equal
grid off
axis off
xlabel('Frontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Trasverse plane [m]');
legend([p1 p2 p3 p4 p5 p6],'Session 1','Session 2','Session 3','Session 4','Session 5','Session 6')
title('Free exploration - wrist motion in 3D space')
hold off


