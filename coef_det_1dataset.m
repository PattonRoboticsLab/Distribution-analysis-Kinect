%% See correlation of same dataset

clear all
clc

% Input data - change in base of need
exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/jialin_FE_4min.csv', 'NumHeaderLines',1);

%% Kinect data acquisition accuracy

accuracy=data_acquisition_accuracy(exercise_sub1);

%% Filtering and homogeneous transformation
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

%%  Number of samples vs coef of determination

nBins=4;
nIterations=100;
[cod,cod_adj,mean_cod]=random(exercise_sub1_wrist,nBins,nIterations);

%% Stacked plot of COD and time 

new_cod(:,1)=cod(:,1);
new_cod(:,2)=cod(:,3);

figure
% grid on
% plot(cod(:,1))
% set(gca,'XTick',0:50:1000)
% set(gca,'XTickLabel',0:16.5:330)
% xticks(0:30:500)
% plot(time(100:319),cod(:,1))
newYlabels = ["COD (0-100%)","Time (s)"];
stackedplot(new_cod,"Title","COD of increasing data samples","DisplayLabels",newYlabels)
xlabel('#samples');
% ylabel('Coefficient of determination (0-100%)');
% title('COD of random samples vs full set of data')

%% Plot with lines for time division

figure
plot(cod(:,1))
hold on 
for n=1:1000
    plot([1 1]*3.3*n, ylim, '--k')               
end           
grid on
set(gca,'XTick',0:50:10000)
set(gca,'XTickLabel',0:16.5:3300)
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
title('COD of random samples vs full set of data')


%% Plot of cod with mean line

figure
plot(cod(:,1))
set(gca,'XTick',0:50:10000)
set(gca,'XTickLabel',0:16.5:3300)
hold on
% normplot(cod(:,1))
% yline(mean(cod(:,1)),'g')
plot(mean_cod)
grid on
hold off
xlabel('Time (s)');
ylabel('Coefficient of determination (0-100%)');
legend('Data','Mean line')
title('COD of random samples vs full set of data')

