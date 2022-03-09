%% See correlation of same dataset

clear all
clc

% Input data - change in base of need
exercise_sub1 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/free_exp_jim.csv', 'NumHeaderLines',1);

%% Kinect data acquisition accuracy

accuracy=data_acquisition_accuracy(exercise_sub1);

%% Filtering and homogeneous transformation
theta1=1.5708;
% rotation1 of 180º around Z axis 
theta2=3.14159;         % since the kinect see in opposite way
% filter: zero-lag-4th-order Butterworth low pass filter with cut off 6Hz
fc=6;           % cut off frequency
fs=1/(33*0.001);      % sample frequency (∂t≈33ms)
[b,a] = butter(5,fc/(fs/2), 'low');     % 5th order

[exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L,time,tot_time]=newref_and_filter(exercise_sub1,b,a,theta1,theta2);

clear a; clear b; clear theta1; clear theta2;

%% check coef determination dividing data in half

L_half=round(L/2);

nBins=4;
[Rsquared,Rsquared_adj,pearson_corr,mov1,mov2]= find_Pcorr_and_COD(exercise_sub1_wrist(1:(L_half-1),:), exercise_sub1_wrist(L_half:L,:), nBins, (L_half-1), L_half);

clear mov1; clear mov2;

%% check coef determination even and odd numbers

nBins=4;
[Rsquared,Rsquared_adj,pearson_corr,mov1,mov2]= find_Pcorr_and_COD(exercise_sub1_wrist(1:2:L,:), exercise_sub1_wrist(2:2:(L-1),:), nBins, (L_half-1), L_half);

clear mov1; clear mov2;

%%  Number of samples vs coef of determination

nBins=4;
nFolds=10;
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

clear fill; clear x_axis;

perc=[1:1:18352];
plot_meanSD(exercise_sub1_wrist(:,1),perc)
