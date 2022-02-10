%% Joint angles results

%% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles

joint_angles=calc_jointangles(exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine);

figure
plot(time,joint_angles)
hold on

xlabel('Time[s]');
ylabel('Joint angles [degree]');
title('Raising - shoulder and elbow angles in time ')
legend('Shoulder abduction/adduction','Shoulder horizontal flexion/extension','Elbow flexion/extension')
hold off

%% Plot histograms of joint angles

figure
nBins=10;
[counts,binLimits,binCenters]=plotDust(joint_angles,nBins)   % plotDust(x,nBins)
                               % X = rows of multidimensional values, cols are dimension
                               % nBins = # equally-spaced bins, for counting
axis equal
xlabel('Shoulder abduction [deg]')
ylabel('Shoulder flexion [deg]')
zlabel('Elbow flexion [deg]')
title('Shoulder and elbow joint angles distribution histogram - free expl')


%% Plot histograms of angular velocity and acceleration

[ja_vel, ja_acc]=joint_angle_vel_acc(joint_angles,exercise_sub1_shoulder,time);

figure
nBins=10;
plotDust(ja_vel,nBins)   % plotDust(x,nBins)
                         % X = rows of multidimensional values, cols are dimension
                         % nBins = # equally-spaced bins, for counting
axis equal
xlabel('Shoulder abduction [deg/s]')
ylabel('Shoulder flexion [deg/s]')
zlabel('Elbow flexion [deg/s]')
title('Shoulder and elbow joint angles velocity distribution histogram')

figure
nBins=10;
plotDust(ja_acc,nBins)   % plotDust(x,nBins)
                         % X = rows of multidimensional values, cols are dimension
                         % nBins = # equally-spaced bins, for counting
axis equal
xlabel('Shoulder abduction [deg/s^2]')
ylabel('Shoulder flexion [deg/s^2]')
zlabel('Elbow flexion [deg/s^2]')
title('Shoulder and elbow joint angles acceleration distribution histogram')


