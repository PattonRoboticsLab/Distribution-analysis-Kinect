%%%%%%%%%%%%%%%%%%% Filtering results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot activity wrist motion in xyz after filtering
figure
plot(time,exercise_sub1_wrist(:,1),'.-')
hold on
plot(time,exercise_sub1_wrist(:,2),'.-')
hold on
plot(time,exercise_sub1_wrist(:,3),'.-')
xlabel('Time[s]');
ylabel('Wrist position [m]');
legend('x','y','z')
title('Raising motion - wrist position (with respect to Kinect reference system) in xyz after filtering and change of reference system')
hold off

%% Plot activity wrist before and after filtering 
figure
plot(time,right_wrist(:,1),'.-')
hold on

plot(time,right_wrist_filt(:,1),'.-')
xlim([110 140])
xlabel('Time[s]');
ylabel('Wrist position [m]');
legend('Original','After filtering')
title('Free exploration - wrist motion before/after filtering in frontal plane')
hold off

h=figure(1);
saveas(h,'filtresult','png')

%% Plot activity wrist motion in 3D after filtering
figure
plotMan(right_shoulder_filt); 
hold on
% plot3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b'); 
scatter3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b'); 
axis equal
xlabel('Frontal plane');
ylabel('Sagittal plane');
zlabel('Trasverse plane');
legend('Wrist motion')
title('Raising - wrist motion in 3D space with respect to Kinect reference system')
hold off
