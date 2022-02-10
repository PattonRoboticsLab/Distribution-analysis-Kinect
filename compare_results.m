%% Compare 2 or more activities - wrist in 3D

% load .csv file
exercise_sub2 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/Codes/data/RaisingJ', 'NumHeaderLines',1);
exercise_sub3 = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/Codes/data/RaisingS', 'NumHeaderLines',1);

% filter, change reference system and time reference 
[exercise_sub2_wrist,exercise_sub2_elbow,exercise_sub2_shoulder,exercise_sub2_hip,exercise_sub2_spine,right_wrist2,right_wrist2_filt,right_elbow2,right_elbow2_filt,right_shoulder2,right_shoulder2_filt,L2,time_sec2,min_time2]=newref_and_filter(exercise_sub2, d, theta1, theta2);
[exercise_sub3_wrist,exercise_sub3_elbow,exercise_sub3_shoulder,exercise_sub3_hip,exercise_sub3_spine,right_wrist3,right_wrist3_filt,right_elbow3,right_elbow3_filt,right_shoulder3,right_shoulder3_filt,L3,time_sec3,min_time3]=newref_and_filter(exercise_sub3, d, theta1, theta2);

% plot movement distribution in 3D 
figure
plotMan(right_shoulder_filt); 
hold on
p1=scatter3(exercise_sub1_wrist(:,1), exercise_sub1_wrist(:,2),exercise_sub1_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
hold on
p2=scatter3(exercise_sub2_wrist(:,1), exercise_sub2_wrist(:,2),exercise_sub2_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','r'); 
hold on
p3=scatter3(exercise_sub3_wrist(:,1), exercise_sub3_wrist(:,2),exercise_sub3_wrist(:,3),'MarkerEdgeColor','k','MarkerFaceColor','g');
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
legend([p1 p2 p3],'Subject 1','Subject 2','Subject 3')
title('Raising movement - wrist motion in 3D space with respect to Kinect reference system')
hold off

%% Calculate elbow-shoulder, shoulder-spine, shoulder-hip, wrist-elbow segments and shoulder (A/A and F/E) elbow (F/E) angles

[shoulder_AA_angle1,shoulder_FE_angle1,elbow_angle1]=calc_jointangles(exercise_sub1_wrist,exercise_sub1_elbow,exercise_sub1_shoulder,exercise_sub1_hip,exercise_sub1_spine);
[shoulder_AA_angle2,shoulder_FE_angle2,elbow_angle2]=calc_jointangles(exercise_sub2_wrist,exercise_sub2_elbow,exercise_sub2_shoulder,exercise_sub2_hip,exercise_sub2_spine);
[shoulder_AA_angle3,shoulder_FE_angle3,elbow_angle3]=calc_jointangles(exercise_sub3_wrist,exercise_sub3_elbow,exercise_sub3_shoulder,exercise_sub3_hip,exercise_sub3_spine);

%% Plot histograms

all_wrist = vertcat(exercise_sub1_wrist,exercise_sub2_wrist,exercise_sub3_wrist);

figure
nBins=10;
plotDust(all_wrist,nBins)   % plotDust(x,nBins)
                            % X = rows of multidimensional values, cols are dimension
                            % nBins = # equally-spaced bins, for counting
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('Wrist movement distribution histogram')