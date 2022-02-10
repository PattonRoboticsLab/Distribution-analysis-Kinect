function [newref_filt_wrist,newref_filt_elbow,newref_filt_shoulder,newref_filt_hip,newref_filt_spine,right_wrist,right_wrist_filt,right_elbow,right_elbow_filt,right_shoulder,right_shoulder_filt,L,time]=newref_and_filter(activity,b,a,theta1,theta2)

%% Create general matrix: convert table into array and fill matrix [(n*L)x3]

n=25;      % total number of joint 

% use table2array to obtain a double variable instead of table 
act_array=table2array(activity(:,2:101));       % first column (time) excluded

% use length to obtain a scalar (total number of rows)
L=length(act_array(:,1)); 

% Build a matrix of zeros aimed to contain the movement in xyz of the activity in all joints
bodyparts=zeros(n*L,3);   

% Filling the matrix bodyparts, composed of 3 columns (x y z) and inside each column
% there are the position of the 25 joints from 1st to 25th
for location=1:n
    for i=1:3
        column = (4*(location-1))+i;
        if(location==1)
            bodyparts(:,i) = vertcat(act_array(:,column),zeros((length(bodyparts(:,1))-L),1));
        else
            bodyparts(:,i) = vertcat(bodyparts(1:(location-1)*L,i),act_array(:,column),zeros((length(bodyparts(:,1))-(location*L)),1));
        end
    end
end


%% Shoulder, elbow, wrist, hip and spine: find in the general matrix bodyparts the part containing the movement in xyz of the 4 joints mentioned

loc_shoulder=9;     %right shoulder
loc_elbow=10;       %right elbow
loc_wrist=11;       %right wrist
loc_hip=17;         %right hip
loc_spine=21;       %spine shoulder

for i=1:L
    right_wrist(i,:)=bodyparts((L*(loc_wrist-1))+i,:);
    right_elbow(i,:)=bodyparts((L*(loc_elbow-1))+i,:);
    right_shoulder(i,:)=bodyparts((L*(loc_shoulder-1))+i,:);
    right_hip(i,:)=bodyparts((L*(loc_hip-1))+i,:);
    spinesh(i,:)=bodyparts((L*(loc_spine-1))+i,:);
end
 
%% Filtering without changing refsystem 

right_wrist_filt(:,1)=filtfilt(b,a,right_wrist(:,1));
right_wrist_filt(:,2)=filtfilt(b,a,right_wrist(:,2));
right_wrist_filt(:,3)=filtfilt(b,a,right_wrist(:,3));

right_elbow_filt(:,1)=filtfilt(b,a,right_elbow(:,1));
right_elbow_filt(:,2)=filtfilt(b,a,right_elbow(:,2));
right_elbow_filt(:,3)=filtfilt(b,a,right_elbow(:,3));

right_shoulder_filt(:,1)=filtfilt(b,a,right_shoulder(:,1));
right_shoulder_filt(:,2)=filtfilt(b,a,right_shoulder(:,2));
right_shoulder_filt(:,3)=filtfilt(b,a,right_shoulder(:,3));

right_hip_filt(:,1)=filtfilt(b,a,right_hip(:,1));
right_hip_filt(:,2)=filtfilt(b,a,right_hip(:,2));
right_hip_filt(:,3)=filtfilt(b,a,right_hip(:,3));

spinesh_filt(:,1)=filtfilt(b,a,spinesh(:,1));
spinesh_filt(:,2)=filtfilt(b,a,spinesh(:,2));
spinesh_filt(:,3)=filtfilt(b,a,spinesh(:,3));

%% Change reference system: rotation
% Activity: build a single matrix of zeros containing the position xyz of the 25 joints

refsystem=zeros(n,3);   % is not n*L, is original point of joints in Kinect
                        % reference system (t=0)

for location=1:n
    for i=1:3
        column = (4*(location-1))+i;
        if(location==1)
            refsystem(:,i) = vertcat(act_array(1,column),zeros(n-1,1));
         else
            refsystem(:,i) = vertcat(refsystem(1:(location-1),i),act_array(1,column),zeros((length(refsystem(:,1))-location),1));
         end
    end
end

% First rotation (78°) on x axis: vector of u = matrix rot1 * vector v
rot1=zeros(3,3);            % matrix of rotation 1
rot1(1,1)=1; 
rot1(2,2)=cos(theta1); 
rot1(2,3)=-sin(theta1); 
rot1(3,2)=sin(theta1); 
rot1(3,3)=cos(theta1);

for i=1:n                   % n = length(refsystem(:,1))
    newposition=rot1*refsystem(i,:)'; refsystem_rot1(i,:)=newposition';
end

for i=1:length(right_wrist(:,1))      % same of elbow, shoulder, hip
    newposition=rot1*right_wrist_filt(i,:)'; right_wrist_rot1(i,:)=newposition';
    newposition=rot1*right_elbow_filt(i,:)'; right_elbow_rot1(i,:)=newposition';
    newposition=rot1*right_shoulder_filt(i,:)'; right_shoulder_rot1(i,:)=newposition';
    newposition=rot1*right_hip_filt(i,:)'; right_hip_rot1(i,:)=newposition';
    newposition=rot1*spinesh_filt(i,:)'; spinesh_rot1(i,:)=newposition';
end

% Second rotation (180°) in z axis: vector of w = matrix rot2 * vector u 
rot2=zeros(3,3);
rot2(2,2)=cos(theta2);
rot2(1,1)=cos(theta2);
rot2(1,2)=-sin(theta2); 
rot2(2,1)=sin(theta2);
rot2(3,3)=1;

for i=1:n           % n = length(refsystem(:,1))
    newposition=rot2*refsystem_rot1(i,:)'; refsystem_rot2(i,:)=newposition';
end

for i=1:length(right_wrist_rot1(:,1))    % same of elbow, shoulder, hip  
    newposition=rot2*right_wrist_rot1(i,:)'; right_wrist_rot2(i,:)=newposition';
    newposition=rot2*right_elbow_rot1(i,:)'; right_elbow_rot2(i,:)=newposition';
    newposition=rot2*right_shoulder_rot1(i,:)'; right_shoulder_rot2(i,:)=newposition';
    newposition=rot2*right_hip_rot1(i,:)'; right_hip_rot2(i,:)=newposition';
    newposition=rot2*spinesh_rot1(i,:)'; spinesh_rot2(i,:)=newposition';
end


%% Change reference system: translation

v1=-refsystem_rot2(21,1)+0.8;           % spine shoulder joint is 21
v2=-refsystem_rot2(21,2)+0.5; 
v3=-refsystem_rot2(21,3)+0.6;
translation=eye(4);                     % translation matrix
translation(1,4)=v1;
translation(2,4)=v2;
translation(3,4)=v3;

% wrist
ones1=ones(length(right_wrist),1);    % same of elbow, shoulder, hip
refsystem_tras_wrist=horzcat(right_wrist_rot2,ones1);

for i=1:length(right_wrist)
    newlocation=translation*refsystem_tras_wrist(i,:)'; newref_filt_wrist(i,:)=newlocation';
end
newref_filt_wrist(:,4)=[];

% elbow
refsystem_tras_elbow=horzcat(right_elbow_rot2,ones1);
for i=1:length(right_elbow)
    newlocation=translation*refsystem_tras_elbow(i,:)'; newref_filt_elbow(i,:)=newlocation';
end
newref_filt_elbow(:,4)=[];

% shoulder
refsystem_tras_shoulder=horzcat(right_shoulder_rot2,ones1);
for i=1:length(right_shoulder)
    newlocation=translation*refsystem_tras_shoulder(i,:)'; newref_filt_shoulder(i,:)=newlocation';
end
newref_filt_shoulder(:,4)=[];

% hip
refsystem_tras_hip=horzcat(right_hip_rot2,ones1);
for i=1:length(right_hip)
    newlocation=translation*refsystem_tras_hip(i,:)'; newref_filt_hip(i,:)=newlocation';
end
newref_filt_hip(:,4)=[];

% spine
refsystem_tras_spine=horzcat(spinesh_rot2,ones1);
for i=1:length(spinesh)
    newlocation=translation*refsystem_tras_spine(i,:)'; newref_filt_spine(i,:)=newlocation';
end
newref_filt_spine(:,4)=[];

%% Time frame 

frames(:,1)=activity(:,1);              % first column

total_sec=(table2array(frames(end,1))-table2array(frames(1,1)))/1000;      % my result is in ms 
                                                                           % I want result in seconds

for i=1:length(table2array(activity(:,1)))
    time(i,1)=(table2array(frames(i,1))-table2array(frames(1,1)))/1000;
end

lengths_time(1,1)=length(time(:,1));
min_time=min(lengths_time);

if(length(time(:,1))==min_time)
    X_time=time;
end


