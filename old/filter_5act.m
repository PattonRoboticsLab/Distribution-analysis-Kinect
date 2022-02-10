%% Creation of new reference system and filter of 1 activity

% It is going to be a function of that activity in wrist, elbow and shoulder
% function [activity_wrist,activity_elbow,activity_shoulder,activity_hip,min_length]=filter_and_HT(activity, d, theta1, theta2)

clc

location = 21;                                      %spine shoulder position
n=25;                                               % total number of joint 
d= designfilt('lowpassiir','FilterOrder',5,'HalfPowerFrequency',0.09); n=10; %To do the means

%% Spinal cord as reference system
for i=1:3
    column = 2+(4*(location-1))+i;                  % such as to have the correspondent column of spinal cord
                                                    % 26 letters in the alphabet so position
                                                    % 82 to 84 correspond
                                                    % to spine shoulder xyz
    spine_1(:,i)=table2array(drinking(:,column));
    spine_2(:,i)=table2array(drinking2(:,column));
    spine_3(:,i)=table2array(drinking3(:,column));
    spine_4(:,i)=table2array(drinking4(:,column));
    spine_5(:,i)=table2array(drinking5(:,column));
end

lengths(1,1)=length(spine_1(:,1));
lengths(1,2)=length(spine_2(:,1));
lengths(1,3)=length(spine_3(:,1));
lengths(1,4)=length(spine_4(:,1));
lengths(1,5)=length(spine_5(:,1));
min_length=min(lengths);                             % creating a matrix of lenghts, where in each
                                                     % position there is the length of the column of spine 
                                                     % x y z we found before

%% Activities: find number of rows and build a single matrix of zeros that includes the movement in xyz of the activities in all joints

% use table2array to obtain a double variable instead of table
drinking_d=table2array(drinking);
drinking2_d=table2array(drinking2);
drinking3_d=table2array(drinking3);
drinking4_d=table2array(drinking4);
drinking5_d=table2array(drinking5);

% use length to obtain a scalar
L1=length(drinking_d(:,1)); 
L2=length(drinking2_d(:,1));
L3=length(drinking3_d(:,1));
L4=length(drinking4_d(:,1));
L5=length(drinking5_d(:,1));

% build a single matrix of zeros that includes the movement in xyz of the activities in all joints
bodyparts1=zeros(n*L1,3);   
bodyparts2=zeros(n*L2,3);
bodyparts3=zeros(n*L3,3);
bodyparts4=zeros(n*L4,3);
bodyparts5=zeros(n*L5,3);

% complete the matrix bodyparts,composed of 3 columns (that correspond to x y z) and inside each column
% there are the position of the 25 joints (following time sequence)
for location=1:n
    for i=1:3
        column = 2+(4*(location-1))+i;
        if(location==1)
            bodyparts1(:,i) = vertcat(drinking_d(:,column),zeros((length(bodyparts1(:,1))-L1),1));
            bodyparts2(:,i) = vertcat(drinking2_d(:,column),zeros((length(bodyparts2(:,1))-L2),1));
            bodyparts3(:,i) = vertcat(drinking3_d(:,column),zeros((length(bodyparts3(:,1))-L3),1));
            bodyparts4(:,i) = vertcat(drinking4_d(:,column),zeros((length(bodyparts4(:,1))-L4),1));
            bodyparts5(:,i) = vertcat(drinking5_d(:,column),zeros((length(bodyparts5(:,1))-L5),1));
        else
            bodyparts1(:,i) = vertcat(bodyparts1(1:(location-1)*L1,i),drinking_d(:,column),zeros((length(bodyparts1(:,1))-(location*L1)),1));
            bodyparts2(:,i) = vertcat(bodyparts2(1:(location-1)*L2,i),drinking2_d(:,column),zeros((length(bodyparts2(:,1))-(location*L2)),1));
            bodyparts3(:,i) = vertcat(bodyparts3(1:(location-1)*L3,i),drinking3_d(:,column),zeros((length(bodyparts3(:,1))-(location*L3)),1));
            bodyparts4(:,i) = vertcat(bodyparts4(1:(location-1)*L4,i),drinking4_d(:,column),zeros((length(bodyparts4(:,1))-(location*L4)),1));
            bodyparts5(:,i) = vertcat(bodyparts5(1:(location-1)*L5,i),drinking5_d(:,column),zeros((length(bodyparts5(:,1))-(location*L5)),1));
        end
    end
end


%% Shoulder, elbow, wrist and hip: find in the general matrix bodyparts, the part containing the movement in xyz of the 4 joints mentioned

loc1=11; %right wrist
loc2=10; %right elbow
loc3=9; %right shoulder
loc4=17; %right hip

for i=1:L1
    right_wrist_1(i,:)=bodyparts1((L1*(loc1-1))+i,:);
    right_elbow_1(i,:)=bodyparts1((L1*(loc2-1))+i,:);
    right_shoulder_1(i,:)=bodyparts1((L1*(loc3-1))+i,:);
    right_hip_1(i,:)=bodyparts1((L1*(loc4-1))+i,:);
end
for i=1:L2
    right_wrist_2(i,:)=bodyparts2((L2*(loc1-1))+i,:);
    right_elbow_2(i,:)=bodyparts2((L2*(loc2-1))+i,:);
    right_shoulder_2(i,:)=bodyparts2((L2*(loc3-1))+i,:);
    right_hip_2(i,:)=bodyparts2((L2*(loc4-1))+i,:);
end
for i=1:L3
    right_wrist_3(i,:)=bodyparts3((L3*(loc1-1))+i,:);
    right_elbow_3(i,:)=bodyparts3((L3*(loc2-1))+i,:);
    right_shoulder_3(i,:)=bodyparts3((L3*(loc3-1))+i,:);
    right_hip_3(i,:)=bodyparts3((L3*(loc4-1))+i,:);
end
for i=1:L4
    right_wrist_4(i,:)=bodyparts4((L4*(loc1-1))+i,:);
    right_elbow_4(i,:)=bodyparts4((L4*(loc2-1))+i,:);
    right_shoulder_4(i,:)=bodyparts4((L4*(loc3-1))+i,:);
    right_hip_4(i,:)=bodyparts4((L4*(loc4-1))+i,:);
end
for i=1:L5
    right_wrist_5(i,:)=bodyparts5((L5*(loc1-1))+i,:);
    right_elbow_5(i,:)=bodyparts5((L5*(loc2-1))+i,:);
    right_shoulder_5(i,:)=bodyparts5((L5*(loc3-1))+i,:);
    right_hip_5(i,:)=bodyparts5((L5*(loc4-1))+i,:);
end

%% Change reference system 
% Activity: build a single matrix of zeros containing the position xyz of the 25 joints

bodyparts01=zeros(n,3);
bodyparts02=zeros(n,3);
bodyparts03=zeros(n,3);
bodyparts04=zeros(n,3);
bodyparts05=zeros(n,3);

for location=1:n
    for i=1:3
        column = 2+(4*(location-1))+i;
        if(location==1)
            bodyparts01(:,i) = vertcat(drinking_d(1,column),zeros(n-1,1));
            bodyparts02(:,i) = vertcat(drinking2_d(1,column),zeros(n-1,1));
            bodyparts03(:,i) = vertcat(drinking3_d(1,column),zeros(n-1,1));
            bodyparts04(:,i) = vertcat(drinking4_d(1,column),zeros(n-1,1));
            bodyparts05(:,i) = vertcat(drinking5_d(1,column),zeros(n-1,1));
         else
            bodyparts01(:,i) = vertcat(bodyparts01(1:(location-1),i),drinking_d(1,column),zeros((length(bodyparts01(:,1))-location),1));
            bodyparts02(:,i) = vertcat(bodyparts02(1:(location-1),i),drinking2_d(1,column),zeros((length(bodyparts02(:,1))-location),1));
            bodyparts03(:,i) = vertcat(bodyparts03(1:(location-1),i),drinking3_d(1,column),zeros((length(bodyparts03(:,1))-location),1));
            bodyparts04(:,i) = vertcat(bodyparts04(1:(location-1),i),drinking4_d(1,column),zeros((length(bodyparts04(:,1))-location),1));
            bodyparts05(:,i) = vertcat(bodyparts05(1:(location-1),i),drinking5_d(1,column),zeros((length(bodyparts05(:,1))-location),1));
        end
    end
end

% First rotation (78°)  on x axis: vector of u = matrix rot1 * vector v
% new body parts is called new_act
theta1=1.36136;
rot1=zeros(3,3); rot1(1,1)=1; rot1(2,2)=cos(theta1); rot1(2,3)=-sin(theta1); rot1(3,2)=sin(theta1); rot1(3,3)=cos(theta1);

for i=1:length(bodyparts01(:,1))
    newposition=rot1*bodyparts01(i,:)'; new_bodyparts01(i,:)=newposition';
end
for i=1:length(bodyparts02(:,1))
    newposition=rot1*bodyparts02(i,:)'; new_bodyparts02(i,:)=newposition';
end
for i=1:length(bodyparts03(:,1))
    newposition=rot1*bodyparts03(i,:)'; new_bodyparts03(i,:)=newposition'; 
end
for i=1:length(bodyparts04(:,1))
    newposition=rot1*bodyparts04(i,:)'; new_bodyparts04(i,:)=newposition';
end
for i=1:length(bodyparts05(:,1))
    newposition=rot1*bodyparts05(i,:)'; new_bodyparts05(i,:)=newposition';
end

for i=1:length(right_wrist_1(:,1))
    newposition=rot1*right_wrist_1(i,:)'; new_act1(i,:)=newposition';
    newposition=rot1*right_elbow_1(i,:)'; new_act1elb(i,:)=newposition';
    newposition=rot1*right_shoulder_1(i,:)'; new_act1sh(i,:)=newposition';
    newposition=rot1*right_hip_1(i,:)'; new_act1hip(i,:)=newposition';
end
for i=1:length(right_wrist_2(:,1))
    newposition=rot1*right_wrist_2(i,:)'; new_act2(i,:)=newposition';
    newposition=rot1*right_elbow_2(i,:)'; new_act2elb(i,:)=newposition';
    newposition=rot1*right_shoulder_2(i,:)'; new_act2sh(i,:)=newposition';
    newposition=rot1*right_hip_2(i,:)'; new_act2hip(i,:)=newposition';
end
for i=1:length(right_wrist_3(:,1))
    newposition=rot1*right_wrist_3(i,:)'; new_act3(i,:)=newposition';
    newposition=rot1*right_elbow_3(i,:)'; new_act3elb(i,:)=newposition';
    newposition=rot1*right_shoulder_3(i,:)'; new_act3sh(i,:)=newposition';
    newposition=rot1*right_hip_3(i,:)'; new_act3hip(i,:)=newposition';
end
for i=1:length(right_wrist_4(:,1))
    newposition=rot1*right_wrist_4(i,:)'; new_act4(i,:)=newposition';
    newposition=rot1*right_elbow_4(i,:)'; new_act4elb(i,:)=newposition';
    newposition=rot1*right_shoulder_4(i,:)'; new_act4sh(i,:)=newposition';
    newposition=rot1*right_hip_4(i,:)'; new_act4hip(i,:)=newposition';
end
for i=1:length(right_wrist_5(:,1))
    newposition=rot1*right_wrist_5(i,:)'; new_act5(i,:)=newposition';
    newposition=rot1*right_elbow_5(i,:)'; new_act5elb(i,:)=newposition';
    newposition=rot1*right_shoulder_5(i,:)';new_act5sh(i,:)=newposition';
    newposition=rot1*right_hip_5(i,:)'; new_act5hip(i,:)=newposition';
end

% Second rotation in z axis
% new body parts is called new2_act
rot2=zeros(3,3); rot2(2,2)=cos(theta2); rot2(1,1)=cos(theta2); rot2(1,2)=-sin(theta2); rot2(2,1)=sin(theta2); rot2(3,3)=1;
for i=1:length(new_bodyparts01(:,1))
    newposition=rot2*new_bodyparts01(i,:)'; new2_bodyparts01(i,:)=newposition';
end
for i=1:length(new_bodyparts02(:,1))
    newposition=rot2*new_bodyparts02(i,:)'; new2_bodyparts02(i,:)=newposition';
end
for i=1:length(new_bodyparts03(:,1))
    newposition=rot2*new_bodyparts03(i,:)'; new2_bodyparts03(i,:)=newposition';
end
for i=1:length(new_bodyparts04(:,1))
    newposition=rot2*new_bodyparts04(i,:)'; new2_bodyparts04(i,:)=newposition';
end
for i=1:length(new_bodyparts05(:,1))
    newposition=rot2*new_bodyparts05(i,:)'; new2_bodyparts05(i,:)=newposition';
end

for i=1:length(new_act1(:,1))
    newposition=rot2*new_act1(i,:)'; new2_act1(i,:)=newposition';
    newposition=rot2*new_act1elb(i,:)'; new2_act1elb(i,:)=newposition';
    newposition=rot2*new_act1sh(i,:)'; new2_act1sh(i,:)=newposition';
    newposition=rot2*new_act1hip(i,:)'; new2_act1hip(i,:)=punewpositionta';
end
for i=1:length(new_act2(:,1))
    newposition=rot2*new_act2(i,:)'; new2_act2(i,:)=newposition';
    putanewposition=rot2*new_act2elb(i,:)'; new2_act2elb(i,:)=newposition';
    newposition=rot2*new_act2sh(i,:)'; new2_act2sh(i,:)=newposition';
    newposition=rot2*new_act2hip(i,:)'; new2_act2hip(i,:)=newposition';
end
for i=1:length(new_act3(:,1))
    newposition=rot2*new_act3(i,:)'; new2_act3(i,:)=newposition';
    newposition=rot2*new_act3elb(i,:)'; new2_act3elb(i,:)=newposition';
    newposition=rot2*new_act3sh(i,:)'; new2_act3sh(i,:)=newposition';
    newposition=rot2*new_act3hip(i,:)'; new2_act3hip(i,:)=newposition';
end
for i=1:length(new_act4(:,1))
    newposition=rot2*new_act4(i,:)'; new2_act4(i,:)=newposition';
    newposition=rot2*new_act4elb(i,:)'; new2_act4elb(i,:)=newposition';
    newposition=rot2*new_act4sh(i,:)'; new2_act4sh(i,:)=newposition';
    newposition=rot2*new_act4hip(i,:)'; new2_act4hip(i,:)=newposition';
end
for i=1:length(new_act5(:,1))
    newposition=rot2*new_act5(i,:)'; new2_act5(i,:)=newposition';
    newposition=rot2*new_act5elb(i,:)'; new2_act5elb(i,:)=newposition';
    newposition=rot2*new_act5sh(i,:)'; new2_act5sh(i,:)=newposition';
    newposition=rot2*new_act5hip(i,:)'; new2_act5hip(i,:)=newposition';
end

%% Translation
% new body parts is called new3_act

v1=-new2_bodyparts01(21,1)+0.8; 
v2=-new2_bodyparts01(21,2)+0.5; 
v3=-new2_bodyparts01(21,3)+0.6;trans1=eye(4); 
trans1(1,4)=v1;
trans1(2,4)=v2;
trans1(3,4)=v3;

ones1=ones(length(right_wrist_1),1); 
new2_act1_ones=horzcat(new2_act1,ones1);
for i=1:length(right_wrist_1)
    puta=trans1*new2_act1_ones(i,:)'; new3_act1(i,:)=puta';
end

new3_act1(:,4)=[];
ones11=ones(length(right_elbow_1),1); new2_act1elb_ones=horzcat(new2_act1elb,ones11);
for i=1:length(right_elbow_1)
    puta=trans1*new2_act1elb_ones(i,:)'; new3_act1elb(i,:)=puta';
end

new3_act1elb(:,4)=[];
ones21=ones(length(right_shoulder_1),1); new2_act1sh_ones=horzcat(new2_act1sh,ones21);
for i=1:length(right_shoulder_1)
    puta=trans1*new2_act1sh_ones(i,:)'; new3_act1sh(i,:)=puta';
end

new3_act1sh(:,4)=[];
ones31=ones(length(right_hip_1),1); new2_act1hip_ones=horzcat(new2_act1hip,ones31);
for i=1:length(right_hip_1)
    puta=trans1*new2_act1hip_ones(i,:)'; new3_act1hip(i,:)=puta';
end

new3_act1hip(:,4)=[];
v1=-new2_bodyparts02(21,1)+0.8; v2=-new2_bodyparts02(21,2)+0.5; v3=-new2_bodyparts02(21,3)+0.6;trans1=eye(4); trans1(1,4)=v1;trans1(2,4)=v2;trans1(3,4)=v3;
ones2=ones(length(right_wrist_2),1); new2_act2_ones=horzcat(new2_act2,ones2);
for i=1:length(right_wrist_2)
    puta=trans1*new2_act2_ones(i,:)';
    new3_act2(i,:)=puta';
end

new3_act2(:,4)=[];
ones12=ones(length(right_elbow_2),1); new2_act2elb_ones=horzcat(new2_act2elb,ones12);
for i=1:length(right_elbow_2)
    puta=trans1*new2_act2elb_ones(i,:)';
    new3_act2elb(i,:)=puta';
end

new3_act2elb(:,4)=[];
ones22=ones(length(right_shoulder_2),1); new2_act2sh_ones=horzcat(new2_act2sh,ones22);
for i=1:length(right_shoulder_2)
    puta=trans1*new2_act2sh_ones(i,:)'; new3_act2sh(i,:)=puta';
end

new3_act2sh(:,4)=[];
ones32=ones(length(right_hip_2),1); new2_act2hip_ones=horzcat(new2_act2hip,ones32);
for i=1:length(right_hip_2)
    puta=trans1*new2_act2hip_ones(i,:)'; new3_act2hip(i,:)=puta';
end

new3_act2hip(:,4)=[];
v1=-new2_bodyparts03(21,1)+0.8; v2=-new2_bodyparts03(21,2)+0.5; v3=-new2_bodyparts03(21,3)+0.6;trans1=eye(4); trans1(1,4)=v1;trans1(2,4)=v2;trans1(3,4)=v3;
ones3=ones(length(right_wrist_3),1); new2_act3_ones=horzcat(new2_act3,ones3);
for i=1:length(right_wrist_3)
    puta=trans1*new2_act3_ones(i,:)';
    new3_act3(i,:)=puta';
end

new3_act3(:,4)=[];
ones13=ones(length(right_elbow_3),1); new2_act3elb_ones=horzcat(new2_act3elb,ones13);
for i=1:length(right_elbow_3)
    puta=trans1*new2_act3elb_ones(i,:)';
    new3_act3elb(i,:)=puta';
end

new3_act3elb(:,4)=[];
ones23=ones(length(right_shoulder_3),1); new2_act3sh_ones=horzcat(new2_act3sh,ones23);
for i=1:length(right_shoulder_3)
    puta=trans1*new2_act3sh_ones(i,:)'; new3_act3sh(i,:)=puta';
end

new3_act3sh(:,4)=[];
ones33=ones(length(right_hip_3),1); new2_act3hip_ones=horzcat(new2_act3hip,ones33);
for i=1:length(right_hip_3)
    puta=trans1*new2_act3hip_ones(i,:)'; new3_act3hip(i,:)=puta';
end

new3_act3hip(:,4)=[];
v1=-new2_bodyparts04(21,1)+0.8; v2=-new2_bodyparts04(21,2)+0.5; v3=-new2_bodyparts04(21,3)+0.6;trans1=eye(4); trans1(1,4)=v1;trans1(2,4)=v2;trans1(3,4)=v3;
ones4=ones(length(right_wrist_4),1); new2_act4_ones=horzcat(new2_act4,ones4);
for i=1:length(right_wrist_4)
    puta=trans1*new2_act4_ones(i,:)';
    new3_act4(i,:)=puta';
end

new3_act4(:,4)=[];
ones14=ones(length(right_elbow_4),1); new2_act4elb_ones=horzcat(new2_act4elb,ones14);
for i=1:length(right_elbow_4)
    puta=trans1*new2_act4elb_ones(i,:)';
    new3_act4elb(i,:)=puta';
end

new3_act4elb(:,4)=[];
ones24=ones(length(right_shoulder_4),1); new2_act4sh_ones=horzcat(new2_act4sh,ones24);
for i=1:length(right_shoulder_4)
    puta=trans1*new2_act4sh_ones(i,:)'; new3_act4sh(i,:)=puta';
end

new3_act4sh(:,4)=[];
ones34=ones(length(right_hip_4),1); new2_act4hip_ones=horzcat(new2_act4hip,ones34);
for i=1:length(right_hip_4)
    puta=trans1*new2_act4hip_ones(i,:)'; new3_act4hip(i,:)=puta';
end

new3_act4hip(:,4)=[];
v1=-new2_bodyparts05(21,1)+0.8; v2=-new2_bodyparts05(21,2)+0.5; v3=-new2_bodyparts05(21,3)+0.6;trans1=eye(4); trans1(1,4)=v1;trans1(2,4)=v2;trans1(3,4)=v3;
ones5=ones(length(right_wrist_5),1); new2_act5_ones=horzcat(new2_act5,ones5);
for i=1:length(right_wrist_5)
    puta=trans1*new2_act5_ones(i,:)';
    new3_act5(i,:)=puta';
end

new3_act5(:,4)=[];
ones15=ones(length(right_elbow_5),1); new2_act5elb_ones=horzcat(new2_act5elb,ones15);
for i=1:length(right_elbow_5)
    puta=trans1*new2_act5elb_ones(i,:)';
    new3_act5elb(i,:)=puta';
end

new3_act5elb(:,4)=[];
ones25=ones(length(right_shoulder_5),1); new2_act5sh_ones=horzcat(new2_act5sh,ones25);
for i=1:length(right_shoulder_5)
    puta=trans1*new2_act5sh_ones(i,:)'; new3_act5sh(i,:)=puta';
end

new3_act5sh(:,4)=[];
ones35=ones(length(right_hip_5),1); new2_act5hip_ones=horzcat(new2_act5hip,ones35);
for i=1:length(right_hip_5)
    puta=trans1*new2_act5hip_ones(i,:)'; new3_act5hip(i,:)=puta';
end
new3_act5hip(:,4)=[];

%ones20=ones(length(right_elbow_10),1); new2_act10elb_ones=horzcat(new2_act10elb,ones20);
