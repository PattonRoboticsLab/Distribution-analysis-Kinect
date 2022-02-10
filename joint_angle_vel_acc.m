%% Angular velocity and acceleration of joint angles

function [ja_vel, ja_acc]=joint_angle_vel_acc(joint_angles,newref_filt_shoulder,time)

for i=1:length(newref_filt_shoulder(:,1))-1
    ja_vel(i,:)=(joint_angles(i+1,:)-joint_angles(i,:))/(time(i+1)-time(i));
end

for i=1:length(newref_filt_shoulder(:,1))-2
    ja_acc(i,:)=(ja_vel(i+1,:)-ja_vel(i,:))/(time(i+1)-time(i));
end


% threshold = 0.1;
% cont1=0;
% vect1=zeros(1,length(new3_act1(:,1))-1);
% for i=1:length(new3_act1(:,1))-1
%     if((abs(vel_act1(i,1))<threshold)&&(abs(vel_act1(i,2))<threshold)&&(abs(vel_act1(i,3))<threshold))
%         vect1(i)=i;
%     end
% 
% end
% vect1(vect1==0) = [];
% vel_act1(vect1,:)=[];
% 
% 

