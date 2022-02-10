%% Angular velocity and acceleration of joint angles

function [ja_vel, ja_acc]=joint_angle_vel_acc(joint_angles,newref_filt_shoulder,time)

for i=1:length(newref_filt_shoulder(:,1))-1
    ja_vel(i,:)=(joint_angles(i+1,:)-joint_angles(i,:))/(time(i+1)-time(i));
end

for i=1:length(newref_filt_shoulder(:,1))-2
    ja_acc(i,:)=(ja_vel(i+1,:)-ja_vel(i,:))/(time(i+1)-time(i));
end


