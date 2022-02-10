%% Calculate velocity and acceleration function

function [vel_act,acc_act,vect1]=calc_vel_acc(new_refsystem_wrist,time)

for i=1:length(newref_filt_wrist(:,1))-1
    vel_act(i,:)=(newref_filt_wrist(i+1,:)-newref_filt_wrist(i,:))/(time(i+1)-time(i));
end

for i=1:length(newref_filt_wrist(:,1))-2
    acc_act(i,:)=(vel_act(i+1,:)-vel_act(i,:))/(time(i+1)-time(i));
end

% 
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
% 
