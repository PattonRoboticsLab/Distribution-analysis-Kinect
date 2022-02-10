%% Calculate the joint angles 
% wrist elbow shoulder --> elbow angle
% elbow shoulder hip --> shoulder angle

function [joint_angles]=calc_jointangles(newref_filt_wrist,newref_filt_elbow,newref_filt_shoulder,newref_filt_hip,newref_filt_spine)

% create vectors for each segment "shoulder-hip","shoulder-spine(@shoulder)", "shoulder-elbow","elbow-wrist"
for i=1:length(newref_filt_shoulder(:,1))                         % all same length
    hipshoulder_vector(i,:)=newref_filt_hip(i,:)-newref_filt_shoulder(i,:);
    elbowshoulder_vector(i,:)=newref_filt_elbow(i,:)-newref_filt_shoulder(i,:);
    wristelbow_vector(i,:)=newref_filt_wrist(i,:)-newref_filt_elbow(i,:);
    spineshoulder_vector(i,:)=newref_filt_shoulder(i,:)-newref_filt_spine(i,:);
end

% shoulder abduction/adduction - angle between hipshoulder and elbowshoulder
% angle = atan2(norm(cross(a,b)),dot(a,b)) or acos??
for i=1:length(newref_filt_shoulder(:,1))
    %shoulder_AA_angle(i,1)= acos(dot(hipshoulder_vector(i,:),elbowshoulder_vector(i,:))/norm(hipshoulder_vector(i,:))*norm(elbowshoulder_vector(i,:)));
    joint_angles(i,1)= (atan2(norm(cross(hipshoulder_vector(i,:),elbowshoulder_vector(i,:))),dot((hipshoulder_vector(i,:)),elbowshoulder_vector(i,:))))*180/pi;
end

% shoulder flexion/extension - angle between spineshoulder and elbowshoulder
for i=1:length(newref_filt_shoulder(:,1))
    %shoulder_FE_angle(i,1) = acos(dot(spineshoulder_vector(i,:),elbowshoulder_vector(i,:))/norm(spineshoulder_vector(i,:))*norm(elbowshoulder_vector(i,:)));
    joint_angles(i,2)= (atan2(norm(cross(spineshoulder_vector(i,:),elbowshoulder_vector(i,:))),dot((spineshoulder_vector(i,:)),elbowshoulder_vector(i,:))))*180/pi;
end

% elbow flexion/extension - angle between elbowwrist and elbowshoulder
for i=1:length(newref_filt_shoulder(:,1))
    %elbow_angle(i,1) = acos(dot(elbowshoulder_vector(i,:),wristelbow_vector(i,:))/norm(elbowshoulder_vector(i,:))*norm(wristelbow_vector(i,:)));
    joint_angles(i,3)= (atan2(norm(cross(elbowshoulder_vector(i,:),wristelbow_vector(i,:))),dot((elbowshoulder_vector(i,:)),wristelbow_vector(i,:))))*180/pi;
end


