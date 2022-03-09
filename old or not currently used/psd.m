%% power spectral density 

% start from raw data
activity = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/jialin_FE_4min.csv', 'NumHeaderLines',1);

n=25;
% double variable instead of table 
act_array=table2array(activity);     
L=length(act_array(:,1)); 
bodyparts=zeros(n*L,3);   
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

%% Time frame 

frames(:,1)=activity(:,1);              
total_sec=(table2array(frames(end,1))-table2array(frames(1,1)))/1000;      

for i=1:length(table2array(activity(:,1)))
    time(i,1)=(table2array(frames(i,1))-table2array(frames(1,1)))/1000;
end

for i=1:length(table2array(activity(:,1)))-1
    frequency(i,1)=1/((time(i+1,1)-time(i,1))); 
end

%% see wrist
loc_wrist=11;
for i=1:L
    right_wrist(i,:)=bodyparts((L*(loc_wrist-1))+i,:);
end

[pxx,f]=pwelch(right_wrist(:,1),[],[],[],33);
axis equal
plot(f,pow2db(pxx))
xlim([0 100])