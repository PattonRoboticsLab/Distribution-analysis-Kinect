%% Control precision of Kinect collecting joint data

function probability=data_acquisition_accuracy(activity)

n=25;      % total number of joint 

% use table2array to obtain a double variable instead of table 
act_array=table2array(activity(:,2:101));       % first column (time) excluded

% use length to obtain a scalar (total number of rows)
L=length(act_array(:,1)); 

% matrix containing only precision
precision_matrix=zeros(L,n);   

for location=1:n
        precision_matrix(:,location)=act_array(:,4*location);
end


final_sum=sum(sum(precision_matrix));
probability = 100*final_sum/(25*2*L); 
fprintf('Kinect precision in collecting joint data is %d %', probability);