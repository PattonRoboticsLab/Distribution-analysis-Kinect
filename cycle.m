%% Cycle of exercise: consider starting and ending moment
% equaldata: matrix containing data (xyz) of percentage that increases
% every 0.1%

function equaldata = cycle(data,time,L)
 
tot_time = time(L)-time(1);
timecycle=zeros(L,1);

for i=1:L
    timecycle(i,:) = 100*time(i)/tot_time;
end

cycledata = [data,timecycle];

M4q = 0:0.1:100;
equaldata = [interp1(cycledata(:,4), cycledata(:,1:3), M4q(:)) M4q(:)];
