%% Load csv file manually

clear all
close all
clc

% noexo = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/plots_data/prova.csv', 'NumHeaderLines',1);
% pre = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/plots_data/pretherapy.csv', 'NumHeaderLines',1);
% post = readtable('/Users/jialinhe1/Desktop/Tesi/Kinect/data/plots_data/posttherapy.csv', 'NumHeaderLines',1);
% noexo_cov=table2array(noexo); 
% pre_cov=table2array(pre); 
% post_cov=table2array(post);

%% coverage position

% noexo_cov=[0,0.0165;1,0.0143;2,0.0179;3,0.0314;4,0.0197;5,0.0261;6,0.0228;7,0.0258];
% pre_cov=[1.1,0.0191;2.1,0.0166;3.1,0.023;4.1,0.0184;5.1,0.0206;6.1,0.0303];
% post_cov=[1.6,0.0117;2.6,0.0191;3.6,0.0191;4.6,0.0217;5.6,0.0163;6.6,0.024];

%% coverage velocity
noexo_cov=[0,0.93;1,0.3718;2,0.307;3,1.0178;4,0.6327;5,0.5805;6,0.5848;7,0.5805];
pre_cov=[1.1,0.2946;2.1,0.3504;3.1,0.9089;4.1,0.3919;5.1,0.5904;6.1,0.3212];
post_cov=[1.6,0.4288;2.6,0.5707;3.6,0.6613;4.6,0.5487;5.6,0.7963;6.6,0.2512];

%% coverage joint angles
% noexo_cov=[2,29.575;3,19.815;4,18.259;5,15.519;6,18.054;7,23.344];
% pre_cov=[2.1,10.533;3.1,3.3406;4.1,7.212;5.1,5.0924;6.1,3.4967];
% post_cov=[2.6,21.523;3.6,1.7321;4.6,6.1255;5.6,3.4967;6.6,3.51];

%% 

%figure
% sz1 = 50;
% sz2 =100;
p1=scatter(noexo_cov(:,1),noexo_cov(:,2),'red','o', 'LineWidth',1.5);
hold on
p2=scatter(pre_cov(:,1),pre_cov(:,2),'black','filled','LineWidth',1);
hold on
p3=scatter(post_cov(:,1),post_cov(:,2),'blue','^','LineWidth',1);
hold on
% g_x=[0:1:5]; 
% g_y=[0:0.02:0.18]; %position
% g_y=[0:0.2:1.6];   % velocity
% for i=1:length(g_x)
%    plot([g_x(i) g_x(i)], [g_y(1) g_y(end)],'k:') %y grid lines
%    hold on    
% end
% for i=1:length(g_y)
%    plot([g_x(1) g_x(end)],[g_y(i) g_y(i)],'k:') %x grid lines
%    hold on    
% end

x=[-0.05 0.05 0.05 -0.05];
%y=[0 0 0.04 0.04];    % position
y=[0 0 1.2 1.2];    % velocity
%y=[0 0 30 30];    % joint angles
behind=patch(x,y,'red');
behind.FaceAlpha=0.1;
for i=1:7
    x=[-0.05 0.05 0.05 -0.05]+i;
    %y=[0 0 0.04 0.04];    % position
    y=[0 0 1.2 1.2];    % velocity
    %y=[0 0 30 30];    % joint angles
    behindred=patch(x,y,'red');
    behindred.FaceAlpha=0.1;
end

x=[0.05 0.15 0.15 0.05];
%y=[0 0 0.04 0.04];    % position
y=[0 0 1.2 1.2];    % velocity
%y=[0 0 30 30];    % joint angles
behind=patch(x,y,'blue');
behind.FaceAlpha=0.1;
for j=1:7
    x=[0.05 0.15 0.15 0.05]+j;
    %y=[0 0 0.04 0.04];    % position
    y=[0 0 1.2 1.2];    % velocity
    %y=[0 0 30 30];    % joint angles
    behind=patch(x,y,'blue');
    behind.FaceAlpha=0.1;
end

x=[0.15 0.55 0.55 0.15];
%y=[0 0 0.04 0.04];    % position
y=[0 0 1.2 1.2];    % velocity
%y=[0 0 30 30];    % joint angles
behind=patch(x,y,'green');
behind.FaceAlpha=0.05;
for k=1:7
    x=[0.15 0.55 0.55 0.15]+k;
    %y=[0 0 0.04 0.04];    % position
    y=[0 0 1.2 1.2];    % velocity
    %y=[0 0 30 30];    % joint angles
    behind=patch(x,y,'green');
    behind.FaceAlpha=0.05;
end

x=[0.55 0.65 0.65 0.55];
%y=[0 0 0.04 0.04];    % position
y=[0 0 1.2 1.2];    % velocity
%y=[0 0 30 30];    % joint angles
behind=patch(x,y,'blue');
behind.FaceAlpha=0.1;
for j=1:7
    x=[0.55 0.65 0.65 0.55]+j;
    %y=[0 0 0.04 0.04];    % position
    y=[0 0 1.2 1.2];    % velocity
    %y=[0 0 30 30];    % joint angles
    behind=patch(x,y,'blue');
    behind.FaceAlpha=0.1;
end

grid off
set(gca,'xtick',[]);
% ylabel('95% coverage  [m^3]');
% ylabel('95% coverage (m/s)^3');
ylabel('95% coverage ')
xlim([-0.05 8])
%legend([p1 p2 p3],'No ExoNET','With ExoNET pre-therapy','With ExoNET post-therapy')
%title('Coverage 95th percentile position')
%title('Coverage 95th percentile velocity')
hold off
