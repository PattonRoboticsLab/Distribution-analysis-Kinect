%% Statistical Parametric Mapping (SPM) 

%% Make time as % of cycle - wrist position

precision=1;

noexo= cycle(exercise_sub1_wrist,time,L1,precision);
exo = cycle(exercise_sub2_wrist,time2,L2,precision);

figure
plot(noexo(:,4),noexo(:,1),'.-')
hold on
plot(exo(:,4),exo(:,1),'.-')
xlabel('Cycle (%)')
ylabel('Wrist position in x')
title('Time results converted into cycle percentage')
hold off

%% T test

fitdist(noexo(:,1),'Normal') % check normal distribution (mean, sd)

no_exo=noexo';
mu=0;

figure 
t= spm1d.stats.ttest(no_exo - mu);
ti = t.inference(0.05,'two_tailed',false);
ti.plot();

%% ANOVA

A=zeros(202,1);
A(102:202,1)=1;
B=[];
B=vertcat(noexo,exo);
C=[];
C=B(:,1);

p=anova1(C,A)

% noexo_v=noexo';
% exo_v=exo';
% data1=noexo_v(1,:);
% data2=exo_v(1,:);
% 
% data=vertcat(data1,data2);
% A=zeros(2,1);
% A(2,:)=1;
% 
% spm = spm1d.stats.anova1(data, A);
% spmi= spm.inference(0.05);
% disp(spmi)
% 
% spmi.plot();
% spmi.plot_threshold_label();
% spmi.plot_p_values();
% 
% clear noexo_v; clear exo_v;
