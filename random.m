%% Create random sample 

function [cod_iter,cod_adj_iter,mean_cod]=random(activity,nBins,nIterations)

cod=zeros(nIterations,1);
cod_adj=zeros(nIterations,1);
itersample=zeros(nIterations,1);
cod_iter=[];
cod_adj_iter=[];
timexlabel=zeros(length(cod),1);
mean_cod = [];

for nsamples=100:100:size(activity,1)
    for i=1:nIterations
        initial = randperm(size(activity,1));
        sample = activity(initial(1:nsamples), :);
        Lsample = length(sample);
        [cod(i,:),cod_adj(i,:)]= find_coef_determination(sample,activity, nBins, Lsample, size(activity,1));
        itersample(i,:) = nsamples;
        timexlabel(i,:)=0.033*nsamples;
        close all
    end
    A = [cod,itersample,timexlabel];
    B= [cod_adj,itersample,timexlabel];
    cod_iter = vertcat(cod_iter,A);
    cod_adj_iter = vertcat(cod_adj_iter,B);
    mean1 = mean(cod);
    mean_cod = vertcat(mean_cod,mean1);
end

