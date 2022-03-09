%% Create random sample 

function [cod_iter,cod_adj_iter,mean_cod,max_cod,min_cod]=random1D(activity,nBins,nFolds)

cod=zeros(nFolds,1);
cod_adj=zeros(nFolds,1);
itersample=zeros(nFolds,1);
cod_iter=[];
cod_adj_iter=[];
timexlabel=zeros(length(cod),1);
time=zeros(nFolds,1);
mean_cod = [];
max_cod = [];
min_cod = [];

for nsamples=100:100:size(activity,1)
    for i=1:nFolds
        initial = randperm(size(activity,1));
        sample = activity(initial(1:nsamples), :);
        Lsample = length(sample);
        [cod(i,:),cod_adj(i,:)]= find_Pcorr_and_COD_1D(sample,activity, nBins, Lsample, size(activity,1));
        itersample(i,:) = nsamples;
        timexlabel(i,:)=0.033*nsamples;
        close all
    end
    A = [cod,itersample,timexlabel];
    B= [cod_adj,itersample,timexlabel];
    cod_iter = vertcat(cod_iter,A);
    cod_adj_iter = vertcat(cod_adj_iter,B);
    time=0.033*nsamples;
    mean1 = mean(cod);
    C=[time,mean1];
    mean_cod = vertcat(mean_cod,C); 
    max1 = max(cod);
    D=[time,max1];
    max_cod = vertcat(max_cod,D);
    min1 = min(cod);
    E=[time,min1];
    min_cod = vertcat(min_cod,E); 
end

