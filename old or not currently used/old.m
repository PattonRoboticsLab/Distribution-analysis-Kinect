%% See number of counts using CDF (cummulative density function) --> forse inutile

nBins=4;
counts1=plotDust(noExo_wrist,nBins)
nBins=4;
counts2=plotDust(withExo_wrist,nBins)

cdf1 = cumsum(counts1); 
cdf1 = cdf1 / cdf1(end); 
data1GT90_ = find(cdf1>= 0.9, 1, 'first')

cdf2 = cumsum(counts2); 
cdf2 = cdf2 / cdf2(end); 
data2GT90 = find(cdf2>= 0.8, 1, 'first')


%% Kullback-Leibler Divergence
% 
% temp_zeros = zeros(size(mov1,1),1);
% %mov1_Label = [mov1, temp_zeros];
% temp_ones = ones(size(mov2,1),1);
% %mov2_Label = [mov2, temp_ones];
% movForEntropy = cat(1, mov1, mov2);
% labelsForEntropy = cat(1, temp_zeros, temp_ones);
% labelsForEntropy_log=logical(labelsForEntropy);
% %writematrix(mov1,'mov1.csv')
% %writematrix(mov2,'mov2.csv')
% Z = relativeEntropy(movForEntropy,labelsForEntropy_log)

