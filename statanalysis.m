%% statistical analysis
% https://www.countbayesie.com/blog/2017/5/9/kullback-leibler-divergence-explained


% The best test of which is better is to ask which distribution preserves the most information
% from our original data source. This is where Kullback-Leibler Divergence comes in.

% entropy defines as "the minimum number of bits it would take us to encode our 
% information" (info would be in my case distribution of motion). It gives
% us the min number of bit needed to encode the number of motion in single case

% The key thing with Entropy is that, simply knowing the theoretical lower bound 
% on the number of bits we need, we have a way to quantify exactly how much information
% is in our data. 
% KL divergence is not symmetric
%