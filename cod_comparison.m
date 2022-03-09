%% just COD without histogram analysis

function [Rsquared,Rsquared_adj]= cod_comparison(data1,data2)
 
mdl =fitlm(data1, data2);
Rsquared = mdl.Rsquared.Ordinary;   % R-squared value
Rsquared_adj = mdl.Rsquared.Adjusted;   % adjusted R-squared value
