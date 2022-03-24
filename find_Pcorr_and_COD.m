%% Find the coefficient of determination inter or intra personal joint movements
% Input:
%   - JointMov x = joint movement subject x
%   - L = length of Jointmov
% Output:
%   -  mov'x'_norm = normalized JointMov x

function [Rsquared,Rsquared_adj,pearson_corr,mov1_norm,mov2_norm]= find_Pcorr_and_COD(JointMov1, JointMov2, nBins, L, L2)

mov_concat = cat(1, JointMov1, JointMov2);
%[counts,binLimits,binCenters]=countInBins(combined_wrist,nBins); % fast-sort histogram
X = mov_concat;

if ~exist('minMax','var'),                    % if not passed 
    mins=min(mov_concat); ranges=range(mov_concat);             % set to points min and max
else
  mins=minMax(1,:); ranges=range(minMax);
end       

[N,nDim]=size(X);                             % find dimensions 

if N<=2, fprintf('X: >2 rows.'); return; end 
fprintf('\n Sorting (%d points, %d dimensions, %d Bins)...',N,nDim,nBins); 
xb=NaN*X;                                     % initiate all NaN

for dim=1:nDim                                % each column is a dimension
  x=X(:,dim);                                 % get this dimension's data
  MIN=mins(dim);  R=ranges(dim);              % stats on it                                   
  xFractOfRange=(x-MIN)/(R+R*eps);            % fraction, alter2include all 
                                              % eps returns the distance from 1.0 to the next 
                                              % larger double-precision number(2e-52)
  xb(:,dim) = uint8(nBins*xFractOfRange +.5); % Xform2binSpace&makeInteger
  binLimits(:,dim)=(MIN)+R/nBins*[0:nBins]';  % delimiters
  binLimits(nBins+1,dim)=binLimits(nBins+1,dim);
  %binCenters(:,dim)=MIN + R/nBins/2 + R/nBins*[0:nBins]'; % halfway
  binCenters(:,dim)=binLimits(:,dim)+R/nBins/2';% halfway
end
binCenters(nBins+1,:)=[];                     % clip off last row

%% Create Counts Function for Joint Movement 1
x1 = xb(1:L, :);
% nested loop counting depends on #dimensions. Construct code then use eval
cmd0=['counts_mov1=NaN*ones(' num2str(nBins) ',' num2str(nDim) '); '];  % init count

cmd1=['']; % init% setup for loops for each dimension
for dim=1:nDim % loop to set up nested for statements
  cmd1=[cmd1 'for j' num2str(dim) '=1:nBins, '];  
end
cmd2=', comp=['; % setup comparison matrix for all dimensions that match
for dim=1:nDim
  cmd2=[cmd2 'j' num2str(dim) '*ones(L,1) '];  
end
cmd2=[cmd2 ']; ']; 
cmd2=[cmd2 'm=(comp==x1); ']; % N by nDim array of mathing bins
if nDim==1, 
  cmd2=[cmd2 'match=(m==ones(L,1)); ']; 
else
  cmd2=[cmd2 'match=(transpose(sum(transpose(m))))==nDim*ones(L,1); '];
end
cmd3=['counts_mov1('];  % init count
for dim=1:nDim
  cmd3=[cmd3 'j' num2str(dim) ','];
end
cmd3(length(cmd3))=[];                                   % removeFinalComma
cmd3=[cmd3 ')=sum( match ); ']; % sumGridMatch
cmd4=['']; % init % wrap up loops in this command string
for dim=1:nDim
  cmd4=[cmd4 'end; '];  
end
cmd=[cmd0 cmd1 cmd2 cmd3 cmd4];   % Put pieces together 
%disp(cmd)
eval(cmd);                        % execute the command string

%fprintf('done. '); drawnow; pause(.0001);

%% Create Counts Function for Joint Movement 2

x2 = xb(L+1:L+L2,:);
% nested loop counting depends on #dimensions. Construct code then use eval
cmd0=['counts_mov2=NaN*ones(' num2str(nBins) ',' num2str(nDim) '); '];  % init count

cmd1=['']; % init% setup for loops for each dimension
for dim=1:nDim % loop to set up nested for statements
  cmd1=[cmd1 'for j' num2str(dim) '=1:nBins, '];  
end
cmd2=', comp=['; % setup comparison matrix for all dimensions that match
for dim=1:nDim
  cmd2=[cmd2 'j' num2str(dim) '*ones(L2,1) '];  
end
cmd2=[cmd2 ']; ']; 
cmd2=[cmd2 'm=(comp==x2); ']; % N by nDim array of mathing bins
if nDim==1, 
  cmd2=[cmd2 'match=(m==ones(L2,1)); ']; 
else
  cmd2=[cmd2 'match=(transpose(sum(transpose(m))))==nDim*ones(L2,1); '];
end
cmd3=['counts_mov2('];  % init count
for dim=1:nDim
  cmd3=[cmd3 'j' num2str(dim) ','];
end
cmd3(length(cmd3))=[];                                   % removeFinalComma
cmd3=[cmd3 ')=sum( match ); ']; % sumGridMatch
cmd4=['']; % init % wrap up loops in this command string
for dim=1:nDim
  cmd4=[cmd4 'end; '];  
end
cmd=[cmd0 cmd1 cmd2 cmd3 cmd4];   % Put pieces together 
%disp(cmd)
eval(cmd);                        % execute the command string

fprintf('done. '); drawnow; pause(.0001);

%% Plot Joint Movement 1

mov1 = [0,0,0,0];
%figure
%plot3(x(:,1),x(:,2),x(:,3),'b.'); ax=axis; hold on;  grid on; 
for j1=1:nBins
for j2=1:nBins
  for j3=1:nBins
    binrow = [binCenters(j1,1),binCenters(j2,2),binCenters(j3,3),nBins*50*counts_mov1(j1,j2,j3)/L+1];
    mov1 = [mov1;binrow];
    %plot3(binCenters(j1,1),binCenters(j2,2),binCenters(j3,3),'o', ...
      %'markerSize',nBins*50*counts_mov1(j1,j2,j3)/L+1, 'MarkerEdgeColor','w', 'MarkerFaceColor','r');
    %disp([binCenters(j1,1), binCenters(j2,2), binCenters(j3,3)]);
    %disp(nBins*50*counts(j1,j2,j3)/N+1);
    %disp(counts(j1,j2,j3))
    
    if counts_mov1(j1,j2,j3)>0,
      text(binCenters(j1,1),binCenters(j2,2),binCenters(j3,3), ...
        ['' num2str(counts_mov1(j1,j2,j3))],'fontSize',7,'Color','k');
    end
%     plot3(binLimits(j1,1)*[1 1], binLimits(j2,2)*[1 1], [ax(5), ax(6)],'g:'); % line showing bins
%     plot3(binLimits(j1,1)*[1 1], [ax(3), ax(4)], binLimits(j3,3)*[1 1],'g:'); % line showing bins
%     plot3([ax(1), ax(2)], binLimits(j2,2)*[1 1], binLimits(j3,3)*[1 1],'g:'); % line showing bins
  end
end
%       drawnow; pause(.01); % update display while you go
end
% plot3(x(:,1),x(:,2),x(:,3),'b.'); % plot again

%% Plot Joint Movement 2
x = JointMov2;

mov2 = [0,0,0,0];
% figure
% plot3(x(:,1),x(:,2),x(:,3),'b.'); ax=axis; hold on;  grid on; 
for j1=1:nBins
for j2=1:nBins
  for j3=1:nBins
    binrow = [binCenters(j1,1),binCenters(j2,2),binCenters(j3,3),nBins*50*counts_mov2(j1,j2,j3)/L2+1];
    mov2 = [mov2;binrow];
%     plot3(binCenters(j1,1),binCenters(j2,2),binCenters(j3,3),'o', ...
%       'markerSize',nBins*50*counts_mov2(j1,j2,j3)/L2+1, 'MarkerEdgeColor','w', 'MarkerFaceColor','r');
%     %disp([binCenters(j1,1), binCenters(j2,2), binCenters(j3,3)]);
    %disp(nBins*50*counts(j1,j2,j3)/N+1);
    %disp(counts(j1,j2,j3))
    if counts_mov2(j1,j2,j3)>0,
      text(binCenters(j1,1),binCenters(j2,2),binCenters(j3,3), ...
        ['' num2str(counts_mov2(j1,j2,j3))],'fontSize',7,'Color','k');
    end
%     plot3(binLimits(j1,1)*[1 1], binLimits(j2,2)*[1 1], [ax(5), ax(6)],'g:'); % line showing bins
%     plot3(binLimits(j1,1)*[1 1], [ax(3), ax(4)], binLimits(j3,3)*[1 1],'g:'); % line showing bins
%     plot3([ax(1), ax(2)], binLimits(j2,2)*[1 1], binLimits(j3,3)*[1 1],'g:'); % line showing bins
  end
end
%       drawnow; pause(.01); % update display while you go
end
% plot3(x(:,1),x(:,2),x(:,3),'b.'); % plot again

%% Normalize data
mov1_norm = normalize(mov1(:,4));
mov2_norm = normalize(mov2(:,4));

%% Calculate Pearson correlation

pearson_corr = corr(mov1(:,4), mov2(:,4));
%fprintf('\n Pearson correlation is %d', pearson_corr);

%% Coefficient of determination

%fit linear regression model
mdl =fitlm(mov1_norm, mov2_norm);
Rsquared = mdl.Rsquared.Ordinary;   % R-squared value
Rsquared_adj = mdl.Rsquared.Adjusted;   % adjusted R-squared value
%fprintf('\n Coefficient of determination is %d', Rsquared);

