%% Calculate coverage function
% calculate first mean, median and standard deviation
% remove everything ouside % percentile 

function volume=calc_coverage(activity,percentile,group)

% figure
center_point=mean(activity);
mediana=median(activity);
% stand_dev=std(activity);
% boxplot(activity,'notch','on')

distance_x=[];
distance_y=[];
distance_z=[];

inside_x=[];
inside_y=[];
inside_z=[];

for i=1:size(activity)
    distance_x(i,1)=activity(i,1)-mediana(1,1); 
    distance_y(i,1)=activity(i,2)-mediana(1,2);
    distance_z(i,1)=activity(i,3)-mediana(1,3);
end

maxcutoffx=prctile(distance_x,percentile);
maxcutoffy=prctile(distance_y,percentile);
maxcutoffz=prctile(distance_z,percentile);
mincutoffx=prctile(distance_x,(100-percentile));
mincutoffy=prctile(distance_y,(100-percentile));
mincutoffz=prctile(distance_z,(100-percentile));

for i=1:size(distance_x)
    if (abs(distance_x(i))<maxcutoffx) && (abs(distance_y(i))<maxcutoffy) && ( ...
            abs(distance_z(i))<maxcutoffz) && (abs(distance_x(i))>mincutoffx) && ( ...
            abs(distance_y(i))>mincutoffy) && ( ...
            abs(distance_z(i))>mincutoffz)
        inside_x=vertcat(inside_x,distance_x(i));
        inside_y=vertcat(inside_y,distance_y(i));
        inside_z=vertcat(inside_z,distance_z(i));
    end
end

inside_std=horzcat(inside_x,inside_y,inside_z);
remainingpoints=inside_std+mediana;

figure
scatter3(activity(:,1),activity(:,2), activity(:,3),1,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
hold on
scatter3(mediana(:,1),mediana(:,2),mediana(:,3),5,'MarkerFaceColor','r');
hold on
axis equal
grid on
xlabel('Horizontal plane [m]');
ylabel('Sagittal plane [m]');
zlabel('Frontal plane [m]');
title('Range of Motion of wrist',group)

[area,volume]=convhull(inside_x,inside_y,inside_z);
trimesh(area,remainingpoints(:,1),remainingpoints(:,2),remainingpoints(:,3));
fprintf('\n Volume of ROM is %d m^3', volume);

