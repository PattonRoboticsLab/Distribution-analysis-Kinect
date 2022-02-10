%% Load csv files --> still have to understand how to save each table from table

% Folder where data are stored (my directory)
fileFolder = ('/Users/jialinhe1/Desktop/Tesi/Kinect/Codes/data');

% Get a list of all files in the folder with the desired file name pattern.
filePattern = fullfile(fileFolder, '*.csv'); % Change to whatever pattern you need.
fileDirectory = dir(filePattern);

for i = 1 : size(fileDirectory,1)
    baseFileName = fileDirectory(i).name;
    fullFileName = fullfile(fileDirectory(i).folder, baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    activity = textscan(fullFileName,'%s')
    drinking_table = readtable(fullFileName, 'NumHeaderLines',1);
    %filename = sprintf( 'Subject_session_freexp_%d', i);
    %writetable(table, filename);
end

