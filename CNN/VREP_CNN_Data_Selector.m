%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% VREP CNN Data Item Selector - Mat Files
% =======================================
% 
% Randomly selects data items from different stages of the VREP Sim
% trajectory for inclusion in a subset from CNN training and validation.
%
% Selected dataitem is retained in it Matlab file format (.mat), which
% contains the 'image' and 'output' (targets).
%
% Paul Zapotezny-Anderson (n4337948)
% 22 May 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear

% directories
dataset = dir('.\dataset');             %relative path to dataset folder
datasetFolder = dataset(1).folder;
trainFolder = fullfile(datasetFolder,'train');
testFolder = fullfile(datasetFolder,'val');

% defining selection space for each trajectory
frontWindow =   15;     %beginning of trajectory where most manoeuvring occurs
backWindow =    10;     %end of trajectory where postioning and stopping occurs

% selection size for each window in the selection space
                        %the first step is always selected
fWinNum =       3;      %number of additional steps to be taken from the front window
mWinNum =       2;      %number of steps to be taken from the middle window
bWinNum =       2;      %number of steps to be taken from the back window

% training and test set proportions
train =         0.7;    %proportion of processed data item for training set
test =          1 - train;    %proportion of processed data item for test set   



select_traj = [];
select_step = [];

% get list of trajectories and their length
for i = 1:length(dataset)
    itemname = dataset(i).name;
    if strncmp(itemname,'MTS',3)
        trajectory = str2num(extractBefore(extractAfter(itemname,'MTS_'),'_'));
        steps = extractBefore(itemname, '.mat');
        steps = str2num(extractAfter(steps,strlength(steps)-2));
        if isempty(find(select_traj==trajectory))
            select_traj = [select_traj;trajectory];
            select_step = [select_step;steps];
        end
    end
end

% select random steps from each trajectory iaw selection space
selectSet = zeros(size(select_traj,1),8);

for i = 1:size(select_traj,1)
    steps = select_step(i);
    selectSet(i,1) = 1;
    selectSet(i,2:1+fWinNum) = randperm(frontWindow-1,fWinNum) + 1;
    selectSet(i,2+fWinNum:1+fWinNum+mWinNum) = ...
        randperm(steps-1-backWindow-frontWindow,mWinNum) + frontWindow;
    selectSet(i,2+fWinNum+mWinNum:1+fWinNum+mWinNum+mWinNum) = ...
        steps - randperm(backWindow,bWinNum) +1;
end

% compile a list of dataitem filenames to pick
count = 0;
list = {};

for i = 1:size(selectSet,1)
    for j=1:size(selectSet,2)
        count = count + 1;
        dataitem = ['MTS_',num2str(select_traj(i)),'_',num2str(selectSet(i,j)),'_', num2str(select_step(i)),'.mat'];
        filename = fullfile(datasetFolder,dataitem);
        list{count} = filename;
        %load(filename);
        %status = movefile(filename, selectFolder);
        %if status==0
         %   display(filename);
        %end
    end
end

% copy ramdomly selected dataitems from picklist to the train or test folder
listsize = length(list);
picklist = randperm(listsize);

trainsize = round(listsize * train);

for i = 1:listsize
    filename = list{picklist(i)};
    if i < trainsize
        copyfile(filename, trainFolder);
    else
        copyfile(filename, testFolder);
    end
end


