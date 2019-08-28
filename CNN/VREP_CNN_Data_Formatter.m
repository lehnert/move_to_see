%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% VREP CNN Data Formatter
% =======================
% 
% Extracts data from Move To See VREP sim recordings, to produce individual
% image with output for CNN training and validation.
%
% Paul Zapotezny-Anderson (n4337948)
% 22 May 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Cycle through each VREP sim recording in the folder to extract image,
% gradients, dPitch and dRoll for each step in the trajectory and save.

% format of filename: MTS_<T>_<S>_<L>
%                   : T = trajectory ('VREP_sim_*') number
%                   : S = step in the trajectory the data is taken from
%                   : L = length, the number of steps in a trajectory

matFiles = dir('.\VREP_training_data_3');

for i = 1:length (matFiles)
    if (strncmp(matFiles(i).name,"vrep_sim", 8))
        matData = load(strcat(matFiles(i).folder,'\',matFiles(i).name));
        steps = length(matData.count);
        for k = 1:steps
            image = squeeze(matData.images(k,:,:,:));
            gradient = squeeze(matData.gradient(k,:,:))';
            dPitch = matData.pose_deltas(k,4);
            dRoll = matData.pose_deltas(k,5);
            output = [gradient, dPitch, dRoll];
            filename = '.\dataset_3\MTS_' + string(i) + '_' + string(k) + '_' + ...
                string(steps);
            save(filename, 'image', 'output');
        end
    end
end

% save VREP_sims processed to help match trajectory number to VREP_sim
save('VREP_sim_list', 'matFiles');

            
            
    

