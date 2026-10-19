% Publishes the classes automatically so they can be used for documentation

close all; 
clearvars; 
clc;

% 1. Safely change workspace folder to this file's directory
[pathstr, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(pathstr)
    cd(pathstr);
end

% 2. Add dependencies relative to current folder
addpath(genpath(fullfile('..', '.')));

%% Initialize Publish Options 

% Use struct() instead of [] for robust structure creation in modern MATLAB
pub_options = struct();
pub_options.format = 'pdf';
pub_options.showCode = true;
pub_options.evalCode = false;
pub_options.outputDir = fullfile('..', '..', 'Documentation', 'Publish_Output');

%% Auto Publish 

all_folders_array = ["Main", "True_Environment", "True_SC", "Software_SC", "True_Sensors_Actuators", "Mission"];

for i = 1:length(all_folders_array)

    % Use fullfile and char conversion safely across OS platforms
    this_folder_name = char(all_folders_array(i));
    this_Dir = fullfile('..', '..', this_folder_name);

    if ~isfolder(this_Dir)
        warning('Folder does not exist: %s', this_Dir);
        continue;
    end

    this_file_list = dir(this_Dir);

    for j = 1:length(this_file_list)

        % Skip directories (e.g., '.', '..')
        if this_file_list(j).isdir
            continue;
        end

        this_file_name = this_file_list(j).name;

        % Ensure file strictly ends with .m extension
        if endsWith(this_file_name, '.m', 'IgnoreCase', true)

            full_file_path = fullfile(this_Dir, this_file_name);

            % Publish this file
            fprintf('Publishing: %s\n', full_file_path);
            publish(full_file_path, pub_options);

        end

    end    

end