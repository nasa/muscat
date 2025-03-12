% Publishes the classes autonmatically, so that they can be used for documentation

close all, clear all, clc

% Change workspace folder to this file
mfile_name          = mfilename('fullpath');
[pathstr,name,ext]  = fileparts(mfile_name);
cd(pathstr);
clear mfile_name pathstr name ext

% Add these paths
addpath(genpath('../.'))


%% Initialize Publish Options 

% Generate view of MATLAB file in specified format: https://www.mathworks.com/help/matlab/ref/publish.html

pub_options = [];
pub_options.format = 'pdf';
pub_options.showCode = true;
pub_options.evalCode = false;
pub_options.outputDir = "../../Documentation/Publish_Output/";

% To enhance the readability of the published document and include additional image snapshots, external file content, and external images, see Publishing Markup: https://www.mathworks.com/help/matlab/matlab_prog/marking-up-matlab-comments-for-publishing.html


%% Auto Publish 

all_folders_array = ["Main", "True_Environment", "True_SC", "Software_SC", "True_Sensors_Actuators", "Mission"];
% all_folders_array = ["True_Sensors_Actuators"];

for i = 1:1:length(all_folders_array)

    this_Dir = ['../../',convertStringsToChars(all_folders_array(i))]

    this_file_list = dir(this_Dir);

    for j = 1:1:length(this_file_list)

        this_file_name = this_file_list(j).name;

        if contains(this_file_name,'.m')

            % Publish this file
            publish([this_Dir, '/', this_file_name],pub_options)

        end

    end    

end
