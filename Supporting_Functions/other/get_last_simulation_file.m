function file = get_last_simulation_file()
path = './Output/Test/';
files = dir(path);
files = files(3:end);
files = {files.name};
files = sort(files);
file = [path files{end} filesep 'all_data.mat'];
end