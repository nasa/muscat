function pos_points = func_load_science_points_v2(num_points)
    % 1 = 500 points, 
    % 2 = 1000 points, 
    % 3 = 2500 points, 
    % 4 = 5000 points, 
    % 5 = 10000 points, 
    % 6 = 50000 points, 
    % 7 = 100000 points, 
    % 8 = 200000 points, 
    % 9 = 400000 points, 
    % 10 = 500000 points, 
    % 11 = 1000000 points, 
    % 12 = 50 points, 
    % 13 = 100 points, 
    % 14 = 200 points, 
    % 15 = 300 points, 
    % 16 = 400 points

    filename = ['../../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_',num2str(num_points),'.mat'];

    if isfile(filename)
        load(filename);
    else
        error('File doesnt exist!')
    end

    %     switch selection
    %         case 1
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_500.mat');
    %         case 2
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_1000.mat');
    %         case 3
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_2500.mat');
    %         case 4
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_5000.mat');
    %         case 5
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_10000.mat');
    %         case 6
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_50000.mat');
    %         case 7
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_100000.mat');
    %         case 8
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_200000.mat');
    %         case 9
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_400000.mat');
    %         case 10
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_500000.mat');
    %         case 11
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_1000000.mat');
    %         case 12
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_50.mat');
    %         case 13
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_100.mat');
    %         case 14
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_200.mat');
    %         case 15
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_300.mat');
    %         case 16
    %             load('../MuSCAT_Supporting_Files/Science_points/pos_point_sphere_v3_400.mat');
    %         otherwise
    %             error('Shouldnt reach here!')
    %     end

end