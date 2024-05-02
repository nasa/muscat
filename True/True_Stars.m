classdef True_Stars < handle
    %TRUE_STARS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        num_stars % Number of stars
        sao_name % Smithsonian Astrophysical Observatory (SAO) Star Catalog%s name of star
        magnitude_visible % Magnitude of star
        all_stars_unit_vector % Unit vector denoting position of all stars
        maximum_magnitude % Maximum magnitude of stars visible to camera
    end

    methods
        function obj = True_Stars(mission_true_time)
            %TRUE_STARS Construct an instance of this class            

            YYYY = str2double(mission_true_time.t_initial_date_string(8:11));

            % SAO Star Catalog : http://tdc-www.harvard.edu/catalogs/sao.html 
            fid=fopen('saoNAN.txt');
            M=textscan(fid, '%f %f %f %f %f %f %f %f', 'headerlines', 1);
            obj.sao_name=M{1}; 
            obj.magnitude_visible=M{3}; 
            RA=(M{5}+(M{7}*(YYYY-2000)))/15; 
            DEC=(M{6}+(M{8}*(YYYY-2000)));

            obj.num_stars = length(obj.sao_name);

            obj.all_stars_unit_vector = zeros(obj.num_stars,3);
            for i=1:1:obj.num_stars

                x_hat = [1 0 0]';

                RA_angle = deg2rad(RA(i)*15); % [rad]
                Dec_angle = deg2rad(DEC(i)); % [rad]

                Rot_Z_star_RA = [cos(RA_angle) -sin(RA_angle) 0;
                    sin(RA_angle)  cos(RA_angle) 0;
                    0                    0 1];

                Rot_Y_star_Dec = [cos(Dec_angle) 0 sin(Dec_angle);
                    0 1 0;
                    -sin(Dec_angle) 0 cos(Dec_angle)];

                Rot_RA_dec = Rot_Y_star_Dec * Rot_Z_star_RA;

                obj.all_stars_unit_vector(i,:) = (Rot_RA_dec*x_hat)';

            end

        end

    end
end

