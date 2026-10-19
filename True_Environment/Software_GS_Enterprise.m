%% Class: Software_GS_Enterprise
% Tracks the status of Spacecraft

classdef Software_GS_Enterprise < handle

    %% Properties
    properties
        %% [ ] Properties: Initialized Variables

        name % [string] 'SW GS Enterprise i' (Optional)

        mode_software_SC_enterprise_selector % [string] Select which Mode to run

        %% [ ] Properties: Variables Computed Internally

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_GS_Enterprise(init_data, mission, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['SW GS Enterprise ',num2str(i_HW)];
            end

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            obj.mode_software_SC_enterprise_selector = init_data.mode_software_SC_enterprise_selector;

            % Initialize Variables to store:
            obj.store = [];

            switch obj.mode_software_SC_enterprise_selector

                case 'Generic'
                    % Do nothing!

                case 'Blue'
                    obj = func_software_GS_enterprise_constructor_blue(obj, mission);

                case 'Red'
                    obj = func_software_GS_enterprise_constructor_red(obj, mission);

                otherwise
                    error('Enterprise Mode: Not defined!')

            end

        end

        %% [ ] Methods: Main
        % Update Software GS Enterprise

        function obj = func_main_software_GS_enterprise(obj, mission, i_HW)

            switch obj.mode_software_SC_enterprise_selector

                case 'Generic'
                    % Do nothing!

                case 'Blue'
                    obj = func_software_GS_enterprise_blue(obj, mission, i_HW);

                case 'Red'
                    obj = func_software_GS_enterprise_red(obj, mission, i_HW);

                otherwise
                    error('Enterprise Mode: Not defined!')

            end

            % Update Storage
            obj = func_update_software_GS_enterprise_store(obj, mission);

        end


        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_software_GS_enterprise_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                % store something

                switch obj.mode_software_SC_enterprise_selector

                    case 'Generic'
                        % Do nothing!

                    case 'Blue'
                        obj = func_software_GS_enterprise_store_blue(obj, mission);

                    case 'Red'
                        obj = func_software_GS_enterprise_store_red(obj, mission);

                    otherwise
                        error('Enterprise Mode: Not defined!')

                end

            end

        end


    end
end

