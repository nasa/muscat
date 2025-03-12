function func_plot_spacecraft_attitude( ...
    kd, mission_storage, mission_true_SC, i_SC, scale_sc, position_offset, Reci2rtn)

if kd == -1
    rot_matrix_SC = Reci2rtn * mission_true_SC{i_SC}.true_SC_adc.rotation_matrix_SC;
else
    rot_matrix_SC = Reci2rtn * squeeze(mission_storage.true_SC{i_SC}.rotation_matrix_SC(kd,:,:));
end

% SC body shape
true_SC_body = mission_true_SC{i_SC}.true_SC_body;
for i_shape = 1:length(true_SC_body.shape_model)
    shape_i = true_SC_body.shape_model{i_shape};
    if isfield(shape_i, 'color')
        color = shape_i.color;
    else
        color = 0.7*[1 1 1];
    end
    ecolor = 'none';
    SC_Shape_Model = [];
    SC_Shape_Model.Vertices = (rot_matrix_SC * shape_i.Vertices')'*scale_sc + position_offset';
    SC_Shape_Model.Faces = shape_i.Faces;
    patch(SC_Shape_Model, 'FaceColor', color, 'EdgeColor', ecolor)
    hold on;
end

% Solar panels
true_SC_solar_panel = mission_true_SC{i_SC}.true_SC_solar_panel;
for i=1:true_SC_solar_panel.num_solar_panels
    SP_data_i = true_SC_solar_panel.solar_panel_data(i);
    SP_Shape_Model = [];
    SP_Shape_Model.Vertices = (rot_matrix_SC *SP_data_i.shape_model.Vertices')' * scale_sc + position_offset';
    SP_Shape_Model.Faces = SP_data_i.shape_model.Faces;
    patch(SP_Shape_Model, 'FaceColor','blue', 'EdgeColor', ecolor)
end

% Microthruster
if true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
    true_SC_micro_thruster_actuator = mission_true_SC{i_SC}.true_SC_micro_thruster_actuator;
    for i=1:true_SC_micro_thruster_actuator.num_micro_thruster
        MT_data_i = true_SC_micro_thruster_actuator.MT_data(i);
        loc = rot_matrix_SC * MT_data_i.location*scale_sc + position_offset;
        dir = rot_matrix_SC * MT_data_i.orientation;
        scale = MT_data_i.commanded_thrust * 0.1 / MT_data_i.maximum_thrust * scale_sc;
        name = ['MT ',num2str(i)];
        quiver3( ...
            loc(1),loc(2),loc(3), ...
            dir(1),dir(2),dir(3), ...
            "LineWidth",3,"DisplayName",name,"AutoScaleFactor",scale)
    end
end

% Chemical Thruster
if true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
    true_SC_chemical_thruster_actuator = mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator;
    for i=1:true_SC_chemical_thruster_actuator.num_chemical_thruster
        CT_data_i = true_SC_chemical_thruster_actuator.chemical_thruster_data(i);
        loc = rot_matrix_SC * CT_data_i.location*scale_sc + position_offset;
        dir = rot_matrix_SC * CT_data_i.orientation;
        scale = CT_data_i.commanded_thrust * 0.15 / CT_data_i.maximum_thrust * scale_sc;
        name = ['CT ',num2str(i)];
        quiver3( ...
            loc(1),loc(2),loc(3), ...
            dir(1),dir(2),dir(3), ...
            "LineWidth",10,"DisplayName",name,"AutoScaleFactor",scale)
    end
end
end
