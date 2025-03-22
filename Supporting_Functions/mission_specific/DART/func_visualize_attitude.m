%% [ ] Methods: Visualize Attitude in True_SC_ADC

function obj = func_visualize_attitude(obj,storage_data, true_SC_body, true_SC_solar_panel, software_SC_control_attitude,true_SC_micro_thruster_actuator, true_SC_chemical_thruster_actuator,mission_true_time)

mArrow3([0 0 0]',[1 0 0]', 'facealpha', 0.1, 'color', 'r', 'stemWidth', 0.01);
hold on
mArrow3([0 0 0]',[0 1 0]', 'facealpha', 0.1, 'color', 'g', 'stemWidth', 0.01);
mArrow3([0 0 0]',[0 0 1]', 'facealpha', 0.1, 'color', 'b', 'stemWidth', 0.01);


mArrow3([0 0 0]',obj.rotation_matrix*[0.5 0 0]', 'facealpha', 1, 'color', 'r', 'stemWidth', 0.005);
mArrow3([0 0 0]',obj.rotation_matrix*[0 0.5 0]', 'facealpha', 1, 'color', 'g', 'stemWidth', 0.005);
mArrow3([0 0 0]',obj.rotation_matrix*[0 0 0.5]', 'facealpha', 1, 'color', 'b', 'stemWidth', 0.005);


% SC body shape
SC_Shape_Model = [];
SC_Shape_Model.Vertices = (obj.rotation_matrix * true_SC_body.shape_model.Vertices')';
SC_Shape_Model.Faces = true_SC_body.shape_model.Faces;
patch(SC_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'none')


% SP shape
for i=1:true_SC_solar_panel.num_solar_panels
    SP_Shape_Model = [];
    SP_Shape_Model.Vertices = (obj.rotation_matrix * true_SC_solar_panel.solar_panel_data(i).shape_model.Vertices')';
    SP_Shape_Model.Faces = true_SC_solar_panel.solar_panel_data(i).shape_model.Faces;
    patch(SP_Shape_Model, 'FaceColor','blue', 'EdgeColor', 'none')
end


% Microthruster
if true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
    for i=1:true_SC_micro_thruster_actuator.num_micro_thruster
        loc = obj.rotation_matrix * true_SC_micro_thruster_actuator.MT_data(i).location;
        dir = obj.rotation_matrix * true_SC_micro_thruster_actuator.MT_data(i).orientation;
        scale = true_SC_micro_thruster_actuator.MT_data(i).commanded_thrust*0.1/true_SC_micro_thruster_actuator.MT_data(i).maximum_thrust;
        quiver3(loc(1),loc(2),loc(3) , dir(1),dir(2),dir(3),"LineWidth",3,"DisplayName",['MT ',num2str(i)],"AutoScaleFactor",scale)
    end
end


% Chemical Thruster
if true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
    for i=1:true_SC_chemical_thruster_actuator.num_chemical_thruster
        loc = obj.rotation_matrix * true_SC_chemical_thruster_actuator.chemical_thruster_data(i).location;
        dir = obj.rotation_matrix * true_SC_chemical_thruster_actuator.chemical_thruster_data(i).orientation;
        scale = true_SC_chemical_thruster_actuator.chemical_thruster_data(i).commanded_thrust*0.15/true_SC_micro_thruster_actuator.MT_data(i).maximum_thrust;
        quiver3(loc(1),loc(2),loc(3) , dir(1),dir(2),dir(3),"LineWidth",10,"DisplayName",['CT ',num2str(i)],"AutoScaleFactor",scale)
    end
end


view([-25,30])
axis equal
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
if software_SC_control_attitude.actuator_to_use == 1
    info_actuator ="Attitude Actuator in use : MT";
elseif software_SC_control_attitude.actuator_to_use == 2
    info_actuator ="Attitude Actuator in use : RWA";
elseif software_SC_control_attitude.actuator_to_use == 3
    info_actuator ="Attitude Actuator in use : both (DESAT)";
else
    % nothing
end
if software_SC_control_attitude.desired_SC_attitude_mode == 1
    info_pointing = "Pointing for : SB";
elseif software_SC_control_attitude.desired_SC_attitude_mode == 2
    info_pointing = "Pointing for : SUN";
elseif software_SC_control_attitude.desired_SC_attitude_mode == 3
    info_pointing = "Pointing for: DELTA V";
elseif software_SC_control_attitude.desired_SC_attitude_mode == 4
    info_pointing = "Pointing for: DTE";
elseif software_SC_control_attitude.desired_SC_attitude_mode == 5
    info_pointing = "Pointing for: INTERSAT comm";
else
    % nothing
end
if sum([true_SC_chemical_thruster_actuator.chemical_thruster_data.command_actuation])>0
    info_thrust = "Thruster firing !";
else
    info_thrust = "Thruster OFF";
end
title ([['Mission Simulation, Time = ',num2str(round(mission_true_time.time)),' sec'],info_actuator,info_pointing,info_thrust])


light
camlight('headlight')
grid on

xlabel('X_{SUN}')
ylabel('Y_{SUN}')
zlabel('Z_{SUN}')


set(gca, 'fontsize',storage_data.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

drawnow limitrate


end
