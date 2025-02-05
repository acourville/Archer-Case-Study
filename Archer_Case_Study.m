clear;
clc;

%% RC Car Specs
Size = 1/16;    % RC Car Scale
GR = 5;      % Gear ratio motor shaft to wheel shaft (N2/N1)
R_wheel_supra = 0.33;           % Full size car tire radius, m
R_wheel = R_wheel_supra*Size;   % RC Car tire radius, m
C_wheel = pi*2*R_wheel;         % RC Car tire circumference, m
width_wheel = 0.254 * Size;     % RC car tire width, m

wheelbase_supra = 2.74;         % Full size car wheelbase, m https://toyotagazooracing.com/gr/supra/specs/
wheelbase = wheelbase_supra * Size; % RC car wheelbase
track = wheelbase / 1.55; % RC car track, m

% Calculate wheel speed needed for max speed
v_car_req = 30;                 % Minimum top speed requirement, m/s
W_wheel_req = v_car_req / C_wheel * 2 * pi; % Minimum wheel speed requirement, Rad/s

%% Battery Specs
V_cell_nom = 3.6;       % Nominal cell voltage, V
V_cell_max = 4.2;       % Max cell voltage, V
V_cell_min = 2.5;       % Min cell voltage, V


%% 2D Plots for motor max speed
motor_specs = readmatrix('Archer Motor Options.csv');

for motor_option = 1:1:10
    for GR = 1:1:10
        w_motor = motor_specs(motor_option,2) * V_cell_min*motor_specs(motor_option,5)/60*2*pi;      % Motor speed, Rad/s
        w_wheel = w_motor * 1/GR;                 % Wheel speed, Rad/s
        v_car(motor_option, GR) = C_wheel * w_wheel / (2*pi);     % Car speed, m/s
    end
    GR_req(motor_option) = interp1(v_car(motor_option,:),1:1:10,v_car_req);
    GR_opt(motor_option) = floor( GR_req(motor_option))+floor(( GR_req(motor_option)-floor( GR_req(motor_option)))/0.5)*0.5;
    
    w_motor = motor_specs(motor_option,2) * V_cell_min*motor_specs(motor_option,5)/60*2*pi;      % Motor speed, Rad/s
    w_wheel = w_motor * 1/GR_opt(motor_option);                 % Wheel speed, Rad/s
    V_car_opt(motor_option) = C_wheel * w_wheel / (2*pi);     % Car speed, m/
 
end

close all
figure;
plot(transpose(v_car));
yline(v_car_req,'--', '30 m/s requirement');
title('Max Speed - 1/16 Scale')
xlabel('Gear Ratio N2/N1')
ylabel('Max Speed, m/s')
legend('Motor 1','Motor 2', 'Motor 3', 'Motor 4', 'Motor 5', 'Motor 6', 'Motor 7', 'Motor 8', 'Motor 9', 'Motor 10');



%% Mass model

    % Import battery options
    battery_spec = readmatrix('Archer Battery Options.csv');

    % wheels
    volume_wheel = pi()*(R_wheel^2)*width_wheel;    %RC car wheel volume, m^3, assume solid cylinder
    rho_rubber =  920;                              %Natural rubber density kg/m^3, https://designerdata.nl/materials/natural-rubber
    m_wheels = volume_wheel * rho_rubber;

    % solar panel mass
    m_solar = wheelbase*track*15;

    % Calculate chassis thickness
    SF_tres = 1.2;              % Safety factor treshold
    G_load = 5.0;               % Vertical G load factor for chassis sizing
    rho_AL = 2700;              % Aluminum density, kg/m^3 https://www.matweb.com/search/datasheet.aspx?MatGUID=b8d536e0b9b54bd7b69e4124d8f1d20a&ckck=1
    yield_AL = 276*10^6;             % Aluminum yield stenght, MPa https://www.matweb.com/search/datasheet.aspx?MatGUID=b8d536e0b9b54bd7b69e4124d8f1d20a&ckck=1
    shear_AL = 207*10^6;             % Aluminum shear stenght, MPa https://www.matweb.com/search/datasheet.aspx?MatGUID=b8d536e0b9b54bd7b69e4124d8f1d20a&ckck=1


    for option = 1:1:10
        SF = 0;                     % Safety factor
        thick_new = 0.0001;     % Chassis starting thickness, m
        m_pack = battery_spec(option,11);   % Battery pack mass, kg

        m_motor = motor_specs(option,6);

        % Electronics
        m_electronics = m_motor * 0.5;

        while SF < SF_tres  %safety factor
            
            thick_chassis = thick_new;
    
            m_chassis(option) = wheelbase * track * thick_chassis * rho_AL;
            m_car = m_pack + m_motor + m_chassis(option) + (4*m_wheels) + m_electronics + m_solar;
    
            M_max = G_load*m_car*9.81*wheelbase/8;             % Max bending moment, Nm 
            V_max = G_load*m_car*9.81/2;                       % Max shear force, N
    
            I_chassis = track*(thick_chassis^3)/12;
            Bend_stress = M_max * (wheelbase/2) / I_chassis;    % Chassis max stress from bending, Pa
            SF_M = yield_AL / Bend_stress;      % Safety factor aggainst bending yield
    
            Shear_stress = V_max / (track * thick_chassis);
            SF_V = yield_AL / Shear_stress;       % Safety factor aggainst shear
    
            SF = min(SF_M, SF_V);       % Limiting safety factor
    
            thick_new = thick_chassis + 0.0001;
        end

        Chassis_opt(option, 1) = option;             % Option label
        Chassis_opt(option, 4) = SF;                 % Safety factor
        Chassis_opt(option, 2) = thick_chassis;      % Optimal thickness, mm
        Chassis_opt(option, 3) = m_car;              % Car mass, mm
    end



%% Calculate Acceleration Time
 for option = 1:1:10
    if battery_spec(option,3) <= battery_spec(option,2)
        P_motor(option) = battery_spec(option,6)*3.6 * battery_spec(option,3);  % Motor power, watts
    else
        display('Current limited by motor');
        P_motor(option) = battery_spec(option,6)*3.6 * battery_spec(option,2); % Motor power, watts
    end
    W_motor_req = W_wheel_req*GR_opt(option); % Min motor speed to meet top speed requirement w/ gear ratio, Rad/s
    T_m = P_motor(option) / W_motor_req;        % Motor torque, n-m
    
    F_fric = T_m * GR_opt(option) / R_wheel; % Net friction propulsion force from car tire to ground, N
    
    a_car(option) = F_fric / Chassis_opt(option, 3);     % Car acceleration m/s^2, assuming no slipping, no drag, no rolling resistance
    t_vmax(option) = v_car_req / a_car(option); % Acceleration time from zero to the requiremd max speed
    t_vmax(option) = round(t_vmax(option),1);   % Round to 1 decimal point

 end

figure;
b=bar(t_vmax);
title('Acceleration Times - 1/16 Scale')
xlabel('Motor Option')
ylabel('Time to V req, seconds')
ylim([0 100])
b(1).Labels = b(1).YData;


%% Cost Model

for option = 1:1:10
    
    Cost(option, 1) = battery_spec(option, 13);      % Battery cost
    Cost(option, 2) = motor_specs(option,8);        % motor cost
    Cost(option, 3) = 0.5 * Cost(option, 2);          % electronics cost
    Cost(option, 4) = 4 * m_wheels * 2 * 2;            % wheel cost
    Cost(option, 5) = m_chassis(option) * 2.6 * 2;    % chassis cost
    Cost(option, 6) = wheelbase * track * 100 * 1.5;      % solar cost

    Cost(option, 7) = Cost(option, 1) + Cost(option, 2) + Cost(option, 3) + Cost(option, 4) + Cost(option, 5) + Cost(option, 6); % Total vehicle cost
    Cost(option, 7) = round(Cost(option, 7),2);

end

figure;
b=bar(Cost(:,7));
title('Vehicle Cost Options - 1/8 Scale')
xlabel('Motor Option')
ylabel('Vehicle Cost $USD')
ylim([0 180])
b(1).Labels = b(1).YData;