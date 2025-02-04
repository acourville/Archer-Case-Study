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

%% Battery Specs
V_cell_nom = 3.6;       % Nominal cell voltage, V
V_cell_max = 4.2;       % Max cell voltage, V
V_cell_min = 2.5;       % Min cell voltage, V
I_cell_rated = 25;      % Cell continuous discharge rating (max) A
m_cell = 0.043;         % Cell mass, kg
P_count = 8;            % Pack P-Count
S_count = 3;            % Pacl S-Count
pack_factor = 1.6;      % Battery pack packing factor, https://www.batterydesign.net/cell-to-pack-mass-ratio/

V_pack_nom = V_cell_nom *S_count; % Nominal Pack Volatage, V
m_pack = pack_factor * m_cell * P_count * S_count; % Total pack mass, kg
I_pack_max = I_cell_rated * P_count;    % Pack continuous discharge current (max) A


%% Motor Specs
m_motor = 0.300;        % Motor Mass, kg
kV = 3500;              % kV Rating, RPM/V
V_motor_max = 15;             % Max Voltage
I_motor_max = 140;            % Max Current
P_max = V_motor_max*I_motor_max;    % Max Power
W_m_max = kV*V_motor_max/60*2*pi; % Motor max speed, Rad/s
T_m_max = P_max/W_m_max;    % Motor max torque


% Calculate motor speed needed for max speed
v_car_req = 30;                 % Minimum top speed requirement, m/s
W_wheel_req = v_car_req / C_wheel * 2 * pi; % Minimum wheel speed requirement, Rad/s
W_motor_req = W_wheel_req*GR; % Min motor speed to meet top speed requirement w/ gear ratio, Rad/s


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

    % wheels
    volume_wheel = pi()*(R_wheel^2)*width_wheel;    %RC car wheel volume, m^3, assume solid cylinder
    rho_rubber =  920;                              %Natural rubber density kg/m^3, https://designerdata.nl/materials/natural-rubber
    m_wheels = volume_wheel * rho_rubber;

    % Electronics
    m_electronics = m_motor * 0.5;

    % Calculate chassis thickness
    
    thick_new = 0.0001;     % Chassis starting thickness, m
    SF = 0;                     % Safety factor
    SF_tres = 1.2;              % Safety factor treshold
    G_load = 5.0;               % Vertical G load factor for chassis sizing
    rho_AL = 2700;              % Aluminum density, kg/m^3 https://www.matweb.com/search/datasheet.aspx?MatGUID=b8d536e0b9b54bd7b69e4124d8f1d20a&ckck=1
    yield_AL = 276*10^6;             % Aluminum yield stenght, MPa https://www.matweb.com/search/datasheet.aspx?MatGUID=b8d536e0b9b54bd7b69e4124d8f1d20a&ckck=1


    while SF < SF_tres  %safety factor
        
        thick_chassis = thick_new;

        m_chassis = wheelbase * track * thick_chassis * rho_AL;
        m_car = m_pack + m_motor + m_chassis + (4*m_wheels) + m_electronics;

        M_max = G_load*m_car*9.81*wheelbase/8;             % Max bending moment, Nm 
        V_max = G_load*m_car*9.81/2;                       % Max shear force, N

        I_chassis = track*(thick_chassis^3)/12;
        Bend_stress = M_max * (wheelbase/2) / I_chassis;
        SF_M = yield_AL / Bend_stress;

        Shear_stress = V_max / (track * thick_chassis);
        SF_V = yield_AL / Shear_stress;

        SF = min(SF_M, SF_V);

        thick_new = thick_chassis + 0.0001;

    end




%% Calculate Acceleration Time

if I_pack_max <= I_motor_max
    P_motor = V_pack_nom * I_pack_max;
else
    display('Current limited by motor');
    P_motor = V_pack_nom * I_motor_max;
end

T_m = P_motor / w_motor;

F_fric = T_m * GR / R_wheel; % Net friction propulsion force from car tire to ground, N

a_car = F_fric / m_car;     % Car acceleration m/s^2, assuming no slipping, no drag, no rolling resistance
t_vmax = v_car_req / a_car; % Acceleration time from zero to the requiremd max speed


%% Solar load

A_solar = wheelbase * track;        % Solar area, m^2
Irradiance = 500;                   % Absorbed solar irradiance, w/m^2
P_solar = A_solar * Irradiance;     % Solar power, W