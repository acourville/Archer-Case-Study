clear;
clc;

%% Sim Parameters
simtime = 86400;

simname = 'Archer_battery_sim';

%% Import Data
battery_spec = readmatrix('Archer Battery Options.csv');
P28A_spec = readmatrix('P28A Cell.csv');
P30B_spec = readmatrix('P30B Cell.csv');

SOC_max = 0.85;
SOC_min = 0.0;


%% RC Car Specs
Size = 1/16;    % RC Car Scale
wheelbase_supra = 2.74;         % Full size car wheelbase, m https://toyotagazooracing.com/gr/supra/specs/
wheelbase = wheelbase_supra * Size; % RC car wheelbase
track = wheelbase / 1.55; % RC car track, m

A_solar = wheelbase * track;        % Solar area, m^2
P_solar = A_solar * 500;            % Solar power, w


%% Charging/Discharging Parameters
Kp   = 100; % Proportional gain CV controller
Ki   = 10;  % Integral gain CV controller
Kaw  = 1;   % Antiwindup gain CV controller
Ts   = 1;   % Sample time (s)

for option = 1
vMax = 4.2 * battery_spec(option, 6); % Maximum cell voltage

    % Setup cell parameters
    if battery_spec(option, 8) == 10.8
        display('P30B Cell')
        
        AH = battery_spec(option, 8) * battery_spec(option, 4) * battery_spec(option, 6);
        SOC_vec = flip(transpose(P30B_spec(:,2)));
        OCV_vec = flip(transpose(P30B_spec(:,3)) .* battery_spec(option, 6));
        R0 = flip(transpose(P30B_spec(:,4)));
        R_tot = (battery_spec(option, 6)/battery_spec(option, 4)) .*  R0;
    
    
    else
        display('P28A Cell')
    
        AH = battery_spec(option, 8) * battery_spec(option, 4) * battery_spec(option, 6);
        SOC_vec = flip(transpose(P28A_spec(:,2)));
        OCV_vec = flip(transpose(P28A_spec(:,3)) .* battery_spec(option, 6));
        R0 = flip(transpose(P28A_spec(:,4)));
        R_tot = (battery_spec(option, 6)/battery_spec(option, 4)) .*  R0;
    end

    if battery_spec(option,3) <= battery_spec(option,2)
        I_100 =  battery_spec(option,3);
    else
        display('Current limited by motor');
        I_100 = battery_spec(option,2);
    end

    I_50 = I_100 * 0.5;
    I_0 = 0;

    t_cycle = 600;  %Cycle duration, seconds
    t_100 = 0.5 * t_cycle;
    t_50 = 0.3 * t_cycle;
    t_0 =  0.2 * t_cycle;
        
    out = sim(simname);
    disp('Sim Complete')

end