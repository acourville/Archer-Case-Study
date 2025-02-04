clear;
clc;

%% Import Data
battery_spec = readmatrix('Archer Battery Options.csv');
P28A_spec = readmatrix('P28A Cell.csv');
P30B_spec = readmatrix('P30B Cell.csv');

SOC_0 = 0.85;

for option = 1:1:10

    % Setup cell parameters
    if battery_spec(option, 8) == 10.8
        display('P30B Cell')
        
        AH = battery_spec(option, 8) * battery_spec(option, 4) * battery_spec(option, 6);
        SOC_vec = transpose(P30B_spec(:,2));
        OCV_vec = transpose(P30B_spec(:,3)) .* battery_spec(option, 6);
        R0 = transpose(P30B_spec(:,4));
        R_tot = (battery_spec(option, 6)/battery_spec(option, 4)) .*  R0;
    
    
    else
        display('P28A Cell')
    
        AH = battery_spec(option, 8) * battery_spec(option, 4) * battery_spec(option, 6);
        SOC_vec = transpose(P28A_spec(:,2));
        OCV_vec = transpose(P28A_spec(:,3)) .* battery_spec(option, 6);
        R0 = transpose(P28A_spec(:,4));
        R_tot = (battery_spec(option, 6)/battery_spec(option, 4)) .*  R0;
    end

    I_100 = 
    
    run('Archer_battery_sim');


end