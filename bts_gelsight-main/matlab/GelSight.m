clc;      clear;    
close all;

addpath('/home/hamid/xdf-Matlab/');

% xdf_file_name = "final_exp_feb28.xdf";  % TODO
xdf_file_name = "004.xdf";
xdf_file_path = "/home/hamid/Documents/CurrentStudy/sub-march1/ses-S001/eeg/exp_march_1/";  % TODO
xdf_file = xdf_file_path + xdf_file_name;
[loaded_xdf, header] = load_xdf(xdf_file);

% Extract of data streams:
for i = 1:length(loaded_xdf)
    fprintf('Stream %d, %s: %s %s\n', i, loaded_xdf{i}.info.name, ...
        loaded_xdf{i}.info.channel_count, loaded_xdf{i}.info.sample_count);

    if loaded_xdf{i}.info.name == "PHIDGET"
        phidget_data = loaded_xdf{i}.time_series;
        phidget_time = loaded_xdf{i}.time_stamps;

        pot_1_data = phidget_data(1,:);
        pot_2_data = phidget_data(2,:);
        bts_data = phidget_data(3,:);

    elseif loaded_xdf{i}.info.name == "PHIDGET_POT1"
        phidget_pot_1_data = loaded_xdf{i}.time_series;
        phidget_pot_1_time = loaded_xdf{i}.time_stamps;

    elseif loaded_xdf{i}.info.name == "PHIDGET_POT2"
        phidget_pot_2_data = loaded_xdf{i}.time_series;
        phidget_pot_2_time = loaded_xdf{i}.time_stamps;
    
    elseif loaded_xdf{i}.info.name == "PHIDGET_BTS"
        bts_data = loaded_xdf{i}.time_series;
        phidget_time = loaded_xdf{i}.time_stamps;

    elseif loaded_xdf{i}.info.name == "GELSIGHT_SCORE"
        gelsight_score_data = loaded_xdf{i}.time_series;
        gelsight_score_time = loaded_xdf{i}.time_stamps;

    elseif loaded_xdf{i}.info.name == "UR5e_EndEffector"
        ur5e_end_effector_data = loaded_xdf{i}.time_series;
        ur5e_end_effector_time = loaded_xdf{i}.time_stamps;

        ur5e_tcp_linear_pos = ur5e_end_effector_data(1:3,:);
        ur5e_tcp_angular_pos = ur5e_end_effector_data(4:6,:);
        ur5e_tcp_linear_speed = ur5e_end_effector_data(7:9,:);
        ur5e_tcp_angular_speed = ur5e_end_effector_data(10:12,:);

    elseif loaded_xdf{i}.info.name == "GelSightMini"
        gelsight_data = loaded_xdf{i}.time_series;
        gelsight_time = loaded_xdf{i}.time_stamps;

    elseif loaded_xdf{i}.info.name == "UR5e_Wrench"
        ur5e_wrench_data = loaded_xdf{i}.time_series;
        ur5e_wrench_time = loaded_xdf{i}.time_stamps;

        ur5e_wrench_force_data = ur5e_wrench_data(1:3,:);
        ur5e_wrench_torque_data = ur5e_wrench_data(4:6,:);

    elseif loaded_xdf{i}.info.name == "DYNAMIXELs"
        dynamixel_data = loaded_xdf{i}.time_series;
        dynamixel_time = loaded_xdf{i}.time_stamps;

        % dxl_1_data = dynamixel_data(1:3,:);
        % dxl_2_data = dynamixel_data(4:6,:);

    end

end

fData = sqrt(ur5e_wrench_data(1,:).^2+...
             ur5e_wrench_data(2,:).^2+...
             ur5e_wrench_data(3,:).^2);
fData = fData';
bData = bts_data';
gData = gelsight_score_data';

fFlag = 0*fData;     gFlag = 0*gData;     bFlag = 0*bData;

[fV, fX] = findpeaks(fData,MinPeakProminence=1.5);      fFlag(fX) = 1;
% [bV, bX] = findpeaks(bData,MinPeakProminence=.09);       bFlag(bX) = 1;                                                   
[gV, gX] = findpeaks(gData,MinPeakProminence=1);        gFlag(gX) = 1;


[bV, bX] = find(bData>2.65);       bFlag(bV) = 1; 


fImpact  = RmMultiObs(fFlag,100);
bImpact  = RmMultiObs(bFlag,10);
gImpact  = RmMultiObs(gFlag,1);



% fImpact(1:2030) = 0;
% bImpact(1:400) = 0;
% gImpact(1:200) = 0;

% idx = find(bImpact == 1, 1);  % Find the index of the first 1
% bImpact(idx) = 0;

% contact force calculation

% fc = [0.61,0.67247,0.69778,0.73803,0.75925,0.81181,0.84390,0.88626,0.93131,0.98921,1.01983,1.01983];

% dynamixel_time = ur5e_wrench_time; 

fcp = gelsight_score_data;


xlimval = [min(ur5e_wrench_time) max(ur5e_wrench_time)];

XL = 20; 
XU = 80;


figure(1);
clf;
t = tiledlayout(4,1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile;       hold on;
plot(ur5e_wrench_time, fData,'LineWidth',2);
plot(ax1,ur5e_wrench_time(fImpact==1), fData(fImpact==1),'r*','LineWidth',2);
plot(ax1,ur5e_wrench_time, fImpact+1,'k','LineWidth',2);
set(gca, 'XTick', []);
xlim(xlimval);
% axis([XL, XU, -1, 6]) 
ylabel("Total Force (N)");


ax2 = nexttile;       hold on;          
bts_smooth = movmean(bts_data, [15 0]);  % Compute backward moving average
plot(phidget_time,bts_data,'LineWidth',2);      
plot(ax2,phidget_time(bImpact==1), bts_data(bImpact==1),'r*','LineWidth',2);
% plot(ax2,phidget_time, bImpact*0.2+2,'k','LineWidth',2);
% axis([XL, XU, -5., 17.])
xlim(xlimval);
set(gca, 'XTick', []);
ylabel("BTS (V)");


ax3 = nexttile;       hold on;          
plot(gelsight_score_time, gelsight_score_data,'LineWidth',2);
plot(ax3,gelsight_score_time(gImpact==1), gelsight_score_data(gImpact==1),'r*','LineWidth',2);
plot(ax3,gelsight_score_time, gImpact-2,'k','LineWidth',2);
set(gca, 'XTick', []);
% axis([XL, XU, 2., 3.])
xlim(xlimval);
ylabel("Gelsight Score");


% fc = ones(1,length(gelsight_score_time));
% dynamixel_time = gelsight_score_time;
% index = 340;
% fc(1:index) = 0;
% fc(index:end) = 1.9772;

ax4 = nexttile;       hold on;          
% plot(ax4,dynamixel_time, fc,'-','LineWidth',2);

plot(dynamixel_time,dynamixel_data,'-*');
% plot(ax4,dynamixel_time, fc,'r*','LineWidth',2);
% axis([XL, XU, -5., 17.])
xlim(xlimval);
set(gca, 'XTick', []);
ylabel("Gripper Force (N)");
xlabel("Time (s)");





fImpactId = find(fImpact==1);
bImpactId = find(bImpact==1);
gImpactId = find(gImpact==1);


% bts_time(bImpactId(1));
% gelsight_score_time(gImpactId(1));
% 
% gelsight_score_time(gImpactId(1)) - bts_time(bImpactId(1));
% 
% 
% bImpactId(1) =[];
% gImpactId(1) =[];6

% fImpactId(end)=[];
% gImpactId(end)=[];

% length(ur5e_wrench_time(fImpactId))
% length(bts_time(bImpactId))
% length(gelsight_score_time(gImpactId))
% 
% bTD = abs(ur5e_wrench_time(fImpactId)-bts_time(bImpactId));
% gTD = abs(ur5e_wrench_time(fImpactId)-gelsight_score_time(gImpactId));
% 
% 
% figure
% plot(bTD,"bo",LineWidth=2);hold on;
% plot(gTD,"ro",LineWidth=2);hold on;

% subploting(common_t, fData, cleaned_Fe, m1Data, m2Data, p1Data, airgap_1, airgap_2);

csv_filename = "final_exp_no_controller_feb28.csv";  % filename
% csv_file_dir = "/home/hamid/projects/Graspio/data/data_for_paper/feb_25/for_Ehsan/csv_files/";  % directory
% csv_file = fullfile(csv_file_dir, csv_file_name);  % combine into full path

header = {'ur5e_wrench_time', 'ur5e_wrench_data', 'fImpact', 'bts_time', 'bts_data', 'bImpact', 'gelsight_score_time', 'gelsight_score_data', 'gImpact', 'gripper_force_time', 'gripper_force'};  % TODO
% data = [ur5e_wrench_time(:), fData(:), fImpact(:), bts_time(:), bData(:), bImpact(:), gelsight_score_time(:), gelsight_score_data(:), gImpact(:), dynamixel_time(:), fc(:)];

% Ensure each variable is a column vector
ur5e_wrench_time = ur5e_wrench_time(:);
fData               = fData(:);
fImpact             = fImpact(:);

bts_time            = phidget_time(:);
bData               = bData(:);
bImpact             = bImpact(:);

gelsight_score_time = gelsight_score_time(:);
gelsight_score_data = gelsight_score_data(:);
gImpact             = gImpact(:);

dynamixel_time      = dynamixel_time(:);
fc                  = fc(:);

% Determine the maximum length among all vectors
maxLength = max([numel(ur5e_wrench_time), numel(fData), numel(fImpact), ...
                 numel(bts_time), numel(bData), numel(bImpact), ...
                 numel(gelsight_score_time), numel(gelsight_score_data), numel(gImpact), ...
                 numel(dynamixel_time), numel(fc)]);

% Helper function to pad a vector with NaNs up to maxLength
padToMax = @(x, n) [x; nan(n - numel(x), 1)];

ur5e_wrench_time = padToMax(ur5e_wrench_time, maxLength);
fData            = padToMax(fData, maxLength);
fImpact          = padToMax(fImpact, maxLength);

bts_time         = padToMax(bts_time, maxLength);
bData            = padToMax(bData, maxLength);
bImpact          = padToMax(bImpact, maxLength);

gelsight_score_time = padToMax(gelsight_score_time, maxLength);
gelsight_score_data = padToMax(gelsight_score_data, maxLength);
gImpact          = padToMax(gImpact, maxLength);

dynamixel_time   = padToMax(dynamixel_time, maxLength);
fc               = padToMax(fc, maxLength);

% Create a table with custom header names
data_table = table(ur5e_wrench_time, fData, fImpact, ...
                   bts_time, bData, bImpact, ...
                   gelsight_score_time, gelsight_score_data, gImpact, ...
                   dynamixel_time, fc, ...
                   'VariableNames', {'UR5eWrenchTime','FData','FImpact', ...
                                     'BTSTime','BData','BImpact', ...
                                     'GelsightScoreTime','GelsightScoreData','GImpact', ...
                                     'DynamixelTime','FC'});

% Specify the CSV file name (adjust the path if needed)
% csv_filename = 'output_data.csv';

% Write the table to a CSV file
writetable(data_table, csv_filename);

disp(['Data saved to ', csv_filename]);


% save_xdf_as_csv_1(common_t(:), data, csv_file, header);

% Function definition
function save_xdf_as_csv_1(data, dir_and_file_name, header)
    % Make sure the timestamps vector is a column vector.
    % Ensure that each column in data is a column vector.
    % (If data is already an N-by-M matrix, each column is automatically a column vector.)
    
    % Combine timestamps and data into one matrix.
    % This creates a matrix where the first column is the timestamps.
    all_data = [data];
    
    % Check that the number of headers matches the number of columns.
    if length(header) ~= size(all_data, 2)
        error('Header length (%d) does not match number of columns (%d).', length(header), size(all_data, 2));
    end
    
    % Convert the matrix to a table with appropriate header names.
    data_table = array2table(all_data, 'VariableNames', header);
    
    % Write the table to a CSV file.
    writetable(data_table, dir_and_file_name);
    fprintf('\nData saved to CSV: %s\n', dir_and_file_name);
end






function save_xdf_as_csv_2(timestamps, data, dir_and_file_name, header)

    data_table = array2table([timestamps', data'], 'VariableNames', header);
    writetable(data_table, dir_and_file_name);
    disp(['\nData saved to CSV: ', dir_and_file_name]);

end


function Fe = measure_vsa_force(pot_1_voltage, dxl_1_pos_bit, dxl_2_pos_bit)

    dxl_resolution = 0.065;
    pot_resolution = 19;
    P1 = 5.5e-6;
    P2 = 6.85e-5;
    P3 = 2.22e-7;

    % inputs are column vectors
    pot_1_voltage = pot_1_voltage(:);
    dxl_1_pos_bit = dxl_1_pos_bit(:);
    dxl_2_pos_bit = dxl_2_pos_bit(:);

    dxl_1_dist = 206.3 - 45.75 - (1024 - dxl_1_pos_bit) .* dxl_resolution;
    dxl_2_dist = 38.5 + dxl_2_pos_bit .* dxl_resolution;

    finger_1_dist = 47.4 + (5 - pot_1_voltage) .* pot_resolution;

    AG_1 = dxl_1_dist - finger_1_dist - 15/2;
    AG_2 = finger_1_dist - dxl_2_dist - 15/2;

    AG_1 = AG_1 / 1000;  % mm to m
    AG_2 = AG_2 / 1000;  % mm to m

    

    Fe = P1 * (1 ./ (AG_2.^3 + P2.*AG_2 + P3) - 1 ./ (AG_1.^3 + P2.*AG_1 + P3));
    Fe = Fe - Fe(1);
end


function subploting(time, ur5e_force, applied_force, dxl_1, dxl_2, pot_1, airgap_1, airgap_2)

    figure;
    subplot(7, 1, 1);
    plot(time, ur5e_force);
    ylabel("ur5e total force");

    subplot(7, 1, 2);
    plot(time, applied_force);
    ylabel("VSA force");

    subplot(7, 1, 3);
    plot(time, dxl_1);
    ylabel("dynamixel_1");

    subplot(7, 1, 4);
    plot(time, dxl_2);
    ylabel("dynamixel_2");

    subplot(7, 1, 5);
    plot(time, pot_1);
    ylabel("pot_1");

    subplot(7, 1, 6);
    plot(time, airgap_1);
    ylabel("airgap_2");

    subplot(7, 1, 7);
    plot(time, airgap_2);
    ylabel("airgap_2");

end


