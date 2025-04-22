%{
*******************************************************************************
Project: graspio
File: xdf_plot.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Jan 28, 2025

Description:
Script for plotting data from xdf files.

License:
This script is licensed under the MIT License.
You may obtain a copy of the License at
    https://opensource.org/licenses/MIT

SPDX-License-Identifier: MIT

Disclaimer:
This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose, and noninfringement. In no event shall the
authors be liable for any claim, damages, or other liability, whether in an
action of contract, tort, or otherwise, arising from, out of, or in connection
with the software or the use or other dealings in the software.

*******************************************************************************
%}
clc; clear; %close all;

addpath('/home/hamid/xdf-Matlab/');  % Add the path to XDF library

% no_controller = "/home/hamid/Documents/CurrentStudy/sub-no_controller/ses-S003/eeg/sub-no_controller_ses-S003_task-Default_run-001_eeg.xdf";
% loaded_xdf_no_controller = load_xdf(no_controller);
% if loaded_xdf_no_controller{1}.info.name == "PHIDGET"
%     phidget_data_1 = loaded_xdf_no_controller{1}.time_series;
%     phidget_time_1 = loaded_xdf_no_controller{1}.time_stamps;
% end
% 
% with_controller = "/home/hamid/Documents/CurrentStudy/sub-with_controller/ses-S003/eeg/sub-with_controller_ses-S003_task-Default_run-001_eeg.xdf";
% loaded_xdf_with_controller = load_xdf(with_controller);
% if loaded_xdf_with_controller{2}.info.name == "BTS"
%     phidget_data_2 = loaded_xdf_with_controller{2}.time_series;
%     phidget_time_2 = loaded_xdf_with_controller{2}.time_stamps;
% end
% 
% with_controller_60 = "/home/hamid/Documents/CurrentStudy/sub-with_controller_60/ses-S003/eeg/sub-with_controller_60_ses-S003_task-Default_run-001_eeg.xdf";
% loaded_xdf_with_controller_60 = load_xdf(with_controller_60);
% if loaded_xdf_with_controller_60{2}.info.name == "BTS"
%     phidget_data_3 = loaded_xdf_with_controller_60{2}.time_series;
%     phidget_time_3 = loaded_xdf_with_controller_60{2}.time_stamps;
% end
% 
% with_controller_90 = "/home/hamid/Documents/CurrentStudy/sub-with_controller_90/ses-S003/eeg/sub-with_controller_90_ses-S003_task-Default_run-001_eeg.xdf";
% loaded_xdf_with_controller_90 = load_xdf(with_controller_90);
% if loaded_xdf_with_controller_90{2}.info.name == "BTS"
%     phidget_data_4 = loaded_xdf_with_controller_90{2}.time_series;
%     phidget_time_4 = loaded_xdf_with_controller_90{2}.time_stamps;
% end
% 
% figure;
% subplot(4,1,1)
% bts_smooth_1 = movmean(phidget_data_1(3,:), [15 0]);
% plot(phidget_time_1, bts_smooth_1);
% ylabel('bts no controller (V)', 'Rotation', 0);
% grid on;
% 
% subplot(4,1,2)
% bts_smooth_2 = movmean(phidget_data_2, [15 0]);
% plot(phidget_time_2, bts_smooth_2);
% ylabel('bts with controller low motor step-size (V)', 'Rotation', 0);
% grid on;
% 
% subplot(4,1,3)
% bts_smooth_3 = movmean(phidget_data_3, [15 0]);
% plot(phidget_time_3, bts_smooth_3);
% ylabel('bts with controller medium motor step-size (V)', 'Rotation', 0);
% grid on;
% 
% subplot(4,1,4)
% bts_smooth_4 = movmean(phidget_data_4, [15 0]);
% plot(phidget_time_4, bts_smooth_4);
% 
% xlabel('time (s)');
% ylabel('bts with controller long motor step-size (V)', 'Rotation', 0);
% grid on;

xdf_file_name = "remove_reading_thread.xdf";  % TODO
% xdf_file_name = "gelsight_small_step_feb25.xdf";  % TODO
% xdf_file_name = "gelsight_medium_step_feb25.xdf";  % TODO
% xdf_file_name = "sub-feb27_ses-S001_task-Default_run-001_eeg.xdf";  % TODO

xdf_file_path = "/home/hamid/projects/Graspio/data/data_for_paper/feb_28/debugging/";  % TODO

% xdf_file_name = "sub-P001_ses-S001_task-Default_run-001_eeg.xdf";  % TODO
% xdf_file_path = "/home/hamid/Documents/CurrentStudy/sub-P001/ses-S001/eeg/";  % TODO

xdf_file = xdf_file_path + xdf_file_name;
loaded_xdf = load_xdf(xdf_file);

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
        bts_time = loaded_xdf{i}.time_stamps;

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

    % elseif loaded_xdf{i}.info.name == "DYNAMIXELs"
    %     dynamixel_data = loaded_xdf{i}.time_series;
    %     dynamixel_time = loaded_xdf{i}.time_stamps;
    % 
    %     dxl_1_data = dynamixel_data(1:3,:);
    %     dxl_2_data = dynamixel_data(4:6,:);

    end

end


% csv_file_name = "vas_motor_2_no_controller.csv";  % TODO
% csv_file_dir = "/home/hamid/projects/Graspio/matlab/";  % TODO
% csv_file = csv_file_dir + csv_file_name;
% save_xdf_as_csv(dynamixel_time, dxl_2_data, csv_file, {'Time', 'pos', 'vel', 'load'});





% XL = 3420; XU =3500;
% 
F = sqrt(ur5e_wrench_data(1,:).^2 + ur5e_wrench_data(2,:).^2 + ur5e_wrench_data(3,:).^2);
subplot(3,1,1);
plot(ur5e_wrench_time, F);
ylabel("Mag ur5e force (N)");
% axis([XL, XU,4, 10])
% 
subplot(3,1,2);
bts_smooth = movmean(bts_data, [15 0]);  % Compute backward moving average
plot(bts_time,bts_data);
ylabel("BTS (v)");
% axis([XL, XU,2.4, 2.8])
% 
subplot(3,1,3);
plot(gelsight_score_time, gelsight_score_data);
ylabel("gelsight score");
% axis([XL, XU, 0, 13])
% 
% subplot(5,1,4);
% plot(dynamixel_time, dxl_2_data(1,:));
% ylabel("dynamixel2 pos (bit)");
% axis([XL, XU,160, 325])
% 
% subplot(5,1,5);
% plot(phidget_pot_1_time, phidget_pot_1_data);
% ylabel("VSA potentiometer1 (v)");
% axis([XL, XU,3.7, 3.76])
% 
% xlabel("time(s)")




ur5e_wrench_data = ur5e_wrench_data';
phidget_time = bts_time;

fData = sqrt(ur5e_wrench_data(:,1).^2+...
             ur5e_wrench_data(:,2).^2+...
             ur5e_wrench_data(:,3).^2);
bData = bts_data;
gData = gelsight_score_data;


fFlag = 0*fData;     gFlag = 0*gData;   bFlag = 0*bData;

%fFlag = [0 ; abs(diff(fData))>1.6];     fImpact = RmMultiObs(fFlag,60);
%gFlag = [0 ; abs(diff(gData))>0.5];     gImpact = RmMultiObs(gFlag,6);

[fV, fX] = findpeaks(fData,MinPeakProminence=1.8);         fFlag(fX) = 1;
[bV, bX] = findpeaks(bData,MinPeakProminence=.1);       bFlag(bX) = 1;                                                   
[gV, gX] = findpeaks(gData,MinPeakProminence=3);        gFlag(gX) = 1;



fImpact  = RmMultiObs(fFlag,50);
bImpact  = RmMultiObs(bFlag',50);
gImpact  = RmMultiObs(gFlag',1);




figure(2)
clf;
t = tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile;       hold on;
plot(ax1,ur5e_wrench_time, fData);
plot(ax1,ur5e_wrench_time, fImpact,'k');
set(gca, 'XTick', []);
% axis([XL, XU, -1, 6]) 
ylabel("Total Force (N)");


ax2 = nexttile;       hold on;          
plot(ax2,phidget_time, bData);
plot(ax2,phidget_time, bImpact*.2 + 2,'k');
set(gca, 'XTick', []);
% axis([XL, XU, 2., 3.])
ylabel("BTS Signal (v)");


ax2 = nexttile;       hold on;          
plot(ax2,gelsight_score_time, gData);      
plot(ax2,gelsight_score_time, gImpact,'k');

% axis([XL, XU, -5., 17.])
set(gca, 'XTick', []);
ylabel("Gelsight Score");



fImpactId = find(fImpact==1);
bImpactId = find(bImpact==1);   %bImpactId = bImpactId(end-5:end);
gImpactId = find(gImpact==1);   %gmpactId = gImpactId(end-5:end);

% bTD = abs(common_t(fImpactId)-common_t(bImpactId));
% gTD = abs(common_t(fImpactId)-common_t(gImpactId));

bImpactId = bImpactId(4:end);

ur5e_wrench_time(fImpactId)
phidget_time(bImpactId)
bTD = abs(ur5e_wrench_time(fImpactId)-phidget_time(bImpactId));
gTD = abs(ur5e_wrench_time(fImpactId)-gelsight_score_time(gImpactId));




figure(20)
plot(bTD,'bs');  hold on;    
plot(gTD,'ok')
ylabel("time difference(s)");
legend('Tf-Tb','Tf-Tg')

categoryGroup = ["BTS", "GELSIGHT"];

figure
boxplot([bTD',gTD']);




% x = (F-min(F))/(max(F)-min(F));
% plot(ur5e_wrench_time, x);
% hold on
% 
% F = movmean(bts_data, [15 0]);  % Compute backward moving average
% 
% x = (bts_data-min(bts_data))/(max(bts_data)-min(bts_data));
% plot(bts_time,x);
% 
% F=gelsight_score_data;
% x = (F-min(F))/(max(F)-min(F));
% plot(gelsight_score_time, x);
% % axis([XL, XU,0,1])
% 
% F=dxl_2_data(1,:);
% x = (F-min(F))/(max(F)-min(F));
% plot(dynamixel_time, x);
% 
% 
% 
% grid on
% legend("Mag ur5e force (N)","BTS (v)","gelsight score","dynamixel2 pos (bit)")







% plot_ur5e_wrench(ur5e_wrench_time, ur5e_wrench_data)
% plot_ur5e_tcp_pos(ur5e_end_effector_time, ur5e_end_effector_data(1:6,:))
% plot_ur5e_tcp_vel(ur5e_end_effector_time, ur5e_end_effector_data(7:12,:))
% plot_pot(phidget_pot_1_time, phidget_pot_1_data)
% plot_pot(phidget_pot_2_time, phidget_pot_2_data)
% plot_move_mean_bts(phidget_time, bts_data)
% plot_dynamixel(dynamixel_time, dxl_1_data)
% plot_dynamixel(dynamixel_time, dxl_2_data)
% plot_gelsight_score(gelsight_score_time, gelsight_score_data)








% Fx = ur5e_wrench_force_data(1,:)';
% Fy = ur5e_wrench_force_data(2,:)';
% Fz = ur5e_wrench_force_data(3,:)';
% F = sqrt(Fx.^2+Fy.^2+Fz.^2);
% 
% BTS_N = (bts_data-mean(bts_data))*20;
% 
% figure;
% plot(ur5e_wrench_time, F);
% % FTIME = (dT*ftime)';
% % plot(dT*ftime, F);
% hold on
% plot(phidget_time, BTS_N-5);
% % PTIME = (dT*ptime)';
% % PTIME_DATA = (BTS_N-5)';
% % plot(dT*ptime, BTS_N-5);
% hold on
% plot(gelsight_score_time, gelsight_score_data+10);
% % GTIME = (dT*gtime)';
% % GTIME_DATA = (gelsight_score_data+10)';
% % plot(dT*gtime, gelsight_score_data+10);
% legend('ur5e avg force', 'bts', 'gelsight score');
% title('controller via gelsight');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Function Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_move_mean_bts(bts_time, bts_data)

    figure;
    % plot(bts_time, bts_data);
    % xlabel('time (s)');
    % ylabel('Voltage (V)');
    
    bts_smooth = movmean(bts_data, [15 0]);  % Compute backward moving average
    plot(bts_time,bts_smooth,'r')
    xlabel('time (s)');
    ylabel('BTS Voltage (V)');
    grid on;

end


function plot_BTS_FFT(bts_time, bts_data)

    crop_time_from = 13;
    crop_time_to = 16;

    bts_time = bts_time - bts_time(1);
    idx_bts = find(bts_time > crop_time_from & bts_time < crop_time_to);
    cropped_timestamps = bts_time(idx_bts);
    cropped_data_bts = bts_data(idx_bts);

    figure;
    subplot(2,1,1)
    plot(cropped_timestamps, cropped_data_bts);
    xlabel('time (s)');
    ylabel('voltage (V)');
    grid on;

    Fs = 1 / mean(diff(cropped_timestamps));  % Estimate from time steps
    L = length(cropped_data_bts);
    Y = fft(cropped_data_bts);  % Perform FFT
    f = Fs * (0:(L/2)) / L;
    P2 = abs(Y / L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2 * P1(2:end-1);

    subplot(2,1,2)
    plot(f, P1);
    xlabel('freq (Hz)');
    ylabel('magnitude');
    grid on;
end


function plot_ur5e_wrench(ur5e_wrench_time, ur5e_wrench_data)

    figure;
    subplot(2,1,1)
    plot(ur5e_wrench_time, ur5e_wrench_data(1:3,:));
    xlabel('time (s)');
    ylabel('Force (N)');
    legend("Fx", "Fy", "Fz");
    grid on;

    subplot(2,1,2)
    plot(ur5e_wrench_time, ur5e_wrench_data(4:6,:));
    xlabel('time (s)');
    ylabel('Torque (Nm)');
    legend("Tx", "Ty", "Tz");
    grid on;

end


function plot_ur5e_tcp_pos(ur5e_tcp_time, ur5e_tcp_pos_data)

    figure;
    subplot(2,1,1)
    plot(ur5e_tcp_time, ur5e_tcp_pos_data(1:3,:));
    xlabel('time (s)');
    ylabel('pos (m)');
    legend("x", "y", "z");
    grid on;

    subplot(2,1,2)
    plot(ur5e_tcp_time, ur5e_tcp_pos_data(4:6,:));
    xlabel('time (s)');
    ylabel('orientation (rad)');
    legend("r_x", "r_y", "r_z");
    grid on;

end


function plot_ur5e_tcp_vel(ur5e_tcp_time, ur5e_tcp_vel_data)

    figure;
    subplot(2,1,1)
    plot(ur5e_tcp_time, ur5e_tcp_vel_data(1:3,:));
    xlabel('time (s)');
    ylabel('linear velocity (m/s)');
    legend("V_x", "V_y", "V_z");
    grid on;

    subplot(2,1,2)
    plot(ur5e_tcp_time, ur5e_tcp_vel_data(4:6,:));
    xlabel('time (s)');
    ylabel('angular velocity (rad/s)');
    legend("\omega_x", "\omega_y", "\omega_z");
    grid on;

end

function save_xdf_as_csv(timestamps, data, dir_and_file_name, header)

    data_table = array2table([timestamps', data'], 'VariableNames', header);
    writetable(data_table, dir_and_file_name);
    disp(['\nData saved to CSV: ', dir_and_file_name]);

end


function plot_dynamixel(dynamixel_time, dynamixel_data)

    figure;
    subplot(3, 1, 1);
    plot(dynamixel_time, dynamixel_data(1,:), 'r-');
    ylabel("position (rad)");
    grid on;

    subplot(3, 1, 2);
    plot(dynamixel_time, dynamixel_data(2,:), 'b-');
    ylabel("velocity (rad/s)");
    grid on;

    subplot(3, 1, 3);
    plot(dynamixel_time, dynamixel_data(3,:), 'g-');
    ylabel("load (Nm)");
    grid on;

    xlabel("time (s)")


end


function plot_pot(pot_time, pot_data)

    figure;
    plot(pot_time, pot_data);
    xlabel('time (s)');
    ylabel('potentiometer (V)');
    grid on;

end

function plot_gelsight_score(gelsight_score_time, gelsight_score_data)

    figure;
    plot(gelsight_score_time, gelsight_score_data);
    xlabel('time (s)');
    ylabel('gelsight score');
    grid on;

end




