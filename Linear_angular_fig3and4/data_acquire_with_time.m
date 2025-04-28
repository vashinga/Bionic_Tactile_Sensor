%Modified with sensor Data acq with defined Hz

experiment_number = 20;


% Initialize the data acquisition session
s = daq('ni');
addinput(s, 'Dev1', 'ai0', 'Voltage')
Fs = 5000;
s.Rate = Fs;

current_time = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
sensor_data = read(s,seconds(7));
sensor_data_vec = abs(sensor_data.Dev1_ai0);
time_vec_sensor = current_time + (sensor_data.Time);

% Plot the acquired data
figure;
plot(time_vec_sensor, sensor_data_vec, '-o');
xlabel('Time (s)');
ylabel('Sensor reading');
title('Sensor reading vs Time');
grid on;

% Convert datetime  desired format
time_strings = time_vec_sensor;

% Create a table from the data
sensor_data_table = table(time_strings, sensor_data_vec, 'VariableNames', {'Time', 'SensorReading'});

% Write the table to an Excel file
% writetable(sensor_data_table, 'sensor_data_19.xlsx');

% Create the filename with multiple numbers
filename = sprintf('sensor_data_exp%d_Sep6.xlsx', experiment_number);

% Save the table to an Excel file
writetable(sensor_data_table, filename);