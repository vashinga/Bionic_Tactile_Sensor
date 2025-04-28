clc;
clear all;

lib_name = '';

% Load Dynamixel libraries
if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_PRESENT_POSITION    = 132;

% Protocol version
PROTOCOL_VERSION            = 2.0;          

% Default setting
DXL_ID                      = 1;            % Dynamixel ID: 1
DXL_ID_2                    = 2;            % Second Dynamixel ID
BAUDRATE                    = 57600;
DEVICENAME                  = 'COM10';       % Check which port is being used on your controller

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;           % Dynamixel minimum position value
DXL_MAXIMUM_POSITION_VALUE  = 450;        % Dynamixel maximum position value

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

dxl_comm_result = COMM_TX_FAIL;           % Communication result

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    return;
end



% Enable Dynamixel Torque for both servos
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    return;
else
    fprintf('Dynamixels have been successfully connected \n');
end

%Enable Data Acquisition 
% deviceID = 'cDAQ1Mod2';  % cDAQ chassis device ID
deviceID = 'Dev1';
% Configure differential data collection (ai0 and ai1 in differential mode)
s = daq('ni');           % Create a session for National Instruments DAQ

% Create the input channels for differential mode
addinput(s, deviceID, 'ai0', 'Voltage');  % First input channel (e.g., FSR sensor)
% addinput(s, deviceID, 'ai1', 'Voltage');  % Second input channel (e.g., ground or reference)
% s.Channels(1).TerminalConfig = 'Differential';  % Set differential mode for ai0 and ai1
% s.Channels(2).TerminalConfig = 'Differential';  % Set differential mode for ai1 and ai0

s.Rate = 50000;    
% Initialize the servo target position
target_position = 250;
 
%Initialize the cutoff for sensor values
sensor_cutoff= 0.035; 
force_cutoff=4.5;

% Sensor reading plot
sensor_reading_list = [];
servo_target_list=[];
force_reading_list=[];

% Make Grasp with low force without feedback:
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(target_position), 'uint32'));
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);   

% Check communication result
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Write new target position to second Dynamixel servo
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, typecast(int32(10), 'uint32'));
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);

% Check communication result
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end    


while true

    % Read sensor reading
    sensor_data = read(s); 
    
    % disp(sensor_data.Properties.VariableNames);

    % Extract numerical data from the timetable
    % slip_reading = sensor_data.cDAQ1Mod2_ai0;
    slip_reading = sensor_data.Dev1_ai0;
    sensor_reading_list = [sensor_reading_list; slip_reading];
    servo_target_list = [servo_target_list; target_position];
    
 
    if slip_reading > sensor_cutoff
        target_position = target_position + 10; 
        % Ensure target position doesn't e xceed maximumx` position value
        if target_position > DXL_MAXIMUM_POSITION_VALUE
            target_position = DXL_MAXIMUM_POSITION_VALUE;
        end
    end    
    disp(target_position);
    
    % Write new target position to Dynamixel for both servos
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(target_position), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);

    
       % Check for servo overload or errors
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        
        % Check the error status of the servo
        error_code = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_ERROR);
        if error_code ~= 0
            fprintf('Error on Servo %d: %s\n', DXL_ID, getErrorDescription(error_code));
        end
        break;
    end
    % Write new target position to second Dynamixel servo
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, typecast(int32(15), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);

    % Check communication result
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        break;
    end
end

% Disable Dynamixel Torque for both servos
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
   fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end
                                    
closePort(port_num);
    
% Unload Library
unloadlibrary(lib_name);
