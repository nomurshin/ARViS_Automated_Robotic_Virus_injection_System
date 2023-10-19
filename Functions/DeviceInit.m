% DeviceInit Loading MATLAB Driver for robotic device and initialization.
% The script begins by loading the PI_MATLAB_Driver_GCS2, which is a driver for the PI GCS controller.
% It establishes a connection to the controller through RS232 or TCP/IP based on the parameters set in the script.
% The connection settings such as COM port and baud rate for RS232 or IP and port for TCP/IP are defined in the script.
% After the connection is established, the script queries the controller identification.
% The axes to be used are defined (in this case, 'x' and 'y') and the stage is referenced.
% The script waits for the referencing process to finish.
%
% You should modify this script and "DeviceWrapper.m" when you use
% a different robotic device.

function C887 = DeviceInit(connection, comPort, baudRate, ip, port)

    %% Set default parameter values if not provided
    if nargin < 5, port = 50000; end
    if nargin < 4, ip = '192.168.0.100'; end  % Replace with your default IP
    if nargin < 3, baudRate = 115200; end
    if nargin < 2, comPort = 1; end
    if nargin < 1, connection = 'TCPIP'; end

    %% Determine connection type
    use_RS232_Connection = false;
    use_TCPIP_Connection = false;
    
    switch connection
        case 'RS232'
            use_RS232_Connection = true;
        case 'TCPIP'
            use_TCPIP_Connection = true;
        otherwise
            error('Invalid connection type. Please choose either "RS232" or "TCPIP".');
    end

    %% Loading the PI_MATLAB_Driver_GCS2 

    if     (strfind(evalc('ver'), 'Windows XP'))
        if (~exist('C:\Documents and Settings\All Users\PI\PI_MATLAB_Driver_GCS2','dir'))
            error('The PI_MATLAB_Driver_GCS2 was not found on your system. Probably it is not installed. Please run PI_MATLAB_Driver_GCS2_Setup.exe to install the driver.');
        else
            addpath('C:\Documents and Settings\All Users\PI\PI_MATLAB_Driver_GCS2');
        end
    elseif (strfind(evalc('ver'), 'Windows'))
        if (~exist('C:\Users\Public\PI\PI_MATLAB_Driver_GCS2','dir'))
            error('The PI_MATLAB_Driver_GCS2 was not found on your system. Probably it is not installed. Please run PI_MATLAB_Driver_GCS2_Setup.exe to install the driver.');
        else
            addpath('C:\Users\Public\PI\PI_MATLAB_Driver_GCS2');
        end
    end


    if(~exist('Controller','var'))
        Controller = PI_GCS_Controller();
    end
    if(~isa(Controller,'PI_GCS_Controller'))
        Controller = PI_GCS_Controller();
    end


    %% Connecting to the C887

    % Open connection
    boolC887connected = false;


    if (~boolC887connected)
        if (use_RS232_Connection)
            C887 = Controller.ConnectRS232(comPort, baudRate);
        end

        if (use_TCPIP_Connection)
            C887 = Controller.ConnectTCPIP(ip, port);
        end
    end

    if (exist('C887','var'))
        if (C887.IsConnected)
            boolC887connected = true;
        end
    end

    assert(boolC887connected, 'Connection to the device is not established.')
    %% Configuration and referencing
    % Query controller identification
    C887.qIDN()

    % Define axes
    axisname = 'x y';

    % Reference stage
    C887.FRF('x');

    % Wait for referencing to finish
    while(0 ~= C887.qFRF('x')==0)
        pause(0.1);
    end
end