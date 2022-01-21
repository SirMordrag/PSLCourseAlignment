%% The Initial Alignemnt based on RLG and accelerometer data
% Euler angles ref - phi = -0.018666, theta = 0.006606, psi = 29.790710
%clc;
clear all, close all;

load('SFin.mat'); SFin = SFin';         % 
load('Win.mat'); Win = Win';
%% Definition of parameters
Par.LATini = deg2rad(51.918465558);     % Latitude of where the data was measured.
Par.w_ie = 7.2921150e-5;                % Earth rate (in rad)
Par.Fs = 2000;                          % the update rate for the IMU data - 2kHz
Par.INITtime = 120;                     % time length of the CA in seconds
%======== UPDATE transformation matrices sensor frame into ENU and ENU2NED
Par.TRs2enu_SF = [];                    % update - converting ACC sensor frame into ENU Body frame
Par.TRs2enu_W = [];                     % update - converting RLG sensor frame into ENU Body frame
Par.TRenu2ned = [];                     % update - converting ENU 2 NED composition of the BODY frame
%----------------------------------------------------------------------------------------------------
% Angular rates/gyroscopes parameters
RawW.units = 'rad/s';
RawW.TRcrossW = [   +0.999998364250954, -0.000068040338606, -0.002309844796597;... % cross-coupling matrix
                    +0.001392401143206, +0.999995101149402, +0.000208905927290;...
                    -0.000301460423864, -0.002057280652560, +0.999997874870133];   % in ENU frame 
RawW.bias = deg2rad([+0.000000412942361; -0.000001185051321; -0.000000255611192]); % SF bias in ENU frame              
% Specific force/accelerometers parameters                
RawSF.units = 'm/s2';             
RawSF.TRcrossSF = [ +1.000015753716061, -0.000074996330332, -0.001769749784857; ... % cross-coupling matrix
                    -0.001345395269523, +0.999993493839768, +0.000103659813067;...
                    +0.001794379465053, +0.000123990276602, +1.000006652591802];    % in ENU
RawSF.bias = [-0.000155131315863; +0.000222021609289; +0.000074785072052];          % RLG bias in ENU    
%% Computation 
%======== UPDATE Earth related parameters 
WieN = ;                                                    % update - Earth rate vector in NED frame
gLOCAL = comp_gravity(Par.LATini, 0);                       % in m/s2 - it is recalculated according to the location of interest 
gN = ;                                                      % local gravity vector
%-----------------------------------------------------------------------------------------------------
%% Course Alinment calculations
%------------ mean values in the INIT time length
N = Par.Fs*Par.INITtime;        % CA length in No. of samples
%======== UPDATE the mean values and std within the INIT time slot
gMin = ; SFstd = ;
wMin = ; Wstd = ;
%======== UPDATE Compensation of sensor erros and transform resultant values into NED
wM = ;              % update - compensated wMin       
gM = ;              % update - compensated gMin; 
%----------------------------------------------------------------------------------------------------
%% ======================================================
% =============== Course alignment =====================
% ======================================================
%======== UPDATE
.......             % update - the whole procedure should be performed here
EAall = ;           % update - evaluate the Euler angles
%----------------------------------------------------------------------------------------------------
%% ======================================================
%======== UPDATE the results confirmation
% PHI and THETA from accelerometers
PHIacc = ;          % update - just for comperison
THacc = ;           % update - just for comperison
EAacc = ([PHIacc,THacc])*180/pi;    % conversion into [deg]
%**************************************************************************
Wstd = Wstd*180/pi; wM = wM*180/pi;
fprintf('Mean of Wraw is [%f;%f;%f] deg/s, 1-sigma is [%f;%f;%f] deg/s\n',wM(1,1),wM(2,1),wM(3,1),Wstd(1,1),Wstd(2,1),Wstd(3,1));
fprintf('Mean of SFraw is [%f;%f;%f] m/s2, 1-sigma is [%f;%f;%f] m/s2\n',-gM(1,1),-gM(2,1),-gM(3,1),SFstd(1,1),SFstd(2,1),SFstd(3,1));
fprintf('\nEstimated Euler angles within the INIT time slot is (deg):\n -- ROLL:%f, PITCH:%f, YAW:%f\n',EAall(1),EAall(2),EAall(3));
fprintf('\nEuler angles taken from the ACC data is (deg):\n -- ROLL:%f, PITCH:%f\n',EAacc(1),EAacc(2));





