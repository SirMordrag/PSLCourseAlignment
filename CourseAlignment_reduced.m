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

% transformation matrices
Cgyro_sensor2ENU = [1 0 0; 0 1 0; 0 0 1];
Cacc_sensor2ENU = [1 0 0; 0 -1 0; 0 0 -1];
C_enu2ned = [0 1 0; 1 0 0; 0 0 -1];
C_ned2enu = C_enu2ned;
% transformation matrices [END]

Par.TRs2enu_SF = Cacc_sensor2ENU;                    % update - converting ACC sensor frame into ENU Body frame
Par.TRs2enu_W = Cgyro_sensor2ENU;                    % update - converting RLG sensor frame into ENU Body frame
Par.TRenu2ned = C_enu2ned;                           % update - converting ENU 2 NED composition of the BODY frame
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

% compute earth rate vector in NED frame
% earth rate vector in ECEF
lat = Par.LATini;
lon = 0;
earth_rate_ENU = [0; Par.w_ie*cos(lat); Par.w_ie*sin(lat)];
% ECEF to NED using DCM
DCM = [-sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat); -sin(lon) cos(lon) 0; -cos(lat)*sin(lon) -cos(lat)*sin(lon) -sin(lat)];

% transform earth rate from ENU to NED
WieN =  Par.TRenu2ned * earth_rate_ENU;                    % update - Earth rate vector in NED frame
gLOCAL = comp_gravity(Par.LATini, 0);                      % in m/s2 - it is recalculated according to the location of interest 
gN = [0 0 gLOCAL];                                         % local gravity vector in NED
% compute earth rate vector in NED frame [END]

%-----------------------------------------------------------------------------------------------------
%% Course Alinment calculations
%------------ mean values in the INIT time length
N = Par.Fs*Par.INITtime;        % CA length in No. of samples
%======== UPDATE the mean values and std within the INIT time slot
gMin = mean(SFin(1:N,:));
SFstd = std(SFin(1:N,:));
wMin = mean(Win(1:N,:));
Wstd = std(Win(1:N,:));
%======== UPDATE Compensation of sensor erros and transform resultant values into NED
wM = Par.TRenu2ned * RawW.TRcrossW*(Par.TRs2enu_W*(wMin') - RawW.bias);            % update - compensated wMin       
gM = Par.TRenu2ned * RawSF.TRcrossSF*(Par.TRs2enu_SF*(gMin') - RawSF.bias);        % update - compensated gMin; 
%----------------------------------------------------------------------------------------------------
%% ======================================================
% =============== Course alignment =====================
% ======================================================
%======== UPDATE

% nas intuitivni pristup:
calculated_east = cross(wM, gM);
actual_east = cross(WieN, gN)';

% Rodriguez rotation, offset by 90 to get north-centric heading, not east-centric heading
EAall = rodriguez_rot_to_eul(calculated_east, actual_east) - [0 0 90]

% z paperu:

gN = [0 0 gLOCAL];                                       
gN = -gN';

% s1
C1_1 = [gN'; WieN'; cross(gN,WieN)'];
C1_2 = [gM'; wM'; cross(gM, wM)'];
C1 = C1_1\C1_2;

% s2
C2_1 = [gN'; cross(gN,WieN)'; cross(cross(gN,WieN),gN)'];
C2_2 = [gM'; cross(gM,wM)'; cross(cross(gM,wM),gM)'];
C2 = C2_1\C2_2;

EAall1 = rad2deg(rotm2eul(C1));
EAall2 = rad2deg(rotm2eul(C2));

EAll_need_to_swap = (EAall1 + EAall2)./(2) - [90 0 0];
EAall = zeros(1,3);
EAall(1) = EAll_need_to_swap(2);
EAall(2) = EAll_need_to_swap(3);
EAall(3) = EAll_need_to_swap(1);

%----------------------------------------------------------------------------------------------------
%% ======================================================
%======== UPDATE the results confirmation
% PHI and THETA from accelerometers
PHIacc = atan2(gM(1), gM(3)) + pi;          % update - just for comparison
THacc = atan2(gM(2), gM(3)) + pi;           % update - just for comparison
EAacc = ([PHIacc,THacc])*180/pi;            % conversion into [deg]
EAacc(1) = -1 * EAacc(1);
%**************************************************************************
Wstd = Wstd*180/pi; wM = wM*180/pi;
fprintf('Mean of Wraw is [%f;%f;%f] deg/s, 1-sigma is [%f;%f;%f] deg/s\n',wM(1),wM(2),wM(3),Wstd(1),Wstd(2),Wstd(3));
fprintf('Mean of SFraw is [%f;%f;%f] m/s2, 1-sigma is [%f;%f;%f] m/s2\n',-gM(1),-gM(2),-gM(3),SFstd(1),SFstd(2),SFstd(3));
fprintf('\nEstimated Euler angles within the INIT time slot is (deg):\n -- ROLL:%f, PITCH:%f, YAW:%f\n',EAall(1),EAall(2),EAall(3));
fprintf('\nEuler angles taken from the ACC data is (deg):\n -- ROLL:%f, PITCH:%f\n',EAacc(1),EAacc(2));





