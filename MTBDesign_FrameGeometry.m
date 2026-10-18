%% MTB DESIGN - FRAME GEOMETRY --------------------------------------------
% Nomenclature:
% 
% 
% Purpose: 
% Determines critical frame and suspension points on frame,
% allowing the sourcing of loads and load application points. Constrains
% the packaging and geometry of MTB assembly -- a critical stepping stone
% to CAD and Bicycle Modeling.
% -------------------------------------------------------------------------


%% Houskeeping
% Clean up workspace
clc; clear; close all;


%% Known Variables
% Suspension
dF = 160; % front travel [mm]
dR = 145; % rear travel [mm]
A2C = 571; % axle-to-crown distance [mm]
d = 44; % fork offset [mm]

% Wheels
rimRad = 622 / 2; % wheel rim radius [mm]
hTF = 63 - 6; % front loaded tire height [mm] (estimated)
hTR = 61 - 6; % rear loaded tire height [mm] (estimated)
RF = rimRad + hTF; % front loaded wheel asy radius [mm]
RR = rimRad + hTR; % rear loaded wheel asy radius [mm]


%% Design Variables - Sweeped
theta = 65; % head angle [deg]
bd = 32; % BB (bottom bracket) drop [mm]
Lcs = 440; % chainstay length (straight line dist from BB to rear axle) [mm]
Re = 470; % reach (horizontal distance from BB to top of headtop) [mm]
Lht = 110; % head tube length [mm]
sL = 10; % lowest headset stack height from bottom of head tube to crown [mm]
sigmaF = 0.2; % fork sag fraction
sigmaR = 0.3; % rear sag fraction


%% Static Geometry (origin at BB, x forward, y up)
% Unit Vectors
u = [cosd(theta); -sind(theta)];
n = [sind(theta); cosd(theta)];

% Rear Axle and BB Height
hBB = RR - bd; % BB height
xR = -sqrt(Lcs^2 - bd^2); % rear axle horizontal position relative to BB
yR = bd; % rear axle vertical position relative to BB
RPos = [xR; yR];

% Front Axle
Lf = A2C + Lht + sL; % total length along steer axis from Htop to axle [mm]
S = bd + (RF - RR) + Lf * sind(theta) - d * cosd(theta); % stack vertical distance from BB to Htop [mm]
Htop = [Re; S]; % top of headtop location [mm]
Hbot = Htop + Lht * u; % bottom of headtop location [mm]
C = Hbot + sL * u; % bottom of crown location (where A2C is measured from) [mm]
FPos = A2C * u + C + d * n; % front axle center location [mm]

% Wheelbase
w = FPos(1) - RPos(1); % wheelbase [mm]

% Mechanical Trail and Ground Trail
tm = RF * cosd(theta) - d; % mechanical trail
c = tm / sind(theta); % ground trail


%% Geometry at Sag (Frame will "rotate" by angle beta)
% Sagging Position
FPosSag = FPos - dF * sigmaF * u; % front axle center location at sag [mm]
RPosSag = RPos + [0; dR * sigmaR]; % rear axle center location at sag (assumed vertical motion) [mm] 

% Angles
alpha = atan2d(FPos(2) - RPos(2), FPos(1) - RPos(1)); % static axle angle [deg]
alphaSag = atan2d(FPosSag(2) - RPosSag(2), FPosSag(1) - RPosSag(1)); % static axle angle at sag [deg]
beta = alphaSag - alpha; % frame pitch rotation at sag [deg]
thetaSag = theta + beta; % head angle at sag [deg]

% Rotation
rot = [cosd(-beta) -sind(-beta); sind(-beta) cosd(-beta)]; % rotate coordinates so the frame is "level" with the ground
FPosRot = rot * FPosSag;
RPosRot = rot * RPosSag;
hBBSag = RR - RPosRot(2);
wSag = FPosRot(1) - RPosRot(1);


%% Printing Results
% 2D Chassis Skeleton
pts = {'BB center',          [0; 0];
       'Rear axle',          RPos;
       'Front axle',         FPos;
       'Head tube top',      Htop;
       'Head tube bottom',   Hbot;
       'Fork crown bottom',  C;
       'Rear contact patch', [RPos(1); -hBB];
       'Front contact patch',[FPos(1); -hBB];
       'Steer axis @ ground',[FPos(1) + c; -hBB]};

fprintf('\n%-22s %10s %10s\n', 'Point', 'x [mm]', 'y [mm]');
fprintf('%s\n', repmat('-', 1, 44));
for k = 1:size(pts, 1)
    fprintf('%-22s %10.1f %10.1f\n', pts{k,1}, pts{k,2}(1), pts{k,2}(2));
end

fprintf('\n%-22s %10s %10s\n', 'Parameter', 'Static', 'Sag');
fprintf('%s\n', repmat('-', 1, 44));
fprintf('%-22s %10.2f %10.2f\n', 'Head angle [deg]',  theta, thetaSag);
fprintf('%-22s %10.1f %10.1f\n', 'BB height [mm]',    hBB,   hBBSag);
fprintf('%-22s %10.1f %10.1f\n', 'Wheelbase [mm]',    w,     wSag);
fprintf('%-22s %10.1f\n',        'Stack [mm]',        S);
fprintf('%-22s %10.1f\n',        'Reach [mm]',        Re);
fprintf('%-22s %10.1f\n',        'Mech. trail [mm]',  tm);
fprintf('%-22s %10.1f\n',        'Ground trail [mm]', c);