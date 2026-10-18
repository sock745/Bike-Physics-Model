%% MTB DESIGN - BICYCLE STATICS -------------------------------------------
% Nomenclature:
% 
% 
% Purpose: 
% Using the geometrically constrained frame points to determine the
% constrains imposed by the statics of the system.
% -------------------------------------------------------------------------


%% Houskeeping
% Clean up workspace
clc; clear; close all;

% Github
cd("/Users/danwanthj/Documents/Projects/MTB");
repo = gitrepo;
pull(repo);

% Structure
data = struct();


%% Inputs
% Bring in Geometry Data
run('MTBDesign_FrameGeometry.m'); 

% Masses
mRider = 82; % rider mass; standard value [kg]
mBike = 17; % projected bike mass [kg]
m = mRider + mBike;

% CoM Location
xG = 30; % from BB; placeholder; need anthropometric model [mm]
yG = 150; % from BB; placeholder; need anthropometric model [mm]
xp = xG - RPos(1); % distance from rear contact
h = yG + hBB; % height above ground

% Accelerations
aBrake = 0.5; % braking [gs]
aAccel = 0; % steady climb [gs]

% Friction
mu = 0.7; % tire ground friction [-]

% Angles
thD = 0:5:35; % descent angle [deg]
thC =  0:5:30; % ascent angle [deg]

% Gravity
g = 9.81; % [m/s^2]


%% Statics
% Level Ground
data.level.Nr = m * g * (1 - xp / w); % rear normal force [N]
data.level.Nf = m * g * xp / w; % front normal force [N]
data.level.frontFrac = xp / w; % fraction of weight on front [-]

% Descending & Braking (ground tilted down by thD)
data.desc.th = thD;
data.desc.Nf = 1 / w * (m * g * (sind(thD) * h + cosd(thD) * xp) + m * aBrake * h); % front normal force [N]
data.desc.Nr = m * g * cosd(thD) - data.desc.Nf; % rear normal force [N]
data.desc.brakingFReq = m * aBrake + m * g * sind(thD); % braking force needed [N]
data.desc.muReq = data.desc.brakingFReq / (m * g * cosd(thD)); % [-]
data.desc.aMaxg = (xp * cosd(thC) - h * sind(thC)) / h; % pitch-over decel / g [gs]
data.desc.thMax = atand((w - xp) / h); % steepest possible descent, no braking [deg]
data.desc.noPitch = data.desc.Nr > 0; % rear wheel stays down 
data.desc.gripOK = data.desc.muReq <= mu; % tires can supply braking force

% Accelerating & Ascending (ground tilted up by thC)
data.climb.th = thC;
data.clumb.Nf = 1 / w * (-m * aAccel * h + m * g * (cosd(thC) * xp - sind(thC) * h)); % front normal force [N]
data.climb.Nr = m * g * cosd(thC) - data.climb.Nf; % rear normal force [N]
data.climb.fr = m * a + m * g * sind(thC); % rear friction force; front wheel not driven [N]
data.climb.muReq = data.climb.fr / data.climb.Nr; % rear friction needed [-]
data.climb.aMaxg = (xp * cosd(thC) - h * sind(thC)) / h; % front lift accel / g [gs]
data.climb.thMax = atand(xp / h); % steepest climb, steady speed [deg]
data.climb.frontDown = data.climb.Nf > 0; % front wheel stays down
data.climb.tractionOK = data.climb.muReq <= mu; % rear tire doesn't spin 


%% Display Results
fprintf('\nFlat: Nf = %.0f N, Nr = %.0f N, front = %.1f%%\n', ...
        data.level.Nf, data.level.Nr, 100*data.level.frontFrac);
fprintf('Steepest descent (no braking): %.1f deg\n', data.desc.thMax);
fprintf('Steepest climb (steady):       %.1f deg\n', data.climb.thMax);

fprintf('\nDescending, braking at %.2f g\n', aBrake/9.81);
fprintf('%6s %8s %8s %8s %10s\n','th[deg]','Nf[N]','Nr[N]','muReq','aMax/g');
fprintf('%6.0f %8.0f %8.0f %8.2f %10.2f\n', [thD; data.desc.Nf; data.desc.Nr; data.desc.muReq; data.desc.aMaxg]);

fprintf('\nClimbing, accel %.2f g\n', aAccel/9.81);
fprintf('%6s %8s %8s %8s %10s\n','th[deg]','Nf[N]','Nr[N]','muReq','aMax/g');
fprintf('%6.0f %8.0f %8.0f %8.2f %10.2f\n', [thC; data.climb.Nf; data.climb.Nr; data.climb.muReq; data.climb.aMaxg]);


%% Push to Github
repo = gitrepo;
add(repo, "MTBDesign_Statics.m");
commit(repo, Message="statics solved!");
push(repo);


%% Debugging