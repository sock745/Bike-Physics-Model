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
data.desc.muReq = data.desc.brakingFReq ./ (m * g * cosd(thD)); % [-]
data.desc.aMaxg = ((w - xp) * cosd(thD) - h * sind(thD)) / h; % pitch-over decel / g [gs]
data.desc.thMax = atand((w - xp) / h); % steepest possible descent, no braking [deg]
data.desc.noPitch = data.desc.Nr > 0; % rear wheel stays down 
data.desc.gripOK = data.desc.muReq <= mu; % tires can supply braking force

% Accelerating & Ascending (ground tilted up by thC)
data.climb.th = thC;
data.climb.Nf = 1 / w * (-m * aAccel * h + m * g * (cosd(thC) * xp - sind(thC) * h)); % front normal force [N]
data.climb.Nr = m * g * cosd(thC) - data.climb.Nf; % rear normal force [N]
data.climb.fr = m * aAccel + m * g * sind(thC); % rear friction force; front wheel not driven [N]
data.climb.muReq = data.climb.fr ./ data.climb.Nr; % rear friction needed [-]
data.climb.aMaxg = (xp * cosd(thC) - h * sind(thC)) / h; % front lift accel / g [gs]
data.climb.thMax = atand(xp / h); % steepest climb, steady speed [deg]
data.climb.frontDown = data.climb.Nf > 0; % front wheel stays down
data.climb.tractionOK = data.climb.muReq <= mu; % rear tire doesn't spin 


%% Display Results
ok = {'FAIL', 'ok'};   % flag labels, indexed by logical+1

fprintf('\n================ STATICS SUMMARY ================\n');
fprintf('Inputs: m = %.1f kg, w = %.1f mm, xp = %.1f mm, h = %.1f mm, mu = %.2f\n', ...
        m, w, xp, h, mu);
fprintf('        braking = %.2f g, climbing accel = %.2f g\n', aBrake/g, aAccel/g);

% Level ground
fprintf('\n--- Level ground ---\n');
fprintf('Front load: %7.1f N  (%.1f%%)\n', data.level.Nf, 100*data.level.frontFrac);
fprintf('Rear load:  %7.1f N  (%.1f%%)\n', data.level.Nr, 100*(1 - data.level.frontFrac));

% Limits
fprintf('\n--- Limits (steady speed) ---\n');
fprintf('Steepest descent before pitch-over: %5.1f deg\n', data.desc.thMax);
fprintf('Steepest climb before front lift:   %5.1f deg\n', data.climb.thMax);

% Descending table
fprintf('\n--- Descending & braking at %.2f g ---\n', aBrake/g);
fprintf('%7s %9s %9s %10s %7s %9s %9s %7s\n', ...
        'th[deg]','Nf[N]','Nr[N]','Fbrake[N]','muReq','aMax[g]','Pitch','Grip');
for k = 1:numel(data.desc.th)
    fprintf('%7.1f %9.1f %9.1f %10.1f %7.2f %9.2f %9s %7s\n', ...
        data.desc.th(k), data.desc.Nf(k), data.desc.Nr(k), ...
        data.desc.brakingFReq(k), data.desc.muReq(k), data.desc.aMaxg(k), ...
        ok{data.desc.noPitch(k)+1}, ok{data.desc.gripOK(k)+1});
end

% Climbing table
fprintf('\n--- Climbing, accel %.2f g ---\n', aAccel/g);
fprintf('%7s %9s %9s %9s %7s %9s %9s %9s\n', ...
        'th[deg]','Nf[N]','Nr[N]','Fdrive[N]','muReq','aMax[g]','FrontDn','Traction');
for k = 1:numel(data.climb.th)
    fprintf('%7.1f %9.1f %9.1f %9.1f %7.2f %9.2f %9s %9s\n', ...
        data.climb.th(k), data.climb.Nf(k), data.climb.Nr(k), ...
        data.climb.fr(k), data.climb.muReq(k), data.climb.aMaxg(k), ...
        ok{data.climb.frontDown(k)+1}, ok{data.climb.tractionOK(k)+1});
end
fprintf('=================================================\n');

%% Push to Github
repo = gitrepo;
add(repo, "MTBDesign_Statics.m");
commit(repo, Message="statics solved, one failure exists!");
push(repo);


%% Debugging