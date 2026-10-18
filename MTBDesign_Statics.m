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


%%



%% Push to Github
repo = gitrepo;
add(repo, "MTBDesign_Statics.m");
commit(repo, Message="statics: new file for bicycle statics");
push(repo);


%% Debugging