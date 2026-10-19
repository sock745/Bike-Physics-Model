%% MTB DESIGN - Github Sync ----------------------------------------------- 
% Purpose: 
% Sync files to github, whenever needed.
% -------------------------------------------------------------------------


%% Houskeeping
% Clean up workspace
clc; clear; close all;

%% MTB Git Sync
% Files
statics = 'MTBDesign_Statics.m';
frameGeom = 'MTBDesign_FrameGeometry.m';
gitsync =  'MTBDesign_GitSync.m';
latdyn = 'MTBDesign_LateralDynamics.m';
whipple = 'whipple_pull_force_abcd.m';

% Push to Github
cd("/Users/danwanthj/Documents/Projects/MTB");
repo = gitrepo;
pull(repo);
add(repo, gitsync); % update second item
msg = 'Added code completion message and new file variables.';
commit(repo, Files=gitsync, Message=msg); % update Files entry
push(repo);
disp("Successfully pushed to Github!");