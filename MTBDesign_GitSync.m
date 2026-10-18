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

% Push to Github
cd("/Users/danwanthj/Documents/Projects/MTB");
repo = gitrepo;
pull(repo);
add(repo, frameGeom);
msg = 'github within file removed!';
commit(repo, Message=msg);
push(repo);