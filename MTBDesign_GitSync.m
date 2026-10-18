%% MTB DESIGN - Github Sync ----------------------------------------------- 
% Purpose: 
% Sync files to github, whenever needed.
% -------------------------------------------------------------------------


%% Houskeeping
% Clean up workspace
clc; clear; close all;

%% MTB Git Sync
cd("/Users/danwanthj/Documents/Projects/MTB");
repo = gitrepo;
pull(repo);
add(repo, ["MTBDesign_FrameGeometry.m", "MTBDesign_Statics.m", "MTBDesign_GitSync.m"]);
msg = 'github within files removed! statics solved! gitsync created!';
commit(repo, Message=msg);
push(repo);