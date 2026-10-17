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


%% Push to Github
repo = gitrepo;
add(repo, "MTBDesign_Statics.m");
commit(repo, Message="statics: new file for bicycle statics");
push(repo);


%% Debugging