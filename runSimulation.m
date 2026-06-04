clear; clc; close all;
cd(fileparts(mfilename('fullpath')));

if ~isfile('navigator.fis')
    build_navigator_fis;
end

mazeSim(@myController);
