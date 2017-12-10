%% Comparing FT data vs estimated data
% %create input parameter is done through params.m for each experiment

%% add required folders for use of functions
addpath external/quadfit
addpath utils

%% general configuration options 
scriptOptions = {};
scriptOptions.forceCalculation=true;%false;
scriptOptions.printPlots=false;%true
scriptOptions.saveData=false;%true
scriptOptions.raw=true;% to calculate the raw data, for recalibration always true
scriptOptions.useInertial=false;

% Script of the mat file used for save the intermediate results
%scriptOptions.saveDataAll=true;
scriptOptions.matFileName='ftDataset';

%% name and paths of the experiment files
%  experimentName='icub-insitu-ft-analysis-big-datasets/2016_07_05/gridMin45';% Name of the experiment;
%experimentName='green-iCub-Insitu-Datasets/2017_12_5_Strain2_3';% 
%experimentName='green-iCub-Insitu-Datasets/2017_10_17Grid';%
experimentName='/green-iCub-Insitu-Datasets/2017_12_7_TestYogaExtendedLeft';

%% We carry the analysis just for a subset of the sensors
% the names are associated to the location of the sensor in the
% in the iCub

%sensorsToAnalize = {'left_leg','right_leg'};
sensorsToAnalize = {'right_leg'};
%sensorsToAnalize = {'left_leg','right_leg','right_foot','left_foot'};

%% Calibration options
%Regularization parameter
lambda=0;
lambdaName='';

%script options
calibOptions.saveMat=false;
calibOptions.usingInsitu=true;
calibOptions.plot=true;
calibOptions.onlyWSpace=true;
calibOptions.IITfirmwareFriendly=true; % in case a calibration matrix that will not be used by iit firmware is estimated
%% Start 
%Read data
[dataset,extraSample]=read_estimate_experimentData(experimentName,scriptOptions);

%Plot for inspection of data
if( scriptOptions.printPlots )
    run('plottinScript.m')
end

%Calibrate
run('CalibMatCorrection.m')

