clear all
close all
clc
%% Calibrate a sensor
% This script allows to calibrate six axis force torque (F/T)
% sensors once they are mounted on the robot. This procedure
% takes advantage of the knowledge of the model of the robot
% to generate the expected wrenches of the sensors during
% some arbitrary motions. It then uses this information to train
% and validate new calibration matrices, taking into account
% the calibration matrix obtained with a classical Workbench
% calibration. For more on the theory behind this script, check [1].
% The data from an experiment is typically logged
% using yarpDataDumper directly form statesAndFtSensorsInertial.xml or
% using sensor-calib-inertial [2] and stored in [3] or [4].
% [1] : F. J. A. Chavez, S. Traversaro, D. Pucci and F. Nori,
%       "Model based in situ calibration of six axis force torque sensors,"
%       2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids), Cancun, 2016
% [2] : https://github.com/robotology-playground/sensors-calib-inertial/tree/feature/integrateFTSensors%
% [3] : https://gitlab.com/dynamic-interaction-control/green-iCub-Insitu-Datasets
% [4] : https://gitlab.com/dynamic-interaction-control/icub-insitu-ft-analysis-big-datasets

%% Instructions before running script
% -Log the experiment using statesAndFtSensorsInertial.xml or
% using sensor-calib-inertial
% -Edit a file params.m based on paramsTemplate.m to match the
% characteristics of the experiment and put it in the experiment folder
% -Verify there is a folder named calibrationMatrices inside the experiment
% folder to store resulting calibration matrices
%    ~ Remark: if nothing changed between experiments (logging method,
%    sensor replacement or use of another robot) params.m can be
%    directly copied for another experiment.
% -Select desired options for reading the experiment
% -Change experimentName to desired experiment folder
% -Select desired options of the calibration procedure
% -Run this script

%% adding required dependencies
addpath external/quadfit
addpath external/walkingDatasetScripts
addpath utils

%% Read data
    % general reading configuration options
readOptions = {};
readOptions.forceCalculation=false;%false;
readOptions.raw=true;
readOptions.saveData=true;
readOptions.multiSens=true;
readOptions.matFileName='ftDataset'; % name of the mat file used for save the experiment data
    % options not from read experiment
readOptions.printPlots=false;%true
    % name and paths of the experiment files
    % change name to desired experiment folder
%    experimentName='/green-iCub-Insitu-Datasets/2018_07_10_multipleTemperatures';
   
%    experimentName='/green-iCub-Insitu-Datasets/2018_07_10_Grid';
    experimentName='/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_tz_3';
% experimentName='icub-insitu-ft-analysis-big-datasets/iCubGenova04/exp_1/poleLeftRight';
% experimentName='/green-iCub-Insitu-Datasets/2018_07_10_LeftYogaWarm';
[dataset,~,input,extraSample]=readExperiment(experimentName,readOptions);

%% Calibration options
    % Select sensors to calibrate the names are associated to the location of
    % the sensor in the robot
    % on iCub  {'left_arm','right_arm','left_leg','right_leg','right_foot','left_foot'};
sensorsToAnalize = {'right_leg','left_leg'};
    %Regularization parameter
lambda=1;
if (lambda==0)
    lambdaName='';
else
    lambdaName=strcat('_l',num2str(lambda));
end
% lambdaName='';
    %calibration script options
calibOptions.saveMat=true;
calibOptions.estimateType=1;%0 only insitu offset, 1 is insitu, 2 is offset on main dataset, 3 is oneshot offset on main dataset, 4 is full oneshot
calibOptions.useTemperature=false;
    % Calibrate
calibrationStep

%% Check results
checkMatrixOptions.plotForceSpace=true;
checkMatrixOptions.plotForceVsTime=false;
checkMatrixOptions.secMatrixFormat=false;
checkMatrixOptions.resultEvaluation=true;
%% logic to select data in which we will test the result
datasetToUse=dataset;
if extraSampleAvailable
    extraSampleNames=fieldnames(extraSample);
    for eSampleIDNum =1:length(extraSampleNames)
        eSampleID = extraSampleNames{eSampleIDNum};
        if (isstruct(extraSample.(eSampleID)))
            for eSamples=1:length(extraSample.(eSampleID))
                datasetToUse=addDatasets(datasetToUse,extraSample.(eSampleID)(eSamples));
            end
        end
    end
end
[reCalibData,offsetInWrenchSpace,MSE]=checkNewMatrixPerformance(datasetToUse,sensorsToAnalize,calibMatrices,offset,checkMatrixOptions,'otherCoeff',temperatureCoeff,'varName','temperature');
MSE
%% save results
%% Save the workspace again to include calib Matrices, scale and offset
%     %save recalibrated matrices, offsets, new wrenches, sensor serial
%     numbers
saveResults=readOptions.saveData; % for the time being save if readOption.saveData is true
if(saveResults)
    results.usedDataset=datasetToUse;
    results.calibrationMatrices=calibMatrices;
    results.fullscale=fullscale;
    results.offset=offset;
    results.temperatureCoeff=temperatureCoeff;
    results.offsetInWrenchSpace=offsetInWrenchSpace;
    results.recalibratedData=reCalibData;
    results.MSE=MSE;
    if ~exist(strcat('data/',experimentName,'/results'),'dir')
        mkdir(strcat('data/',experimentName,'/results'));
    end
    save(strcat('data/',experimentName,'/results/results.mat'),'results')
end

    % Plot for inspection of data
if( readOptions.printPlots )
    run('plottinScript.m')
end

