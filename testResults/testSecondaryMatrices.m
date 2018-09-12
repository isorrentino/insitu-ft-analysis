clear all
close all
clc

% clear all;
addpath utils
addpath external/quadfit
%% Prepare options of the test

scriptOptions = {};
scriptOptions.testDir=true;% to calculate the raw data, for recalibration always true
scriptOptions.matFileName='ftDataset';
scriptOptions.printAll=true;
% Script of the mat file used for save the intermediate results
%scriptOptions.saveDataAll=true;

%% Select datasets with which the matrices where generated and lambda values
%Use only datasets where the same sensor is used
% experimentNames={   
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid';% Name of the experiment;    
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid_warm';% Name of the experiment;
%     '/green-iCub-Insitu-Datasets/2018_07_10_multipleTemperatures';% Name of the experiment;
%     }; %this set is from iCubGenova04
experimentNames={ %iCubGenova02 experiments
%     '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_Grid';% Name of the experiment;
%     '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_left_yoga';% Name of the experiment;
%     '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_right_yoga';% Name of the experiment;
    '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_MixedDataSets';% Name of the experiment;
    };
names={'Workbench';
%     'grid';
    %'decemberNoTemp';
%     'leftYoga';
    'mixedDataset';
    };% except for the first one all others are short names for the expermients in experimentNames

% lambdas=[1];
% estimationTypes=[1,4];
% useTempBooleans=[1,4];
lambdas=[0;
    1;
    5;
    10;
    50;
    100;
    1000;
    5000;
    10000;
    50000;
    100000;
    500000;
    1000000];
% estimation types
% estimationTypes=[1,2,3,4,1,2,3,4];
% useTempBooleans=[0,0,0,0,1,1,1,1];
estimationTypes=[1,1,3,3];
useTempBooleans=[0,1,0,1];
% Create appropiate names for the lambda variables
for namingIndex=1:length(lambdas)
    if (lambdas(namingIndex)==0)
        lambdasNames{namingIndex}='';
    else
        lambdasNames{namingIndex}=strcat('_l',num2str(lambdas(namingIndex)));
    end
end
lambdasNames=lambdasNames';
%
if ~exist('estimationTypes','var')
    estimationNames={''};
else
    for namingIndex=1:length(estimationTypes)
        switch estimationTypes(namingIndex)
            case 1
                name='_sphere_offset';
            case 2
                name='_noMeanOnMain_offset';
            case 3
                name='_noMean_offset';
            case 4
                name='_oneShot_offset';
        end
        if useTempBooleans(namingIndex)
            name=strcat(name,'_temp');
        else
            name=strcat(name,'_noTemp');
        end
        estimationNames{namingIndex}=name;
    end
end
%
cNamesNumber=1;
for j=1:length(lambdasNames)
    for k=1:length(estimationNames)
        calibrationFileNames{cNamesNumber}=strcat(lambdasNames{j},estimationNames{k});
        cNamesNumber=cNamesNumber+1;
    end
end
calibrationFileNames=calibrationFileNames';
% do something similar but with estimation types, consider temperature on
% and off as different estimation types

names2use{1}=names{1};
num=2;
for i=2:length(names)
    for j=1:length(lambdasNames)
        for k=1:length(estimationNames)
            names2use{num}=strcat(names{i},lambdasNames{j},estimationNames{k});
            num=num+1;
        end
    end
end
names2use=names2use';

%%  Select sensors and frames to analize
% sensorsToAnalize = {'left_leg','right_leg'};  %load the new calibration matrices
% framesToAnalize={'l_upper_leg','r_upper_leg'};
% sensorName={'l_leg_ft_sensor','r_leg_ft_sensor'};
reduceBy=10; % value used in datasampling;
sensorsToAnalize = {'left_leg'};  %load the new calibration matrices
framesToAnalize={'l_upper_leg'};
sensorName={'l_leg_ft_sensor'};


%% Read the calibration matrices to evaluate

[cMat,secMat,WorkbenchMat,extraCoeff]=readGeneratedCalibMatrices(experimentNames,scriptOptions,sensorsToAnalize,names2use,calibrationFileNames);

% %% Select datasets in which the matrices will be evaluated

toCompare={'/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_Grid_2','/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_tz_2','/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_left_yoga_2','/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_right_yoga_2'};
toCompareNames={'Grid39Degree','Tz39Degree','LeftYoga39Degree','RightYoga39Degree'}; % short Name of the experiments for iCubGenova02

% toCompare={'green-iCub-Insitu-Datasets/yoga in loop','green-iCub-Insitu-Datasets/yoga left cold session'};
% toCompareNames={'yogaLoog','yogaLeftCold'}; % short Name of the experiments

compareDatasetOptions = {};
compareDatasetOptions.forceCalculation=false;%false;
compareDatasetOptions.saveData=true;%true
compareDatasetOptions.matFileName='iCubDataset';
compareDatasetOptions.testDir=true;
compareDatasetOptions.raw=false;
%compareDatasetOptions.testDir=true;% to calculate the raw data, for recalibration always true
compareDatasetOptions.filterData=false;
compareDatasetOptions.estimateWrenches=true;
compareDatasetOptions.useInertial=false;

data=struct();
for c=1:length(toCompare)
    [data.(toCompareNames{c}),estimator,input]=readExperiment(toCompare{c},compareDatasetOptions);
    dataFields=fieldnames(data.(toCompareNames{c}));
    if ~ismember('temperature',dataFields)
        withTemperature=false;
    else
        withTemperature=true;
    end
    %TODO: have a more general structure with multiple datasets, and multiple
    %inputs to be able to check on multiple experiments in the end maybe
    %external forces can be combined from both experiments just to get the best
    %matrix considering all evaluated datasets.
    
    %% Calculate offset with a previously selected configuration of the robot during the experment
    %offsetContactFrame={'r_sole','l_sole'}; %'l_sole' 'root_link'
    %inspect data to select where to calculate offset
    robotName='iCubGenova04';
    onTestDir=true;
    %iCubVizWithSlider(data.(toCompareNames{c}),robotName,sensorsToAnalize,input.contactFrameName{1},onTestDir);
    
    %% Calculate offsets for each secondary matrix for each comparison dataset
    sampleInit=[40,40,1040,1040];
    sampleEnd=[60,60,1060,1060];
    if length(toCompareNames)~=length(sampleInit) || length(toCompareNames)~=length(sampleEnd)
        error('testSecondaryMatrices: begining and end of the samples in which the offset will be calculated should be provided for all data sets to compare');
        
    end
    % subsample dataset to speed up computations
    for i=1:length(names2use)
        [offset.(toCompareNames{c}).(names2use{i})]=calculateOffsetUsingWBD(estimator,data.(toCompareNames{c}),sampleInit(c),sampleEnd(c),input,secMat.(names2use{i}));
    end
    
    %TODO: should I filter before sampling? Or avoid data sampling and filtering to have more real like results
    fprintf('Filtering %s \n',(toCompareNames{c}));
    [data.(toCompareNames{c}).ftData,mask]=filterFtData(data.(toCompareNames{c}).ftData);
    data.(toCompareNames{c})=applyMask(data.(toCompareNames{c}),mask);
    [data.(toCompareNames{c}),~]= dataSampling(data.(toCompareNames{c}),reduceBy);
    
    
    
    %% Comparison
    framesNames={'l_sole','r_sole','l_upper_leg','r_upper_leg','root_link','l_elbow_1','r_elbow_1',}; %there has to be atleast 6
    timeFrame=[0,15000];
    sMat={};
    for j=1:length(sensorsToAnalize) %why for each sensor? because there could be 2 sensors in the same leg
        %% Calculate external forces
        for i=1:length(names2use)
            sMat.(sensorsToAnalize{j})=secMat.(names2use{i}).(sensorsToAnalize{j});% select specific secondary matrices
            
            
            cd ../
            if ~withTemperature
                [results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).externalForces,...
                    results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).eForcesTime,~,...
                    results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame]=...
                    estimateExternalForces... %obtainExternalForces ... %
                    (input.robotName,data.(toCompareNames{c}),sMat,input.sensorNames,...
                    input.contactFrameName,timeFrame,framesNames,offset.(toCompareNames{c}).(names2use{i}),...
                    'sensorsToAnalize',sensorsToAnalize(j));
            else
                sensorsExtraCoeff.(sensorsToAnalize{j})=extraCoeff.(names2use{i}).(sensorsToAnalize{j});
                [results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).externalForces,...
                    results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).eForcesTime,~,...
                    results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame]=...
                    estimateExternalForces... %obtainExternalForces ... %
                    (input.robotName,data.(toCompareNames{c}),sMat,input.sensorNames,...
                    input.contactFrameName,timeFrame,framesNames,offset.(toCompareNames{c}).(names2use{i}),...
                    'sensorsToAnalize',sensorsToAnalize(j),'extraVar',data.(toCompareNames{c}).temperature,'extraCoeff',sensorsExtraCoeff);
            end
            % we restrict the offset to be used to only the sensor we are
            % analizing by passing in sensorstoAnalize only the sensor we
            % are analizing at the moment, otherwise it induces errors in
            % when it uses the offset on a part that do not requires it
            
            cd testResults/
        end
        if c==1
            stackedResults.(sensorsToAnalize{j})=results.(toCompareNames{c}).(sensorsToAnalize{j});
        else
            stackedResults.(sensorsToAnalize{j})=addDatasets(stackedResults.(sensorsToAnalize{j}),results.(toCompareNames{c}).(sensorsToAnalize{j}));
            
        end
    end
    
end
%% Save external forces
extForceResults.results=stackedResults;
extForceResults.lambdas=lambdas;
extForceResults.estimationTypes=estimationTypes;
extForceResults.useTempBooleans=useTempBooleans;
extForceResults.names.names2use=names2use;
extForceResults.names.toCompare=toCompareNames;
extForceResults.names.experimentNames=names;
extForceResults.toCompare=toCompare;
extForceResults.experimentNames=experimentNames;
if strfind(pwd,'testResults')>0  
    prefix='../';
else
    prefix='';
    
end
if ~exist(strcat(prefix,'data/generalResults'),'dir')
    mkdir(strcat(prefix,'data/generalResults'));
    
end

save(strcat(prefix,'data/generalResults/extForceResults_',date,'_',input.robotName,'.mat'),'extForceResults')
%% Evaluate error
run('evaluateSecondaryMatrixError');