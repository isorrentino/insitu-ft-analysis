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

% %Use only datasets where the same sensor is used
% experimentNames={
%     'green-iCub-Insitu-Datasets/2017_10_31_3';
%     'green-iCub-Insitu-Datasets/2017_10_31_4'% Name of the experiment;
%     'green-iCub-Insitu-Datasets/2017_10_17Grid';
%     'green-iCub-Insitu-Datasets/2017_10_30';
%     }; %this set is from iCubGenova02
% names={'Workbench';
%     'bestleftleg';
%     'bestleftfoot';
%     'fristGreenSample';
%     'day30';    
%     };% except for the first one all others are short names for the expermients in experimentNames

%Use only datasets where the same sensor is used
experimentNames={
    'green-iCub-Insitu-Datasets/2018_04_09_Grid_2';
     'icub-insitu-ft-analysis-big-datasets/iCubGenova04/exp_1/poleLeftRight';
    }; %this set is from iCubGenova04
names={'Workbench';
    'withTz';
    'DecemberData';
    };% except for the first one all others are short names for the expermients in experimentNames


%lambdas=[0];
lambdas=[0;
    10
    50;
    1000;
    10000;
    50000;
    100000;
    500000;
    1000000;
    5000000;
    10000000];
% Create appropiate names for the lambda variables
for namingIndex=1:length(lambdas)
    if (lambdas(namingIndex)==0)
        lambdasNames{namingIndex}='';
    else
        lambdasNames{namingIndex}=strcat('_l',num2str(lambdas(namingIndex)));
    end
end
lambdasNames=lambdasNames';


names2use{1}=names{1};
num=2;
for i=2:length(names)
    for j=1:length(lambdasNames)
        names2use{num}=(strcat(names{i},lambdasNames{j}));
        num=num+1;
    end
end
names2use=names2use';

%%  Select sensors and frames to analize
sensorsToAnalize = {'left_leg','right_leg'};  %load the new calibration matrices
framesToAnalize={'l_upper_leg','r_upper_leg'};
sensorName={'l_leg_ft_sensor','r_leg_ft_sensor'};

% sensorsToAnalize = {'left_leg'};  %load the new calibration matrices
% framesToAnalize={'l_upper_leg'};
% sensorName={'l_leg_ft_sensor'};


%% Read the calibration matrices to evaluate

[cMat,secMat,WorkbenchMat]=readGeneratedCalibMatrices(experimentNames,scriptOptions,sensorsToAnalize,names2use,lambdasNames);

%% Select datasets in which the matrices will be evaluated
%toCompare={'iCubGenova04/exp_1/yogaLeft','iCubGenova04/exp_1/yogaRight'};%datasets name 'leftYoga' 'failedLeftYoga'
toCompare={'icub-insitu-ft-analysis-big-datasets/iCubGenova04/exp_2/yogaRight','icub-insitu-ft-analysis-big-datasets/iCubGenova04/exp_2/yogaLeft'};
toCompareNames={'yogaRight','yogaLeft'}; % short Name of the experiments

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

for c=1:length(toCompare)
    [data.(toCompareNames{c}),estimator,input]=readExperiment(toCompare{c},compareDatasetOptions);
    
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
    sampleInit=[40,40];
    sampleEnd=[60,60];
    % subsample dataset to speed up computations
     for i=1:length(names2use)
      [offset.(toCompareNames{c}).(names2use{i})]=calculateOffsetUsingWBD(estimator,data.(toCompareNames{c}),sampleInit(c),sampleEnd(c),input,secMat.(names2use{i}));
     end
    
    %TODO: should I filter before sampling? Or avoid data sampling and filtering to have more real like results
    fprintf('Filtering %s \n',(toCompareNames{c}));
    [data.(toCompareNames{c}).ftData,mask]=filterFtData(data.(toCompareNames{c}).ftData);
    data.(toCompareNames{c})=applyMask(data.(toCompareNames{c}),mask);
    [data.(toCompareNames{c}),~]= dataSampling(data.(toCompareNames{c}),2);
    
   
    
    %% Comparison
    framesNames={'l_sole','r_sole','l_upper_leg','r_upper_leg','root_link','l_elbow_1','r_elbow_1',}; %there has to be atleast 6
    timeFrame=[0,15000];
    sMat={};
    for j=1:length(sensorsToAnalize) %why for each sensor? because there could be 2 sensors in the same leg
        %% Calculate external forces
        for i=1:length(names2use)
            sMat.(sensorsToAnalize{j})=secMat.(names2use{i}).(sensorsToAnalize{j});% select specific secondary matrices
             
  
            cd ../
            [results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).externalForces,...
                results.(toCompareNames{c}).(sensorsToAnalize{j}).(names2use{i}).eForcesTime]=...
                estimateExternalForces... %obtainExternalForces ... %
                (input.robotName,data.(toCompareNames{c}),sMat,input.sensorNames,...
                input.contactFrameName,timeFrame,framesNames,offset.(toCompareNames{c}).(names2use{i}),{sensorsToAnalize{j}});
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

%% SimpleOddometry init
% This is to understand which axis from the ft Sensor affects each axis in the evaluation frame 
odom = iDynTree.SimpleLeggedOdometry();
odom.setModel(estimator.model());
jointPos = iDynTree.JointPosDoubleArray(estimator.model());
odom.updateKinematics(jointPos);
odom.init('root_link','root_link');
% Note: we will assume that mapping of the axis is a one to one
% relationship

%% Evaluate error
run('evaluateSecondaryMatrixError');