%close all
clc
addpath utils
addpath external/quadfit
%% Prepare options of the test
scriptOptions = {};
scriptOptions.testDir=true;
scriptOptions.matFileName='ftDataset';
scriptOptions.printAll=true;
% Script of the mat file used for save the intermediate results
%   scriptOptions.saveDataAll=true;
%% Select datasets with which the matrices where generated and lambda values
%Use only datasets where the same sensor is used
% experimentNames={
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid';% Name of the experiment;
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid_warm';% Name of the experiment;
%     '/green-iCub-Insitu-Datasets/2018_07_10_multipleTemperatures';% Name of the experiment;
%     }; %this set is from iCubGenova04
% experimentNames={ %iCubGenova02 experiments
% %     '/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_noTz';% Name of the experiment;
%     '/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_multipleTemperatures';% Name of the experiment;
% %     '/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_AllGeneral';% Name of the experiment;
%     };
experimentNames={ %iCubGenova04 experiments
    '/green-iCub-Insitu-Datasets/2018_12_11/noTz';
    '/green-iCub-Insitu-Datasets/2018_12_11/onlySupportLegs';
    '/green-iCub-Insitu-Datasets/2018_12_11/allTogether';
    };

names={'Workbench';
    'noTz';
    'SuppOnly';
    'all';
    };% except for the first one all others are short names for the expermients in experimentNames


% lambdas=[
%     0;
%     1;
%     5;
%     10;
%     50;
%     100;
%     1000;
%     5000;
%     10000;
%     50000;
%     100000;
%     500000;
%     1000000
%     ];
% estimation types/
% estimationTypes=[1,1,1,3,3,3,4,4,4];
% useTempBooleans=[0,1,1,0,1,1,0,1,1];
% useTempOffset  =[0,0,1,0,0,1,0,0,1];
lambdas=[0];
estimationTypes=[1,1,4,4];
useTempBooleans=[0,1,0,1];
useTempOffset  =[0,1,0,1];
%% Create appropiate names for the calibration matrices to be tested
lambdasNames=generateLambdaNames(lambdas);
if ~exist('estimationTypes','var')
    estimationNames={''};
else
    estimationNames=generateEstimationTypeNames(estimationTypes,useTempBooleans,useTempOffset);
end
calibrationFileNames=generateCalibrationFileNames(lambdasNames,estimationNames);
names2use=generateCalibrationFileNames(names(2:end),calibrationFileNames);
names2use=[names{1};names2use];
%%  Select sensors and frames to analize
% sensorsToAnalize = {'right_leg','left_leg','right_foot','left_foot'};
% sensorsToAnalize = {'left_leg','right_leg'};  %load the new calibration matrices
% framesToAnalize={'r_upper_leg','l_upper_leg'};
% sensorName={'r_leg_ft_sensor','l_leg_ft_sensor','r_foot_ft_sensor','l_foot_ft_sensor'};
% sensorsToAnalize = {'right_leg'};  %load the new calibration matrices
% framesToAnalize={'r_upper_leg'};
% sensorName={'r_leg_ft_sensor'};
sensorsToAnalize = {'left_leg'};  %load the new calibration matrices
framesToAnalize={'l_upper_leg'};
sensorName={'l_leg_ft_sensor'};
j=1;

%% Read the calibration matrices to evaluate

[cMat,secMat,WorkbenchMat,extraCoeff,offsets,extraCoeffOffset]=readGeneratedCalibMatrices(experimentNames,scriptOptions,sensorsToAnalize,names2use,calibrationFileNames);

% %% Select datasets in which the matrices will be evaluated
toCompare={'/green-iCub-Insitu-Datasets/2018_12_10/Grid_2','/green-iCub-Insitu-Datasets/2018_12_10/tz_2','/green-iCub-Insitu-Datasets/2018_12_10/leftyoga','/green-iCub-Insitu-Datasets/2018_12_04/leftyoga_3','/green-iCub-Insitu-Datasets/2018_12_04/rightyoga_3',};
toCompareNames={'Grid27Degree','Tz27Degree','LeftYoga34Degree','LeftYoga38Degree','RightYoga38Degree'}; % short Name of the experiments for iCubGenova02
reduceBy=[100,1,1,1,1]; % value used in datasampling;
useKnownOffset=false;
% offset times for each comparison dataset
sampleInit=[40,40,1040,1040,1040];
sampleEnd=[60,60,1060,1060,1060];
if length(toCompareNames)~=length(sampleInit) || length(toCompareNames)~=length(sampleEnd)
    error('testSecondaryMatrices: begining and end of the samples in which the offset will be calculated should be provided for all data sets to compare');
    
end


% readExperiment Options for the compare datasets
compareDatasetOptions = {};
compareDatasetOptions.forceCalculation=false;%false;
compareDatasetOptions.saveData=true;%true
compareDatasetOptions.matFileName='ftDataset';
compareDatasetOptions.testDir=true;
compareDatasetOptions.raw=true;
compareDatasetOptions.filterData=true;
compareDatasetOptions.estimateWrenches=true;
compareDatasetOptions.useInertial=false;
% CheckMatrixPerformance Options
checkMatrixOptions.plotForceSpace=true;
checkMatrixOptions.plotForceVsTime=false;
checkMatrixOptions.secMatrixFormat=false;
counter=1;
data=struct();
%% Section to prepare variables that will receive results
combinationNumber=length(experimentNames)*length(lambdas)*length(estimationTypes);
MSEvalues=zeros(length(toCompare),combinationNumber,6);
if ~useKnownOffset
offstetValues=zeros(length(toCompare),combinationNumber,6);
end
for c=1:length(toCompare)
    [data.(toCompareNames{c}),estimator,input,~]=readExperiment(toCompare{c},compareDatasetOptions);
    dataFields=fieldnames(data.(toCompareNames{c}));
    if ~ismember('temperature',dataFields)
        withTemperature=false;
    else
        withTemperature=true;
    end
    
    %% Calculate offsets for each secondary matrix for each comparison dataset
    % compute offset or store known offset
    % workbench does not currently have a known offset
    offset.(toCompareNames{c}).(names2use{1})=calculateOffsetUsingWBD(estimator,data.(toCompareNames{c}),sampleInit(c),sampleEnd(c),input,secMat.(names2use{1}));
    % for all other matrices
    for i=2:length(names2use)
        calculatedOffset=calculateOffsetUsingWBD(estimator,data.(toCompareNames{c}),sampleInit(c),sampleEnd(c),input,secMat.(names2use{i}));
        if useKnownOffset
            sensorFieldNames=fieldnames(calculatedOffset);
            for sensor=1:length(sensorFieldNames)
                sIndx= find(strcmp(sensorsToAnalize,sensorFieldNames(sensor)));
                if(isempty(sIndx)) || isempty(offsets.(names2use{i}).(sensorFieldNames{sensor})) % TODO: probably the sIndx condition will be removed
                    offsetToUse.(sensorFieldNames{sensor})=calculatedOffset.(sensorFieldNames{sensor});
                else
                    offsetToUse.(sensorFieldNames{sensor})=offsets.(names2use{i}).(sensorFieldNames{sensor});
                end
            end
        else
            offsetToUse=calculatedOffset;
        end
        offset.(toCompareNames{c}).(names2use{i})=offsetToUse;
    end
    
    [data.(toCompareNames{c}),~]= dataSampling(data.(toCompareNames{c}),reduceBy);
    
    
    for i=1:length(names2use)
        sMat.(sensorsToAnalize{j})=secMat.(names2use{i}).(sensorsToAnalize{j});% select specific secondary matrices
        offsetToUse=offset.(toCompareNames{c}).(names2use{i});
        %TODO: use or estimation of offsets
        
        
        if ~withTemperature
            [reCalibData,offsetInWrenchSpace,MSE,MSE_p]=calculateMatrixPerformance(data.(toCompareNames{c}),sensorsToAnalize,cMat.(names2use{i}),checkMatrixOptions);
        else
          checkMatrixOptions.otherCoeffValAsOffset= extraCoeffOffset.(names2use{i}).(sensorsToAnalize{j});
          [reCalibData,offsetInWrenchSpace,MSE,MSE_p]=calculateMatrixPerformance(data.(toCompareNames{c}),sensorsToAnalize,cMat.(names2use{i}),offsetToUse,checkMatrixOptions,'otherCoeff',extraCoeff.(names2use{i}),'varName','temperature','rawOffset',false);
                      
        end
        
        MSEvalues(c,i,:)=MSE.(sensorsToAnalize{j});
        MSEvalues_p(c,i,:)=MSE_p.(sensorsToAnalize{j});
        if ~useKnownOffset
            offsetValues(c,i,:)=offsetInWrenchSpace.(sensorsToAnalize{j});
        end
        clear results;
        clear calibMatrices;
    end
    
end
%% Comparison

% Calculate performance index


%% Save
mseValuesArray=struct2array(MSEvalues);
mseValues=reshape(mseValuesArray,length(sensorsToAnalize)*6,length(mseValuesArray)/(length(sensorsToAnalize)*6));
mseValues=mseValues'; % each 6 columns is a sensor

mseValuesArray_p=struct2array(MSEvalues_p);
mseValues_p=reshape(mseValuesArray_p,length(sensorsToAnalize)*6,length(mseValuesArray_p)/(length(sensorsToAnalize)*6));
mseValues_p=mseValues_p'; % each 6 columns is a sensor

offsetValuesArray=struct2array(offsetValues);
offsetValuesArray=offsetValuesArray';
if length(sensorsToAnalize)>1
    OffsetValues=[offsetValuesArray(1:2:end,:) offsetValuesArray(2:2:end,:)];
end
% each 6 columns is a sensor


fullNames=names2use;
if(scriptOptions.saveResults )
    allResults.fullNames=fullNames;
    for sta=1:length(sensorsToAnalize)
        sensor=sensorsToAnalize{sta};
        allResults.MSEvalues.(sensor)=mseValues(:,(sta-1)*6+1:sta*6);
        allResults.MSEvalues_p.(sensor)=mseValues_p(:,(sta-1)*6+1:sta*6);
        allResults.offsetValues.(sensor)=OffsetValues(:,(sta-1)*6+1:sta*6);
    end
    
    %     allResults.MSEvalues=MSEvalues;
    %     allResults.offsetValues=offsetValues;
    allResults.experimentNames=experimentNames;
    allResults.toCompare=toCompare;
    if strfind(pwd,'testResults')>0
        prefix='../';
    else
        prefix='';
        
    end
    if ~exist(strcat(prefix,'data/generalResults'),'dir')
        mkdir(strcat(prefix,'data/generalResults'));
        
    end
    save(strcat('data/generalResults/results_',date,'_',input.robotName,'.mat'),'allResults')
end