clear all
close all
clc
% add required folders for use of functions
addpath external/quadfit
addpath utils

%% read experiment related variables
% obtain data from all listed experiments, is better if all experiments
% belong to the same robot or sensors. This is not a hard constraint though

% experimentNames={ %iCubGenova04 Experiments
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid';% Name of the experiment;
%     '/green-iCub-Insitu-Datasets/2018_07_10_Grid_warm';% Name of the experiment;
%     '/green-iCub-Insitu-Datasets/2018_07_10_multipleTemperatures';% Name of the experiment;
%     };

experimentNames={ %iCubGenova02 experiments
    '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_Grid';% Name of the experiment;
    '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_left_yoga';% Name of the experiment;
    '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_right_yoga';% Name of the experiment;
    '/icub-insitu-ft-analysis-big-datasets/2018_09_07_ICRA/2018_09_07_MixedDataSets';% Name of the experiment;
    };
% read experiment options
readOptions = struct();
readOptions.forceCalculation=false;%false;
readOptions.raw=true;
readOptions.saveData=true;
readOptions.multiSens=true;
readOptions.matFileName='ftDataset'; % name of the mat file used for save the experiment data

readOptions.visualizeExp=false;
readOptions.printPlots=true;%true
%% Calibration related variables options
% Select sensors to calibrate the names are associated to the location of
% the sensor in the robot
% on iCub  {'left_arm','right_arm','left_leg','right_leg','right_foot','left_foot'};
sensorsToAnalize = {'left_leg','right_leg'};
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
estimationTypes=[1,1,3,3];
useTempBooleans=[0,1,0,1];

% lambdas=0;
% Create appropiate names for the lambda variables
for namingIndex=1:length(lambdas)
    if (lambdas(namingIndex)==0)
        lambdasNames{namingIndex}='';
    else
        lambdasNames{namingIndex}=strcat('_l',num2str(lambdas(namingIndex)));
    end
end
lambdasNames=lambdasNames';

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
%
combinationNumber=length(experimentNames)*length(lambdas)*length(estimationTypes);
fullNames{combinationNumber}='';
MSEvalues(combinationNumber)=struct();
offstetValues(combinationNumber)=struct();
for staIdx=1:length(sensorsToAnalize)
    fieldName=sensorsToAnalize{staIdx};
    MSEvalues(combinationNumber).(fieldName)=[0,0,0,0,0];
    offstetValues(combinationNumber).(fieldName)=[0,0,0,0,0];
end
%calibration options
calculate=true;
calibOptions.saveMat=true;
calibOptions.estimateType=1;%0 only insitu offset, 1 is insitu, 2 is offset on main dataset, 3 is oneshot offset on main dataset, 4 is full oneshot
calibOptions.useTemperature=true;
% checking options
checkMatrixOptions.plotForceSpace=false;
checkMatrixOptions.plotForceVsTime=false;
checkMatrixOptions.secMatrixFormat=false;
%%
counter=1;
for experimentIndex=1:length(experimentNames)
    [data.(strcat('e',num2str(experimentIndex))),~,input,data.(strcat('extra',num2str(experimentIndex)))]=readExperiment(experimentNames{experimentIndex},readOptions);
    
    if(calculate)
        dataset=data.(strcat('e',num2str(experimentIndex)));
        extraSample=data.(strcat('extra',num2str(experimentIndex)));
        experimentName=experimentNames{experimentIndex};
        datasetToUse=dataset;
        if isstruct(extraSample)
            extraSampleAvailable=true;
        else
            extraSampleAvailable=false;
        end
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
        for in=1:length(lambdas)
            lambda=lambdas(in);
            lambdaName=lambdasNames{in};
            for type=1:length(estimationTypes)
                calibOptions.estimateType=estimationTypes(type);%0 only insitu offset, 1 is insitu, 2 is offset on main dataset, 3 is oneshot offset on main dataset, 4 is full oneshot
                calibOptions.useTemperature=useTempBooleans(type);
                estimationName=estimationNames{type};
                disp(strcat(estimationName,lambdaName));
                % calibrate
                calibrationStep
                % check performance in the data set
                [reCalibData,offsetInWrenchSpace,MSE]=checkNewMatrixPerformance(datasetToUse,sensorsToAnalize,calibMatrices,offset,checkMatrixOptions,'otherCoeff',temperatureCoeff,'varName','temperature');
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
                    save(strcat('data/',experimentName,'/results/results',estimationName,lambdaName,'.mat'),'results')
                end
                fullNames{counter}=strcat('e',num2str(experimentIndex),estimationName,lambdaName);
                MSEvalues(counter)=MSE;
                offsetValues(counter)=offsetInWrenchSpace;
                counter=counter+1;
                clear results;
            end
        end
        clear dataset;
        clear reCalibData;
        clear extraSample;
        clear datasetToUse;
    end
end

% convert to matrix for applying math operations
mseValuesArray=struct2array(MSEvalues);
mseValues=reshape(mseValuesArray,length(sensorsToAnalize)*6,length(mseValuesArray)/(length(sensorsToAnalize)*6));
mseValues=mseValues'; % each 6 columns is a sensor

offsetValuesArray=struct2array(offsetValues);
offsetValuesArray=offsetValuesArray';
OffsetValues=[offsetValuesArray(1:2:end,:) offsetValuesArray(2:2:end,:)];
 % each 6 columns is a sensor


 fullNames=fullNames';
 if(saveResults)
     allResults.fullNames=fullNames;
     for sta=1:length(sensorsToAnalize)
         sensor=sensorsToAnalize{sta};
         allResults.MSEvalues.(sensor)=mseValues(:,(sta-1)*6+1:sta*6);
         allResults.offsetValues.(sensor)=OffsetValues(:,(sta-1)*6+1:sta*6);
     end
     
     %     allResults.MSEvalues=MSEvalues;
     %     allResults.offsetValues=offsetValues;
     allResults.experimentNames=experimentNames;
     if ~exist(strcat('data/generalResults'),'dir')
         mkdir(strcat('data/generalResults'));
     end
     save(strcat('data/generalResults/results_',date,'_',input.robotName,'.mat'),'allResults')
 end

