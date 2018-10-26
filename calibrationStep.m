%% Calibration matrix script
% estimate new calibration matrices
% assuming is run at the end of calibrateFTsensor, or after calibrateFTsensor
%% Calibration
narin=0;
varInput={};
if isstruct(extraSample)
    varInput{narin+1}='useExtraSample';
    varInput{narin+3}='extraSample';
    varInput{narin+2}=true;
    varInput{narin+4}=extraSample;
    narin=narin+4;
    extraSampleAvailable=true;
else
    extraSampleAvailable=false;
end
dataFields=fieldnames(dataset);
if calibOptions.useTemperature
    if ~ismember('temperature',dataFields)
        calibOptions.useTemperature=false;
        warning('calibrateAndCheck: expected temperature info but not available, calibration will be perform without temperature');
    else
        varInput{narin+1}='withTemperature';
        varInput{narin+2}=true;
        narin=narin+2;
        % temperature offset
        if calibOptions.temperatureOffset
            varInput{narin+1}='temperatureOffset';
            varInput{narin+2}=true;
            narin=narin+2;
        end
    end
end
[calibMatrices,fullscale,offset,temperatureCoeff]=useLinearModelToCalibrate(dataset,sensorsToAnalize,...
    'estimationType',calibOptions.estimateType,'cMat',dataset.cMat,'lambda',lambda,'useFilteredData',true,...
    varInput{:});
%% Write calibration matrices file
if(calibOptions.saveMat)
    names=fieldnames(dataset.ftData);
    if ~exist(strcat('data/',experimentName,'/calibrationMatrices'),'dir')
        mkdir(strcat('data/',experimentName,'/calibrationMatrices'));
    end
    for ftIdx =1:length(sensorsToAnalize)
        ft = sensorsToAnalize{ftIdx};
        ftIndex=find(strcmp(ft, names));
        if (round(dataset.cMat.(ft))==eye(6))
            calibOptions.firstTime=true;
        else
            calibOptions.firstTime=false;
        end
        if ~exist('estimationName','var')
            estimationName='';
        end
        if (calibOptions.firstTime)
            if (any(strcmp('calibOutputNames', fieldnames(input))))
                filename=strcat('data/',experimentName,'/calibrationMatrices/', input.calibOutputNames{ftIndex},lambdaName,estimationName);
            else
                prompt={'First time sensor:                 insert serial number or      desired sensor name'};
                name = 'Sensor name';
                defaultans = {'SN00001'};
                answer = inputdlg(prompt,name,[1 30],defaultans);
                if (~isempty(answer))
                    filename=strcat('data/',experimentName,'/calibrationMatrices/',answer{1},lambdaName);
                else
                    disp('Sensor name canceled, not saving calibration matrix');
                    filename=strcat('data/',experimentName,'/calibrationMatrices/',defaultans{1},lambdaName);
                    break;
                end
            end
        else
            
            filename=strcat('data/',experimentName,'/calibrationMatrices/',dataset.calibMatFileNames{ftIndex},lambdaName,estimationName);
        end
        firmwareMat=calibMatrices.(ft);
        full_scale=fullscale.(ft);
        if ~isempty(temperatureCoeff)
            offsetInForce=firmwareMat*offset.(ft)';
            if calibOptions.temperatureOffset
                allOffsets=[offsetInForce;dataset.temperature.(ft)(1)];
            else
                allOffsets=[offsetInForce;0];
            end
            writeCalibMat(firmwareMat, full_scale, filename,'extraCoeffs',temperatureCoeff.(ft),'offsets',allOffsets)
        else
            writeCalibMat(firmwareMat, full_scale, filename)
        end
    end
end