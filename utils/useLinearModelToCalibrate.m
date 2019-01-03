function [calibMatrices,fullscale,offset,temperatureCoefficients]=useLinearModelToCalibrate(dataset,sensorsToAnalize,varargin)
%TODO: expand to use beyond only adding temperature to any amount of extra
%linear variables (check estimateCalibrationMatrix for reference)
%This function takes a dataset that is expected to have at least rawData
%and estimatedFtData fields. It uses the information in the dataset to
%calculate a calibration matrix for the sensors assuming a linear model
%% copy datset variables to local variables
rawData=dataset.rawData;
estimatedFtData=dataset.estimatedFtData;
dataFields=fieldnames(dataset);
outputSize = size(estimatedFtData.(sensorsToAnalize{1}),2);
inputSize= size(rawData.(sensorsToAnalize{1}),2);
%% Default values
useFilteredData=true;
useExtraSample=true;
extraSamplesAvailable=false;
withRegularization=true;
withTemperature=false;
temperatureOffset=false;
estimationType=1; %0 only insitu offset, 1 is shpere offset , 2 is no mean offset on main dataset, 3 is no mean offset on all dataset, 4 is oneshot
narin=0;
cMat=[];
lambda=0;
temperature=[];
tempOffset=0;
varInput={};
%% Check varargin for variables
for v=1:2:length(varargin)
    if(ischar(  varargin{v}))
        switch varargin{v}
            case {'previousCalibration','cMat','preCalib'}
                if (isstruct(varargin{v+1}))
                    vnames= fieldnames(varargin{v+1});
                    ok=true;
                    for sensIndex=1:length(sensorsToAnalize)
                        if ~ismember(sensorsToAnalize{sensIndex},vnames)
                            ok=false;
                        end
                    end
                    if (ismatrix(varargin{v+1}.(vnames{1})))
                        if ~(size(varargin{v+1}.(vnames{1}),1)==outputSize && size(varargin{v+1}.(vnames{1}),2)==inputSize) % check if dimensions are correct
                            ok=false;
                            warning('useLinearModelToCalibrate: matrix inerted is not the right dimensions, so is not a calibration matrix');
                        end
                    end
                    if ok
                        cMat=varargin{v+1};
                    else
                        warning('useLinearModelToCalibrate: incorrect struct, no previous calibration matrix available');
                    end
                else
                    warning('useLinearModelToCalibrate: Expected struct,  no previous calibration matrix.')
                end
            case {'lambda','Lambda','LAMBDA'}
                if isnumeric(varargin{v+1})
                    lambda=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected numeric, using default lambda value of 0.')
                end
            case {'withTemperature','withtemperature','temperature'}
                if logical(varargin{v+1})
                    withTemperature=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected logical, using default withTemperature value.')
                end
            case {'temperatureOffset','temperatureoffset','tempOffset'}
                if logical(varargin{v+1})
                    temperatureOffset=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected logical, using default withTemperature value.')
                end
            case {'useExtraSample','useextraSample','useextrasample','useSamples'}
                if logical(varargin{v+1})
                    useExtraSample=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected logical, using default useExtraSample value.')
                end
            case {'useFilteredData','usefilteredData'}
                if logical(varargin{v+1})
                    useFilteredData=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected logical, using default useFilteredData value.')
                end
            case {'estimationType','estimationtype','ESTIMATIONTYPE','type','eType'}
                if isnumeric(varargin{v+1})
                    estimationType=varargin{v+1};
                else
                    warning('useLinearModelToCalibrate: Expected numeric, using default estimation type of 1.')
                end
            case {'extraSample','extraSamples','eSample','eSamples'}
                if (isstruct(varargin{v+1}))
                    extraSample=varargin{v+1};
                    extraSamplesAvailable=true;
                else
                    extraSamplesAvailable=false;
                    warning('useLinearModelToCalibrate: Expected struct,  no extra samples.')
                end
            otherwise
                warning('useLinearModelToCalibrate: Unexpected option.')
        end
    end
end


%% extra logic
% if want to use one shot but no extra samples
if ~extraSamplesAvailable && estimationType==3
    estimationType=2;
end
if withTemperature
    if ~ismember('temperature',dataFields)
        withTemperature=false;
        warning('useLinearModelToCalibrate: expected temperature info but not available, calibration will be perform without temperature');
    else
        temperature=dataset.temperature;
    end
end
if isempty(cMat)
    withRegularization=false;
    warning('useLinearModelToCalibrate: no previous calibration matrix, calibration will be perform without regularization');
    
end
if useFilteredData
    if ~ismember('rawDataFiltered',dataFields)
        useFilteredData=false;
        warning('useLinearModelToCalibrate: expected filtered data info but not available, calibration will be perform using non filterd raw data');
    else
        rawData=dataset.rawDataFiltered;
    end
end
if estimationType>2
    varInput{narin+1}='estimateoffset';
    varInput{narin+2}=true;
    offsetIndex=narin+2;
    narin=narin+2;
end
% Stack names to use in stackLogic
if extraSamplesAvailable && useExtraSample
    fieldsToStack{1}='estimatedFtData';
    if useFilteredData
        fieldsToStack{2}='rawDataFiltered';
    else
        fieldsToStack{2}='rawData';
    end
    if withTemperature
        fieldsToStack{3}='temperature';
    end
end
if withRegularization
    varInput{narin+1}='cMat';
    varInput{narin+3}='lambda';
    varInput{narin+4}=lambda;
    cmatIndex=narin+2;
    narin=narin+4;
end
if withTemperature
    narin=narin+2;
    temperatureDataIndex=narin;
end
%% Call appropiate methods
for ftIdx =1:length(sensorsToAnalize)
    ft = sensorsToAnalize{ftIdx};
    if estimationType>2
        varInput{offsetIndex}=true;
    end
    if withRegularization
        varInput{cmatIndex}=cMat.(ft);
    end
    if withTemperature
        if any(strcmp(ft,fieldnames(temperature)))            
            if temperatureOffset
                tempOffset=temperature.(ft)(1);
            end
            varInput{temperatureDataIndex}=temperature.(ft)-tempOffset;
            varInput{temperatureDataIndex-1}='addLinearVariable';
        else
            varInput{temperatureDataIndex-1}='nothing';
            warning('useLinearModelToCalibrate: no temperature info for sensor %s',ft);
        end
    end
    %% first 3 options only insitu offset , estimation with insitu offset , offset in main dataset
    if estimationType<3
        if estimationType<2 %% calculate insitu offset
            offset.(ft)=estimateOffsetUsingInSitu(rawData.(ft), estimatedFtData.(ft)(:,1:3));
            rawToUse=rawData.(ft)+repmat(offset.(ft),size(rawData.(ft),1),1);
            expectedWrench=estimatedFtData.(ft);
        else %% prepare mean values for no mean offset
            meanFt=mean(rawData.(ft));
            meanEst=mean(estimatedFtData.(ft));
            rawToUse=rawData.(ft)-repmat(meanFt,size(rawData.(ft),1),1);
            expectedWrench=estimatedFtData.(ft)-repmat(meanEst,size(estimatedFtData.(ft),1),1);
        end
        if ( (estimationType==1 && ~extraSamplesAvailable) || estimationType>1)%% not only insitu offset
            [calibMatrices.(ft),fullscale.(ft),~,tempCoeff]=estimateCalibrationMatrix(rawToUse,expectedWrench,varInput{:});
            if estimationType==2
                if sum(tempCoeff)~=0
                    offsetInForce=meanEst'-(calibMatrices.(ft)*meanFt'+ tempCoeff*mean(temperature.(ft)-tempOffset));
                else
                    offsetInForce=meanEst'-calibMatrices.(ft)*meanFt';
                end
                offset.(ft)=calibMatrices.(ft)\offsetInForce;
            end
        end
    end
    %% no mean dataset
%     if estimationType==3
%         rawToUse=rawData.(ft);
%         expectedWrench=estimatedFtData.(ft);
%         [calibMatrices.(ft),fullscale.(ft),offsetInForce,tempCoeff]=...
%             estimateCalibrationMatrix(rawToUse,expectedWrench,varInput{:});
%         offset.(ft)=calibMatrices.(ft)\offsetInForce;
%         varInput{offsetIndex}=false;
%     end
    if estimationType<3
        %% correct dimensions of the offset if needed before use
        [rows,columns]=size(offset.(ft));
        if rows==6 && columns==1
            offset.(ft)=offset.(ft)';
        end
    end
    %% extra sample section
    if useExtraSample && extraSamplesAvailable && estimationType>0
        if withTemperature
            [calibrationRequired,stackedExpectedWrench,stackedRawtoUse, stackedTemperature]= stackLogic(dataset,ft,extraSample,fieldsToStack);
            if calibrationRequired %% insert temperature  in format for calibration
                varInput{temperatureDataIndex}= stackedTemperature-tempOffset;
                tempToUse=varInput{temperatureDataIndex};
            else
                tempToUse=temperature.(ft)-tempOffset;
            end
        else
            [calibrationRequired,stackedExpectedWrench,stackedRawtoUse]= stackLogic(dataset,ft,extraSample,fieldsToStack);
        end
        if calibrationRequired
            expectedWrench=stackedExpectedWrench;
            rawToUse=stackedRawtoUse;
        end
        if estimationType<3 %% offset is known, remove from the raw data
            if calibrationRequired
                rawToUse=stackedRawtoUse+repmat(offset.(ft),size(stackedRawtoUse,1),1);
            end
            [calibMatrices.(ft),fullscale.(ft),~,tempCoeff]=...
                estimateCalibrationMatrix(rawToUse,expectedWrench,varInput{:});
        else % estimate offset as well
            %             if  withTemperature %TODO: decide to keep this or not
            %                 varInput{narin+1}='previousOffset';
            %                 narin=narin+2;
            %                 prevOffsetIndex=narin;
            %                 varInput{prevOffsetIndex}=expectedWrench(1,:)'-cMat.(ft)*rawToUse(1,:)';
            %             end
            if estimationType==3
                if calibrationRequired
                    meanFt=mean(stackedRawtoUse);
                    meanEst=mean(stackedExpectedWrench);
                    rawToUse=stackedRawtoUse-repmat(meanFt,size(stackedRawtoUse,1),1);
                    expectedWrench=stackedExpectedWrench-repmat(meanEst,size(stackedExpectedWrench,1),1);
                else
                    meanFt=mean(rawData.(ft));
                    meanEst=mean(estimatedFtData.(ft));
                    rawToUse=rawData.(ft)-repmat(meanFt,size(rawData.(ft),1),1);
                    expectedWrench=estimatedFtData.(ft)-repmat(meanEst,size(estimatedFtData.(ft),1),1);
                end
                [calibMatrices.(ft),fullscale.(ft),~,tempCoeff]=estimateCalibrationMatrix(rawToUse,expectedWrench,varInput{:});
                if sum(tempCoeff)~=0
                    offsetInForce=meanEst'-(calibMatrices.(ft)*meanFt'+ tempCoeff*mean(tempToUse));
                else
                    offsetInForce=meanEst'-calibMatrices.(ft)*meanFt';
                end
            else % full one shot
                [calibMatrices.(ft),fullscale.(ft),offsetInForce,tempCoeff]=...
                    estimateCalibrationMatrix(rawToUse,expectedWrench,varInput{:});
            end
            offset.(ft)=calibMatrices.(ft)\offsetInForce;
            %% correct dimensions of the offset if needed after estimation
            [rows,columns]=size(offset.(ft));
            if rows==6 && columns==1
                offset.(ft)=offset.(ft)';
            end
        end
    end
    if sum(tempCoeff)~=0
        temperatureCoefficients.(ft)=tempCoeff;
    end
    clear rawToUse expectedWrench;
    if exist('meanEst','var')
        clear meanEst meanFt;
    end
end
if estimationType==0 % only interested in the insitu offset
    calibMatrices=[];
    fullscale=[];
end
if ~exist('temperatureCoefficients','var')
    temperatureCoefficients=[];
end