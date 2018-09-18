%TODO: possible to move dave to the end and everything else inside the same ft loop to make it into
%a function that receives the data directly and doesnt need to deal with
%the sensors themselves
function [reCalibData,offsetInWrenchSpace,varargout]=checkNewMatrixPerformance(datasetToUse,sensorsToAnalize,calibMatrices,offset,checkMatrixOptions,varargin)
%% Check required fields in dataset are there
dataFields=fieldnames(datasetToUse);
otherCoeff=[];
otherCoeffVarName={};
isRawOffset=true;
if (~any(strcmp('rawData', dataFields)))
    error(' %s field required','rawData');
end
if (~any(strcmp('estimatedFtData', dataFields)))
    error(' %s field required','estimatedFtData');
end
if (~any(strcmp('filteredFtData', dataFields)))
    error(' %s field required','filteredFtData');
end
if (~any(strcmp('time', dataFields)))
    error(' %s field required','time');
end
if (~any(strcmp('cMat', dataFields)))
    error(' %s field required','cMat');
end
%% Default values for checkMatrixOptions
optionsFieldNames=fieldnames(checkMatrixOptions);
if (~any(strcmp('plotForceSpace', optionsFieldNames)))
    checkMatrixOptions.plotForceSpace=false;
    disp('checkNewMatrixPerformance: Using default value plotForceSpace=false');
end
if (~any(strcmp('plotForceVsTime', optionsFieldNames)))
    checkMatrixOptions.plotForceVsTime=false;
    disp('checkNewMatrixPerformance: Using default value plotForceVsTime=false');
end
if (~any(strcmp('secMatrixFormat', optionsFieldNames)))
    checkMatrixOptions.secMatrixFormat=false;
    disp('checkNewMatrixPerformance: Using default value secMatrixFormat=false');
end
if (~any(strcmp('resultEvaluation', optionsFieldNames)))
    checkMatrixOptions.resultEvaluation=true;
    disp('checkNewMatrixPerformance: Using default value resultEvaluation=true');
end
%% Check varargin logic
for v=1:2:length(varargin)
    if(ischar(  varargin{v}))
        switch varargin{v}
            case {'otherCoeff'}
                if (isstruct(varargin{v+1}))
                    vnames= fieldnames(varargin{v+1});
                    ok=true;
                    for sensIndex=1:length(sensorsToAnalize)
                        if ~ismember(sensorsToAnalize{sensIndex},vnames)
                            ok=false;
                        end
                    end
                    if ok
                        if v+3<=length(varargin) % needs 3 more options in varargin to check this
                            vtoCheck=varargin{v+2};
                            vtoCheckValue=varargin{v+3};
                            if sum(strcmp(vtoCheck,{'otherCoeffFieldName','coeffName','varName'}))>0
                                if (~any(strcmp(vtoCheckValue, dataFields)))
                                    ok=false;
                                    error(' %s field required',vtoCheckValue);
                                else
                                    otherCoeffVarName=vtoCheckValue;
                                    otherCoeff=varargin{v+1};
                                end
                            end
                        end
                        
                    end
                end
            case {'offsetIsInRawSpace','OffsetIsRaw','offsetisraw','OFFSETISRAW','rawOffset','isRawOffset'}
                if islogical(tempV)
                    isRawOffset=tempV;
                else
                    warning('checkNewMatrixPerformance: expected a boolean variable for option OffsetIsRaw, ignoring');
                end
        end
    end
end
% for each sensor to analize
for ftIdx =1:length(sensorsToAnalize)
    ft = sensorsToAnalize{ftIdx};
    %% Generate wrenches with new calibration matrix
    recabnarin=0;
    recabInput={};
    if ~isempty(otherCoeff)
        recabInput{recabnarin+1}='addLinVarVal';
        recabInput{recabnarin+3}='varCoeff';
        recabInput{recabnarin+2}=datasetToUse.(otherCoeffVarName).(ft);
        recabInput{recabnarin+4}=otherCoeff.(ft);
        recabnarin=recabnarin+4;
    end
    [reCalibData.(ft),offsetInWrenchSpace.(ft)]=recalibrateData(datasetToUse.rawData.(ft),calibMatrices.(ft),...
        'offset',offset.(ft),'isRawOffset',isRawOffset,recabInput{:});
    % take out offset in the filtered data for comparison
    %             filteredOffset.(ft)=(datasetToUse.cMat.(ft)*offset.(ft)')';
    filteredOffset.(ft)=mean(datasetToUse.filteredFtData.(ft)- datasetToUse.estimatedFtData.(ft)) ;
    filteredNoOffset.(ft)=datasetToUse.filteredFtData.(ft) -repmat(filteredOffset.(ft),size(datasetToUse.filteredFtData.(ft),1),1);
    
    
    %% Plotting section
    % plot 3D graph
    if (checkMatrixOptions.plotForceSpace)
        if (round(datasetToUse.cMat.(ft))==eye(6))
            namesdatasets={'estimatedData','reCalibratedData'};
            force3DPlots(namesdatasets,(ft),datasetToUse.estimatedFtData.(ft),reCalibData.(ft));
        else
            namesdatasets={'measuredDataNoOffset','estimatedData','reCalibratedData'};
            force3DPlots(namesdatasets,(ft),filteredNoOffset.(ft),datasetToUse.estimatedFtData.(ft),reCalibData.(ft));
        end
        legendmarkeradjust(20);
    end
    % plot forces with time as x axis
    if(checkMatrixOptions.plotForceVsTime)
        FTplots(struct(ft,reCalibData.(ft),strcat('estimate',ft),datasetToUse.estimatedFtData.(ft)),datasetToUse.time,'forcecomparison');
        %FTplots(struct(strcat('measure',ft),filteredNoOffset.(ft),strcat('estimate',ft),modifiedDataset.estimatedFtData.(ft)),modifiedDataset.time,'forcecomparison');
        %FTplots(struct(ft,reCalibData.(ft),strcat('measure',ft),filteredNoOffset.(ft)),modifiedDataset.time,'forcecomparison');
    end
    
    %%  secondary matrix format
    if (checkMatrixOptions.secMatrixFormat)
        secMat.(ft)= calibMatrices.(ft)/datasetToUse.cMat.(ft);
        xmlStr=cMat2xml(secMat.(ft),ft)% print in required format to use by WholeBodyDynamics
    end
    
    %% Evaluation of results
    if (checkMatrixOptions.resultEvaluation)
        %disp(ft)
        Workbench_no_offset_mse=mean((filteredNoOffset.(ft)-datasetToUse.estimatedFtData.(ft)).^2);
        New_calibration_no_offset_mse.(ft)=mean((reCalibData.(ft)-datasetToUse.estimatedFtData.(ft)).^2);
        MSE_percentage.(ft)=(1-(New_calibration_no_offset_mse.(ft)./Workbench_no_offset_mse))*100;
    end
end
if exist('New_calibration_no_offset_mse','var')
    varargout{1}= New_calibration_no_offset_mse;
    varargout{2}= MSE_percentage;
end

