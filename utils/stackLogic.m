function [calibrationRequired,varargout]= stackLogic(dataset,ft,extraSample,fieldsToStack)
%% This function performs the logic to stack the extra samples only if required depending on the type of extra sample and the selected ft
%%
calibrationRequired=false;
extraSampleNames=fieldnames(extraSample);
dataFields=fieldnames(dataset);
%% check if fields are in the dataset
varoutnum=0;
for ftS=1:length(fieldsToStack)
    if ~ismember(fieldsToStack{ftS},dataFields)
        warning('stackLogic: %s not found among the dataset variables. Will be ignored',fieldsToStack{ftS});
    else
        varoutnum=varoutnum+1;
        varargout{varoutnum}=dataset.(fieldsToStack{ftS}).(ft);
        availableNames{varoutnum}=fieldsToStack{ftS};
    end
end
% go through all possible extra samples
for eSampleIDNum =1:length(extraSampleNames)
    eSampleID = extraSampleNames{eSampleIDNum};
    toStack=[];
    if (isstruct(extraSample.(eSampleID)))
        sampleFields=fieldnames(extraSample.(eSampleID));
        for eSamples=1:length(extraSample.(eSampleID))
            for varToStack=1:varoutnum
                if ~ismember((availableNames{varToStack}),sampleFields)
                    error('stackLogic: %s not found among the extra sample %s variables',(availableNames{varToStack}),(eSampleID));
                else
                    if (~strcmp(ft,'right_arm') && ~strcmp(ft,'left_arm')) %new samples that work mainly on legs
                        switch eSampleID
                            case 'right'
                                if (strcmp(ft,'right_leg') || strcmp(ft,'right_foot' )) % if calibrating right side use samples specific for the right
                                    toStack=extraSample.(eSampleID)(eSamples).(availableNames{varToStack});
                                    calibrationRequired=true;
                                end
                            case 'left'
                                if (strcmp(ft,'left_leg') || strcmp(ft,'left_foot') ) % if calibrating left side use samples specific for the left
                                    toStack=extraSample.(eSampleID)(eSamples).(availableNames{varToStack});
                                    calibrationRequired=true;
                                end
                            case 'Tz'
                                toStack=extraSample.(eSampleID)(eSamples).(availableNames{varToStack});
                                calibrationRequired=true;
                        end
                    end
                    % Only general extra sample can be considered for adding info to the arms, since in theory it could affect all sensors
                    if ( strcmp(eSampleID,'general' ))
                        toStack=extraSample.(eSampleID)(eSamples).(availableNames{varToStack});
                        calibrationRequired=true;
                    end
                    
                    %% stack them
                    if(isstruct(toStack))
                        varargout{varToStack}=[varargout{varToStack};toStack.(ft)];
                    end
                end
            end
        end
    end
end