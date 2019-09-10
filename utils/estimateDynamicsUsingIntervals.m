function [dataset,endMask,contactFrame]=estimateDynamicsUsingIntervals(dataset,estimator,input,useInertial)
calculateAcc=false;
endMask=~logical(dataset.time);
contactFrame=cell(length(dataset.time),1);
%% Manage intervals
if (any(strcmp('intervals', fieldnames(input))))
    intervalsNames=fieldnames(input.intervals);
    if(~isempty(intervalsNames))
        if (any(strcmp('hanging', intervalsNames)) && isfield(dataset,'inertialData'))
            mask=dataset.time>=dataset.time(1)+input.intervals.hanging.initTime & dataset.time<=dataset.time(1)+input.intervals.hanging.endTime;
            fprintf('estimateDynamicsUsingIntervals: estimating interval hanging with contact frame %s from %d s to %d s \n',input.intervals.hanging.contactFrame,input.intervals.hanging.initTime,input.intervals.hanging.endTime);
            [inertialEstimatedFtData]=obtainEstimatedWrenches(estimator,dataset.time,{input.intervals.hanging.contactFrame},dataset,'mask',mask,'inertialdata',dataset.inertialData,'calculatedq',false,'calculateddq',calculateAcc);
            
            inertial.ftData=inertialEstimatedFtData.ftData;
            inertial.time=inertialEstimatedFtData.time;
            %inertial.mask=mask;
            
            sensorNames=fieldnames(inertialEstimatedFtData.estimatedFtData);
            % match field names with sensor loaded through readDataDumper
            %
            matchup=zeros(size(input.sensorNames,1),1);
            for i=1:size(input.sensorNames,1)
                matchup(i) = find(strcmp(sensorNames, input.sensorNames{i}));
            end
            %replace the estored estimatedFtData for one with the same order as the
            %ftData
            for i=1:size(input.ftNames,1)
                inertial.estimatedFtData.(input.ftNames{i})=inertialEstimatedFtData.estimatedFtData.(sensorNames{matchup(i)});
            end
            
        end
        %% Use not hanging intervals
        % generalize ordering of intervals
        for i=1:size(input.ftNames,1)
            estimatedFtData.(input.ftNames{i})=zeros(size(dataset.ftData.(input.ftNames{i})));
        end
        newOrdering=true;
        
        % extract sub-intervals in the case of walking datasets
        if (any(strcmp('type', fieldnames(input))))
            if (strcmp('walking',input.type))
                isLeftLegIntervDef = ismember('leftLeg',intervalsNames);
                isRightLegIntervDef = ismember('rightLeg',intervalsNames);
                if(isLeftLegIntervDef || isRightLegIntervDef)
                    [~, initStanceLeft, endStanceLeft, initStanceRight, endStanceRight] = getStancePeriodsFromWalkingFTdata(...
                        dataset.time, dataset.ftData.left_foot,...
                        dataset.time, dataset.ftData.right_foot,...
                        input.gait.minSwingLength, input.gait.tolerancePercentage, input.gait.shrinkPercentage);
                    % set the sub-intervals
                    if(isLeftLegIntervDef)
                        input.intervals.leftLeg=struct('initTime',initStanceLeft,'endTime',endStanceLeft,'contactFrame','l_sole');
                    end
                    if(isRightLegIntervDef)
                        input.intervals.rightLeg=struct('initTime',initStanceRight,'endTime',endStanceRight,'contactFrame','r_sole');
                    end
                end
            end
        end
        %   generalize ordering of intervals end first section
        for index=1:length(intervalsNames)
            
            if(~strcmp('hanging', intervalsNames{index}))
                intName=intervalsNames{index};
                mask=~logical(dataset.time);
                
                % process the intervals
                for timeIntervals=1:length(input.intervals.(intName).initTime)
                    tempMask=dataset.time>=dataset.time(1)+input.intervals.(intName).initTime(timeIntervals) & dataset.time<=dataset.time(1)+input.intervals.(intName).endTime(timeIntervals);
                    fprintf('estimateDynamicsUsingIntervals: estimating interval %s with contact frame %s from %d s to %d s \n',(intName),input.intervals.(intName).contactFrame,input.intervals.(intName).initTime(timeIntervals),input.intervals.(intName).endTime(timeIntervals));
                    mask=mask | tempMask;
                end
                
                %create a cell array with the contact frame used in each
                %time sample
                contactFrame(mask)={input.intervals.(intName).contactFrame};
                
                % create the endMask;
                endMask=endMask | mask;
                
                if (useInertial && isfield(dataset,'inertialData'))
                    disp('estimateDynamicsUsingIntervals: using floating base for estimation');
                    [dataset2]=obtainEstimatedWrenches(estimator,dataset.time,{input.intervals.(intName).contactFrame},dataset,'mask',mask,'inertialdata',dataset.inertialData,'calculatedq',false,'calculateddq',calculateAcc);
                else
                    disp('estimateDynamicsUsingIntervals: using fixed base for estimation');
                    [dataset2]=obtainEstimatedWrenches(estimator,dataset.time,{input.intervals.(intName).contactFrame},dataset,'mask',mask,'calculatedq',false,'calculateddq',calculateAcc);
                end
                sensorNames=fieldnames(dataset2.estimatedFtData);
                
                % match field names with sensor loaded through readDataDumper
                matchup=zeros(size(input.sensorNames,1),1);
                for i=1:size(input.sensorNames,1)
                    matchup(i) = find(strcmp(sensorNames, input.sensorNames{i}));
                end
                
                if (newOrdering)
                    % generalize intervals ordering section 2
                    
                    for i=1:size(input.ftNames,1)
                        estimatedFtData.(input.ftNames{i})(mask,:)=dataset2.estimatedFtData.(sensorNames{matchup(i)});
                    end
                    data.estimatedFtData=estimatedFtData;
                    % generalize intervals ordering section 2 end
                else
                    %replace the estored estimatedFtData for one with the same order and name as the
                    %ftData
                    for i=1:size(input.ftNames,1)
                        estimatedFtData.(input.ftNames{i})=dataset2.estimatedFtData.(sensorNames{matchup(i)});
                    end
                    dataset2.estimatedFtData=estimatedFtData;
                    
                    % deal with the ordering of the intervals
                    if (length(intervalsNames)==1)
                        data=dataset2;
                    else
                        if (any(strcmp('hanging', intervalsNames)) && strcmp('hanging', intervalsNames{index-1}))
                            data=dataset2;
                        else
                            if (input.intervals.(intervalsNames{index-1}).initTime<input.intervals.(intName).initTime)
                                data=addDatasets(data,dataset2);
                            else
                                data=addDatasets(dataset2,data);
                            end
                        end
                    end
                end
            end
        end
        dataset=data;
        if(newOrdering)
            dataset.contactFrame=contactFrame;
            dataset=applyMask(dataset,endMask);
        end
        contactFrame=contactFrame(endMask);
        if exist('inertial','var')%(any(strcmp('hanging', intervalsNames)) && isfield(dataset,'inertialData'))
            dataset.inertial=inertial;
        end
    else
        disp('estimateDynamicsUsingIntervals: intervals is empty avoiding all estimation');
    end
else
    disp('estimateDynamicsUsingIntervals: input.intervals needs to exist to estimate, no estimation has been done');
end