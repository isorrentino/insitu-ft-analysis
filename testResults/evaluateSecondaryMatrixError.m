%% Evaluate error
useMean=true; %select which means of evaluation should be considered is either mean or standard deviation.
plotResults=true;
for j=1:length(sensorsToAnalize) %why for each sensor? because there could be 2 sensors in the same leg
    for frN=1:length(framesToAnalize)
        
        
        for i=1:length(names2use)
            error.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=norm(mean(abs(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(:,1:3))));
            errorXaxis.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i,:)=mean(abs(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})));
            strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=std(mean(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(:,1:3)));
            strd_axis.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i,:)=std(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j}));
            % we probably want the mean of the standard deviations of the
            % forces during experiment. the lower the variability the
            % better
            
            
            %             for num=1:size(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForces.(framesToAnalize{frN}),1)
            %                 testStd(num)=std(stackedResults.(sensorsToAnalize{j}).(names2use{i}).externalForces.(framesToAnalize{frN})(num,:));
            %             end
            %             strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=mean(testStd);
            % it seems easier from matlab notation to apply the std of the
            % mean which I am not sure is the similar enough to what we
            % want
        end
        
        if ( std(error.(sensorsToAnalize{j}).(framesToAnalize{frN}))> 1*10^(-10))
            if useMean
                [minErrall,minIndall]=min(error.(sensorsToAnalize{j}).(framesToAnalize{frN}));
                fprintf('Matrix with least external force is from %s for %s when considering %s frame, with a total of %.5f N on average \n',names2use{minIndall},(sensorsToAnalize{j}),(framesToAnalize{frN}), minErrall);
                
            else
                [minErrall,minIndall]=min(strd.(sensorsToAnalize{j}).(framesToAnalize{frN}));
                fprintf('Matrix with least standard deviation is from %s for %s when considering %s frame, with a total of %.5f std \n',names2use{minIndall},(sensorsToAnalize{j}),(framesToAnalize{frN}), minErrall);
                
            end
            sCalibMat.(sensorsToAnalize{j})=cMat.(names2use{minIndall}).(sensorsToAnalize{j})/(WorkbenchMat.(sensorsToAnalize{j}));%calculate secondary calibration matrix
            bestCMat.(sensorsToAnalize{j})=cMat.(names2use{minIndall}).(sensorsToAnalize{j});
            bestName.(sensorsToAnalize{j})=names2use{minIndall};
            xmlStr.(sensorName{j})=cMat2xml(sCalibMat.(sensorsToAnalize{j}),sensorName{j});% print in required format to use by WholeBodyDynamics
           
            axisName={'fx','fy','fz','tx','ty','tz'};
            for axis=1:6
                if useMean
                    totalerrorXaxis=errorXaxis.(sensorsToAnalize{j}).(framesToAnalize{frN})(:,:,axis);
                    fprintf('Matrix with least external force on %s sensor evaluted on %s frame',(sensorsToAnalize{j}),(framesToAnalize{frN}));
                    
                else
                    totalerrorXaxis=strd_axis.(sensorsToAnalize{j}).(framesToAnalize{frN})(:,:,axis);
                    fprintf('Matrix with least variation on %s sensor evaluted on %s frame',(sensorsToAnalize{j}),(framesToAnalize{frN}));
                    
                end                              
                % select the calibration matrix with less error for this
                % axis
                [minErr,minInd]=min(totalerrorXaxis);
                if useMean
                    fprintf(' in %s is from %s , with a total of %.5f N or Nm on average \n',axisName{axis},names2use{minInd}, minErr);
                else
                    fprintf(' in %s is from %s , with a total of %.5f std \n',axisName{axis},names2use{minInd}, minErr);
                end
                
                frankieMatrix.(sensorsToAnalize{j})(axis,:)=cMat.(names2use{minInd}).(sensorsToAnalize{j})(axis,:);
                frankieData.(framesToAnalize{frN})(:,axis)=stackedResults.(sensorsToAnalize{j}).(names2use{minInd}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(:,axis);
            end
            fCalibMat.(sensorsToAnalize{j})=frankieMatrix.(sensorsToAnalize{j})/(WorkbenchMat.(sensorsToAnalize{j}));%calculate secondary calibration matrix
            xmlStrFrankie.(sensorName{j})=cMat2xml(fCalibMat.(sensorsToAnalize{j}),sensorName{j});% print in required format to use by WholeBodyDynamics
            
            if plotResults
                comparisonData.(framesToAnalize{frN})=stackedResults.(sensorsToAnalize{j}).Workbench.externalForces.(framesToAnalize{frN});
                newData.(framesToAnalize{frN})=stackedResults.(sensorsToAnalize{j}).(names2use{minIndall}).externalForces.(framesToAnalize{frN});
            end
            
            
        else
            fprintf('Effect of %s on %s frame is neglegible \n',(sensorsToAnalize{j}),(framesToAnalize{frN}));
        end
        
    end
end

for j=1:length(sensorName)
    disp('BEST over all')
    disp(xmlStr.(sensorName{j}))
    fprintf('\n');
    disp('BEST by axis ')
    disp( xmlStrFrankie.(sensorName{j})   )
end

if plotResults
    FTplots(newData,stackedResults.(sensorsToAnalize{j}).(names2use{minIndall}).eForcesTime,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime, comparisonData,'Best General');
    FTplots(frankieData,stackedResults.(sensorsToAnalize{j}).(names2use{minIndall}).eForcesTime,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime, comparisonData,'Best axis');
end