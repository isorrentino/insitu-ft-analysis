%% Evaluate error
loadResult=true;
% convert from extForceResults structure to what is used here
if exist('stackedResults','var') && ~loadResult
    names2evaluate=names2use;
    selectEstTypes=1:length(estimationNames);
    selectDataset=(2:length(names))-1;
    selectLambdas=1:length(lambdasNames);
    timeLength=length(stackedResults.(sensorsToAnalize{1}).(names2evaluate{1}).eForcesTime);
    selectTimeSamples=1:timeLength;
else
    if ~exist ('extForceResults','var') % select result file
        [file,path] = uigetfile;
        load(strcat(path,file));
    end
    % convert info form extforce resutls to variables used
    stackedResults=extForceResults.results;
    names2use=extForceResults.names.names2use;
    cMat=extForceResults.cMat;
    offsets=extForceResults.offsets;
    extraCoeff=extForceResults.extraCoeff;
    extraCoeffOffset=extForceResults.extraCoeffOffset;
    WorkbenchMat=cMat.Workbench;
    lambdas=extForceResults.lambdas;
    lambdasNames=generateLambdaNames(extForceResults.lambdas);
    estimationNames=generateEstimationTypeNames(extForceResults.estimationTypes,extForceResults.useTempBooleans,extForceResults.useTempOffset);
    names=extForceResults.names.experimentNames;
%     sensorsToAnalize = {'right_leg'};  %load the new calibration matrices
%     framesToAnalize={'r_upper_leg'};
%     sensorName={'r_leg_ft_sensor'};
         sensorsToAnalize = {'left_leg'};  %load the new calibration matrices
    framesToAnalize={'l_upper_leg'};
    sensorName={'l_leg_ft_sensor'};
    
    % create selector matrix
    orderArrangement=[1 3 2];
    namesMatrix=reshape(names2use(2:end),length(estimationNames),length(lambdasNames),length(names)-1);
    namesMatrix=permute(namesMatrix,orderArrangement); % this makes so that the lambdas are in the last dimension.
    % select conditions of the evaluation
    selectEstTypes=1:length(estimationNames);
    selectDataset=(2:length(names))-1;
    selectLambdas=1:length(lambdasNames);
    timeLength=length(stackedResults.(sensorsToAnalize{1}).Workbench.eForcesTime);
    %selectTimeSamples=[1:20:3000,3001:timeLength];
    selectTimeSamples=[1:timeLength];
    
    % create names based on selected values
    names2check=namesMatrix(selectEstTypes,selectDataset,selectLambdas);
    names2evaluate=names2check(:); %typically
    names2evaluate=[names2use(1);names2evaluate];
    %% clear variables
    clear error errorXaxis strd strd_axisN
end
useMean=true; %select which means of evaluation should be considered is either mean or standard deviation.
plotResults=true;
for j=1:length(sensorsToAnalize) %why for each sensor? because there could be 2 sensors in the same leg
    for frN=1:length(framesToAnalize)
        
        
        for i=1:length(names2evaluate)
            error.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=mean(normOfRows(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,1:3)));
            errorXaxis.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i,:)=mean(abs(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,:)));
            strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=mean(std(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,1:3)));
            strd_axis.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i,:)=std(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,:));
%             % testing r_squared metric
% %             r_squared=1-(sum((y-predicted_y)^2)/(sum((y-avg_y)^2)=1-MSE/VAR;
% %             in our case predicted_y =0
% %             for the magnitudes
%             y=normOfRows(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,1:3));
%             MSE=sum(y.^2);
%             VAR=sum((y-mean(y)).^2);
%             r_squared.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=1-(MSE/VAR);
% %             for axis
%             y_axis=stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(selectTimeSamples,:);
%             MSE_axis=sum(y_axis.^2);
%             VAR_axis=sum((y_axis-mean(y_axis)).^2);
%             r_squared_axis.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i,:)=1-(MSE_axis./VAR_axis);           
%             
%             %
            % we probably want the mean of the standard deviations of the
            % forces during experiment. the lower the variability the
            % better
            
            
            %             for num=1:size(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForces.(framesToAnalize{frN}),1)
            %                 testStd(num)=std(stackedResults.(sensorsToAnalize{j}).(names2evaluate{i}).externalForces.(framesToAnalize{frN})(num,:));
            %             end
            %             strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(1,i)=mean(testStd);
            % it seems easier from matlab notation to apply the std of the
            % mean which I am not sure is the similar enough to what we
            % want
        end
        
        if ( std(error.(sensorsToAnalize{j}).(framesToAnalize{frN}))> 1*10^(-10))
            if useMean
                [minErrall,minIndall]=min(error.(sensorsToAnalize{j}).(framesToAnalize{frN}));
                fprintf('Matrix with least external force is from %s for %s when considering %s frame, with a total of %.5f N on average \n',names2evaluate{minIndall},(sensorsToAnalize{j}),(framesToAnalize{frN}), minErrall);
                
            else
                [minErrall,minIndall]=min(strd.(sensorsToAnalize{j}).(framesToAnalize{frN}));
                fprintf('Matrix with least standard deviation is from %s for %s when considering %s frame, with a total of %.5f std \n',names2evaluate{minIndall},(sensorsToAnalize{j}),(framesToAnalize{frN}), minErrall);
                
            end
            sCalibMat.(sensorsToAnalize{j})=cMat.(names2evaluate{minIndall}).(sensorsToAnalize{j})/(WorkbenchMat.(sensorsToAnalize{j}));%calculate secondary calibration matrix
            bestCMat.(sensorsToAnalize{j})=cMat.(names2evaluate{minIndall}).(sensorsToAnalize{j});
            bestName.(sensorsToAnalize{j})=names2evaluate{minIndall};
            bestExtraCoeff.(sensorsToAnalize{j})=extraCoeff.(names2evaluate{minIndall}).(sensorsToAnalize{j});
            bestExtraCoeffOffset.(sensorsToAnalize{j})=extraCoeffOffset.(names2evaluate{minIndall}).(sensorsToAnalize{j});
            bestOffset.(sensorsToAnalize{j})=offsets.(names2evaluate{minIndall}).(sensorsToAnalize{j});
            
            xmlStr.(sensorName{j})=cMat2xml(sCalibMat.(sensorsToAnalize{j}),sensorName{j});% print in required format to use by WholeBodyDynamics
            
            axisName={'fx','fy','fz','tx','ty','tz'};
            for axisN=1:6
                if useMean
                    totalerrorXaxis=errorXaxis.(sensorsToAnalize{j}).(framesToAnalize{frN})(:,:,axisN);
                    fprintf('Matrix with least external force on %s sensor evaluted on %s frame',(sensorsToAnalize{j}),(framesToAnalize{frN}));
                    
                else
                    totalerrorXaxis=strd_axis.(sensorsToAnalize{j}).(framesToAnalize{frN})(:,:,axisN);
                    fprintf('Matrix with least variation on %s sensor evaluted on %s frame',(sensorsToAnalize{j}),(framesToAnalize{frN}));
                    
                end
                % select the calibration matrix with less error for this
                % axis
                [minErr,minInd]=min(totalerrorXaxis);
                if useMean
                    fprintf(' in %s is from %s , with a total of %.5f N or Nm on average \n',axisName{axisN},names2evaluate{minInd}, minErr);
                else
                    fprintf(' in %s is from %s , with a total of %.5f std \n',axisName{axisN},names2evaluate{minInd}, minErr);
                end
                
                frankieMatrix.(sensorsToAnalize{j})(axisN,:)=cMat.(names2evaluate{minInd}).(sensorsToAnalize{j})(axisN,:);
                frankieCoeffs.(sensorsToAnalize{j})(axisN,:)=extraCoeff.(names2evaluate{minInd}).(sensorsToAnalize{j})(axisN,:);
                frankieCoeffsOffset.(sensorsToAnalize{j})(axisN,:)=extraCoeffOffset.(names2evaluate{minInd}).(sensorsToAnalize{j});
                if strcmp((names2evaluate{minInd}),'Workbench')
                    frankieOffset.(sensorsToAnalize{j})(axisN,:)=-99999;
                else
                frankieOffset.(sensorsToAnalize{j})(axisN,:)=offsets.(names2evaluate{minInd}).(sensorsToAnalize{j})(axisN,:);
                end
                frankieData.(framesToAnalize{frN})(:,axisN)=stackedResults.(sensorsToAnalize{j}).(names2evaluate{minInd}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j})(:,axisN);
            end
            fCalibMat.(sensorsToAnalize{j})=frankieMatrix.(sensorsToAnalize{j})/(WorkbenchMat.(sensorsToAnalize{j}));%calculate secondary calibration matrix
            xmlStrFrankie.(sensorName{j})=cMat2xml(fCalibMat.(sensorsToAnalize{j}),sensorName{j});% print in required format to use by WholeBodyDynamics
            
            if plotResults
                comparisonData.(framesToAnalize{frN})=stackedResults.(sensorsToAnalize{j}).Workbench.externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j});
                newData.(framesToAnalize{frN})=stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).externalForcesAtSensorFrame.(framesToAnalize{frN}).(sensorsToAnalize{j});
                % bar3 plot
                xBarNames=generateCalibrationFileNames(names(selectDataset+1),estimationNames(selectEstTypes));
                if useMean
                    extForceToPlot=reshape(error.(sensorsToAnalize{j}).(framesToAnalize{frN})(2:end),length(xBarNames),length(selectLambdas));
                    wkbenchErr=error.(sensorsToAnalize{j}).(framesToAnalize{frN})(1)*ones(1,length(selectLambdas));
                else
                    extForceToPlot=reshape(strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(2:end),length(xBarNames),length(selectLambdas));
                    wkbenchErr=strd.(sensorsToAnalize{j}).(framesToAnalize{frN})(1)*ones(1,length(selectLambdas));
                end
                extForceToPlot=[extForceToPlot;wkbenchErr];
                xBarNames{length(xBarNames)+1}='Workbench';
                figure,
                b=bar3(extForceToPlot);                
                set(gca,'YTick',1:length(xBarNames));
                set(gca,'YTickLabel',escapeUnderscores(   xBarNames));
                set(gca,'XTickLabel',lambdas');
                ylabel('Dataset+estimationType');
                xlabel('\lambda');
                zlabel('External Force (N)');
                title(strcat('Second Validation Procedure Results ',escapeUnderscores((sensorsToAnalize{j}))));
                colorbar;
                for k = 1:length(b)
                    zdata = b(k).ZData;
                    b(k).CData = zdata;
                    b(k).FaceColor = 'interp';
                end
                axis tight;
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
    OffsetFT=bestOffset.(sensorsToAnalize{j})'
    TempCoeffAndTempOff=[bestExtraCoeff.(sensorsToAnalize{j})',bestExtraCoeffOffset.(sensorsToAnalize{j})]
    disp('BEST by axis ')
    disp( xmlStrFrankie.(sensorName{j})   )
end

if plotResults
%     FTplots(comparisonData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime, newData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'Best General','byChannel');
%     FTplots(comparisonData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime,frankieData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'Best axis','byChannel');
    
%     FTplots(comparisonData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime, newData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'Best General','byChannel','USESAMPLES');
%     FTplots(comparisonData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime,frankieData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'Best axis','byChannel','USESAMPLES');
%      FTplots(newData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime,frankieData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'Best axis','byChannel','USESAMPLES');
   
h1= FTplots(comparisonData,stackedResults.(sensorsToAnalize{j}).Workbench.eForcesTime,'byChannel','USESAMPLES');
h2= FTplots(newData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'byChannel','USESAMPLES');
h3= FTplots(frankieData,stackedResults.(sensorsToAnalize{j}).(names2evaluate{minIndall}).eForcesTime,'byChannel','USESAMPLES');
mergeFTplots(h2,h3);
mergeFTplots(h1,h2,'thisLegendNames',{'workbench','bestOverAll','bestByAxis'});
    % 3D plot
    outputFolder='';
   fieldNamesh1=fieldnames(h1);
   for fnh=1:length(fieldNamesh1)
       fnameh1=fieldNamesh1{fnh};
%       saveas(h1.(fnameh1),[outputFolder,'/',fnameh1],'epsc') 
%       saveas(h1.(fnameh1),[outputFolder,'/',fnameh1],'png') 
        saveas(h1.(fnameh1),fnameh1,'png') 
       
   end
    
end
%% clear variables
% clear error errorXaxis strd strd_axis