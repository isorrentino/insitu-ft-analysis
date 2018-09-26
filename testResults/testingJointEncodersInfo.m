%% actual script 
readOptions = {};
readOptions.forceCalculation=true;%false;
readOptions.raw=true;
readOptions.saveData=false;
readOptions.filterData=false;
readOptions.estimateWrenches=false;
readOptions.multiSens=true;
readOptions.matFileName='ftDataset'; % name of the mat file used for save the experiment data
 
    % name and paths of the experiment files
    % change name to desired experiment folder
%    experimentName='/green-iCub-Insitu-Datasets/2018_07_10_multipleTemperatures';
   
   experimentName='/green-iCub-Insitu-Datasets/2018_07_10_Grid';
%    experimentName= '/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_Grid_2';% Name of the experiment;
% experimentName='icub-insitu-ft-analysis-big-datasets/iCubGenova04/exp_1/poleLeftRight';
% experimentName='/green-iCub-Insitu-Datasets/2018_07_10_LeftYogaWarm';
[datasetNoChange,estimator,input,extraSample]=readExperiment(experimentName,readOptions);
count=1;
for tests=4:10:50
dataset=datasetNoChange;
% filter data
N=2;
F=tests+1;
mask= false(length(dataset.qj),1);
for joint=1:size(dataset.qj,2)
y = dataset.dqj(:,joint);
        nrOfSamples = length(dataset.qj);
        [b,g] = sgolay(N,F);
        HalfWin  = ((F+1)/2) -1;        
        for n = (F+1)/2:nrOfSamples-(F+1)/2
            % Zeroth derivative (smoothing only)
            filteredQj(n,joint) = dot(g(:,1),y(n - HalfWin:n + HalfWin));
        end
end     
mask((F+1)/2:nrOfSamples-(F+1)/2)=1;
dataset.dqj=filteredQj;
% modify value
[filteredFtData,mask]=filterFtData(dataset.ftData);
        dataset=applyMask(dataset,mask);
        dataset.filteredFtData=applyMask(filteredFtData,mask);

        [estimatedDataset,intervalMask,contactFrame]=estimateDynamicsUsingIntervals(dataset,estimator,input,false);
        dataset=applyMask(dataset,intervalMask);
        dataset.estimatedFtData=estimatedDataset.estimatedFtData;
        dataset.contactFrame=contactFrame;
        
%         Evaluate
ft='right_leg';
    ftOffset.(ft)=mean(dataset.ftData.(ft)- dataset.estimatedFtData.(ft)) ;
    ftNoOffset.(ft)=dataset.ftData.(ft) -repmat(ftOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
 mse(count,:)=mean((ftNoOffset.(ft)-dataset.estimatedFtData.(ft)).^2);
   filteredOffset.(ft)=mean(dataset.filteredFtData.(ft)- dataset.estimatedFtData.(ft)) ;
    filteredNoOffset.(ft)=dataset.filteredFtData.(ft) -repmat(filteredOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
 mseFilt(count,:)=mean((filteredNoOffset.(ft)-dataset.estimatedFtData.(ft)).^2);
           FTplots(struct(strcat('measure',ft),ftNoOffset.(ft),strcat('estimate',ft),dataset.estimatedFtData.(ft)),dataset.time,'forcecomparison');
 data(count)=dataset;
 count=count+1;
%%
end
%     dqj_all=numericalDifferentiation(dataset.qj,timeDelta);
%     
%     dqj_all=numericalDifferentiation(filteredQj,timeDelta);    
%    ddqj_all=numericalDifferentiation(dqj_all,timeDelta);
% 
%   
%    
%     joint=8;
%     figure,   
% %     plot(rad2deg(dqj_all(:,joint))); hold on
% %      plot(rad2deg(dataset.dqj(:,joint))); hold on
% %     legend('numerical','state ext' )    
%      plot(rad2deg(dataset.dqj(:,joint))); hold on
%       plot(rad2deg(dqj_all(:,joint))); hold on
%     legend('state ext','numerical')
%     title(strcat(' dq joint ',{' '}, escapeUnderscores(dataset.jointNames(joint))));
%     axis tight      
%       
%     figure,   
% %     plot(rad2deg(ddqj_all(:,joint))); hold on
% %      plot(rad2deg(dataset.ddqj(:,joint)))
% %     legend('numerical','state ext' )
%      plot(rad2deg(dataset.ddqj(:,joint))); hold on
%       plot(rad2deg(ddqj_all(:,joint))); hold on
%     legend('state ext','numerical')
%     title(strcat(' ddq joint ',{' '}, escapeUnderscores(dataset.jointNames(joint))));
%     axis tight
%     
%     
%     filterFtData
%     
%     
% mean(difInV)
% difInV=(rad2deg(dqj_all-dataset.dqj));
% mean(difInA)
% difInA=(rad2deg(ddqj_all-dataset.ddqj));
% plot(difInA(:,1));
% 
% plot(difInV(:,1));
% 
% % filter data
% N=2;
% F=21;
% mask= false(length(dataset.qj),1);
% 
% y = dataset.qj(:,joint);
%         nrOfSamples = length(dataset.qj);
%         [b,g] = sgolay(N,F);
%         HalfWin  = ((F+1)/2) -1;        
%         for n = (F+1)/2:nrOfSamples-(F+1)/2
%             % Zeroth derivative (smoothing only)
%             for joints=1:size(dataset.qj,2)
%             filteredQj(n,joints) = dot(g(:,1),y(n - HalfWin:n + HalfWin));
%             end
%         end
%         
% mask((F+1)/2:nrOfSamples-(F+1)/2)=1;