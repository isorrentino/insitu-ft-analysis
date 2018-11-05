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
   
   experimentName='/green-iCub-Insitu-Datasets/2018_07_10_LeftYoga';
%    experimentName= '/icub-insitu-ft-analysis-big-datasets/2018_09_07/2018_09_07_Grid_2';% Name of the experiment;
% experimentName='/green-iCub-Insitu-Datasets/2018_07_10_LeftYogaWarm';
[datasetNoChange,estimator,input,extraSample]=readExperiment(experimentName,readOptions);
count=1;

timeDelta=datasetNoChange.time(2)-datasetNoChange.time(1);
polyWinVector=[4,6,4,6,8,4,8,12,12 ;
               15,15,25,25,25,35,35,35,45 ];
for tests=1:length(polyWinVector)
dataset=datasetNoChange;% filter data

mask= false(length(dataset.qj),1);
for joint=1:size(dataset.qj,2)
% y = dataset.dqj(:,joint);
        nrOfSamples = length(dataset.qj);
%         [b,g] = sgolay(N,F);
%         HalfWin  = ((F+1)/2) -1;        
%         for n = (F+1)/2:nrOfSamples-(F+1)/2
%             % Zeroth derivative (smoothing only)
%             filteredQj(n,joint) = dot(g(:,1),y(n - HalfWin:n + HalfWin));
%         end
        [filt,deriv]=sgolayFilterAndDerivate(polyWinVector(1,tests),polyWinVector(2,tests),2,dataset.qj(:,joint),timeDelta);%derivOrder,xRaw,dt)
        sfiltddq(:,joint)=deriv(:,2);
        sfiltdq(:,joint)=deriv(:,2);
end     
mask((polyWinVector(2,tests)+1)/2:nrOfSamples-(polyWinVector(2,tests)+1)/2)=1;
if tests==1
    dataset.ddqj=zeros(size(sfiltddq));
else
dataset.ddqj=sfiltddq;
end
% ddqj_all=numericalDifferentiation(filteredQj,timeDelta);
% modify value

       [filteredFtData,mask]=filterFtData(dataset.ftData);
        dataset=applyMask(dataset,mask);
        dataset.filteredFtData=applyMask(filteredFtData,mask);

        [estimatedDataset,intervalMask,contactFrame]=estimateDynamicsUsingIntervals(dataset,estimator,input,false);
        
        dataset=applyMask(dataset,mask);
         dataset=applyMask(dataset,intervalMask);
         dataset.estimatedFtData=estimatedDataset.estimatedFtData;
        [filteredEstimatedFtData,mask]=filterFtData(dataset.estimatedFtData);
        dataset=applyMask(dataset,mask);
        filteredEstimatedFtData=applyMask(filteredEstimatedFtData,mask);
        
        dataset.contactFrame=contactFrame;
        
%         Evaluate
ft='right_leg';
    ftOffset.(ft)=mean(dataset.ftData.(ft)- dataset.estimatedFtData.(ft)) ;
    ftNoOffset.(ft)=dataset.ftData.(ft) -repmat(ftOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
 mse(count,:)=mean((ftNoOffset.(ft)-dataset.estimatedFtData.(ft)).^2);
   filteredOffset.(ft)=mean(dataset.filteredFtData.(ft)- filteredEstimatedFtData.(ft)) ;
    filteredNoOffset.(ft)=dataset.filteredFtData.(ft) -repmat(filteredOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
 mseFilt(count,:)=mean((filteredNoOffset.(ft)-filteredEstimatedFtData.(ft)).^2);
     %      FTplots(struct(strcat('measure',ft),ftNoOffset.(ft),strcat('estimate',ft),filteredEstimatedFtData.(ft)),dataset.time,'forcecomparison');
 data(count)=dataset;
 count=count+1;
%%
end

%% posproccesing tests

dataset=data(1);


[filteredNoFilt,mask]=filterFtData(data(1).estimatedFtData);
dataset=applyMask(dataset,mask);
filteredNoFilt=applyMask(filteredNoFilt,mask);

ftOffset.(ft)=mean(dataset.ftData.(ft)- dataset.estimatedFtData.(ft)) ;
ftNoOffset.(ft)=dataset.ftData.(ft) -repmat(ftOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
filteredOffset.(ft)=mean(dataset.filteredFtData.(ft)- filteredNoFilt.(ft)) ;
filteredNoOffset.(ft)=dataset.filteredFtData.(ft) -repmat(filteredOffset.(ft),size(dataset.filteredFtData.(ft),1),1);
%Plot
n=6;
[filteredEst,mask]=filterFtData(data(n).estimatedFtData);
filteredEst=applyMask(filteredEst,mask);

FTplots(struct(strcat('noFIlt'),data(1).estimatedFtData.(ft),strcat('pol',num2str( polyWinVector(1,n)),'_win', num2str( polyWinVector(2,n))),data(n).estimatedFtData.(ft)),data(1).time,'forcecomparison');

FTplots(struct(strcat('noFIlt'),filteredNoFilt.(ft),strcat('pol',num2str( polyWinVector(1,n)),'_win', num2str( polyWinVector(2,n))),filteredEst.(ft)),dataset.time,'forcecomparison');
hold on;
plot(dataset.time-dataset.time(1),filteredNoOffset.(ft)(:,1:3),'.')
l=legend;
legend2=l.String;
legend2{7}='F_{x_3}';
legend2{8}='F_{y_3}';
legend2{9}='F_{z_3}';

legend(legend2)
legendmarkeradjust(20)
