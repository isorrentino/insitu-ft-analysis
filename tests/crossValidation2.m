%% Test calibration matrices obtained from different datasets
%use from test directory
%load calibration matrices to compare with workbench calibration matrix
% clear all;
addpath ../utils
addpath ../external/quadfit

%serialNumber='SN191';%from iCubGenova05
%sensorName='r_foot_ft_sensor';

 serialNumber='SN192';%from iCubGenova05
 sensorName='r_leg_ft_sensor';
%Use only datasets where the same sensor is used
% experimentNames={
%     'icub-insitu-ft-analysis-big-datasets/16_03_2016/leftRightLegsGrid';...% Name of the experiment;
%     'icub-insitu-ft-analysis-big-datasets/21_03_2016/yogaLeft1';...% Name of the experiment;
%     'icub-insitu-ft-analysis-big-datasets/2016_04_21/extendedYoga4StandingOnLeft';...% Name of the experiment;
%     }; %this set is from iCubGenova02
experimentNames={
%    'icub-insitu-ft-analysis-big-datasets/2016_06_08/yoga';% Name of the experiment;
%    'icub-insitu-ft-analysis-big-datasets/2016_06_17/normal';% Name of the experiment;
%    'icub-insitu-ft-analysis-big-datasets/2016_06_17/fast';% Name of the experiment;
    'icub-insitu-ft-analysis-big-datasets/2016_07_04/normal';% Name of the experiment;
%    'icub-insitu-ft-analysis-big-datasets/2016_07_04/fast';% Name of the experiment;
    'icub-insitu-ft-analysis-big-datasets/2016_07_05/gridMin30';% Name of the experiment;
    'icub-insitu-ft-analysis-big-datasets/2016_07_05/gridMin45'% Name of the experiment;
    };%this set is form iCubGenova05
names2use={'Workbench';
%    'Yoga';
%    'Yogapp1st';
%    'fastYogapp';
    'Yogapp2nd';
%    'fastYogapp2';
    'gridMin30';
    'gridMin45'};% except for the first one all others are short names for the expermients in experimentNames
%toCompareWith='Yogapp2nd'; %choose in which experiment will comparison be made

scriptOptions.matFileName='ftDataset';
scriptOptions.printAll=false;
%% Load matrices and rawData
cMat.(names2use{1}) = readCalibMat(strcat('../data/sensorCalibMatrices/matrix_',serialNumber,'.txt'));

for i=1:length(experimentNames)
    cMat.(names2use{i+1}) = readCalibMat(strcat('../data/',experimentNames{i},'/calibrationMatrices/',serialNumber));
    load(strcat('../data/',experimentNames{i},'/',scriptOptions.matFileName,'.mat'),'dataset');
    [data.(names2use{i+1})]=dataset;
    
end
for toCompare=2:length(names2use)
    toCompareWith=names2use{toCompare}; %choose in which experiment will comparison be made
    
    inertiaOffset=0;
    nametemp=fieldnames(data.(toCompareWith));
    if(sum(strcmp(nametemp,'inertial'))==1)
        inertiaOffset=1;
    end
    
    index=0;
    
    for in=1:length(data.(names2use{2}).calibMatFileNames)
        if( find(strcmp(serialNumber, data.(names2use{2}).calibMatFileNames{in})))
            index=in;
        end
    end

if (index~=0)
    ftNames= fieldnames( data.(names2use{2}).ftData);
    sensorsToAnalize=ftNames{index};
    
    for i=1:length(names2use)
        if (inertiaOffset==0)
            for j=1:size(data.(toCompareWith).rawData.( sensorsToAnalize),1)
                reCalibData.(names2use{i})(j,:)=cMat.(names2use{i})*(data.(toCompareWith).rawData.( sensorsToAnalize)(j,:)');
            end
            %remove offset
            
            ftDataNoOffset.(names2use{i})=removeOffset(reCalibData.(names2use{i}),data.(toCompareWith).estimatedFtData.(sensorsToAnalize));
        else
            [inerOffset,inertialRaw]=calculateOffset(sensorsToAnalize,data.(toCompareWith).inertial.ftData,data.(toCompareWith).inertial.estimatedFtData,data.(toCompareWith).cMat,cMat.(names2use{i}));
            for j=1:size(data.(toCompareWith).rawData.( sensorsToAnalize),1)
                reCalibData.(names2use{i})(j,:)=cMat.(names2use{i})*(data.(toCompareWith).rawData.( sensorsToAnalize)(j,:)');
                ftDataNoOffset.(names2use{i})(j,:)=  reCalibData.(names2use{i})(j,:)+inerOffset.(sensorsToAnalize)';
            end
            
        end
        
    end
    
    if(scriptOptions.printAll)
    for i=2:length(names2use)
        figure,plot3_matrix( ftDataNoOffset.(names2use{i})(:,1:3));hold on;
        plot3_matrix(data.(toCompareWith).estimatedFtData.(sensorsToAnalize)(:,1:3)); grid on;
        legend(names2use{i},'estimatedData','Location','west');
    title(strcat('Wrench space on ',toCompareWith));
        xlabel('F_{x}');
        ylabel('F_{y}');
        zlabel('F_{z}');
    end
    end
    figure,
    plot3_matrix(data.(toCompareWith).estimatedFtData.(sensorsToAnalize)(:,1:3)); grid on;  hold on;
    for i=1:length(names2use)
        plot3_matrix( ftDataNoOffset.(names2use{i})(:,1:3));hold on;
    end
    legend([{'estimatedData'};names2use],'Location','west');
    
    title(strcat('Wrench space on ',toCompareWith));
    xlabel('F_{x}');
    ylabel('F_{y}');
    zlabel('F_{z}');
    
    for i=1:length(names2use)
        error(toCompare-1,i)=sum(sum(abs(data.(toCompareWith).estimatedFtData.(sensorsToAnalize)-ftDataNoOffset.(names2use{i}))));
         errorXaxis(toCompare-1,i,:)=sum(abs(data.(toCompareWith).estimatedFtData.(sensorsToAnalize)-ftDataNoOffset.(names2use{i})));
    end
    
    clear ftDataNoOffset reCalibData
   
    
end

end
for i=1:size(error,1)
    errorP(i,:)=(error(i,:)/error(i,1));
    for axis=1:6
    errorPaxis(i,:,axis)=(errorXaxis(i,:,axis)/errorXaxis(i,1,axis));
    end
end
 totalerror=mean(errorP);
 %totalerrorXaxis=mean(errorPaxis,3);
 [minErr,minInd]=min(totalerror);
 fprintf('The calibration matrix with least error among all datasets is from %s , with a total of %d percentage on average \n',names2use{minInd}, minErr);
 sCalibMat=cMat.(names2use{minInd})/(cMat.Workbench);%calculate secondary calibration matrix 
 xmlStr=cMat2xml(sCalibMat,sensorName);% print in required format to use by WholeBodyDynamics
 bestCMat=cMat.(names2use{minInd});
 bestName=names2use{minInd};
 axisName={'fx','fy','fz','tx','ty','tz'};
 for axis=1:6
      totalerrorXaxis=mean(errorPaxis(:,:,axis));
      
     [minErr,minInd]=min(totalerrorXaxis);
     fprintf('The calibration matrix with least error on %s among all datasets is from %s , with a total of %d percentage on average \n',axisName{axis},names2use{minInd}, minErr);
     frankieMatrix(axis,:)=cMat.(names2use{minInd})(axis,:);
 end
 fCalibMat=frankieMatrix/(cMat.Workbench);%calculate secondary calibration matrix 
 xmlStrF=cMat2xml(fCalibMat,sensorName);% print in required format to use by WholeBodyDynamics
