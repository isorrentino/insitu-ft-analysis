%% Comparing FT data vs estimated data
%
% %create input parameter
% % input.experimentName='dumperRightLegNoIMU';% Name of the experiment
% input.ftPortName='analog:o'; % (arm, foot and leg have FT data)
% input.statePortName='stateExt:o'; % (only foot has no state data)
% input.ftNames={'left_arm';'right_arm';'left_leg';'right_leg';'left_foot';'right_foot'}; %name of folders that contain ft measures
% sensorNames={'l_arm_ft_sensor'; 'r_arm_ft_sensor'; 'l_leg_ft_sensor'; 'r_leg_ft_sensor'; 'l_foot_ft_sensor'; 'r_foot_ft_sensor';};
% input.sensorNames=sensorNames; %make sensor names match the order of the names of the folders
%
% %input.stateNames={'head','left_arm','right_arm','left_leg','right_leg','torso'}; % name of the folders that contain state information
% %DoF=[6,16,6,16,6,3];% degrees of freedom in the same order of dataStateDirs (head, left_arm _leg, right_arm _leg, torso)
% head='head'; value1={'neck_pitch';'neck_roll';'neck_yaw';'eyes_tilt';'eyes_tilt';'eyes_tilt'};
% left_arm='left_arm'; value2={'l_shoulder_pitch';'l_shoulder_roll';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_shoulder_yaw';'l_hand_finger';...
%     'l_thumb_oppose';'l_thumb_proximal';'l_thumb_distal';'l_index_proximal';'l_index_distal';'l_middle_proximal';'l_middle_distal';' l_pinky'};
% left_leg='left_leg'; value3={'l_hip_pitch';'l_hip_roll';'l_hip_yaw';'l_knee';'l_ankle_pitch';'l_ankle_roll'};
% right_arm='right_arm'; value4={'r_shoulder_pitch';'r_shoulder_roll';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_shoulder_yaw';'r_hand_finger';...
%     'r_thumb_oppose';'r_thumb_proximal';'r_thumb_distal';'r_index_proximal';'r_index_distal';'r_middle_proximal';'r_middle_distal';' r_pinky'};
% right_leg='right_leg'; value5={'r_hip_pitch';'r_hip_roll';'r_hip_yaw';'r_knee';'r_ankle_pitch';'r_ankle_roll'};
% torso='torso'; value6={'torso_yaw';'torso_roll';'torso_pitch'};
%
% input.stateNames=struct(head,{value1},left_arm,{value2},left_leg,{value3},right_arm,{value4},right_leg,{value5},torso,{value6});
% input.robotName='iCubGenova02';

%%
%add required folders for use of functions
addpath external/quadfit
addpath utils
% name and paths of the data files

experimentName='icub-insitu-ft-analysis-big-datasets/16_03_2016/leftRightLegsGrid';% Name of the experiment;
paramScript=strcat('data/',experimentName,'/params.m');
run(paramScript)

if (exist(strcat('data/',experimentName,'/dataset.mat'),'file')==2)
    %% Load from workspace
    %     %load meaninful data, estimated data, meaninful data no offset
    load(strcat('data/',experimentName,'/dataset.mat'),'dataset')
    
else
    ftDataName=strcat(input.ftPortName,'/data.log'); % (arm, foot and leg have FT data)
    stateDataName=strcat(input.statePortName,'/data.log');  % (only foot has no state data)
    %params.m is expected to have contactInfo (1 right ,0 left ), relevant (if
    %there is an specific interval desired to study (1 true, 0 false ) and
    %rData which is a double array 1x2 that has begining and ending of interval in seconds
    
    
    for i=1:size(input.ftNames,1)
        dataFTDirs{i}=strcat('data/',experimentName,'/icub/',input.ftNames{i},'/',ftDataName);
        
    end
    stateNames=fieldnames(input.stateNames);
    for i=1:size(stateNames,1)
        dataStateDirs{i}=strcat('data/',experimentName,'/icub/',stateNames{i},'/',stateDataName);
        
    end
    %TODO: replace "icub" for robot model? so that it can be used for other
    %robots, although this is dependent on the output kind of the data dumper
    
    
    %% load FT data
    [ftData.(input.ftNames{1}),time]=readDataDumper(dataFTDirs{1});
    nanIndex=0;
    nanCount=0;
    for i=2:size(input.ftNames,1)
        %read from dataDumper
        [ftData_temp,time_temp]=readDataDumper(dataFTDirs{i});
        %resample FT data
        ftData.(input.ftNames{i})=resampleFt(time,time_temp,ftData_temp);
        %if the initial time of the time_temp is less than time it might return
        %NaN values for the those first values, so we will take into account
        %which has the biggest amount of nans and remove those values with
        %applyMask later
        if (sum(isnan(ftData.(input.ftNames{i})(:,1)))>nanCount)
            nanIndex=i;
            nanCount=sum(isnan(ftData.(input.ftNames{i})(:,1)));
        end
    end
    
    %% load state and calculate estimated wrenches for comparison
    [dataset]=obtainEstimatedWrenches(dataStateDirs,input.stateNames,input.robotName,time,contactFrameName);
    
    dataset.time=time;
    dataset.ftData=ftData;
    
    sensorNames=fieldnames(dataset.estimatedFtData);
    
    %match field names with sensor loaded through readDataDumper
    %
    matchup=zeros(size(input.sensorNames,1),1);
    for i=1:size(input.sensorNames,1)
        matchup(i) = find(strcmp(sensorNames, input.sensorNames{i}));
    end
    
    %replace the estored estimatedFtData for one with the same order as the
    %ftData
    for i=1:size(input.ftNames,1)
        estimatedFtData.(input.ftNames{i})=dataset.estimatedFtData.(sensorNames{matchup(i)});
    end
    dataset.estimatedFtData=estimatedFtData;
    
    if (relevant==1)
        mask=dataset.time>dataset.time(1)+rData(1) & dataset.time<dataset.time(1)+rData(2);
        dataset=applyMask(dataset,mask);
    end
    
    %simple visual exploration suggests an offset problem, this parts aims
    %to calculate the offset and then compare the data with the offset
    %removed
    
    %compute offset on meaningful data
    for i=1:size(input.ftNames,1)
        ftDataNoOffset.(input.ftNames{i})=removeOffset(dataset.ftData.(input.ftNames{i}),dataset.estimatedFtData.(input.ftNames{i}));
    end
    dataset.ftDataNoOffset=ftDataNoOffset;
    
    %% Save the workspace
    %     %save meaninful data, estimated data, meaninful data no offset
    save(strcat('data/',experimentName,'/dataset.mat'),'dataset')
end
%% Data exploration
% Plot ftDataNoOffset and/vs estimatedFtData
% for i=4:4
%     %     for i=1:size(input.ftNames,1)
%     FTplots(struct(input.ftNames{i},dataset.ftData.(input.ftNames{i}),strcat('estimated',input.ftNames{i}),dataset.estimatedFtData.(input.ftNames{i})),dataset.time);
% end

% Plot ftDataNoOffset and/vs estimatedFtData
% for i=4:4
%     %     for i=1:size(input.ftNames,1)
%     FTplots(struct(input.ftNames{i},dataset.ftDataNoOffset.(input.ftNames{i}),strcat('estimated',input.ftNames{i}),dataset.estimatedFtData.(input.ftNames{i})),dataset.time);
% end

% Plot forces in 3D space
% %with offset
% for i=4:4
%     %     for i=1:size(input.ftNames,1)
%     figure,plot3_matrix(dataset.ftData.(input.ftNames{i})(:,1:3));hold on;
%     plot3_matrix(dataset.estimatedFtData.(input.ftNames{i})(:,1:3)); grid on;
% end


%without offset
for i=4:4
    %     for i=1:size(input.ftNames,1)
    figure,plot3_matrix(dataset.ftDataNoOffset.(input.ftNames{i})(:,1:3));hold on;
    plot3_matrix(dataset.estimatedFtData.(input.ftNames{i})(:,1:3)); grid on;
end

% ellipsoid

%TODO:
%
% for i=4:4
%     %     %     for i=1:size(input.ftNames,1)
%     %     figure,plot3_matrix(dataset.ftDataNoOffset.(input.ftNames{i})(:,1:3));hold on;
%
%     %     plot3_matrix(dataset.estimatedFtData.(input.ftNames{i})(:,1:3)); grid on;
%
%     % ellipsoidfit(dataset.ftDataNoOffset.(input.ftNames{i})(:,1),dataset.ftDataNoOffset.(input.ftNames{i})(:,2),dataset.ftDataNoOffset.(input.ftNames{i})(:,3))
%     % ellipsoidfit_leastsquares(dataset.estimatedFtData.(input.ftNames{i})(:,1),dataset.estimatedFtData.(input.ftNames{i})(:,2),dataset.estimatedFtData.(input.ftNames{i})(:,3))
%     ft_raw=dataset.ftDataNoOffset.(input.ftNames{i})(:,1:3);
%     acc=dataset.estimatedFtData.(input.ftNames{i})(:,1:3);
%     ft_raw_no_mean = ft_raw-ones(size(ft_raw,1),1)*mean(ft_raw);
%     ft_raw_mean = mean(ft_raw);
%
%     [U_raw,S_ft_raw,V_raw] = svd(ft_raw_no_mean,'econ');
%     bar((S_ft_raw));
%     ft_raw_projector = V_raw(:,1:3)';
%     ft_raw_projected = (V_raw(:,1:3)'*ft_raw_no_mean')';
%     title('Raw sensor data singular values')
%
%     normalize = @(x) (x-ones(size(x,1),1)*mean(x))./(ones(size(x,1),1)*std(x));
%     normalize_isotropically = @(x) (x-ones(size(x,1),1)*mean(x))/mean(std(x));
%
%     % normalize data
%     ft_raw_projected_norm = normalize(ft_raw_projected);
%     %%
%     %Plotting ellipsoid fitted in raw space
%     fprintf(['Fitting ft ellipsoid\n']);
%     [p_ft_norm,ft_proj_norm_refitted]   = ellipsoidfit_smart(ft_raw_projected_norm,acc);
%     fprintf(['Fitting acc ellipsoid\n']);
%     p_acc  = ellipsoidfit(acc(1:end,1),acc(1:end,2),acc(1:end,3));
%
%     figure
%     plot_ellipsoid_im(p_ft_norm);
%     plot3_matrix(ft_raw_projected_norm(1:end,:))
%     axis equal
%     title('Ellipsoid fitted in FT raw space');
%
% end


%%Calibration matrix correction
% filtered ft data
[filteredFtData,mask]=filterFtData(dataset.ftData);

dataset2=applyMask(dataset,mask);
filterd=applyMask(filteredFtData,mask);
dataset2.filteredFtData=filterd;
% for i=4:4
%     %     for i=1:size(input.ftNames,1)
%     FTplots(struct(input.ftNames{i},filterd.(input.ftNames{i}),strcat('estimated',input.ftNames{i}),dataset2.estimatedFtData.(input.ftNames{i})),dataset2.time);
% end
%getting raw datat
[dataset2.rawData,cMat]=getRawData(dataset2.filteredFtData,input.calibMatPath,input.calibMatFileNames);
[calibMatrices,offset]=estimateMatrices(dataset2.rawData,dataset2.estimatedFtData);
eC=cMat.left_leg-calibMatrices.left_leg;

for i=4:4
    for j=1:size(dataset2.rawData.(input.ftNames{i}),1)
        reCalibData(j,:)=calibMatrices.(input.ftNames{i})*dataset2.rawData.(input.ftNames{i})(j,:)';
    end
end

for i=4:4
    %     for i=1:size(input.ftNames,1)
    figure,plot3_matrix(reCalibData(:,1:3));hold on;
    plot3_matrix(dataset.estimatedFtData.(input.ftNames{i})(:,1:3)); grid on;
end
