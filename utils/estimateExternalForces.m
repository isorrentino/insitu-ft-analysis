function [externalWrenches,time,jointTorques,externalWrenchesAtSensorFrame]= estimateExternalForces(robotName,dataset,secMat,sensorNames,contactFrameName,timeFrame,framesNames,offset,varargin)
%% This function will estimate the external forces using the measurements in the case of the desired sensor to analize and the estimated wrenches otherwise.
% The aim of doing this is to reduce the sources of uncertainty to only one
% sensor at a time.
% estimateExternalForces estimate external forces for a iCub dataset
%   estimator       : variable which contains the model of the robot
%   dataset         : contains the joint position velocities and
%   accelerations
%   resampleTime   : time that will be used as reference time
%   contactFrameName : the name of the frame on which it is assumed that an
%                      external contact is (tipically the
%                      root_link, r_sole or l_sole)

%% Check varargin
useInertial=false;
mask=[];
sensorsToAnalize=NaN;
useEstimatedData=true;
extraCoeff=NaN;
extraLinearVariables=NaN;
extraOffset=NaN;
numberOfSamples=length(dataset.time);
secNames=fieldnames(secMat);
outputSize=size(secMat.(secNames{1}),1);
extraCoeffDefaultValue=zeros(outputSize,1);
if(~isempty(varargin))
    skip=0;
    if mod(length(varargin),2)==0
        for v=1:2:length(varargin)
            if ~skip
                content=varargin{v+1};
                if(ischar(  varargin{v}))
                    switch lower(varargin{v})
                        case {'mask'}
                            if islogical(content)
                                if(size(mask)==size(dataset.time))
                                    mask=content;
                                else
                                    warning('estimateExternalForces:Mask is the wrong size, parameter ignored');
                                end
                            end
                        case {'sensorstoanalize','sensors to analize','analize only this sensors'}%'sensorsToAnalize'
                            if iscell(content)
                                sensorsToAnalize=content;
                            else
                                warning('estimateExternalForces: sensorsToAnalize should be a cell array, parameter ignored');
                            end
                        case {'inertial','inertialdata','inertial data'}
                            if isstruct(content) % it means inertial data is provided
                                inertialData=content;
                                inertialFields=fieldnames(inertialData);
                                if length(inertialFields)==2
                                    useInertial=true;
                                else
                                    warning('estimateExternalForces: Expected inertial data that has only 2 fields');
                                end
                            end
                        case {'extravar','extra variable','extralinearvariable','add linear variable','extra linear variable value','addlinvar','addextlinvarval'}
                            if isstruct(content)
                                contentNames= fieldnames(content);
                                extraLinearVariables=struct();
                                extraCoeff=struct();
                                extraOffset=struct();
                                for namesIndex=1:length(contentNames)
                                    contentName=contentNames{namesIndex};
                                    if length(content.(contentName))==numberOfSamples
                                        if ~ismember(contentName,fieldnames(extraLinearVariables))
                                            extraLinearVariables.(contentName)=content.(contentName);
                                        else
                                            extraLinearVariables.(contentName)=[extraLinearVariables.(contentName) content.(contentName)];
                                        end
                                        %% check if there is previous information on this new linear variable to consider
                                        tempPrevLinVar=extraCoeffDefaultValue;                                        
                                        varOff=0;
                                        if v+3<=length(varargin) % needs 3 more options in varargin to check this
                                            vtoCheck=varargin{v+2};
                                            vtoCheckValue=varargin{v+3};
                                            if sum(strcmpi(vtoCheck,{'extracoeff','extra linear variable coefficients','varcoeff', 'extra coefficients'}))>0
                                                if ismember(contentName,fieldnames(vtoCheckValue))
                                                    if isvector(vtoCheckValue.(contentName))
                                                        if (sum(size(vtoCheckValue.(contentName))) ==(outputSize+1) && (size(vtoCheckValue.(contentName),1)==outputSize || size(vtoCheckValue.(contentName),2)==outputSize) )
                                                            tempPrevLinVar=vtoCheckValue.(contentName);
                                                            skip=skip+1;
                                                            % check if there is some offset
                                                            % for this variable
                                                            if v+5<=length(varargin) % needs 3 more options in varargin to check this
                                                                vtoCheck2=varargin{v+4};
                                                                vtoCheckValue2=varargin{v+5};
                                                                if sum(strcmpi(vtoCheck2,{'extvaroffset','LinVarOff','varOff','extOff','linear variable offset','extra linear variable offset'}))>0
                                                                    if length(vtoCheckValue2.(contentName))==1
                                                                        varOff=vtoCheckValue2.(contentName);
                                                                        skip=skip+1;
                                                                    else
                                                                        warning('estimateExternalForces: expected one value for the offset');
                                                                    end
                                                                end
                                                            end
                                                        else
                                                            warning('estimateExternalForces: this vector is not of the right dimensions, ignoring vector');
                                                        end
                                                    else
                                                        warning('estimateExternalForces: Expected a vector, using default value of 0.')
                                                    end
                                                else
                                                    % there is no sensor so put 0's
                                                end
                                            end
                                        else
                                            warning('estimateExternalForces: No coefficient available so coeff will be set to 0, the variable will have no effec in recalibrating the data.')
                                        end
                                        if ~ismember(contentName,fieldnames(extraCoeff))
                                            extraCoeff.(contentName)=tempPrevLinVar;
                                        else
                                            extraCoeff.(contentName)=[extraCoeff.(contentName) tempPrevLinVar];
                                        end
                                        if ~ismember(contentName,fieldnames(extraOffset))
                                            extraOffset.(contentName)=varOff;
                                        else
                                            extraOffset.(contentName)=[extraOffset.(contentName) varOff];
                                        end
                                    end
                                end
                            else
                                warning('estimateExternalForces: Expected a vector, using default ignoring variable.')
                            end
                        case {'useestimateddata','use estimated data','use estimated data on sensors not to analize','dont use ft measurements'}
                            if islogical(content)
                                useEstimatedData=content;
                            else
                                warning('estimateExternalForces: expecte logical, by default estimation will use estimated data for the sensors not being analized')
                            end
                    end
                else
                    warning('estimateExternalForces: Unexpected option.')
                end
            else
                % since we already skipped make skip false again
                skip=skip-1;
            end
        end
    else
        error( 'estimateExternalForces: varargin should contain and even number of parameters, something is wrong');
    end
end

%% resize data to desired timeFrame
if (isempty(mask))
    mask=dataset.time>dataset.time(1)+timeFrame(1) & dataset.time<dataset.time(1)+timeFrame(2);
end
dataset=applyMask(dataset,mask);

sNames=fieldnames(dataset.ftData);
if ~iscell(sensorsToAnalize)
    sensorsToAnalize=fieldnames(secMat);
end
% fill with zeros when no extra coeff are available
for sIndex=1:length(sensorsToAnalize)
    ft=sensorsToAnalize{sIndex};
    if ~isstruct(extraCoeff)
        extraCoeff=struct();
        extraLinearVariables=struct();
        extraOffset=struct();
        extraCoeff.(ft)=zeros(outputSize,1);
        extraLinearVariables.(ft)=zeros(size(dataset.time));
        extraOffset.(ft)=0;
    else
        if ~strcmp((ft),fieldnames(extraCoeff))
            extraCoeff.(ft)=zeros(outputSize,1);
            extraLinearVariables.(ft)=zeros(size(dataset.time));
            extraOffset.(ft)=0;
        end
    end
    desiredSensorsIndex(sIndex)= find(strcmp(sNames,(ft)));
end

%% Load the estimator

% Create estimator class
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
% estimator.loadModelAndSensorsFromFile(strcat('./robots/',robotName,'.urdf'));
dofs = length(dataset.jointNames);
consideredJoints = iDynTree.StringVector();
for i=1:dofs %-4 ensures avoiding the 3 last neck joints
 
    consideredJoints.push_back( (dataset.jointNames{i}));
end
for i=1:length(sensorNames)
    consideredJoints.push_back( (sensorNames{i}));    
end

estimatorLoader = iDynTree.ModelLoader();
estimatorLoader.loadReducedModelFromFile(strcat('./robots/',robotName,'.urdf'),consideredJoints);
estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);

% Create KinDynComputations class variable
kinDyn = iDynTree.KinDynComputations();
kinDyn.loadRobotModel(estimator.model);

% Check if the model was correctly created by printing the model
%estimator.model().toString()

%store number of sensors
nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

%For more info on iCub frames check: http://wiki.icub.org/wiki/ICub_Model_naming_conventions
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);
wrench_idyn= iDynTree.Wrench();
qj_all=dataset.qj;
dqj_all=dataset.dqj;
ddqj_all=dataset.ddqj;
if (useInertial)
    angVel_idyn = iDynTree.Vector3();
    angAcc_idyn = iDynTree.Vector3();
end
if (length(contactFrameName)==1)
    % Set the contact information in the estimator
    disp(strcat('estimateExternalForces:using contact frame',{' '},char(contactFrameName)));
    contact_index = estimator.model().getFrameIndex(char(contactFrameName));
end

% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

%% We can use the same class also for performing external wrenches estimation,
%% assuming that calibrated (i.e. without offset) F/T sensor measurements are available
%% For the sake of the example, we use the same FT measurements estimated, but
%% if actual FT sensor measurements were available we could set them in the SensorsMeasurements
%% object by calling the setMeasurements method.

% We first need a new set of unknowns, as we now need 7 unknown wrenches, one for
% each submodel in the estimator
fullBodyUnknownsExtWrenchEst = iDynTree.LinkUnknownWrenchContacts(estimator.model());

% framesNames={'l_sole','r_sole','l_lower_leg','r_lower_leg','root_link','l_elbow_1','r_elbow_1'};
for frame=1:length(framesNames)
    fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex(framesNames{frame}));
end

% We also need to allocate the output of the estimation: a class for estimated contact wrenches and one for joint torques
dofs = estimator.model().getNrOfDOFs();
qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);
% The estimated external wrenches
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(estimator.model());

% The estimated joint torques
estJointTorquesExtWrenchesEst = iDynTree.JointDOFsDoubleArray(dofs);

%match names of sensors
for ftIndex = 0:(nrOfFTSensors-1)
    sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex).getName();
    matchup(ftIndex+1) = find(strcmp(sensorNames,sens ));
end



%size of array with the expected Data
externalWrenchData=zeros(length(framesNames),size(dataset.time,1),6);
% TODO: initialize externalWrenchAtSensorFrame, which is a struct with the
% frames to analize and sensors to analize
jointTorques=zeros(size(qj_all));
%% For each time instant
fprintf('estimateExternalForces: Computing the estimated wrenches\n');
for t=1:length(dataset.time)
    tic
    qj=qj_all(t,:);
    dqj=dqj_all(t,:);
    ddqj=ddqj_all(t,:);
    
    
    qj_idyn.fromMatlab(qj);
    dqj_idyn.fromMatlab(dqj);
    ddqj_idyn.fromMatlab(ddqj);
    
    if(length(contactFrameName)>1)
        contact_index = estimator.model().getFrameIndex(char(contactFrameName(t)));
    end
    
    % print progress test
    if( mod(t,10000) == 0 )
        fprintf('estimateExternalForces: process the %d sample out of %d\n',t,length(dataset.time))
    end
    % store the estimated measurements
    for ftIndex = 0:(nrOfFTSensors-1)
        sIndx= find(strcmp(sensorsToAnalize,sNames(matchup(ftIndex+1))));
        
        if(~isempty(sIndx))
            wrench_idyn.fromMatlab( (secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sNames{matchup(ftIndex+1)})(t,:)')+offset.(sNames{matchup(ftIndex+1)})...
                + extraCoeff.(sensorsToAnalize{sIndx})*(extraLinearVariables.(sNames{matchup(ftIndex+1)})(t,:)-extraOffset.(sNames{matchup(ftIndex+1)}))');
        else
            if useEstimatedData
                wrench_idyn.fromMatlab( dataset.estimatedFtData.(sNames{matchup(ftIndex+1)})(t,:)');
            else
                wrench_idyn.fromMatlab( dataset.ftData.(sNames{matchup(ftIndex+1)})(t,:)'+offset.(sNames{matchup(ftIndex+1)}));
            end
        end
        ok = estFTmeasurements.setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex,wrench_idyn);
        
    end
    if (useInertial)
        grav_idyn.fromMatlab(inertialData.linAcc(t,:));
        angVel_idyn.fromMatlab(inertialData.angVel(t,:));
        angAcc_idyn.fromMatlab([0;0;0]);
        % Set the kinematics information in the estimator
        ok = estimator.updateKinematicsFromFloatingBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn,angVel_idyn,angAcc_idyn);
    else
        ok = estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn);
    end
    % update robot state in the kindyncomputations variable, use this since
    % we only care about relative transforms
    kinDyn.setRobotState(qj_idyn,dqj_idyn,grav_idyn);
    
    % Now we can call the estimator
    estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknownsExtWrenchEst,estFTmeasurements,estContactForcesExtWrenchesEst,estJointTorquesExtWrenchesEst);
    
    
    % We can now print the estimated external forces : as the FT sensor measurements where estimated
    % under the assumption that the only external wrench is acting on the left foot, we should see
    % that the only non-zero wrench is the one on the left foot (frame: l_sole)
    % fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    % fprintf('External wrenches estimated using the F/T offset computed in the previous step\n');
    % fprintf('%s',estContactForcesExtWrenchesEst.toString(estimator.model()));
    
    % Wrenches values can easily be obtained as matlab vectors
    %estContactForcesExtWrenchesEst.contactWrench(estimator.model().getLinkIndex('l_foot'),0).contactWrench().getLinearVec3().toMatlab()
    
    
    % LinkContactWrenches is a structure that can contain multiple contact wrench for each link,
    % but usually is convenient to just deal with a collection of net wrenches for each link
    linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());%
    estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);
    
    for i=1:length(framesNames)
        wrench = linkNetExtWrenches(estimator.model().getFrameLink(estimator.model().getFrameIndex(framesNames{i})));
        %wrench.toMatlab();
        externalWrenchData(i,t,:)=wrench.toMatlab();
        for dsi=1:length(desiredSensorsIndex)
        ft_h_contactFrame=kinDyn.getRelativeTransform((sensorNames{desiredSensorsIndex(dsi)}),(framesNames{i}));
        externalWrenchesAtSensorFrame_temp=ft_h_contactFrame.asAdjointTransformWrench.toMatlab()*wrench.toMatlab();
        externalWrenchesAtSensorFrame.(framesNames{i}).(sensorsToAnalize{dsi})(t,:)=externalWrenchesAtSensorFrame_temp;
        end
    end
    jointTorques(t,:)=estJointTorquesExtWrenchesEst.toMatlab();
end
for i=1:length(framesNames)
    
    externalWrenches.(framesNames{i})=squeeze(externalWrenchData(i,:,:));
end
time=dataset.time;