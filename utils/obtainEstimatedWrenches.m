function [dataset]=obtainEstimatedWrenches(estimator,resampledTime,contactFrameName,dataset,varargin)
% OBTAINESTIMATEDWRENCH Get (model) estimated wrenches for a iCub dataset
%   estimator       : variable which contains the model of the robot
%   dataset         : contains the joint position velocities and
%   accelerations
%   resampleTime   : time that will be used as reference time
%   contactFrameName : the name of the frame on which it is assumed that an
%                      external contact is (tipically the
%                      root_link, r_sole or l_sole)

%TODO: add contactInfo (which part of the robot is in contact) obtained from minimal knowledge on the FT sensors
% contactInfo should in the end have this information for every time step,
% for now assuming contact doesnt change
%
%% Check varargin
useInertial=false;
calculatedq=false;
calculateddq=false;
timeDelta=resampledTime(2)-resampledTime(1);
if(~isempty(varargin))
    if mod(length(varargin),2)==0
        for v=1:2:length(varargin)
            content=varargin{v+1};
            if(ischar(  varargin{v}))
                switch lower(varargin{v})
                    case {'mask'}
                        if islogical(content)
                            if(size(content)==size(dataset.time))
                                mask=content;                                
                            else
                                warning('obtainEstimatedWrenches:Mask is the wrong size, parameter ignored');
                            end
                        end
                    case {'inertial','inertialdata','inertial data'}
                        if isstruct(content) % it means inertial data is provided
                            inertialData=content;
                            inertialFields=fieldnames(inertialData);
                            if length(inertialFields)==2
                                useInertial=true;
                            else
                                warning('obtainEstimatedWrenches: Expected inertial data that has only 2 fields');
                            end
                        end
                    case{'diffpos','differentiateposition','calculatedq',}
                        if islogical(content)
                            calculatedq=content;
                        else
                            warning('obtainEstimatedWrenches: expected boolean for calculate dq, parameter ignored');
                        end
                    case{'diffvel','differentiatevelocity','calculateddq',}
                        if islogical(content)
                            calculateddq=content;
                        else
                            warning('obtainEstimatedWrenches: expected boolean for calculate ddq, parameter ignored');
                        end
                end
            else
                warning('obtainEstimatedWrenches: Unexpected option.')
            end
        end
    else
        error( 'obtainEstimatedWrenches: varargin should contain and even number of parameters, something is wrong');
    end
end

%% Take the used position from the dataset
qj_all=dataset.qj;
% if calculateddq
%     diffddq=diff(dataset.dqj)/timeDelta;
%     ddqj_all=[zeros(size(qj_all(1,:)));diffddq];
%     dataset.ddqj=ddqj_all;
% else
%     %for now acc values are unreliable so zeroes
%     %ddqj_all=dataset.ddqj;
%     ddqj_all=zeros(size(dataset.ddqj));
% end
if calculatedq
    dqj_all=numericalDifferentiation(dataset.qj,timeDelta);
    dataset.dqj=dqj_all;
else
    dqj_all=dataset.dqj;
end
if calculateddq
    ddqj_all=numericalDifferentiation(dqj_all,timeDelta);
    dataset.ddqj=ddqj_all;
else
    %for now acc values are unreliable so zeroes
%     ddqj_all=dataset.ddqj;
    ddqj_all=zeros(size(dataset.ddqj));
end


if exist('mask','var')
    dataset=applyMask(dataset,mask);
    resampledTime=applyMask(resampledTime,mask);
    qj_all=applyMask(qj_all,mask);
    dqj_all=applyMask(dqj_all,mask);
    ddqj_all=applyMask(ddqj_all,mask);
end

dofs = estimator.model().getNrOfDOFs();

grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

if (useInertial)
    angVel_idyn = iDynTree.Vector3();
    angAcc_idyn = iDynTree.Vector3();
end
%store number of sensors
nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

%size of array with the expected Data
ftData=zeros(nrOfFTSensors,size(resampledTime,1),6);
%% Specify unknown wrenches

% We need to set the location of the unknown wrench. For the time being it
% is assumed that the contact will not change either on left or right foot
% so it is only defined once.
%TODO: recognize when the feet are in contact either left or right or both
%and establish the unkown wrench accordingly



unknownWrench = iDynTree.UnknownWrenchContact();
unknownWrench.unknownType = iDynTree.FULL_WRENCH;

% the position is the origin, so the conctact point wrt to l_sole is zero
unknownWrench.contactPoint.zero();

% The fullBodyUnknowns is a class storing all the unknown external wrenches
% acting on a class
fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
fullBodyUnknowns.clear();

if (length(contactFrameName)==1)
    % Set the contact information in the estimator
    contact_index = estimator.model().getFrameIndex(char(contactFrameName));
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_index,unknownWrench);
end
% Print the unknowns to make sure that everything is properly working
%fullBodyUnknowns.toString(estimator.model())

% Prepare the datastructure that we used in the loop outside the loop
% (to improve performances, as otherwise we allocate memory at each
% loop step, and allocating memory is usually quite a slow operation)
qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);


% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% The estimated joint torques
estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

% The estimated contact forces
estContactForces = iDynTree.LinkContactWrenches(estimator.model());

% Sensor wrench buffer
estimatedSensorWrench = iDynTree.Wrench();


%% For each time instant
fprintf('obtainEstimatedWrenches: Computing the estimated wrenches\n');
for t=1:length(resampledTime)
    tic
    % print progress test
    if( mod(t,10000) == 0 )
        fprintf('obtainedEstimatedWrenches: process the %d sample out of %d\n',t,length(resampledTime))
    end
    
    qj=qj_all(t,:);
    dqj=dqj_all(t,:);
    ddqj=ddqj_all(t,:);
    
    %    % velocity and acceleration to 0 to prove if they are neglegible. (slow
    %    % experiment scenario)
    %     dqj=zeros(size(qj));
    % ddqj=zeros(size(qj)); % temprorary change due to problem with the acc in the firmware
    
    qj_idyn.fromMatlab(qj);
    dqj_idyn.fromMatlab(dqj);
    ddqj_idyn.fromMatlab(ddqj);
    
    if(length(contactFrameName)>1)
        fullBodyUnknowns.clear();
        contact_index = estimator.model().getFrameIndex(char(contactFrameName(t)));
        fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_index,unknownWrench);
        
    end
    
    
    if (useInertial)
        grav_idyn.fromMatlab(inertialData.linAcc(t,:));
        angVel_idyn.fromMatlab(inertialData.angVel(t,:));
        angAcc_idyn.fromMatlab([0;0;0]);
        % Set the kinematics information in the estimator
        ok = estimator.updateKinematicsFromFloatingBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn,angVel_idyn,angAcc_idyn);
        
    else
        % Set the kinematics information in the estimator
        ok = estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn);
    end
    
    %% Run the prediction of FT measurements
    
    % There are three output of the estimation, FT measurements, contact
    % forces and joint torques (they are declared outside the loop for
    % performance reason)
    
    % run the estimation
    estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,estFTmeasurements,estContactForces,estJointTorques);
    
    % store the estimated measurements
    for ftIndex = 0:(nrOfFTSensors-1)
        %sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex);
        ok = estFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex,estimatedSensorWrench);
        %store in the correct variable, format from readDataDumpre results in (time,sensorindex)
        %TODO: generalized code for any number of sensors , idea create a
        %struct with the name of the sensor as field Data as values and
        %compare in the main with the name of the sensors.
        
        ftData(ftIndex+1,t,:)=estimatedSensorWrench.toMatlab();
    end
    
    % collect also joint torques which were already estimated
    dataset.jointTorques(t,:)=estJointTorques.toMatlab();
    
    % print the estimated contact forces
    %estContactForces.toString(estimator.model())
end


nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);
% sensorNames{nrOfFTSensors}='';
for ftIndex = 0:(nrOfFTSensors-1)
    sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex);
    %sensorNames{ftIndex+1}=sens.getName();
    %squeeze(ftData(ftIndex+1,:,:)) to remove singleton of ftIndex
    dataset.estimatedFtData.(sens.getName())=squeeze(ftData(ftIndex+1,:,:));
end


