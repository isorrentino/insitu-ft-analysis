function [offset]=calculateOffsetUsingWBD(estimator,dataset,sampleInit,sampleEnd,input,varargin)
%% estimate ft to calculate offset used when starting wbd using iterative mean
% calculateOffsetUsingWBD estimated wrenches for a iCub dataset to
% calculate the offset
%   estimator       : variable which contains the model of the robot
%   dataset         : contains the joint position velocities and
%   accelerations
%   sampleInit   : first sample from which we asume is only on one contact and
%   relatively still
%   sampleEnd : last sample from which we asume is only on one contact and
%   relatively still
%   input: information regarding the params.m file
%   varargin: able to have secondary matrixes or not
% assuming contact doesnt change%
secMat=NaN;
if (length(varargin)==1)
    if (isstruct(varargin{1}))
        secMat= varargin{1};
        sensorsToAnalize=fieldnames(secMat);
    end
end
sensorNames=fieldnames(dataset.ftData);
for n=1:length(sensorNames)          
    offset.(sensorNames{n})=[0;0;0;0;0;0];     
end
%% Prerpare joint variables

dofs = estimator.model().getNrOfDOFs();
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);


%    % velocity and acceleration to 0 to prove if they are neglegible. (slow
%    % experiment scenario)
%     dqj=zeros(size(qj));
%     ddqj=zeros(size(qj));

qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);




%% Prepare estimator variables
%store number of sensors
nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% The estimated external wrenches
estContactForces = iDynTree.LinkContactWrenches(estimator.model());

% The estimated joint torques
estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

%match names of sensors
for ftIndex = 0:(nrOfFTSensors-1)
    sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex).getName();
    matchup(ftIndex+1) = find(strcmp(input.sensorNames,sens ));
end

 estimator.model().toString();
%% Iterate over joint positions from sample Init to sample End
for sample=sampleInit:sampleEnd
    
    q=dataset.qj(sample,:);
    dq=zeros(size(dataset.dqj(sample,:)));
    ddq=dataset.ddqj(sample,:);
    
    qj_idyn.fromMatlab(q);
    dqj_idyn.fromMatlab(dq);
    ddqj_idyn.fromMatlab(ddq);
    
    %% Update robot kinematics
    
      
    
    disp(strcat('using contact frame',{' '},char(input.contactFrameName)));
    % Set the contact information in the estimator
    contact_index = estimator.model().getFrameIndex(char(input.contactFrameName));
    ok = estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,contact_index,grav_idyn);
    
    
    
    %% Specify unknown wrenches
    
    % We need to set the location of the unknown wrench. For the time being it
    % is assumed that the contact will not change
    
    unknownWrench = iDynTree.UnknownWrenchContact();
    %% Run the prediction of FT measurements
    
    % There are three output of the estimation, FT measurements, contact
    % forces and joint torques (they are declared outside the loop for
    % performance reason)
    
    fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
    fullBodyUnknowns.clear();
    unknownWrench.unknownType = iDynTree.FULL_WRENCH;
    unknownWrench.contactPoint = iDynTree.Position.Zero();
    %fullBodyUnknowns.addNewContactForLink(contact_index,unknownWrench);
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_index,unknownWrench);
    
    % Print the unknowns to make sure that everything is properly working
    %fullBodyUnknowns.toString(estimator.model())
    % Sensor wrench buffer
    estimatedSensorWrench = iDynTree.Wrench();
    estimatedSensorWrench.fromMatlab(zeros(1,6));
    % run the estimation
    
    ok=estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,estFTmeasurements,estContactForces,estJointTorques);
    
    % store the estimated measurements
    for ftIndex = 0:(nrOfFTSensors-1)
        ok = estFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex,estimatedSensorWrench);
        estimatedFT.(sensorNames{matchup(ftIndex+1)})=estimatedSensorWrench.toMatlab()';
%         if sum(abs(estimatedFT.(sensorNames{matchup(ftIndex+1)})) >330)
%             guilty=(abs(estimatedFT.(sensorNames{matchup(ftIndex+1)})) >330)
%             indexes=find(guilty)
%             for gg=1:length(indexes)
%                 sprintf('huge number %d for sensor %s axis %d at sample %d \n', estimatedFT.(sensorNames{matchup(ftIndex+1)})(indexes(gg)),(sensorNames{matchup(ftIndex+1)}),indexes(gg),sample)
%             end
%         end
    end
    %estimatedFT
    
    %% Calculate offset with iterative mean
    count=sample-sampleInit+1;
    for n=1:length(sensorNames)
        sIndx= find(strcmp(sensorsToAnalize,sensorNames(n)));        
        if(isempty(sIndx))
            offset.(sensorNames{n})=((count-1)/count)*offset.(sensorNames{n}) + (1/count)*(dataset.ftData.(sensorNames{n})(sample,:)-estimatedFT.(sensorNames{n}))';
        else
            recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{n})(sample,:)')';
             offset.(sensorNames{n})=((count-1)/count)*offset.(sensorNames{n}) + (1/count)*(recalibData-estimatedFT.(sensorNames{n}))';
%              if sum(abs(offset.(sensorNames{n}))>300 )
%                      sprintf('offset exceeded 300N is now  %d for sensor %s at sample %d', offset.(sensorNames{n}),(sensorNames{n}),sample)      
%              end
        end
    end   
end

    
end