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
sensorsToAnalize={};
tempCoefficients=zeros(6,1);
tempOffset=0;
if (length(varargin)==1)
    if (isstruct(varargin{1}))
        secMat= varargin{1};
        sensorsToAnalize=fieldnames(secMat);
    end
end

if (mod(length(varargin),2)==0)
    for v=1:2:length(varargin)
        switch  varargin{v}
            case {'secondaryMatrix','secMat','sMat'}
                secMat= varargin{v+1};
                sensorsToAnalize=fieldnames(secMat);
            case {'temperatureCoefficients','tempCoefficients','tempCoeff'}
                TempCoefficients= varargin{v+1};
            case {'temperatureOffset','tempOffset','tempOff'}
                TempOffset= varargin{v+1};
        end
    end
end
sensorNames=fieldnames(dataset.ftData);
for n=1:length(sensorNames)
    offset.(sensorNames{n})=[0;0;0;0;0;0];
end
tempAvailable=true;
if any(strcmp('temperature',fieldnames(dataset)))
    tempAvailable=true;
end

if any(strcmp('estimatedFtData',fieldnames(dataset)))
    %% If estimated variables are already available
    sampleInterval=sampleInit:sampleEnd;
    mask=logical(zeros(length(dataset.time),1));
    mask(sampleInterval)=1;
    dataset=applyMask(dataset,mask);
    for sensor=1:length(sensorNames)
        sIndx= find(strcmp(sensorsToAnalize,sensorNames(sensor)));
        if(isempty(sIndx))
            offset.(sensorNames{sensor})=mean(dataset.estimatedFtData.(sensorNames{sensor})-dataset.ftData.(sensorNames{sensor}))';
        else
            if ~tempAvailable
                recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{sensor})')';
            else
                if exist('TempCoefficients','var')
                    tempCoefficients=TempCoefficients.(sensorNames{sensor});
                end
                if exist('TempOffset','var')
                    tempOffset=TempOffset.(sensorNames{sensor});
                end
                recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{sensor})')'+(tempCoefficients*(dataset.temperature.(sensorNames{sensor})-tempOffset)')';
            end
            offset.(sensorNames{sensor})=mean(dataset.estimatedFtData.(sensorNames{sensor})-recalibData)';
        end
    end
else
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
        ddq=zeros(size(dataset.dqj(sample,:)));
        %ddq=dataset.ddqj(sample,:); % acc measuremnts are not reliable
        %some times
        
        qj_idyn.fromMatlab(q);
        dqj_idyn.fromMatlab(dq);
        ddqj_idyn.fromMatlab(ddq);
        
        %% Update robot kinematics
        
        %     disp(strcat('calculateOffsetUsingWBD: using contact frame',{' '},char(input.contactFrameName)));
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
                offset.(sensorNames{n})=((count-1)/count)*offset.(sensorNames{n}) + (1/count)*(estimatedFT.(sensorNames{n})-dataset.ftData.(sensorNames{n})(sample,:))';
            else
                %                 recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{n})(sample,:)')';
                
                if ~tempAvailable
                    recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{n})(sample,:)')';
                else
                    if exist('TempCoefficients','var')
                        tempCoefficients=TempCoefficients.(sensorNames{n});
                    end
                    if exist('TempOffset','var')
                        tempOffset=TempOffset.(sensorNames{n});
                    end
                    recalibData=(secMat.(sensorsToAnalize{sIndx})*dataset.ftData.(sensorNames{n})(sample,:)')'+(tempCoefficients*(dataset.temperature.(sensorNames{n})(sample,:)-tempOffset)')';
                end
                
                
                offset.(sensorNames{n})=((count-1)/count)*offset.(sensorNames{n}) + (1/count)*(estimatedFT.(sensorNames{n})-recalibData)';
                %              if sum(abs(offset.(sensorNames{n}))>300 )
                %                      sprintf('offset exceeded 300N is now  %d for sensor %s at sample %d', offset.(sensorNames{n}),(sensorNames{n}),sample)
                %              end
            end
        end
    end
    
end
end