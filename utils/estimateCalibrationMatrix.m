function [calibrationMatrix,full_scale,offset,extraCoeff]=estimateCalibrationMatrix(rawData,expectedWrench,varargin)
% This functions applies linear regression to obtain a
% calibration matrix using least squares with regularization.
% inputs:
%  rawData: is the raw data coming from ft sensors
%  expectedWrench: is the reference of the regression
%  varargin: contains options that may change the behavior
%    previous_Calibration: is a previous calibration matrix
%    lambda: is the regularization parameter
%    previous_offset: in case there is a known offset we can use that information in the
%  regularization
%    estimateOffset: request to estimate the offset at the same time
%
% outputs:
%  calibrationMatrix: the resulting calibration matrix
%  full_scale: the values obtained if the raw values are maxed out
%  offset: the resulting offset
%  extraCoeff: coefficients corresponding to the extra linear variables to
%  consider
%% initial values
[n,outputSize] = size(expectedWrench);
inputSize= size(rawData,2);
calibrationMatrixLength=outputSize*inputSize;
previous_Calibration=eye(inputSize,outputSize);
lambda=0;
extraLinearVariablesNumber=0;
% possibly required values
previous_offset=zeros(size(expectedWrench,2),1);
previous_extraLinearVariable=[];
extraLinearVariables=[];
% default output values
% full_scale=zeros(outputSize,1);
% calibrationMatrix=eye(outputSize,inputSize);
extraCoeff=zeros(outputSize,1);
offset=zeros(outputSize,1);
% options
withRegularization=false;
estimateOffset=false;
offsetAvailable=false;
previousLinearVariables=false;
skip=false;
%% check varargin
for v=1:2:length(varargin)
    if ~skip
        if(ischar(  varargin{v}))
            tempV=varargin{v+1};
            switch varargin{v}
                case {'previousCalibration','cMat','preCalib'}
                    if (ismatrix(tempV))
                        ok=true;
                        if ~(size(tempV,1)==outputSize && size(tempV,2)==inputSize) % check if dimensions are correct
                            ok=false;
                            warning('estimateCalibrationMatrix: matrix inerted is not the right dimensions, so is not a calibration matrix');
                        end
                    end
                    if ok
                        previous_Calibration=tempV;
                        withRegularization=true;
                    else
                        withRegularization=false;
                        warning('estimateCalibrationMatrix: incorrect struct, no previous calibration matrix');
                    end
                case {'lambda','Lambda','LAMBDA'}
                    if isnumeric(tempV)
                        lambda=tempV;
                    else
                        warning('estimateCalibrationMatrix: Expected numeric, using default lambda value of 0.')
                    end
                case {'prevOffset','previousOffset','previousOFFSET'}
                    if isvector(tempV)
                        if (sum(size(tempV)) ==(outputSize+1) && (size(tempV,1)==outputSize || size(tempV,2)==outputSize) )
                            if size(tempV,2)==outputSize
                                previous_offset=tempV';
                            else
                                previous_offset=tempV;
                            end
                            offsetAvailable=true;
                        else
                            warning('estimateCalibrationMatrix: this vector is not an offset of the right dimensions, ignoring vector');
                        end
                    else
                        warning('estimateCalibrationMatrix: Expected a vector, using default offset value of 0.')
                    end
                case {'addLinearVariable','addExtraLinearVariable','extraLinearVariable','addLinVar','addExtLinVar'}
                    if isvector(tempV)
                        if length(tempV)==n
                            extraLinearVariables=[extraLinearVariables tempV];
                            extraLinearVariablesNumber=extraLinearVariablesNumber+1;
                            %% check if there is previous information on this new linear variable to consider
                            previousLinearVariables(extraLinearVariablesNumber)=false;
                            tempPrevLinVar=zeros(size(expectedWrench,2),1);
                            if v+3<=length(varargin) % needs 3 more options in varargin to check this
                                vtoCheck=varargin{v+2};
                                vtoCheckValue=varargin{v+3};
                                if sum(strcmp(vtoCheck,{'previousExtraLinearVariable','prevExtLinVar','prevLinVar'}))>0
                                    if isvector(vtoCheckValue)
                                        if (sum(size(vtoCheckValue)) ==(outputSize+1) && (size(vtoCheckValue,1)==outputSize || size(vtoCheckValue,2)==outputSize) )
                                            if size(vtoCheckValue,2)==outputSize
                                                tempPrevLinVar=vtoCheckValue';
                                            else
                                                tempPrevLinVar=vtoCheckValue;
                                            end
                                            previousLinearVariables(extraLinearVariablesNumber)=true;
                                            skip=true;
                                        else
                                            skip=false;
                                            warning('estimateCalibrationMatrix: this vector is not of the right dimensions, ignoring vector');
                                        end
                                    else
                                        warning('estimateCalibrationMatrix: Expected a vector, using default value of 0.')
                                    end
                                end
                            end
                            previous_extraLinearVariable=[previous_extraLinearVariable tempPrevLinVar];
                        else
                            warning('estimateCalibrationMatrix: length of vecotr incorrect, not extra linear variable.')
                        end
                    else
                        warning('estimateCalibrationMatrix: Expected logical, using default withTemperature value.')
                    end
                case {'estimateoffset','estimateOffset','estimateOFFSET'}
                    if islogical(tempV)
                        estimateOffset=tempV;
                    else
                        warning('useLinearModelToCalibrate: Expected logical, using default withRegularization value.')
                    end
                otherwise
                    warning('estimateCalibrationMatrix: Unexpected option.')
            end
        else
            warning('estimateCalibrationMatrix: Unexpected option.')
        end
    end
end
%% extra logic
if estimateOffset
    extraLinearVariablesNumber=extraLinearVariablesNumber+1;
    extraLinearVariables=[extraLinearVariables ones(n,1)];
    previous_extraLinearVariable=[previous_extraLinearVariable previous_offset];
    previousLinearVariables(extraLinearVariablesNumber)=offsetAvailable;
end
if ~withRegularization
    lambda=0;
    toPenalize=eye(outputSize*(inputSize+extraLinearVariablesNumber));
    toPenalizeReference=eye(outputSize*(inputSize+extraLinearVariablesNumber),1);
else %% build toPenalize based on previousLinearVariables info
    previous_Calibration=[previous_Calibration previous_extraLinearVariable];
    toPenalizeReference=previous_Calibration(:);
    %TODO: check if it should be depending on output size or input size
    toPenalize=eye(outputSize*(inputSize+extraLinearVariablesNumber));
    for elv=1:extraLinearVariablesNumber
        toModify=calibrationMatrixLength+(elv-1)*outputSize+1:calibrationMatrixLength+elv*outputSize;
        if ~previousLinearVariables(elv)
            toPenalize(toModify,toModify)=zeros(outputSize);
        end
    end
end
overlineR = [rawData extraLinearVariables];
%% variables that do not change
expectedWrench_trans=expectedWrench';
kA = kron(overlineR,eye(outputSize));% should eye be eye(outputSize,inputSize)?
kb=expectedWrench_trans(:);
% A=kA'*kA+lambda*toPenalize;
% b=kA'*kb+lambda*toPenalizeReference;
%% TODO: select if division by kaSize helps or not,
% using it makes it more sensible to the regularization parameters and is
% also the way we were using it in the main estimate function
kaSize=size(kA,1);
A=(kA'*kA)/kaSize+lambda*toPenalize;
b=(kA'*kb)/kaSize+lambda*toPenalizeReference;

%% apply least squares (linear regression)
% x=pinv(A)*b;
%% QR solution to least squares
[Q,R]=qr(A);
x=R\(Q'*b);

calibrationMatrix = reshape(x(1:calibrationMatrixLength), outputSize, inputSize);
%% depending on options
if length(x)>calibrationMatrixLength
    for extCoeff=1:extraLinearVariablesNumber
        extraCoeff(:,extCoeff)=x(calibrationMatrixLength+(extCoeff-1)*outputSize+1:calibrationMatrixLength+extCoeff*outputSize);
    end
    if estimateOffset
        if extraLinearVariablesNumber>1
            offset=extraCoeff(:,end);
            extraCoeff=extraCoeff(:,end-1);
        else
            offset=extraCoeff;
            extraCoeff=zeros(outputSize,1);
        end
    end
end
% calculate full scale range
maxs = sign(calibrationMatrix)*32767;
full_scale = diag(calibrationMatrix*maxs');
max_Fx = ceil(full_scale(1));
if length(full_scale)>1
    max_Fy = ceil(full_scale(2));
    if length(full_scale)>2
        max_Fz = ceil(full_scale(3));
        if length(full_scale)>3
            max_Tx = ceil(full_scale(4));
            if length(full_scale)>4
                max_Ty = ceil(full_scale(5));
                if length(full_scale)>5
                    max_Tz = ceil(full_scale(6));
                end
            end
        end
    end
end
end
