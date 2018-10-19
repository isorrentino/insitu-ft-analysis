function [] = writeCalibMat(calibMat, full_scale, filename,varargin)
% WRITECALIBMAT Write on file the calibration matrix
%
% calibMat is a 6x6 calibration matrix that maps raw output of
% 16-bit ADC connected to the strain gauges (going from 2^15 to 2^15-1)
% to the Force-Torque values, expressed in Newtons / Newton-meters .
%
% full_scale is a 6 vector of the full scale for each channel of the F/T
% sensor, expressed in Newtons (first three values) and Newton-meters (last
% three values).
%
% filename is the name of the file in which the calibration matrix will be
% written .
%

% logic copied from the write_matrix script in ftSensCalib repository
max_Fx = full_scale(1);
max_Fy = full_scale(2);
max_Fz = full_scale(3);
max_Tx = full_scale(4);
max_Ty = full_scale(5);
max_Tz = full_scale(6);

Wf = diag([1/max_Fx 1/max_Fy 1/max_Fz 1/max_Tx 1/max_Ty 1/max_Tz]);
maxRaw = 2^15-1;
Ws = diag([1/maxRaw 1/maxRaw 1/maxRaw 1/maxRaw 1/maxRaw 1/maxRaw]);

% Calibration matrix ready to be implemented into the firmware, maps
% the raw values to values expressed with respect to the fullscale of the
% sensor
Cs = Wf * calibMat * inv(Ws);

if(sum(sum(Cs>1))==0 && sum(sum(Cs<-1))==0)
    disp('writeCalibMat: Matrix can be implemented in the DSP (i.e. coeffs in [-1 1])')
else
    disp('writeCalibMat: ERROR!!!! Matrix cannot be implemented in the DSP (i.e. coeffs not in [-1 1])')
end

fid = fopen(filename, 'w+');
for iy=1:6
    for ix=1:6
        temp=convert_onedotfifteen(Cs(iy,ix));
        fprintf(fid, '%s\r\n', temp);
    end
end
fprintf(fid, '%d\r\n', 1);
fprintf(fid, '%d\r\n', ceil(full_scale(1)));
fprintf(fid, '%d\r\n', ceil(full_scale(2)));
fprintf(fid, '%d\r\n', ceil(full_scale(3)));
fprintf(fid, '%d\r\n', ceil(full_scale(4)));
fprintf(fid, '%d\r\n', ceil(full_scale(5)));
fprintf(fid, '%d\r\n', ceil(full_scale(6)));

if fclose(fid) == -1
    error('writeCalibMat: [ERROR] there was a problem in closing the file')
end

if ~isempty(varargin)
    for v=1:2:length(varargin)
        if(ischar(  varargin{v}))
            switch varargin{v}
                case {'extraCoeff','extracoeff','extraCoeffs','extracoeffs'}
                    fid = fopen(strcat(filename,'_extraCoeff'), 'w+');
                    vinput=varargin{v+1};
                    if isvector(vinput) && isnumeric(vinput)
                        if mod(length(vinput),6)==0
                            for i=1:length(vinput)
                                fprintf(fid, '%f\r\n', vinput(i));
                            end
                        end
                    else
                        if ismatrix(vinput) && isnumeric(vinput)
                            if size( vinput,1)==6
                                for iy=1:6
                                    for ix=1:size( vinput,2)
                                        temp=vinput(iy,ix);
                                        fprintf(fid, '%f\r\n', temp);
                                    end
                                end
                            end
                        end
                    end                    
                    if fclose(fid) == -1
                        error('writeCalibMat: [ERROR] there was a problem in closing the file')
                    end
                case {'offsets'}
                    fid = fopen(strcat(filename,'_offsets'), 'w+');
                    vinput=varargin{v+1};
                    if isvector(vinput) && isnumeric(vinput)
                        if length(vinput)>=6
                            for i=1:length(vinput)
                                fprintf(fid, '%f\r\n', vinput(i));
                            end     
                        else
                            error('writeCalibMat: [ERROR] offsets should be at least 6')
                        end
                    else
                     error('writeCalibMat: [ERROR] offsets should be a vector')
                    end                    
                    if fclose(fid) == -1
                        error('writeCalibMat: [ERROR] there was a problem in closing the file')
                    end
            end
        end
    end
end