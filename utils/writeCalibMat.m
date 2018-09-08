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
    disp('Matrix can be implemented in the DSP (i.e. coeffs in [-1 1])')
else
    disp('ERROR!!!! Matrix cannot be implemented in the DSP (i.e. coeffs not in [-1 1])')
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
    error('[ERROR] there was a problem in closing the file')
end

if ~isempty(varargin)
    fid = fopen(strcat(filename,'_extraCoeff'), 'w+');
    for v=1:length(varargin)
        if isvector(varargin{v}) && isnumeric(varargin{v})
            if mod(length(varargin{v}),6)==0
                for i=1:length(varargin{v})
                    fprintf(fid, '%f\r\n', varargin{v}(i));
                end
            end
        else
            if ismatrix(varargin{v}) && isnumeric(varargin{v})
                if size( varargin{v},1)==6
                    for iy=1:6
                        for ix=1:size( varargin{v},2)
                            temp=varargin{v}(iy,ix);
                            fprintf(fid, '%f\r\n', temp);
                        end
                    end
                end
            end
        end
    end
    if fclose(fid) == -1
        error('[ERROR] there was a problem in closing the file')
    end
end