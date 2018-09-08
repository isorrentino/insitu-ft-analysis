function [cMat,full_scale] = readCalibMat(filename)
%read the calibration matrix delivered by the calibration procedure
defaultFullScale=false;


fid = fopen(filename);

if( fid == -1 )
    error(strcat('[ERROR] error in opening file ',filename))
end

format = '%X';
vec=fscanf(fid,format);
calibMat=reshape(vec(1:36),[6,6])';
calibMat=calibMat/(2^15);
mask=calibMat>1;
calibMat(mask)=calibMat(mask)-2;

if fclose(fid) == -1
   error('[ERROR] there was a problem in closing the file')
end
%these values were originally base 10, due to format they were converted
%as if they were hex so dec2hex returns them to real base 10 value in this
%case
if defaultFullScale
    defaultScale=[32767,32767,32767,32767,32767,32767]; %% this might work only for strain2 for reasons unknown
    full_scale=defaultScale;
else
    full_scale=str2num(dec2hex(vec(38:end)));
end
max_Fx = full_scale(1);
max_Fy = full_scale(2);
max_Fz = full_scale(3);
max_Tx = full_scale(4);
max_Ty = full_scale(5);
max_Tz = full_scale(6);

Wf = diag([1/max_Fx 1/max_Fy 1/max_Fz 1/max_Tx 1/max_Ty 1/max_Tz]);
Ws = diag([1/32767 1/32767 1/32767 1/32767 1/32767 1/32767]);
cMat = inv(Wf) * calibMat *Ws;

if (exist(strcat(filename,'_extraCoeff'),'file')==2)   
    vec2=load(strcat(filename,'_extraCoeff'));
    columns=length(vec2)/6;
    extraCoeff=reshape(vec2,[6,columns]);
    cMat=[cMat  extraCoeff];
end
