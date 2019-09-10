function[h]=Plot_variousFTData_fromMatrices(namesDatasets,varargin)
if (length(namesDatasets) <=length(varargin)/2)
    %plotting
    for n=1:length(namesDatasets)
        ftDataNumber=(n-1)*2+1;
        timeDataNumber=ftDataNumber+1;
        if ismatrix(varargin{ftDataNumber})
            tempFTdata=struct('FTdata',varargin{ftDataNumber});
            if(length(namesDatasets)<=length(varargin)/2)
                h(n)=    FTplots(tempFTdata,varargin{timeDataNumber},'byChannel','noTimeOffset');
            else
                h(n)=    FTplots(tempFTdata,varargin{timeDataNumber});
            end
        else
            warning('Something is wrong, a matrix was expected');
        end
    end
    %merging
    for n=length(namesDatasets):-1:2
        mergeFTplots(h(n-1),h(n),'thisLegendNames',namesDatasets);
    end
    
else
    warning('Insuficient arguments');
end

