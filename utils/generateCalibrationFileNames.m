function [calibrationFileNames]=generateCalibrationFileNames(lambdasNames,estimationTypeNames)
cNamesNumber=1;
for j=1:length(lambdasNames)
    for k=1:length(estimationTypeNames)
        calibrationFileNames{cNamesNumber}=strcat(lambdasNames{j},estimationTypeNames{k});
        cNamesNumber=cNamesNumber+1;
    end
end
calibrationFileNames=calibrationFileNames';