function [cMat,secMat,WorkbenchMat,extraCoeff,offsets,extraCoeffOffset]=readGeneratedCalibMatrices(experimentNames,scriptOptions,sensorsToAnalize,names2use,calibrationFileNames)

for i=1:length(experimentNames)

    if(scriptOptions.testDir==false)
        prefixDir='';
    else
        prefixDir='../';
    end

    paramScript=strcat(prefixDir,'data/',experimentNames{i},'/params.m');
    run(paramScript)
    ftNames=input.ftNames;

    for ii=1:size(input.calibMatFileNames,1) % no offset in workbench matrix
        WorkbenchMat.(ftNames{ii})=getWorkbenchCalibMat(strcat(prefixDir,input.calibMatPath),input.calibMatFileNames{ii});
        secMat.(names2use{1}).(ftNames{ii})=eye(6);
        extraCoeff.(names2use{1}).(ftNames{ii})=zeros(6,1);
        extraCoeffOffset.(names2use{1}).(ftNames{ii})=0;
    end

    cMat.(names2use{1})=WorkbenchMat; %first dataset to compare is the orignal workbench

    for lam=1:length(calibrationFileNames)
        (names2use{(i-1)*length(calibrationFileNames)+1+lam});
        for j=1:length(sensorsToAnalize)
            sIndx= find(strcmp(ftNames,sensorsToAnalize{j}));
            [cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j}),~,offset] = readCalibMat(strcat(prefixDir,'data/',experimentNames{i},'/calibrationMatrices/',input.calibMatFileNames{sIndx},calibrationFileNames{lam}));
            secMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})= cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})(1:6,1:6)/WorkbenchMat.(sensorsToAnalize{j});
            if size(cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j}),2)>6
                extraCoeff.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})(:,7:end);
                cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=cMat.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})(1:6,1:6);
                if length(offset)>6
                    extraCoeffOffset.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=offset(7:end);
                    offsets.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=offset(1:6);
                end
            else
                extraCoeff.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=zeros(6,1);
                extraCoeffOffset.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=0;
                offsets.(names2use{(i-1)*length(calibrationFileNames)+1+lam}).(sensorsToAnalize{j})=offset;
            end
        end
    end
end
