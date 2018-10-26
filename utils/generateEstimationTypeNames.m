function [estimationTypeNames]=generateEstimationTypeNames(estimationTypes,useTemperatureBooleans,useTemperatureOffset)
estimationTypeNames{length(estimationTypes)}='';
  for namingIndex=1:length(estimationTypes)
        switch estimationTypes(namingIndex)
            case 1
                name='_S';%'_sphere_offset';
            case 2
                name='_Cm';%'_noMeanOnMain_offset';
            case 3
                name='_C';%'_noMean_offset';
            case 4
                name='_O';%'_oneShot_offset';
        end
        if useTemperatureBooleans(namingIndex)
            name=strcat(name,'wT');
        else
            name=strcat(name,'nT');
        end
        if useTemperatureOffset(namingIndex)
            name=strcat(name,'rTO');
        else
            name=strcat(name,'dTO');
        end
        estimationTypeNames{namingIndex}=name;
  end
  estimationTypeNames=estimationTypeNames';  