

[file,path]=uigetfile('*.log','open logged filtered from robot');
[data1,time1]=readDataDumper([path,file]);
[file,path]=uigetfile('*.log','open logged filtered with temp');
[data2,time2]=readDataDumper([path,file]);
[file,path]=uigetfile('*.log','open logged filtered with temp and offset');
[data3,time3]=readDataDumper([path,file]);


names={'no temp','temp','temp & offset'};
[H,h]=Plot_variousFTData_fromMatrices(names,data1,time1,data2,time2,data3,time3);

calibOffset=mean(dataset.estimatedInertialData.ftData.left_leg-dataset.estimatedInertialData.estimatedFtData.left_leg);
dataNoCalibOffset=dataset.ftData.left_leg-calibOffset;
names={'no temp','temp','temp & offset','estimated','orignal_{measure}'};
% [h2]=Plot_variousFTData_fromMatrices(names,data1,time1,data2,time2,data3,time3,dataset.estimatedFtData.left_leg,dataset.time,dataNoCalibOffset,dataset.time);
fullEstimated=[dataset.estimatedInertialData.estimatedFtData.left_leg;dataset.estimatedFtData.left_leg];
fullEstimatedTime=[dataset.estimatedInertialData.time;dataset.time];
[h2]=Plot_variousFTData_fromMatrices(names,data1,time1,data2,time2,data3,time3,fullEstimated,fullEstimatedTime,dataNoCalibOffset,dataset.time);

 fieldNamesh1=fieldnames(h(1));
   for fnh=1:length(fieldNamesh1)
       fnameh1=fieldNamesh1{fnh};
%       saveas(h1.(fnameh1),[outputFolder,'/',fnameh1],'epsc') 
%       saveas(h1.(fnameh1),[outputFolder,'/',fnameh1],'png') 
        saveas(h2(1).(fnameh1),fnameh1,'png') 
       
   end