function calibMatrix=getWorkbenchCalibMat(pathFile,serialNumber)
    if (exist(strcat(pathFile,'matrix_',serialNumber,'.txt'),'file')==2)
        calibMatrix=readCalibMat(strcat(pathFile,'matrix_',serialNumber,'.txt'));        
    else
        destinationFile=strcat(pathFile,'matrix_',serialNumber,'.txt');
        disp(strcat({'getRawData: Calibration Matrix '},serialNumber,{' not found in the specified folder. Trying in default folders'}))
        if (exist(strcat('external/ftSensCalib/software/sensAquisitionArchive/',serialNumber,'/','matrix_',serialNumber,'.txt'),'file')==2)
            calibMatrix=readCalibMat(strcat('external/ftSensCalib/software/sensAquisitionArchive/',serialNumber,'/','matrix_',serialNumber,'.txt'));
            sourceFile=strcat('external/ftSensCalib/software/sensAquisitionArchive/',serialNumber,'/','matrix_',serialNumber,'.txt');
        else
            if (exist(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12188_A2/',serialNumber,'/','matrix_',serialNumber,'.txt'),'file')==2)
                calibMatrix=readCalibMat(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12188_A2/',serialNumber,'/','matrix_',serialNumber,'.txt'));
                sourceFile=strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12188_A2/',serialNumber,'/','matrix_',serialNumber,'.txt');
            else
                if (exist(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12491/',serialNumber,'/','matrix_',serialNumber,'.txt'),'file')==2)
                    calibMatrix=readCalibMat(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12491/',serialNumber,'/','matrix_',serialNumber,'.txt'));
                    sourceFile=strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_12491/',serialNumber,'/','matrix_',serialNumber,'.txt');
                else
                    if (exist(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_683/',serialNumber,'/','matrix_',serialNumber,'.txt'),'file')==2)
                        calibMatrix=readCalibMat(strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_683/',serialNumber,'/','matrix_',serialNumber,'.txt'));
                        sourceFile=strcat('external/ftSensCalib/software/sensAquisitionArchive/cod_iit_683/',serialNumber,'/','matrix_',serialNumber,'.txt');
                    else
                        disp(strcat({'getRawData: Calibration Matrix '},serialNumber,{' not found in the default folder.'}))
                    end
                end
            end
        end
        
    end
    if exist('sourceFile','var')
        copyfile(sourceFile,destinationFile);
    end
