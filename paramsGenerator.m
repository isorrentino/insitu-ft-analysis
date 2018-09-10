function [paramsFile]=paramsGenerator(experimentName,robotName)
%prompt to ask if we should copy from another experiment
if copyFromExperiment
    paramsFile=fileread(strcat(experimentNameRef,'/params.m'));
else
paramsTemplate=fileread('paramsTemplate.m')
endLine='\n';
sectionStart='%%';
emptyStringsChars=''';';
fillVariable='input';
defaultNumber='0';
emptyCellArray='{};';
endLineIndexes= regexp(paramsTemplate,endLine);
sections=regexp(paramsTemplate,sectionStart); % ignore first section
stringsToFill=regexp(paramsTemplate,emptyStringsChars); % expecting a string
numbersToChange=regexp(paramsTemplate,defaultNumber); % this should be changed
cellArrayToFill=regexp(paramsTemplate,emptyCellArray); % expecting a cell array
% collect inital description of the params file
outputParams=paramsTemplate(1:sections(2)-1);
%lines in section
%% Variables that depend on the type of experiment
linesInSection=endLineIndexes(endLineIndexes>sections(2) & endLineIndexes<sections(3));

% eval('ftNames={''left_arm'';''right_arm'';''left_leg'';''right_leg'';''left_foot'';''right_foot''};')

end

fid = fopen(strcat(experimentName,'/params.m'),'w');
    fprintf(fid, '%s', paramsFile);
    fclose(fid);