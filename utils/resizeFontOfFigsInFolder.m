%  /media/fandradechavez/OS/Users/fandradechavez/Documents/GitHub/PhD/NewFontFigs/bigChanges
fontSize=24;
markerSize=30;

%%
inputFolder = uigetdir;
outputFolder = uigetdir;
%read files with .fig ending
list=dir([inputFolder,'/*.fig'])
for fileN=1:size(list,1)
   fid= open([inputFolder,'/',list(fileN).name]);
      axis tight
% set font size to 14
set(gca,'FontSize',fontSize)
   % set marker size
   try 
  l= legendmarkeradjust(markerSize,fontSize);
   catch ME
       
   end

saveas(gcf,[outputFolder,'/',list(fileN).name(1:end-4)],'epsc')
%pause
close(fid);
end

%%
[inputFile,pathfile] = uigetfile;
outputFolder = uigetdir;
   fid= open([pathfile,'/',inputFile]);
      axis tight
% set font size to 14
fontSize=14;

set(gca,'FontSize',fontSize)
   % set marker size
   try 
  l= legendmarkeradjust(markerSize,fontSize);
   catch ME
       
   end

saveas(gcf,[outputFolder,'/',inputFile(1:end-4)],'epsc')
% pause
close(fid);
