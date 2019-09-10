function [H]=FTplots(data,time,varargin)
% This function plots the information received from an FT sensor be it the wrenches or the raw data.
% It assumes wrenches are in a double matrix (time x wrench), wrench [F,T]
% The x axis is the relative time of the experiment. This means it
% considers the first sample as time 0 always.
%% inputs
% data: the actual information of the sensor
% time: a vector the same size as the rows of data containing the timestamp
% varargin: this allows to receive multiple configuration parameters.
%  if varargin is a string it assumes it is one of the configuration
%  variables otherwise it assumes that is the desired name for the
%  reference data
%  if varagin is a struct it assumes is another set of FT sensor data that
%  will be used to compare the main data information
%% configurations
%  onlyFOrce: if enabled it will only plot the forces (first 3 columns)
%  raw: if enabled it will change the legends to reflect the raw channels
%  byChannel: if enable it will generate a different plot for every axis or
%  channel.
onlyForce=false;
raw=false;
byChannel=false;
reference={};
referenceName='reference';
showDifference=false;
referenceTime=[];
xAxisOption='TimeStamp';
xAxisOffset=true;
if (length(varargin)==1)
    if(ischar(  varargin{1}))
        switch varargin{1}
            case {'onlyForce','OnlyForce','onlyforce'}
                onlyForce=true;
            case {'raw','Raw','RAW'}
                raw=true;
            case {'byChannel','ByChannel','bychannel'}
                byChannel=true;
            case {'forceComparison','forcecomparison','ForceComparison'}
                showDifference=true;
            case {'noTimeStamp','NOTIMESTAMP','USESAMPLES','useSamples'}
                xAxisOption='Samples';
            case {'noOffsetOnX','xAxisAsIs','noTimeOffset'}
                xAxisOffset=false;
            otherwise
                warning('FTplots: Unexpected option going by default options.')
        end
    end
    if(isstruct(varargin{1}))
        reference=varargin{1};
    end
else
    if (length(varargin)>1 && length(varargin)<9)
        for count=1:length(varargin)
            if(ischar(  varargin{count}))
                switch varargin{count}
                    case {'onlyForce','OnlyForce','onlyforce'}
                        onlyForce=true;
                    case {'raw','Raw','RAW'}
                        raw=true;
                    case {'byChannel','ByChannel','bychannel'}
                        byChannel=true;
                    case {'showDifference','showdifference','ShowDifference'}
                        showDifference=true;
                    case {'noTimeStamp','NOTIMESTAMP','notimestamp','USESAMPLES','useSamples','usesamples'}
                        xAxisOption='Samples';
                    case {'noOffsetOnX','xAxisAsIs','noTimeOffset'}
                        xAxisOffset=false;
                    otherwise
                        referenceName=varargin{count};
                end
            else
                if(isstruct(varargin{count}))
                    reference=varargin{count};
                else
                    if (isvector(varargin{count}))
                        referenceTime=varargin{count};
                    else                        
                    warning('FTplots: Not an option for FTplots. Ignored');
                    end
                end
            end
        end
    end
end



% xPlotOptions = 'r.';
% yPlotOptions = 'g.';
% zPlotOptions = 'b.';
% x2PlotOptions = 'c.';% 'm.';
% y2PlotOptions = {'Color',[0.5412 0.1686 0.8863],'Marker','.','LineStyle','none'};
% %z2PlotOptions = 'y.';% 'c.';
% z2PlotOptions = {'Color',[1.0000 0.5490 0],'Marker','.','LineStyle','none'};
% % z2PlotOptions{:}
colors=[ 0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;];

xPlotOptions = {'Color',colors(1,:),'Marker','.','LineStyle','none'};
yPlotOptions ={'Color',colors(2,:),'Marker','.','LineStyle','none'};
zPlotOptions = {'Color',colors(3,:),'Marker','.','LineStyle','none'};
x2PlotOptions = {'Color',colors(4,:),'Marker','.','LineStyle','none'};
y2PlotOptions = {'Color',colors(5,:),'Marker','.','LineStyle','none'};
%z2PlotOptions = 'y.';% 'c.';
z2PlotOptions = {'Color',colors(6,:),'Marker','.','LineStyle','none'};
% z2PlotOptions{:}

% use colors from colormap(lines)
colors=[ 0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;];

locationLegend='northeast';
fields=fieldnames(data);
if ~isempty(reference)
    rfields=fieldnames(reference);
end

if (isempty(referenceTime) && ~isempty(reference) && (length(time) == size(reference.(rfields{1}),1)))
    referenceTime=time    ;
else
    if(~isempty(reference) && (length(referenceTime) ~= size(reference.(rfields{1}),1)))
    error('FTplots: mismatch between size of the time vector for the reference and the reference size');
    end
end

if (size(fields,1)==2 && showDifference && isempty(reference))
    temp.(fields{1})=data.(fields{1});
    reference.(fields{1})=data.(fields{2});
    referenceName=(fields{2});
    referenceTime=time ;
    rfields=fieldnames(reference);
    fields=fieldnames(temp);
    %FTplots(temp,time,(fields{2}),reference);
end

if  strcmp(xAxisOption,'TimeStamp')
    if xAxisOffset
        xAxis=time-time(1);
        if ~isempty(reference)
            xAxisReference=referenceTime-referenceTime(1);
        end
    else
        xAxis=time;
        if ~isempty(reference)
            xAxisReference=referenceTime;
        end
    end
else
    xAxis=1:length(time);
    if ~isempty(reference)
        xAxisReference=1:length(referenceTime);
    end
end

if (~byChannel && isempty(reference))
    for i=1:size(fields,1)
         H.force=figure('WindowStyle','docked');
        plot(xAxis,data.(fields{i})(:,1),xPlotOptions{:});hold on;
        plot(xAxis,data.(fields{i})(:,2),yPlotOptions{:});hold on;
        plot(xAxis,data.(fields{i})(:,3),zPlotOptions{:});hold on;
        if raw
             legend('ch1','ch2','ch3','Location',locationLegend);
        else
            legend('F_{x}','F_{y}','F_{z}','Location',locationLegend);
        end
        title(escapeUnderscores((fields{i})));
        xlabel(xAxisOption);
        ylabel('N');
        legendmarkeradjust(20);
        grid on;
        axis tight;
    end
    if(~onlyForce)
        for  i=1:size(fields,1)
            H.torque= figure('WindowStyle','docked');
            plot(xAxis,data.(fields{i})(:,4),x2PlotOptions{:});hold on;
            plot(xAxis,data.(fields{i})(:,5),y2PlotOptions{:});hold on;
            plot(xAxis,data.(fields{i})(:,6),z2PlotOptions{:});hold on;
            if raw               
                legend('ch4','ch5','ch6','Location',locationLegend);
            else
                legend('\tau_{x}','\tau_{y}','\tau_{z}','Location',locationLegend);
            end
            title(escapeUnderscores((fields{i})));
            xlabel(xAxisOption);
            ylabel('Nm');
            legendmarkeradjust(20);
            grid on;
            axis tight;
        end
    end
end

if (~isempty(reference) && ~byChannel)
    for i=1:size(fields,1)
        H.force= figure('WindowStyle','docked');
        plot(xAxis,data.(fields{i})(:,1),xPlotOptions{:});hold on;
        plot(xAxis,data.(fields{i})(:,2),yPlotOptions{:});hold on;
        plot(xAxis,data.(fields{i})(:,3),zPlotOptions{:});hold on;
        plot(xAxisReference,reference.(rfields{i})(:,1),x2PlotOptions{:});hold on;
        plot(xAxisReference,reference.(rfields{i})(:,2),y2PlotOptions{:});hold on;
        plot(xAxisReference,reference.(rfields{i})(:,3),z2PlotOptions{:});hold on;
        if raw
            legend('ch1','ch2','ch3','ch1_2','ch2_2','ch3_2','Location',locationLegend);
        else
            legend('F_{x}','F_{y}','F_{z}','F_{x2}','F_{y2}','F_{z2}','Location',locationLegend);
        end
        title(escapeUnderscores( strcat((fields{i}),{' and  '},referenceName,{' '},(rfields{i}))));
        xlabel(xAxisOption);
        ylabel('N');
        legendmarkeradjust(20);
        grid on;
        axis tight;
    end
    if(~onlyForce)
        for  i=1:size(fields,1)
            H.torque= figure('WindowStyle','docked');
            plot(xAxis,data.(fields{i})(:,4),x2PlotOptions{:});hold on;
            plot(xAxis,data.(fields{i})(:,5),y2PlotOptions{:});hold on;
            plot(xAxis,data.(fields{i})(:,6),z2PlotOptions{:});hold on;
            plot(xAxisReference,reference.(rfields{i})(:,4),xPlotOptions{:});hold on;
            plot(xAxisReference,reference.(rfields{i})(:,5),yPlotOptions{:});hold on;
            plot(xAxisReference,reference.(rfields{i})(:,6), zPlotOptions{:});hold on;
            if raw                
                legend('ch4','ch5','ch6','ch4_2','ch5_2','ch6_2','Location',locationLegend);
            else
                legend('\tau_{x}','\tau_{y}','\tau_{z}','\tau_{x2}','\tau_{y2}','\tau_{z2}','Location',locationLegend);
            end
            title(escapeUnderscores( strcat((fields{i}),{' and  '},referenceName,{' '},(rfields{i}))));
            xlabel(xAxisOption);
            ylabel('Nm');
            legendmarkeradjust(20);
            grid on;
            axis tight;
        end
    end
    if (length(time)== length(referenceTime) && showDifference)
    if (sum(referenceTime==time)==length(time))
        for  i=1:size(fields,1)
            H.forceDiff=figure('WindowStyle','docked');
            plot(xAxis,abs(data.(fields{i})(:,1))-abs(reference.(rfields{i})(:,1)),xPlotOptions{:});hold on;
            plot(xAxis,abs(data.(fields{i})(:,2))-abs(reference.(rfields{i})(:,2)),yPlotOptions{:});hold on;
            plot(xAxis,abs(data.(fields{i})(:,3))-abs(reference.(rfields{i})(:,3)),zPlotOptions{:});hold on;
            if raw
                
                legend('ch1','ch2','ch3','Location',locationLegend);
            else
                legend('F_{x}','F_{y}','F_{z}','Location',locationLegend);
            end
            title(escapeUnderscores( strcat((fields{i}),{' -  '},referenceName,{' '},(rfields{i}))));
            xlabel(xAxisOption);
            ylabel('N');
            legendmarkeradjust(20);
            grid on;
            axis tight;
        end
        if(~onlyForce)
            for  i=1:size(fields,1)
                H.torqueDiff=figure('WindowStyle','docked');
                plot(xAxis,abs(data.(fields{i})(:,4))-abs(reference.(rfields{i})(:,4)),x2PlotOptions{:});hold on;
                plot(xAxis,abs(data.(fields{i})(:,5))-abs(reference.(rfields{i})(:,5)),y2PlotOptions{:});hold on;
                plot(xAxis,abs(data.(fields{i})(:,6))-abs(reference.(rfields{i})(:,6)),z2PlotOptions{:});hold on;
                if raw
                    legend('ch4','ch5','ch6','Location',locationLegend);
                else
                    legend('\tau_{x}','\tau_{y}','\tau_{z}','Location',locationLegend);
                end
                title(escapeUnderscores( strcat((fields{i}),{' -  '},referenceName,{' '},(rfields{i}))));
                xlabel(xAxisOption);
                ylabel('Nm');
                legendmarkeradjust(20);
                grid on;
                axis tight;
            end
        end
    end
    end
end

if (byChannel)
    if raw
        legendNames={'ch1','ch2','ch3','ch4','ch5','ch6'};
    else
        legendNames={'F_{x}','F_{y}','F_{z}','\tau_{x}','\tau_{y}','\tau_{z}'};
    end
    if onlyForce
        count=3;
    else
        count=6;
    end
    for i=1:size(fields,1)
        for n=1:count
            H.(strcat('axis',num2str(n)))= figure('WindowStyle','docked');
            plot(xAxis,data.(fields{i})(:,n),xPlotOptions{:});
            grid on;
            axis tight;
            if ~isempty(reference)
                hold on; plot(xAxisReference,reference.(rfields{i})(:,n),yPlotOptions{:});
                legend((legendNames{n}),strcat((legendNames{n}),'_2'));
                title(strcat((legendNames{n}),{' : '},escapeUnderscores((fields{i})),{' and  '},escapeUnderscores(strcat(referenceName,{' '},(rfields{i})))));
            else
                legend((legendNames{n}));
                title(strcat((legendNames{n}),{' : '},escapeUnderscores((fields{i}))));
            end           
            legendmarkeradjust(20);
            xlabel(xAxisOption);
            if n<4
                ylabel('N');
            else
                ylabel('Nm');
            end
        end
    end
    if showDifference
        if (length(time)== length(referenceTime))
            if (sum(referenceTime==time)==size(time,1))
                for i=1:size(fields,1)
                    for n=1:count
                        H.(strcat('axis',num2str(n),'Diff'))=figure('WindowStyle','docked'),
                        plot(xAxis,abs(data.(fields{i})(:,n))-abs(reference.(rfields{i})(:,n)),xPlotOptions{:});hold on;
                        grid on;
                        axis tight;
                        legend(strcat((legendNames{n}),' error'));
                        title(strcat((legendNames{n}),{' : '},escapeUnderscores((fields{i})),{' - '},escapeUnderscores(strcat(referenceName,{' '},(rfields{i})))));                        
                        xlabel(xAxisOption);
                        legendmarkeradjust(20);
                        if n<4
                            ylabel('N');
                        else
                            ylabel('Nm');
                        end
                    end
                end
            end
        end
    end
end