function []= mergePlots(h1,h2,varargin)
%function to merge the plots in h2 into h1
adjustColors=false;
if mod(length(varargin),2)==0
    for v=1:length(varargin)
        switch varargin{v}
            case {'colorOrder'}
                adjustColors=true;
                colorOrder=varargin{v+1}
            case {'noRepetition'}
                adjustColors=true;
        end
    end
else
    for v=1:length(varargin)
        switch varargin{v}
            case {'noRepetition'}
                adjustColors=true;
        end
    end
end
        Lineh2 = findobj(h2,'type','line'); %gives them in reverse order in which they were inputed.
        otherLegend=findobj(h2,'type','legend');
        %Lineh1 = findobj(h1,'type','line');
        h1Axes=findobj(h1,'type','axes')
        copyobj(Lineh2,h1Axes);
        newLines = findobj(h1,'type','line');
        theLegend=findobj(h1,'type','legend');
        % handle legends (TODO: think of a better way to avoid repetition)
        legend(h1Axes,[theLegend.String,otherLegend.String]);          
        % handle colors
        if adjustColors
            if exist('colorOrder','var')
                count=1;
                for l=length(newLines):-1:1
                    modL=mod(count,length(colorOrder));
                    newLines(l).Color=colorOrder(modL,:);
                    count=count+1;
                end
            else
                %check that colors are not repeated
                for l=length(newLines):-1:1
                    colorsUsed(l,:)=newLines(l).Color;
                end
                uniqueColors=unique(colorsUsed,'rows');
                if size(uniqueColors,1)<length(newLines) %some colors are repeated
                    colorOrder=selectColors(length(newLines));
                    count=1;
                    for l=length(newLines):-1:1
                        newLines(l).Color=colorOrder(count,:);
                        count=count+1;
                    end
                end
            end
        end
        legendmarkeradjust(h1,20);
        
    function [colorOrder]=selectColors(numberNeeded)
        switch numberNeeded
            case num2cell(21:64)
                colorOrder=parula(numberNeeded);
            case num2cell(20)
                rgb20=[
                    0.2235    0.2314    0.4745;
                    0.3216    0.3294    0.6392;
                    0.4196    0.4314    0.8118;
                    0.6118    0.6196    0.8706;
                    0.3882    0.4745    0.2235;
                    0.5490    0.6353    0.3216;
                    0.7098    0.8118    0.4196;
                    0.8078    0.8588    0.6118;
                    0.5490    0.4275    0.1922;
                    0.7412    0.6196    0.2235;
                    0.9059    0.7294    0.3216;
                    0.9059    0.7961    0.5804;
                    0.5176    0.2353    0.2235;
                    0.6784    0.2863    0.2902;
                    0.8392    0.3804    0.4196;
                    0.9059    0.5882    0.6118;
                    0.4824    0.2549    0.4510;
                    0.6471    0.3176    0.5804;
                    0.8078    0.4275    0.7412;
                    0.8706    0.6196    0.8392;];
                colorOrder=rgb20;
                
            case num2cell(14:19)
                my_colorOrder19=[
                    254,178,76
                    253,141,60
                    252,78,42
                    227,26,28
                    189,0,38
                    128,0,38
                    158,202,225
                    107,174,214
                    66,146,198
                    33,113,181
                    8,81,156
                    8,48,107
                    161,217,155
                    116,196,118
                    65,171,93
                    35,139,69
                    0,109,44
                    0,68,27
                    102,37,6
                    ]/255;
                %
                colorOrder=my_colorOrder19;
                
            case num2cell(10:13)
                my_colorOrder13=[
                    253,141,60;
                    252,78,42;
                    227,26,28;
                    177,0,38;
                    65,182,196;
                    29,145,192;
                    34,94,168;
                    12,44,132;
                    116,196,118;
                    65,171,93;
                    35,139,69;
                    0,90,50;
                    140,45,4;
                    ]/255;
                colorOrder=my_colorOrder13;
                
            case num2cell(8:9)
                my_colorOrder=[lines(7);
                    0.8745    0.3961    0.6902;
                    0.6510    0.3373    0.1569;];
                colorOrder=my_colorOrder;
                
            case num2cell(1:7)
                colorOrder=colormap(lines);
        end