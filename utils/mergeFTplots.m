function [ ]=mergeFTplots(h1,h2,varargin)
cleanUp=true;
if mod(length(varargin),2)==0 && length(varargin)>0
    switch varargin{1}
        case {'groupNames'}
            groupNames=varargin{1+1}
        case {'cleanUp'};
            cleanUp=varargin{1+1};
    end
end
fields1=fieldnames(h1);
fields2=fieldnames(h2);
if length(fields2)<=length(fields1) && sum(ismember(fields1,fields2))==length(fields2) % all fields in h2 should be contained in h1 for merging
    for f=1:length(fields2)
         mergePlots(h1.(fields2{f}),h2.(fields2{f}),'noRepetition');
         theLegend=findobj(h1.(fields2{f}),'type','legend');
         raw=contains(theLegend.String{1},'ch');
         number=length(theLegend.String);
        switch (fields2{f})
            case {'force'}
             legendNames= produceForceLegends(number,raw)   ;             
            case{'torque'}
             legendNames= produceTorqueLegends(number,raw);
                
            case{'axis1','axis2','axis3','axis4','axis5','axis6'}
                name=(theLegend.String{1});
              legendNames=  produceLegendNames(number,name);
        end
        legend(findobj(h1.(fields2{f}),'type','axes'),legendNames);
        legendmarkeradjust(h1.(fields2{f}),20);
        if cleanUp
            close(h2.(fields2{f}));
        end
    end
else
    error('Some fields of the sencond set of handles do not exist in the handle to merge');
end




    function [legendNames]=produceForceLegends(number,raw)        
        if raw
        legendnames={'ch1','ch2','ch3'};
        else
        legendnames={'F_{x}','F_{y}','F_{z}'};
        end
        if mod(number,3)==0
        legendNames=createLegendNames(number,legendnames)
        else
            error('Expecting multiples of 3');
        end
    end

    function [ legendNames]=produceTorqueLegends(number,raw) 
        if raw
        legendnames={'ch4','ch5','ch6'};
        else
        legendnames={'\tau_{x}','\tau_{y}','\tau_{z}'};
        end
         if mod(number,3)==0        
        legendNames=createLegendNames(number,legendnames)
        else
            error('Expecting multiples of 3');
         end      
    end

    function [legendNames ]=produceLegendNames(number,name)
        for n=1:number
            legendNames{n}=appendNumberAtEnd(name,n);
        end
    end

    function [legendNames]=createLegendNames(number,legendnames)        
            counter=1;
        for set=1:number/3
            for axis=1:3
            legendNames{counter}=appendNumberAtEnd((legendnames{axis}),set);
            counter=counter+1;
            end
        end   
    end

    function [newString]=appendNumberAtEnd(s,number)
        newString=strcat(s,'_',num2str(number));
    end

    function [newString]=appendNameAtEnd(s,name)
        newString=strcat(s,'_{',name,'}');
    end

end