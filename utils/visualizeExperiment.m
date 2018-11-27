function []=visualizeExperiment(dataset,input,sensorsToAnalize,varargin)
%Author: Francisco Andrade
%% This function has the aim of been able to see the icub posture while seen the devolpment of the ft forces in the wrench space
%Input
% dataset: has the joint and forces information
% input: contains other required variables such as robotName and fixedFrame
%robotName: is used to load the robot model to know which joints to add the
% visualization part, it is asumed its the same type of robot than the one found in
% model.urdf
%fixedFrame: is to give a reference for the visualizer which frame is fixed
%sensorsToAnalize: is to plot only the relevant sensors
%varargin: it enables certain behaviours depending on what it receives
% video: creates a video of the forces and the iCub moving based on the
% given dataset
% testDir: tells the program if you are in the main folder or a subfolder
% torque: selects torques to be ploted instead of forces
%TODO: extend to the use of intervals for selection of the fixed frame
%TODO: a way to add also the reference from world to fixed frame
% For using fully the functionality of the button save we should declare
%Remark: install also the irrlicht library (sudo apt install
%libirrlicht-dev ) required , and enable the `IDYNTREE_USES_MATLAB` and `IDYNTREE_USES_IRRLICHT`
%testDir=false; % true if in test directory
%TODO read the following 3 options from varargin
video=false;
fixedFrame='root_link';
n=100;
testDir=false;
skipNextIteration=false;
useTorque=false;
%% deal with extra variables
if (length(varargin)==1)
    if(ischar(  varargin{1}))
        switch varargin{1}
            case {'video','VIDEO','Video'}
                video=true;
            case {'testDir','TestDir','testdir'}
                testDir=true;
            case {'torque','TORQUE','Torque'}
                useTorque=true;
            otherwise
                warning('Unexpected option going by default options.')
        end
    end
else
    if (length(varargin)>1)
        for count=1:length(varargin)
            if (~skipNextIteration)
                
                if(ischar(  varargin{count}))
                    switch varargin{count}
                        case {'video','VIDEO','Video'}
                            video=true;
                        case {'testDir','TestDir','testdir'}
                            testDir=true;
                        case {'torque','TORQUE','Torque'}
                            useTorque=true;
                        case {'fixedFrame','contactFrame','FixedFrame','ContactFrame'}
                            if(length(varargin)>count)
                                fixedFrame=varargin{count+1};
                                skipNextIteration=true;
                                continue;
                            else
                                warning('Missing the actual name of the contact frame.')
                            end
                        otherwise
                            warning('Unexpected option going by default options.')
                    end
                else
                    if(isnumeric(  varargin{count}))
                        n=round(varargin{count});
                    end
                end
            else
                skipNextIteration=false;
                continue
            end
        end
    end
end
global storedInis storedEnds storedTimeInis storedTimeEnds intervalIni
%% start
if testDir
    currentDir=pwd;
    cd ('../')
end
addpath external/iCubViz

%selecting which ft Data to plot
if (any(strcmp('ftDataNoOffset', fieldnames(dataset))))
    whichFtData='ftDataNoOffset';
else
    if (any(strcmp('filteredFtData', fieldnames(dataset))))
        whichFtData='filteredFtData';
    else
        whichFtData='ftData';
    end
end

if (any(strcmp('estimatedFtData', fieldnames(dataset))))
    estimatedAvailable=true;
else
    estimatedAvailable=false;
end
if(~input.calibFlag && estimatedAvailable)
    whichFtData='estimatedFtData';
    disp('calibFlag is off and estimatedFT is available. Only estimated data will be plotted.')
    estimatedAvailable=false;
end

%% Getting names to put into visualizer
% Create estimator class
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% % Load model and sensors from the URDF file
% estimator.loadModelAndSensorsFromFile(strcat('./robots/',input.robotName,'.urdf'));

dofs = length(dataset.jointNames);
consideredJoints = iDynTree.StringVector();
for i=1:dofs %-4 ensures avoiding the 3 last neck joints
 
    consideredJoints.push_back( (dataset.jointNames{i}));
end
for i=1:length(input.sensorNames)
    consideredJoints.push_back( (input.sensorNames{i}));    
end

estimatorLoader = iDynTree.ModelLoader();
estimatorLoader.loadReducedModelFromFile(strcat('./robots/',input.robotName,'.urdf'),consideredJoints);
estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);

%% set iCubViz variables
cd ('external/iCubViz/')
mdlLdr = iDynTree.ModelLoader();
%mdlLdr.loadReducedModelFromFile(strcat('external/iCubViz/','model.urdf'),consideredJoints);
mdlLdr.loadReducedModelFromFile('model.urdf',consideredJoints);
model = mdlLdr.model();
viz3 = iDynTree.Visualizer();
viz3.init();
viz3.addModel(model,'icub');
viz3.draw();
cd ('../../')
%camara positioning
if (strcmp('root_link',fixedFrame))
    cPos=iDynTree.Position(-.65,-.2,1); %depends on the initial position of the reference frame. Root link has a -x direction
else
    cPos=iDynTree.Position(.65,.2,1); %depends on the initial position of the reference frame. Root link has a -x direction
end
cTarg=iDynTree.Position(0,0,0.5); %depends on the initial position of the reference frame. Root link has a -x direction
viz3.camera().setTarget(cTarg)
viz3.camera().setPosition(cPos);
%% Setting axis and handlers for forces plots
H.handle=figure,
for indx=1:length(sensorsToAnalize)
    ft =sensorsToAnalize{indx};
    H.(ft).sub=subplot(1,length(sensorsToAnalize),indx);
    H.(ft).old=plot3(0,0,0); hold on;
    H.(ft).old2=plot3(0,0,0);
    
    %get axis values
    if estimatedAvailable
        if useTorque
            minF=[min(dataset.(whichFtData).(ft)(:,4:6));
                min(dataset.estimatedFtData.(ft)(:,4:6))];
            maxF=[max(dataset.(whichFtData).(ft)(:,4:6));
                max(dataset.estimatedFtData.(ft)(:,4:6))];
        else
            minF=[min(dataset.(whichFtData).(ft));
                min(dataset.estimatedFtData.(ft))];
            maxF=[max(dataset.(whichFtData).(ft));
                max(dataset.estimatedFtData.(ft))];
        end
        minF=min(minF);
        maxF=max(maxF);
    else
        if useTorque
            minF=min(dataset.(whichFtData).(ft)(:,4:6));
            maxF=max(dataset.(whichFtData).(ft)(:,4:6));
        else
            minF=min(dataset.(whichFtData).(ft));
            maxF=max(dataset.(whichFtData).(ft));
        end
    end
    %tempMax=max(abs(minF-maxF));
    %H.(ft).minMaxForces=[minF(1),minF(1)+tempMax,minF(2),minF(2)+tempMax,minF(3),minF(3)+tempMax];
    H.(ft).minMaxForces=[minF(1),maxF(1),minF(2),maxF(2),minF(3),maxF(3)];
    
    axis(H.(ft).minMaxForces);
    %axis equal;
end
%% Start variables for viz
jointPos = iDynTree.JointPosDoubleArray(model);
% Assuming that the l_sole frame is fixed and it is the world, compute the
% world_H_base that correspond to the specified joints
odom = iDynTree.SimpleLeggedOdometry();
odom.setModel(model);
%odom.init(fixedFrame,'r_sole');
odom.updateKinematics(jointPos);
odom.init(fixedFrame,fixedFrame);
%% Plot and visualize at each time sample
if(~video)
    storedInis=[];
    storedEnds=[];
    storedTimeInis=0;
    storedTimeEnds=(dataset.time(end)-dataset.time(1));
    
    intervalIni=true;
    
    uiHandles.txt = uicontrol('Parent',H.handle,'Style','text',...
        'Position',[0,5,75,32],...            ..
        'String',"S ="+num2str(0)+newline+sprintf('t= %.2f',(0)));
    
    uiHandles.edtxt = uicontrol('Parent',H.handle,'Style','edit',...
        'Position',[5,59,50,32],...
        'String',num2str(0));
    
    uiHandles.sliderHandle = uicontrol('Parent',H.handle,'Style','slider',...
        'Units','normalized','Position',[0.15,0,0.75,0.03],...
        'value',1, 'min',1, 'max',length(dataset.time),...
        'SliderStep', [1/length(dataset.time) 1/length(dataset.time)]);

    if useTorque
        set(uiHandles.sliderHandle, 'Callback', {@callBackSlider,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,uiHandles,'torque'});
        set(uiHandles.edtxt, 'CallBack',{@editText,uiHandles,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,'torque'});
    else
        set(uiHandles.sliderHandle, 'Callback', {@callBackSlider,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,uiHandles});
        set(uiHandles.edtxt, 'CallBack',{@editText,uiHandles,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model});
    end
    % create save button
    saveButton = uicontrol('Style', 'pushbutton', 'String', 'Save',...
        'Position', [10 40 40 20],...
        'Callback', {@callBackSaveButton,dataset,uiHandles.sliderHandle,viz3});
    % create button group
    radioButtonsGroup = uibuttongroup(H.handle,'Visible','off',...
        'Position',[0.9 0 0.1 .05],...
        'SelectionChangedFcn',@bselection);
    
    % Create 2 radio buttons in the button group.
    radioButton1 = uicontrol(radioButtonsGroup,'Style',...
        'radiobutton',...
        'String','Ini',...
        'Units','normalized',...
        'Position',[0 0.5 1 0.5],...
        'HandleVisibility','off');
    
    radioButton2 = uicontrol(radioButtonsGroup,'Style','radiobutton',...
        'String','End',...
        'Units','normalized',...
        'Position',[0 0 1 0.5],...
        'HandleVisibility','off');
    
    % Make the uibuttongroup visible after creating child objects.
    radioButtonsGroup.Visible = 'on';
    
else
    init_time = 1;
    baseT=odom.getWorldLinkTransform(odom.model.getDefaultBaseLink());
    %create view vector for rotating the view on the plot figure
    % 360 is a full turn default view starts at -37.5  322.5
    nViews= round(length(dataset.qj(:,1))-init_time)/n;
    views=-37.5:720/nViews:682.5;
    for i=init_time:n:length(dataset.qj(:,1))
        tic
        
        joints = dataset.qj(i,1:dofs-3)';
        jointPos.fromMatlab(joints);
        %      odom.updateKinematics(jointPos);
        %     odom.init(fixedFrame,fixedFrame);
        %viz3.modelViz(0).setPositions(odom.getWorldLinkTransform(model.getDefaultBaseLink()),jointPos);
        viz3.modelViz(0).setPositions(baseT,jointPos);
        viz3.draw();
        t = toc;
        if useTorque
            dataToPlot=4:6;
        else
            dataToPlot=1:3;
        end
        for indx=1:length(sensorsToAnalize)
            ft =sensorsToAnalize{indx};
            subplot( H.(ft).sub)
            h= plot3_matrix(dataset.(whichFtData).(ft)(1:i,dataToPlot),'r');%
            hold on;
            delete(H.(ft).old);
            H.(ft).old=h;
            if estimatedAvailable
                h2= plot3_matrix(dataset.estimatedFtData.(ft)(1:i,dataToPlot),'b');
                delete(H.(ft).old2);
                H.(ft).old2=h2;
                legend('measuredData','estimatedData','Location','west');
            else
                legend('measuredData','Location','west');
            end
            title(strcat({'Wrench space '},escapeUnderscores(ft)));
            if useTorque
                xlabel('\tau_{x}');
                ylabel('\tau_{y}');
                zlabel('\tau_{z}');
            else
                xlabel('F_{x}');
                ylabel('F_{y}');
                zlabel('F_{z}');
            end
            axis(H.(ft).minMaxForces);
            %axis equal;
            grid on;
            axis equal;
            view(views(round(i/n)+1),30);
            drawnow;
            F(i) = getframe(gcf);
        end
        %pause(max(0,0.01-t))
    end
    
    %make the video of the plot.
    v = VideoWriter('forces.avi');
    open(v);
    for k = init_time:n:length(dataset.qj(:,1))
        writeVideo(v,F(k));
    end
    close(v);
end

if testDir
    cd ( currentDir)
end

%% Callback functions
    function callBackSlider(hObject,evt,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,uiHandles,varargin)
        sample = hObject.Value;
        set(uiHandles.txt,'String',"S ="+num2str(round(sample))+newline+sprintf('t= %.2f',(dataset.time(round(sample))-dataset.time(1))));
        set(uiHandles.edtxt,'String',num2str(round(sample)));
        [H]=plotForceAndVizFromSample(sample,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,varargin)
    end

    function bselection(source,event)
        % global   intervalIni ;
        display(['Previous: ' event.OldValue.String]);
        display(['Current: ' event.NewValue.String]);
        display('------------------');
        if strcmp('Ini',event.NewValue.String)
            intervalIni=true;
        else
            intervalIni=false;
        end
    end

    function callBackSaveButton(hObject,evt,dataset,sliderHandle,viz3)
        sliderValue=round(sliderHandle.Value);
        sample=round(sliderValue);
        timeSample=dataset.time(round(sliderValue))-dataset.time(1);
        imageName=strcat('robotPosition_',num2str(sample));
        if intervalIni
            storedInis=[storedInis sample];
            storedTimeInis=[storedTimeInis timeSample];
            storedTimeInis=sort(storedTimeInis);
            imageName=strcat(imageName,'_ini');
        else
            storedEnds=[storedEnds sample];
            storedTimeEnds=[timeSample storedTimeEnds];
            storedTimeEnds=sort(storedTimeEnds);
            imageName=strcat(imageName,'_end');
        end
        viz3.drawToFile(strcat(imageName,'.png'));
    end

    function editText(Hobj,evt,uiHandles,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,varargin)
        a = get(Hobj,'string');
        disp(['The string in the editbox is: ',a])
        sample=str2num(a);
        if sample>uiHandles.sliderHandle.Max
            sample=uiHandles.sliderHandle.Max;
        end
        if sample<uiHandles.sliderHandle.Min
            sample=uiHandles.sliderHandle.Min;
        end
        set(uiHandles.sliderHandle,'Value',sample);
        set(uiHandles.txt,'String',"S ="+num2str(round(sample))+newline+sprintf('t= %.2f',(dataset.time(round(sample))-dataset.time(1))));
        [H]=plotForceAndVizFromSample(sample,dataset,sensorsToAnalize,odom,viz3,H,whichFtData,estimatedAvailable,fixedFrame,jointPos,model,varargin)
    end
end