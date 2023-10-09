%% Modified from RobotCow.m and Lab1 Code

classdef insertModel < handle
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 1.5;
    end
    
    properties % Defaults
        modelCount = 1; % Number of models to insert
       
        robotModel; % cell structure of models

        modelName = 'Whisky'; % String name for model (name of model file)

        basePose = transl(0, 0, 0) * trotx(0) * troty(0) * trotz(0); % Location of model to be placed
        
        %> workspaceSize in meters
        workspaceSize = [3.5,3.5];        
        
        %> Dimensions of the workspace in regard to the workspace size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = insertModel(modelName, modelCount, basePose)
            if 0 < nargin
                self.modelCount = modelCount;
                self.modelName = modelName;
                self.basePose = basePose;
            end
            
            self.workspaceDimensions = [-self.workspaceSize(1)/2, self.workspaceSize(1)/2 ...
                                       ,-self.workspaceSize(2)/2, self.workspaceSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of models
            for i = 1:self.modelCount
                [self.robotModel{i}, colour] = self.GetModel(self.modelName, i);
                
                self.robotModel{i}.base = self.robotModel{i}.base.T * basePose{i} * trotx(-pi/2);

                 % Plot 3D model
                plot3d(self.robotModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                
                if i == 1 
                    hold on;
                end
              
                % Taken from RobotBaseClass 
                handles = findobj('Tag', self.robotModel{i}.name);
                h = get(handles,'UserData');
                % From RobotBaseClass -> h.link(linkIndex + 1),
                % Since model is link1 (from GetModel)
                % linkIndex = 1, therefore h.link(2)
                h.link(2).Children.FaceVertexCData = colour;
                h.link(2).Children.FaceColor = 'interp';
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end   
    end
    
    methods (Static)
        %% GetBrickModel, renamed half red green brick to just Brick.ply
        function [model, vertexColours] = GetModel(name, index)
            [f,v, data] = plyread(strcat(name, '.ply'),'tri');
            link1 = Link('alpha',pi/2,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',[name, num2str(index)]);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], f};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], v};

            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        end
    end    
end