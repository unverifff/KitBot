%% Modified from insertModel.m
% To call on this class from main Script:
% insertModelAtPose(modelName, poseIndex, basePose);
%     modelName -> Name of PLY model (do not include .ply syntax).
%     poseIndex -> which Pose (from cell) to insert model at. 
%     basePose -> location of modelName.
%         Where basePose = transl(x, y, z) * trotz(rz) * troty(ry) *
%             trotx(rx);

classdef insertModelAtPose < handle
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 3;
    end
    
    properties % Defaults
        poseIndex = 1; % Corresponds to the Pose Index of Pose Cell
       
        robotModel; % cell structure of models

        modelName = 'Whisky'; % String name for model (name of model file)

        basePose = transl(0, 0, 0) * trotx(0) * troty(0) * trotz(0); % Location of model to be placed
        
        %> workspaceSize in meters
        workspaceSize = [4,4];        
        
        %> Dimensions of the workspace in regard to the workspace size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = insertModelAtPose(modelName, poseIndex, basePose)
            if 0 < nargin
                self.poseIndex = poseIndex;
                self.modelName = modelName;
                self.basePose = basePose;
            end
            
            self.workspaceDimensions = [-self.workspaceSize(1), self.workspaceSize(1) ...
                                       ,-self.workspaceSize(2), self.workspaceSize(2) ...
                                       ,0,self.maxHeight];

            % Create the required number of models
            [self.robotModel, colour] = self.GetModel(self.modelName, poseIndex);
            
            self.robotModel.base = self.robotModel.base.T * basePose{poseIndex} * trotx(-pi/2);

             % Plot 3D model
            plot3d(self.robotModel,0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
          
            % Taken from RobotBaseClass 
            handles = findobj('Tag', self.robotModel.name);
            h = get(handles,'UserData');
            % From RobotBaseClass -> h.link(linkIndex + 1),
            % Since model is link1 (from GetModel)
            % linkIndex = 1, therefore h.link(2)
            h.link(2).Children.FaceVertexCData = colour;
            h.link(2).Children.FaceColor = 'interp';

            view(135, 30);
            % axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end    
    end
    
    methods (Static)
        %% GetModel
        function [model, vertexColours] = GetModel(name, index)
            [f,v, data] = plyread(strcat(name, '.ply'),'tri');
            link1 = Link('alpha',pi/2,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name', [name, num2str(index)]);
            
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