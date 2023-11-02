classdef Gripper < handle  % Assuming the gripper is a handle class for reference behavior.
    properties
        plyFileNameStemBase = 'RG6_v3_Base_v3_ply';
        plyFileNameStemLeftFinger = 'RG6_v3_L1_v3_ply';
        plyFileNameStemLeftFingerTip = 'RG6_v3_L2_v3_ply';
        plyFileNameStemRightFinger = 'RG6_v3_L1F_v3ply';
        plyFileNameStemRightFingerTip = 'RG6_v3_L2F_v3_ply';
        
        left;
        right;
        useTool = true;
        toolFilename = 'GripperTool.ply';  % Assuming a tool for the gripper (change as needed).
        model;  % Might be needed depending on your implementation and use of 'PlotAndColourRobot'
    end
    
    methods
        %% Initialization function 
        function self = Gripper(baseTr)
            if nargin < 1
                baseTr = eye(4);				
            end
            
            self.CreateModel();

            % Assuming the base is centered and fingers are symmetrically positioned:
            distance_from_center = 0.13765/2;  % Half of the base width (assuming symmetry)
            self.left.base = transl(-distance_from_center, 0, 0);
            self.right.base = transl(distance_from_center, 0, 0);
            
            self.PlotAndColourRobot();  % If you have this function defined elsewhere
        end
        
        function CreateModel(self)
            % Define the link parameters for the gripper fingers

            % Link 1 - Finger Length
            link(1) = Link('d', 0, 'a', 0.11484, 'alpha', pi/2, 'qlim', deg2rad([-30 30]), 'offset', 0);  
    
            % Link 2 - Finger Tip
            link(2) = Link('d', 0, 'a', 0.07735, 'alpha', 0, 'qlim', deg2rad([-30 30]), 'offset', 0);  
    
            % Create left and right fingers
            self.left = SerialLink(link, 'name', 'left_finger');
            self.right = SerialLink(link, 'name', 'right_finger');
        end
        
        % Add the function 'PlotAndColourRobot' if you have it, to handle visualization.
        
    end
end
