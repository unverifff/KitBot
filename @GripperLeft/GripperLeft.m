classdef GripperLeft < RobotBaseClass  % Assuming the gripper is a handle class for reference behavior.

    properties(Access = public)
        plyFileNameStem = 'GripperLeft';

        % left;
        % right;
    end

    methods
        %% Initialization function 
        function self = GripperLeft(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);				
            end

            self.model.base = self.model.base.T * baseTr;

            self.PlotAndColourRobot(); 
        end

        function CreateModel(self)
            % Define the link parameters for the gripper fingers

            link(1) = Link('d',0.13,'a',-0.02,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);

            % Link 2 - Finger Length
            link(2) = Link('d', 0, 'a', -0.08, 'alpha', 0, 'qlim', deg2rad([-30 30]), 'offset', 0);  
            
            % Link 3 - Finger Tip
            link(3) = Revolute('d', 0, 'a', 0.0, 'alpha', 0, 'offset', 0); 

            % % Create left and right fingers
            % self.left = SerialLink(link, 'name', 'left_finger');
            % self.right = SerialLink(link, 'name', 'right_finger');
            % 
            % % Assuming the base is centered and fingers are symmetrically positioned:
            % distance_from_center = 0.13765/2;  % Half of the base width (assuming symmetry)
            % self.left.base = transl(-distance_from_center, 0, 0);
            % self.right.base = transl(distance_from_center, 0, 0);

            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-0.1 60]*pi/180;
            link(3).qlim = [-60 0]*pi/180;


            self.model = SerialLink(link, 'name', self.name);
            % self.left = SerialLink(link, 'name', self.name);
            % self.right = SerialLink(link, 'name', self.name);
            % 
            % self.right.base = trotz(pi);
            

        end

        % Add the function 'PlotAndColourRobot' if you have it, to handle visualization.

        function operateGripper(self, robotName, open, workspace, scale)
            % Have graphical gripper on UR3 Wrist, inline with Robotiq model
            rfkine = robotName.model.fkine(robotName.model.getpos()); 
            f1.base = rfkine.T * troty(-pi/2) * trotx(-pi/2);
            f2.base = rfkine.T * troty(-pi/2) * trotx(-pi/2);

            % Open State Gripper Joint Angles
            q_f1 = [-deg2rad(45), 0, deg2rad(45)]; 
            q_f2 = [deg2rad(45), 0, -deg2rad(45)];
            % Closed State Gripper Joint Angles (Matching Width of Bricks)
            q_f1Closed = [-deg2rad(22.5), 0, deg2rad(22.5)];
            q_f2Closed = [deg2rad(22.5), 0, -deg2rad(22.5)];

            % Initial State of Gripper
            if open == true % Open State
                f1.plot(q_f1,'workspace', workspace, 'scale', scale, 'nowrist', 'noname');
                hold on
                f2.plot(q_f2,'workspace', workspace, 'scale', scale, 'nowrist', 'noname');
                hold on
            elseif open == false % Closed State
                f1.plot(q_f1Closed,'workspace', workspace, 'scale', scale, 'nowrist', 'noname');
                hold on
                f2.plot(q_f2Closed,'workspace', workspace, 'scale', scale, 'nowrist', 'noname');
                hold on
            end



        end
    end

end


