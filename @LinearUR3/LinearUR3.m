%% Industrial Robotics Lab 1 Assignment
% Modified LinearUR5 to LinearUR3
% Gavin Liang (13205430)

classdef LinearUR3 < RobotBaseClass
    %% LinearUR3 UR3 on a non-standard linear rail modified from the LinearUR5 file 

    properties(Access = public)              
        plyFileNameStem = 'LinearUR3';
    end
    
    methods
%% Define robot Function 
        function self = LinearUR3(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
          
            % Add Robotiq 2F 140 Gripper End-Effector
            % Robotiq 2F 140 https://robotiq.com/support
            % self.useTool = true;
            % self.toolFilename = ['Gripper.ply'];
            
            self.PlotAndColourRobot();
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR3 model mounted on a linear rail
            % Syntax: DH = [THETA D A ALPHA SIGMA OFFSET]
            % SIGMA = 1 is Prismatic
            link(1) = Link([pi  0         0        pi/2  1]); % PRISMATIC Link
            link(2) = Link([0   0.1519    0        pi/2  0]);
            link(3) = Link([0   0        -0.24365  0     0]);
            link(4) = Link([0   0        -0.21325  0     0]);
            link(5) = Link([0   0.11235   0        pi/2  0]);
            link(6) = Link([0   0.08535   0       -pi/2	 0]);
            link(7) = Link([0   0.0819    0        0     0]);

            % Incorporate joint limits
            % link(1).qlim = [0.01 0.8];
            link(1).qlim = [-1.5 0];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-170 170]*pi/180;
            link(5).qlim = [-350 350]*pi/180;
            link(6).qlim = [-350 350]*pi/180;
            link(7).qlim = [-350 350]*pi/180;
        
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
    end
end