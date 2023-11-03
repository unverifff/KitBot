classdef LinearTM12 < RobotBaseClass
    %% LinearTM12 UR5 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'LinearTM12';
    end
    
    methods
%% Define robot Function 
        function self = LinearTM12(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR5 model mounted on a linear rail 0.2659
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.195,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0.18,'a',0.621,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',-0.1297,'a',0.5725,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.106,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.107,'a',0,'alpha',pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.055,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            
            % Incorporate joint limits
            link(1).qlim = [-1.9 0];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-170 170]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;
        
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end