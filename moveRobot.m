%% Functions to get 6DOF (with/without linear rail) moving 
% Call in main script with:
%     moveRobot.solveIK(robotName, itemPose, q0)
%         robotName -> name of robot
%         itemPose -> cell of item poses
%         q0 -> inital joint guess
%     moveRobot.trapTraj(robotName, time, initQ, nextQ, itemName, itemIndex)
%         robotName -> name of robot
%         time -> time in seconds for how long movement should take
%         initQ -> starting joint angle pose
%         nextQ -> next joint angle pose
%         itemName -> name of item to move with end effector
%             leave blank if no item is required to be moved
%         itemIndex -> index within the cell of itemName 
%             leave blank if no item is required to be moved

classdef moveRobot
    methods(Static)
        function trapTraj(robotName, time, initQ, nextQ, itemName, itemIndex)
            % Variables
            deltaT = 0.02; % Control Frequency, steps * deltaT = time (s)
            steps = time/deltaT;
            epsilon = 0.1; % Threshold Value for Manipulability
            qMatrix = nan(steps, 7);
            s = lspb(0, 1, steps); % Using Trapezoidal Velocity Profile
        
            for p = 1:steps
                qMatrix(p, :) = (1 - s(p)) * initQ + s(p) * nextQ; % Create Joint Trajectory
                
                % RMRC Stuff 
                J = robotName.model.jacob0(qMatrix(p, :)); % Compute Jacobian at Current Pose
                w = sqrt(det(J * J')); % Measure of Manipulability
        
                % Modify Damping Factor Lambda based on DLS (based off Lab
                % 9 Solutions)
                if w < epsilon
                    lambda = (1 - w/epsilon) * 5E-2;
                else
                    lambda = 0;
                end
        
                if p < steps
                    % Compute Desired End Effector Velocity
                    deltaX = (1-s(p+1))*initQ + s(p+1)*nextQ - qMatrix(p, :); 
                    % Check if robotName is 6DOF or 6DOF on Linear Rail
                    if length(deltaX) > 6 
                        deltaXDot = deltaX(2:7)'/deltaT; % Ignoring Linear Rail, therefore 2:7
                    elseif length(deltaX) == 6
                        deltaXDot = deltaX'/deltaT;
                    else 
                        disp('This only accepts 6DOF Manipulators with/without Linear Rail!!!');
                        break;
                    end
                    
                    pseudoDLS = J' * (J*J' + lambda*eye(6))^-1; % PseudoDLS Formula
                    qDot = pseudoDLS * deltaXDot; % Joint Velocity
        
                    qMatrix(p+1, :) = qMatrix(p, :) + (qDot' * deltaT); % Integrate qDot to get adjusted qMatrix
                end
            end
        
            % Animate Robot & Item 
            for o = 1:length(qMatrix)
                robotName.model.animate(qMatrix(o, :)); % Animate
        
                % Checks if itemName is provided to move with End Effector
                if nargin >= 5 && ~isempty(itemName) && nargin >= 6 && ~isempty(itemIndex) 
                    endEffectorLocation = robotName.model.fkine(robotName.model.getpos());
                    itemName.robotModel{itemIndex}.base = endEffectorLocation.T  * transl(0, 0, 0.135) * trotx(pi/2) * troty(0) * trotz(0);
                    itemName.robotModel{itemIndex}.animate(0);
                end
        
                drawnow();
            end
        end

        %% Solve IK (insertModel.m) Function 
        function desiredJoint = solveIK(robotName, itemPose, q0)
            itemNo = numel(itemPose); % Size of brickPose cell/array
            desiredJoint = cell(1, itemNo);
            for i = 1:itemNo
                % Offset Z position so that the Tips of Gripper makes
                % contact with itemName
                endEffectorPose = itemPose{i} * trotx(deg2rad(180)) * transl(0, 0, -0.135);
                desiredJoint{i} = robotName.model.ikcon(endEffectorPose, q0{i});
            end
        end
    end
end