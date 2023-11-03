%% Functions to get 6DOF (with/without linear rail) moving 
% Call in main script with:
%     moveRobot.solveIK(robotName, itemPose, q0)
%         robotName -> name of robot
%         itemPose -> cell of item poses
%         q0 -> inital joint guess, can be left blank
%     moveRobot.solveIK12(robotName, itemPose, q0)
%         FOR TIM12
%         robotName -> name of robot
%         itemPose -> cell of item poses
%         q0 -> inital joint guess, can be left blank
%     moveRobot.trapTraj(robotName, time, initQ, nextQ, isRMRC itemName, itemIndex)
%         robotName -> name of robot
%         time -> time in seconds for how long movement should take
%         initQ -> starting joint angle pose
%         nextQ -> next joint angle pose
%         isRMRC -> True to turn on RMRC, else put False
%         itemName -> name of item to move with end effector
%             leave blank if no item is required to be moved
%         itemIndex -> index within the cell of itemName 
%             leave blank if no item is required to be moved
%     moveRobot.trapTraj12(robotName, time, initQ, nextQ, isRMRC, gripperR, gripperL, gripperQR, gripperQL, itemName, itemIndex)
%         FOR TM12
%         robotName -> name of robot
%         time -> time in seconds for how long movement should take
%         initQ -> starting joint angle pose
%         nextQ -> next joint angle pose
%         isRMRC -> True to turn on RMRC, else put False
%         gripperR -> Right Finger Model Name
%         gripperL -> Left Finger Model Name
%         gripperQR -> Right Finger Pose
%         gripperQL -> Left Finger Pose
%         itemName -> name of item to move with end effector
%             leave blank if no item is required to be moved
%         itemIndex -> index within the cell of itemName 
%             leave blank if no item is required to be moved
%     moveRobot.RMRC(robotName, time, initQ, nextQ, isRMRC itemName, itemIndex)
%         robotName -> name of robot
%         time -> time in seconds for how long movement should take
%         initPose -> starting cartesian pose
%         nextPose -> next cartesian pose
%         isRMRC -> True to turn on RMRC, else put False
%         itemName -> name of item to move with end effector
%             leave blank if no item is required to be moved
%         itemIndex -> index within the cell of itemName 
%             leave blank if no item is required to be moved


classdef moveRobot
    methods(Static)
        function trapTraj(robotName, time, initQ, nextQ, isRMRC, itemName, itemIndex)
            % Variables
            deltaT = 0.02; % Control Frequency, steps * deltaT = time(s)
            steps = time/deltaT; % Number of Steps for Simulation
            epsilon = 0.1; % Threshold Value for Manipulability
            qMatrix = nan(steps, 7); % Allocating array of joint angles
            s = lspb(0, 1, steps); % Using Trapezoidal Velocity Profile
            W = diag([0.5 0.5 0.5 1 1 1]); % Weight Matrix for Velocity Vector

            for p = 1:steps
                qMatrix(p, :) = (1 - s(p)) * initQ + s(p) * nextQ; % Create Joint Trajectory
            end

            % RMRC Stuff (based off Lab 9 Solutions)
            % if isRMRC the for loop
            if isRMRC == true
                for p = 1:steps 
                    J = robotName.model.jacob0(qMatrix(p, :)); % Compute Jacobian at Current Pose
                    JMod = J;
                    JMod(:, 1) = []; % Removing the first link (linear rail) from being modified
                    % MoM = sqrt(det(J * J')); % Measure of Manipulability
                    MoM = sqrt(det(JMod * JMod')); % Measure of Manipulability
    
                    if MoM < epsilon
                        lambda = (1 - MoM/epsilon) * 5E-2; 
                    else
                        lambda = 0;
                    end
    
                    if p < steps
                        % Compute Desired End Effector Velocity
                        % deltaX = qMatrix(p+1, :) - qMatrix(p, :);
                        T1 = robotName.model.fkine(qMatrix(p,:)).T; % Fkine for Current Transformation
                        T2 = robotName.model.fkine(qMatrix(p+1,:)).T; % Fkine for Next Transformation
                        deltaX = T2(1:3, 4) - T1(1:3, 4); % Get Position Error to Waypoint
                        
                        Rd = T2(1:3,1:3); % Next RPY angles
                        Ra = T1(1:3,1:3); % Current RPY angles                           
                        Rdot = (1/deltaT)*(Rd - Ra); % Calculate Rotation Error Matrix 
                        S = Rdot*Ra'; 
                        linVel = (1/deltaT)*deltaX;
                        angVel = [S(3,2);S(1,3);S(2,1)];
                        xDot = W * [linVel;angVel]; % Calculate End Effector Velocity
    
                        % pseudoDLS = J' * (J*J' + lambda*eye(6))^-1; % PseudoDLS Formula
                        pseudoDLS = JMod' * (JMod*JMod' + lambda*eye(6))^-1; % PseudoDLS Formula
                        % qDot = pseudoDLS * deltaXDot; % Joint Velocity
                        qDot = pseudoDLS * xDot; % Joint Velocity
    
                        % qMatrix(p+1, :) = qMatrix(p, :) + (qDot' * deltaT) % Integrate qDot to get adjusted qMatrix
                        qMatrix(p+1, 2:7) = qMatrix(p, 2:7) + (qDot' * deltaT); % Integrate qDot to get adjusted qMatrix
                    end
                end
            end
        
            % Animate Robot & Item 
            for o = 1:length(qMatrix)
                robotName.model.animate(qMatrix(o, :)); % Animate
        
                % Checks if itemName is provided to move with End Effector
                if nargin >= 6 && ~isempty(itemName) && nargin >= 7 && ~isempty(itemIndex) 
                    endEffectorLocation = robotName.model.fkine(robotName.model.getpos());
                    offset = transl(0, 0, 0.135) * trotx(pi/2) * troty(0) * trotz(0); % Offset for Gripper
                    itemName.robotModel{itemIndex}.base = endEffectorLocation.T  * offset;
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
                % Allow for the option for if no q0 is inputted.
                if nargin >= 3 && ~isempty(q0)
                    desiredJoint{i} = robotName.model.ikcon(endEffectorPose, q0{i});
                else
                    desiredJoint{i} = robotName.model.ikcon(endEffectorPose);
                end
            end
        end

        function trapTraj12(robotName, time, initQ, nextQ, isRMRC, gripperR, gripperL, gripperQR, gripperQL, itemName, itemIndex)
            % Variables
            deltaT = 0.02; % Control Frequency, steps * deltaT = time(s)
            steps = time/deltaT; % Number of Steps for Simulation
            epsilon = 0.1; % Threshold Value for Manipulability
            qMatrix = nan(steps, 7); % Allocating array of joint angles
            s = lspb(0, 1, steps); % Using Trapezoidal Velocity Profile
            W = diag([0.5 0.5 0.5 1 1 1]); % Weight Matrix for Velocity Vector

            for p = 1:steps
                qMatrix(p, :) = (1 - s(p)) * initQ + s(p) * nextQ; % Create Joint Trajectory
            end

            % RMRC Stuff (based off Lab 9 Solutions)
            % if isRMRC the for loop
            if isRMRC == true
                for p = 1:steps 
                    J = robotName.model.jacob0(qMatrix(p, :)); % Compute Jacobian at Current Pose
                    JMod = J;
                    JMod(:, 1) = []; % Removing the first link (linear rail) from being modified
                    % Mom = sqrt(det(J * J')); % Measure of Manipulability
                    MoM = sqrt(det(JMod * JMod')); % Measure of Manipulability
    
                    if MoM < epsilon
                        lambda = (1 - MoM/epsilon) * 5E-2; 
                    else
                        lambda = 0.01;
                    end
    
                    if p < steps
                        % Compute Desired End Effector Velocity
                        % deltaX = qMatrix(p+1, :) - qMatrix(p, :);
                        T1 = robotName.model.fkine(qMatrix(p,:)).T; % Fkine for Current Transformation
                        T2 = robotName.model.fkine(qMatrix(p+1,:)).T; % Fkine for Next Transformation
                        deltaX = T2(1:3, 4) - T1(1:3, 4); % Get Position Error to Waypoint
                        
                        Rd = T2(1:3,1:3); % Next RPY angles
                        Ra = T1(1:3,1:3); % Current RPY angles                           
                        Rdot = (1/deltaT)*(Rd - Ra); % Calculate Rotation Error Matrix 
                        S = Rdot*Ra'; 
                        linVel = (1/deltaT)*deltaX;
                        angVel = [S(3,2);S(1,3);S(2,1)];
                        xDot = W * [linVel;angVel]; % Calculate End Effector Velocity
    
                        % pseudoDLS = J' * (J*J' + lambda*eye(6))^-1; % PseudoDLS Formula
                        pseudoDLS = JMod' * (JMod*JMod' + lambda*eye(6))^-1; % PseudoDLS Formula
                        % qDot = pseudoDLS * deltaXDot; % Joint Velocity
                        qDot = pseudoDLS * xDot; % Joint Velocity
    
                        % qMatrix(p+1, :) = qMatrix(p, :) + (qDot' * deltaT) % Integrate qDot to get adjusted qMatrix
                        qMatrix(p+1, 2:7) = qMatrix(p, 2:7) + (qDot' * deltaT); % Integrate qDot to get adjusted qMatrix
                    end
                end
            end
        
            % Animate Robot & Item 
            for o = 1:length(qMatrix)
                robotName.model.animate(qMatrix(o, :)); % Animate

                EFF = robotName.model.fkine(robotName.model.getpos());
                gripperR.model.base = EFF;
                gripperL.model.base = EFF;
                gripperR.model.animate(gripperR.model.getpos());
                hold on
                gripperL.model.animate(gripperL.model.getpos());

                % Checks if itemName is provided to move with End Effector
                if nargin >= 10 && ~isempty(itemName) && nargin >= 11 && ~isempty(itemIndex) 
                    endEffectorLocation = robotName.model.fkine(robotName.model.getpos());
                    offset = transl(0, 0, 0.15) * trotx(pi/2) * troty(0) * trotz(0); % Offset for Gripper * trotx(pi/2) z -0.135
                    itemName.robotModel{itemIndex}.base = endEffectorLocation.T* offset;
                    itemName.robotModel{itemIndex}.animate(0);
                end
        
                drawnow();
            end
        end

        %% Solve IK (insertModel.m) Function 
        function desiredJoint = solveIK12(robotName, itemPose, q0)
            itemNo = numel(itemPose); % Size of brickPose cell/array
            desiredJoint = cell(1, itemNo);
            for i = 1:itemNo
                % Offset Z position so that the Tips of Gripper makes
                % contact with itemName
                endEffectorPose = itemPose{i} * trotx(deg2rad(180)) * transl(0, 0, -0.15);
                % Allow for the option for if no q0 is inputted.
                if nargin >= 3 && ~isempty(q0)
                    desiredJoint{i} = robotName.model.ikcon(endEffectorPose, q0{i});
                else
                    desiredJoint{i} = robotName.model.ikcon(endEffectorPose);
                end
            end
        end

        function animateClaw(gripperR, gripperL, gripperQR, gripperQL)
            % fprintf('animate');
                stepsG = 50;
                qMatrixR = nan(stepsG, 3);
                qMatrixL = nan(stepsG, 3);
                d = lspb(0, 1, stepsG); % Using Trapezoidal Velocity Profile
                currentGL = gripperL.model.getpos();
                currentGR = gripperR.model.getpos();
                for c = 1:stepsG
                    qMatrixR(c, :) = (1 - d(c)) * currentGR + d(c) * gripperQR; % Create Joint Trajectory
                    qMatrixL(c, :) = (1 - d(c)) * currentGL + d(c) * gripperQL; % Create Joint Trajectory
                    size(qMatrixL)
                    size(qMatrixR)
                end
                for e = 1:stepsG
                    gripperR.model.animate(qMatrixR(e, :));
                    hold on;
                    gripperL.model.animate(qMatrixL(e, :));
                    hold on;
                end
            % fprintf('animate2');
        end

        function RMRC(robotName, time, initPose, nextPose, isRMRC, itemName, itemIndex)
            % Variables
            deltaT = 0.02; % Control Frequency, steps * deltaT = time(s)
            steps = time/deltaT; % Number of Steps for Simulation
            epsilon = 0.1; % Threshold Value for Manipulability
            s = lspb(0, 1, steps); % Using Trapezoidal Velocity Profile
            W = diag([0.5 0.5 0.5 1 1 1]); % Weight Matrix for Velocity Vector
            MoM = zeros(steps,1);             % Array for Measure of Manipulability
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory

            for i = 1:steps
                x(1,i) = (1 - s(i)) * initPose(1) + s(i) * nextPose(1); % Points in x
                x(2,i) = (1 - s(i)) * initPose(2) + s(i) * nextPose(2); % Points in y
                x(3,i) = (1 - s(i)) * initPose(3) + s(i) * nextPose(3); % Points in z
                theta(1,i) = pi;                 % Roll angle 
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
            q0 = zeros(1,7);                                                   % Initial guess for joint angles, 7DOF
            qMatrix(1,:) = robotName.model.ikcon(T,q0);                            % Solve joint angles to achieve first waypoint

            % RMRC Stuff (based off Lab 9 Solutions)
            % if isRMRC the for loop
            if isRMRC == true
                for p = 1:steps 
                    J = robotName.model.jacob0(qMatrix(p, :)); % Compute Jacobian at Current Pose
                    JMod = J;
                    JMod(:, 1) = []; % Removing the first link (linear rail) from being modified
                    % MoM = sqrt(det(J * J')); % Measure of Manipulability
                    MoM(p) = sqrt(det(JMod * JMod')); % Measure of Manipulability
    
                    if MoM(p) < epsilon
                        lambda = (1 - MoM(p)/epsilon) * 5E-2; 
                    else
                        lambda = 0;
                    end
    
                    if p < steps
                        % Compute Desired End Effector Velocity
                        % deltaX = qMatrix(p+1, :) - qMatrix(p, :);
                        T1 = robotName.model.fkine(qMatrix(p,:)).T; % Fkine for Current Transformation
                        T2 = x(:,p+1); % Fkine for Next Transformation
                        deltaX = T2(1:3, 4) - T1(1:3, 4); % Get Position Error to Waypoint
                        
                        Rd = rpy2r(theta(1,p+1),theta(2,p+1),theta(3,p+1)); % Next RPY angles
                        Ra = T1(1:3,1:3); % Current RPY angles                           
                        Rdot = (1/deltaT)*(Rd - Ra); % Calculate Rotation Error Matrix 
                        S = Rdot*Ra'; 
                        linVel = (1/deltaT)*deltaX;
                        angVel = [S(3,2);S(1,3);S(2,1)];
                        xDot = W * [linVel;angVel]; % Calculate End Effector Velocity
    
                        % pseudoDLS = J' * (J*J' + lambda*eye(6))^-1; % PseudoDLS Formula
                        pseudoDLS = JMod' * (JMod*JMod' + lambda*eye(6))^-1; % PseudoDLS Formula
                        % qDot = pseudoDLS * deltaXDot; % Joint Velocity
                        qDot = pseudoDLS * xDot; % Joint Velocity
    
                        % qMatrix(p+1, :) = qMatrix(p, :) + (qDot' * deltaT) % Integrate qDot to get adjusted qMatrix
                        qMatrix(p+1, 2:7) = qMatrix(p, 2:7) + (qDot' * deltaT); % Integrate qDot to get adjusted qMatrix
                    end
                end
            end

            % Animate Robot & Item 
            for o = 1:length(qMatrix)
                robotName.model.animate(qMatrix(o, :)); % Animate
        
                % Checks if itemName is provided to move with End Effector
                if nargin >= 6 && ~isempty(itemName) && nargin >= 7 && ~isempty(itemIndex) 
                    endEffectorLocation = robotName.model.fkine(robotName.model.getpos());
                    offset = transl(0, 0, 0.135) * trotx(pi/2) * troty(0) * trotz(0); % Offset for Gripper
                    itemName.robotModel{itemIndex}.base = endEffectorLocation.T  * offset;
                    itemName.robotModel{itemIndex}.animate(0);
                end
        
                drawnow();
            end
        end
    end
end