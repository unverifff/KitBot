%% Based on Lab 9 Solutions
% Call in main script with:
% RMRC(robotName, initPose, finalPose, itemName;
%     robotName -> name of Robot
%     initPose -> in this instance Joint Poses (q)
%     finalPose -> in this instance Joint Poses (q)
%     itemName -> to animate item with robot (does not work correctly)
%              -> '0' for no item

function RMRC(robotName, initPose, finalPose, itemName)
    % Parameters
    t = 5;              % Total time
    deltaT = 0.02;       % Control Frequency
    steps = t/deltaT;    % Number of steps for simulation
    % delta = 2*pi/steps;  % Small angle change
    epsilon = 0.1;       % Threshold Value for Manipulability
    W = diag([1 1 1 1 1 1]); % Weighting Matrix for Velocity Vector (Putting more emphasis on Linear Velocities than Angular)
    
    startPose = initPose;
    endPose = finalPose;
    
    % Allocate Array Data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,7);       % Array for joint anglesR, 7DOF
    qdot = zeros(steps,7);          % Array for joint velocities, 7DOF
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps);    % For plotting trajectory error

    % Trajectory, Initial Pose 
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    qMatrix = nan(steps, 7);
    for i=1:steps
        % x(1,i) = (1 - s(i)) * startPose(1) + s(i) * endPose(1); % Points in x
        % x(2,i) = (1 - s(i)) * startPose(2) + s(i) * endPose(2); % Points in y
        % x(3,i) = (1 - s(i)) * startPose(3) + s(i) * endPose(3); % Points in z
        % theta(1,i) = pi;                 % Roll angle 
        % theta(2,i) = 0;                 % Pitch angle
        % theta(3,i) = 0;                 % Yaw angle
        qMatrix(i, :) = (1 - s(i)) * startPose + s(i) * endPose;
    end

    % T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
    % q0 = zeros(1,7);                                                   % Initial guess for joint angles, 7DOF
    % qMatrix(1,:) = robotName.model.ikcon(T,q0);                            % Solve joint angles to achieve first waypoint

    % 1.4) Track the trajectory with RMRC
    % for i = 1:steps-1
    %     % UPDATE: fkine function now returns an SE3 object. To obtain the 
    %     % Transform Matrix, access the variable in the object 'T' with '.T'.
    %     T = robotName.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
    %     deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    %     Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    %     Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    %     Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    %     S = Rdot*Ra';                                                           % Skew symmetric!
    %     linear_velocity = (1/deltaT)*deltaX;
    %     angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    %     deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    %     xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    %     J = robotName.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    %     m(i) = sqrt(det(J*J'));
    %     if m(i) < epsilon  % If manipulability is less than given threshold
    %         lambda = (1 - m(i)/epsilon)*5E-2;
    %     else
    %         lambda = 0;
    %     end
    %     invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
    %     qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
    %     for j = 1:7                                                             % Loop through joints 1 to 7
    %         if qMatrix(i,j) + deltaT*qdot(i,j) < robotName.model.qlim(j,1)          % If next joint angle is lower than joint limit...
    %             qdot(i,j) = 0; % Stop the motor
    %         elseif qMatrix(i,j) + deltaT*qdot(i,j) > robotName.model.qlim(j,2)      % If next joint angle is greater than joint limit ...
    %             qdot(i,j) = 0; % Stop the motor
    %         end
    %     end
    %     qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
    %     positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    %     angleError(:,i) = deltaTheta;                                           % For plotting
    % end

    % Animate Movement
    for k = 1:length(qMatrix)
        robotName.model.animate(qMatrix(k,:));
        % endEffectorLocation = robotName.model.fkine(robotName.model.getpos());

        % if itemName ~= 0
        % % To move Item with End Effector
        %     itemName.robotModel{1}.base = endEffectorLocation.T;
        %     itemName.robotModel{1}.animate(0);
        % else
            % drawnow();
        % end

        drawnow();
    end
end
