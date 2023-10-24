%% TESTING FUNCTIONS & CLASSES
clf;
clearvars;

% Load In Environment
% Environment();
% hold on;

% Declare how many pans & dinnerware to add
noDinnerSet = 6;
noPan = 8;

% Get Poses
[platePose, bowlPose, whiskyPose, knifePose, ...
    forkPose, spoonPose] = kitchenPoses.getDiningPose(noDinnerSet);

[plateWashed, bowlWashed, whiskyWashed, knifeWashed, ...
    forkWashed, spoonWashed] = kitchenPoses.getWashedPose(noDinnerSet);

% panPose = kitchenPoses.getPanPose(noPan);
% panWashedPose = kitchenPoses.getWashedPanPose(noPan);

%Insert dinnerware at Table
plate = insertModel('Plate', noDinnerSet, platePose);
bowl = insertModel('Bowl', noDinnerSet, bowlPose);
whisky = insertModel('Whisky', noDinnerSet, whiskyPose);
knife = insertModel('Knife', noDinnerSet, knifePose);
fork = insertModel('Fork', noDinnerSet, forkPose);
spoon = insertModel('Spoon', noDinnerSet, spoonPose);

%Insert dinnerware in Wash Basket
% plate = insertModel('Plate', noDinnerSet, plateWashed);
% bowl = insertModel('Bowl', noDinnerSet, bowlWashed);
% whisky = insertModel('Whisky', noDinnerSet, whiskyWashed);
% knife = insertModel('Knife', noDinnerSet, knifeWashed);
% fork = insertModel('Fork', noDinnerSet, forkWashed);
% spoon = insertModel('Spoon', noDinnerSet, spoonWashed);

% Insert pans in Dishwashing Machine
% for i = 1:noPan
%     switch i
%         case 1
%             name = 'Wok';
%         case 2
%             name = 'Saucepan';
% 
%         case 3
%             name = 'Pot';
% 
%         case 4
%             name = 'Cast Iron Saucepan';
% 
%         case 5
%             name = 'Cast Iron Skillet';            
% 
%         case 6
%             name = 'Cast Iron Pan';
% 
%         case 7
%             name = 'Pot';
% 
%         case 8
%             name = 'Saucepan';
%     end
% 
%     insertModelAtPose(name, i, panPose);
%     % insertModelAtPose(name, i, panWashedPose);
% end

%% LinearUR3
baseTr = transl([-0.65 2.5 0.63]) * trotz(deg2rad(180)); % Linear UR3 built into Table
ur3 = LinearUR3(baseTr);
q0 = [0, 0, 0, 0, 0, 0, 0];

%% Dining ware Waypoints & Joint Angles
% Joint Angle Guesses for IKCON & Waypoints for items
qJointGuess = cell(1, noDinnerSet);
qJointGuess{1} = [-0.5, 0, pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];
qJointGuess{2} = [-1.3, 0, pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];
qJointGuess{3} = [0, deg2rad(-25), pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];
qJointGuess{4} = [-0.75, 0, pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];
qJointGuess{5} = [-1.3, deg2rad(-195), pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];
qJointGuess{6} = [0, 0, pi/4, deg2rad(65), deg2rad(-20), -pi/2, 0];

qWashedGuess = cell(1, noDinnerSet);
for k = 1:numel(qWashedGuess)
    qWashedGuess{k} = [0, pi/2, -pi/4, -pi/4, 0, pi/2, 0];
end

% Waypoints for Which Side of Table
qRight = [0, 0, -pi/4, -pi/4, 0, pi/2, 0];
qLeft = [0, pi, -pi/4, -pi/4, 0, pi/2, 0];

% Joint Angles at Dining ware
plateJoint = solveIK(ur3, platePose, qJointGuess);
bowlJoint = solveIK(ur3, bowlPose, qJointGuess);
whiskyJoint = solveIK(ur3, whiskyPose, qJointGuess);
knifeJoint = solveIK(ur3, knifePose, qJointGuess);
forkJoint = solveIK(ur3, forkPose, qJointGuess);
spoonJoint = solveIK(ur3, spoonPose, qJointGuess);

plateWashedJoint = solveIK(ur3, plateWashed, qWashedGuess);
bowlWashedJoint = solveIK(ur3, bowlWashed, qWashedGuess);
whiskyWashedJoint = solveIK(ur3, whiskyWashed, qWashedGuess);
knifeWashedJoint = solveIK(ur3, knifeWashed, qWashedGuess);
forkWashedJoint = solveIK(ur3, forkWashed, qWashedGuess);
spoonWashedJoint = solveIK(ur3, spoonWashed, qWashedGuess);

%% Move Robot
for k = 1:6
    % Loops through each Item to be Moved
    switch k
        case 1
            itemName = 'plate';
            itemJoint = plateJoint;
            itemWashed = plateWashedJoint;

        case 2
            itemName = 'bowl';
            itemJoint = bowlJoint;
            itemWashed = bowlWashedJoint;

        case 3
            itemName = 'whisky';
            itemJoint = whiskyJoint;
            itemWashed = whiskyWashedJoint;

        case 4
            itemName = 'knife';
            itemJoint = knifeJoint;
            itemWashed = knifeWashedJoint;

        case 5
            itemName = 'fork';
            itemJoint = forkJoint;
            itemWashed = forkWashedJoint;

        case 6
            itemName = 'spoon';
            itemJoint = spoonJoint;
            itemWashed = spoonWashedJoint;
    end

    
    for i = noDinnerSet:-1:1
        steps = 20;

        % Move to just above Wash Basket
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, qWashedGuess{i}, 0);
        trapTraj(ur3, steps, qCurrent, qWashedGuess{i});

        % Move to Washed Item
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, itemWashed{i}, 0);
        trapTraj(ur3, steps, qCurrent, itemWashed{i});
    
        % Grab Washed Item

        % Move to just above Wash Basket
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, qWashedGuess{i}, 0);
        trapTraj(ur3, steps, qCurrent, qWashedGuess{i});
    
        if itemJoint{i}(1, 2) < 0
            % Reorientate Joints for Dining ware on right
            qCurrent = ur3.model.getpos();
            % RMRC(ur3, qCurrent, qRight, 0);
            trapTraj(ur3, steps, qCurrent, qRight);
        elseif itemJoint{i}(1, 2) > 0    
            % Reorientate Joints for Dining ware on right
            qCurrent = ur3.model.getpos();
            % RMRC(ur3, qCurrent, qLeft, 0);
            trapTraj(ur3, steps, qCurrent, qLeft);
        end

        % Move to Just Above Item
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, qJointGuess{i}, 0);
        trapTraj(ur3, steps, qCurrent, qJointGuess{i});
    
        % Move to Item
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, itemJoint{i}, 0);
        trapTraj(ur3, steps, qCurrent, itemJoint{i});
    
        % Let go of Item

        % Move to Just Above Item
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, qJointGuess{i}, 0);
        trapTraj(ur3, steps, qCurrent, qJointGuess{i});

        % Return to Default Pose
        qCurrent = ur3.model.getpos();
        % RMRC(ur3, qCurrent, q0, 0);
        trapTraj(ur3, steps, qCurrent, q0);

    end
end

% ur3.model.teach(q0);

%% Move to Basket Waypoint Function
% function baskWaypoint(robotName, steps)
%     % Waypoint Joint States
%     qBaskWaypoint1 = [-1.4000, 0, 0, 0, 0, 0, 0];
%     qBaskWaypoint2 = [-1.4000, -1.5708, 0, -1.5708, 0, 1.5708, 0];
% 
%     % Waypoint 1
%     qCurrent = robotName.model.getpos();
%     trapTraj(robotName, steps, qCurrent, qBaskWaypoint1);
% 
%     % Waypoint 2 
%     qCurrent = robotName.model.getpos();
%     trapTraj(robotName, steps, qCurrent, qBaskWaypoint2);
% end

%% Solve IK (insertModel.m) Function 
function desiredJoint = solveIK(robotName, itemPose, q0)
    itemNo = numel(itemPose); % Size of brickPose cell/array
    desiredJoint = cell(1, itemNo);
    for i = 1:itemNo
        % Offset Z position so that the Tips of Gripper makes contact with
        % brickPose rather than UR3 wrist.
        % endEffectorPose = itemPose{i} * transl(0, 0, -0.15);
        endEffectorPose = itemPose{i} * trotx(deg2rad(180)) * transl(0, 0, 0);
        desiredJoint{i} = robotName.model.ikcon(endEffectorPose, q0{i});
    end
end

%% Trapezoidal Trajectory (Joint Angles) Generator Function
function trapTraj(robotName, steps, initQ, nextQ)
    s = lspb(0, 1, steps);
    qMatrix = nan(steps, 7);
    for p = 1:steps
        qMatrix(p, :) = (1 - s(p)) * initQ + s(p) * nextQ;
    end
    for n = 1:length(qMatrix)
        robotName.model.animate(qMatrix(n, :));
        drawnow();
        pause(0.025);
    end
end

%% Trapezoidal Trajectory (Pose) Generator Function
% Imported from Lab 9 solutions (idk if it works)
function trapTrajT(robotName, steps, initTr, nextTr, rx, ry, rz)
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    
    s = lspb(0, 1, steps);
    for p = 1:steps
        x(1,p) = (1 - s(p)) * initTr(1) + s(p) * nextTr(1); % Points in x
        x(2,p) = (1 - s(p)) * initTr(2) + s(p) * nextTr(2); % Points in y
        x(3,p) = (1 - s(p)) * initTr(3) + s(p) * nextTr(p); % Points in z
        theta(1,p) = rx;                 % Roll angle 
        theta(2,p) = ry;                 % Pitch angle
        theta(3,p) = rz;                 % Yaw angle
    end

    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];
    qMatrix(1,:) = robotName.model.ikcon(T);

    for n = 1:length(qMatrix)
        robotName.model.animate(qMatrix(n, :));
        drawnow();
        pause(0.025);
    end
end
