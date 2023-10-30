%% TESTING FUNCTIONS & CLASSES
% Majority of Functions to be Put in separate classes.
clf;
clearvars;

% Load In Environment
Environment();
hold on;

% Declare how many pans & dinnerware to add
noDinnerSet = 6;
noPan = 8;

% Get Poses
[platePose, bowlPose, whiskyPose, knifePose, ...
    forkPose, spoonPose] = kitchenPoses.getDiningPose(noDinnerSet);

[plateWashed, bowlWashed, whiskyWashed, knifeWashed, ...
    forkWashed, spoonWashed] = kitchenPoses.getWashedPose(noDinnerSet);

% If want to x times the number of poses:
% Create a Cell -> ___Pose = cell(1:x)
% ___Pose{1} = transl.......
% ___Pose{2} = transl.......
% ___Pose{x} = transl.......
% Do the same for wash___Pose
wokPose{1} = transl(1.15, -1.6, 1.03) * trotx(0) * troty(0) * trotz(0);
saucePose{1} = transl(0.75, -1.65, 0.985) * trotx(0) * troty(0) * trotz(0);
potPose{1} = transl(0.6, -0.4, 0.65) * trotx(0) * troty(0) * trotz(deg2rad(180));
skilletPose{1} = transl(1.75, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
castSaucePose{1} = transl(0.6, -2.205, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
castPanPose{1} = transl(-0.6, -2.205, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);

washWokPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washPotPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSkilletPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washCastSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washCastPanPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));

%Insert dinnerware at Table (Comment out the other if using this)
plate = insertModel('Plate', noDinnerSet, platePose);
bowl = insertModel('Bowl', noDinnerSet, bowlPose);
whisky = insertModel('Whisky', noDinnerSet, whiskyPose);
knife = insertModel('Knife', noDinnerSet, knifePose);
fork = insertModel('Fork', noDinnerSet, forkPose);
spoon = insertModel('Spoon', noDinnerSet, spoonPose);

%Insert dinnerware in Wash Basket (Comment out the other if using this)
% plate = insertModel('Plate', noDinnerSet, plateWashed);
% bowl = insertModel('Bowl', noDinnerSet, bowlWashed);
% whisky = insertModel('Whisky', noDinnerSet, whiskyWashed);
% knife = insertModel('Knife', noDinnerSet, knifeWashed);
% fork = insertModel('Fork', noDinnerSet, forkWashed);
% spoon = insertModel('Spoon', noDinnerSet, spoonWashed);

% Insert pans (Comment out the other if using this)
wok = insertModel('Wok', 1, wokPose);
sauce = insertModel('Saucepan', 1, saucePose);
pot = insertModel('Pot', 1, potPose);
skillet = insertModel('Cast Iron Skillet', 1, skilletPose);
castSauce = insertModel('Cast Iron Saucepan', 1, castSaucePose);
castPan= insertModel('Cast Iron Pan', 1, castPanPose);

% Insert pans in washing machine (Comment out the other if using this)
% wok = insertModel('Wok', 1, washWokPose);
% sauce = insertModel('Saucepan', 1, washSaucePose);
% pot = insertModel('Pot', 1, washPotPose);
% skillet = insertModel('Cast Iron Skillet', 1, washSkilletPose);
% castSauce = insertModel('Cast Iron Saucepan', 1, washCastSaucePose);
% castPan= insertModel('Cast Iron Pan', 1, washCastPanPose);

%% LinearUR3
baseTr = transl([-0.7 2.5 0.63]) * trotz(deg2rad(180)); % Linear UR3 built into Table
ur3 = LinearUR3(baseTr);
q0 = [0, 0, 0, 0, 0, 0, 0];

%% Dining ware Waypoints & Joint Angles
% Joint Angle Guesses for IKCON & Waypoints for items
qJointGuess = cell(1, noDinnerSet);
qJointGuess{1} = [-0.5, 0, deg2rad(50), deg2rad(-20), deg2rad(-20), -pi/2, 0];
qJointGuess{2} = [-1.3, 0, deg2rad(50), deg2rad(-20), deg2rad(-20), -pi/2, 0];
qJointGuess{3} = [0, deg2rad(-25), deg2rad(50), deg2rad(-20), deg2rad(-20), -pi/2, 0];
qJointGuess{4} = [-0.75, deg2rad(165), deg2rad(50), deg2rad(-20), deg2rad(-20), -pi/2, deg2rad(-90)];
qJointGuess{5} = [-1.3, deg2rad(165), deg2rad(50), deg2rad(-20), deg2rad(-20), deg2rad(-90), deg2rad(-90)];
qJointGuess{6} = [0, deg2rad(165), deg2rad(50), deg2rad(-20), deg2rad(-20), -pi/2, deg2rad(-90)];

qWashedGuess = cell(1, noDinnerSet);
for k = 1:numel(qWashedGuess)
    qWashedGuess{k} = [0, pi/2, -deg2rad(45), -deg2rad(5), 0, pi/2, 0];
end

% Waypoints for Which Side of Table
qRight = [-0.7, pi, deg2rad(50), deg2rad(-20), 0, -pi/2, 0];
qLeft = [-0.7, 0, deg2rad(50), deg2rad(-20), 0, -pi/2, 0]; % Need to fix the Joints to match qGuess

% Joint Angles at Dining ware
plateJoint = moveRobot.solveIK(ur3, platePose, qJointGuess);
bowlJoint = moveRobot.solveIK(ur3, bowlPose, qJointGuess);
whiskyJoint = moveRobot.solveIK(ur3, whiskyPose, qJointGuess);
knifeJoint = moveRobot.solveIK(ur3, knifePose, qJointGuess);
forkJoint = moveRobot.solveIK(ur3, forkPose, qJointGuess);
spoonJoint = moveRobot.solveIK(ur3, spoonPose, qJointGuess);

plateWashedJoint = moveRobot.solveIK(ur3, plateWashed, qWashedGuess);
bowlWashedJoint = moveRobot.solveIK(ur3, bowlWashed, qWashedGuess);
whiskyWashedJoint = moveRobot.solveIK(ur3, whiskyWashed, qWashedGuess);
knifeWashedJoint = moveRobot.solveIK(ur3, knifeWashed, qWashedGuess);
forkWashedJoint = moveRobot.solveIK(ur3, forkWashed, qWashedGuess);
spoonWashedJoint = moveRobot.solveIK(ur3, spoonWashed, qWashedGuess);

%% Move Robot
for k = 1:6
    % Loops through each Item to be Moved
    switch k
        case 1
            itemName = knife;
            itemJoint = knifeJoint;
            itemWashed = knifeWashedJoint;

        case 2
            itemName = fork;
            itemJoint = forkJoint;
            itemWashed = forkWashedJoint;

        case 3
            itemName = spoon;
            itemJoint = spoonJoint;
            itemWashed = spoonWashedJoint;

        case 4
            itemName = plate;
            itemJoint = plateJoint;
            itemWashed = plateWashedJoint;

        case 5
            itemName = bowl;
            itemJoint = bowlJoint;
            itemWashed = bowlWashedJoint;

        case 6
            itemName = whisky;
            itemJoint = whiskyJoint;
            itemWashed = whiskyWashedJoint;       
    end


    for i = noDinnerSet:-1:1
        time = 1;

        % Move to just above Wash Basket
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, qWashedGuess{i}, true);

        % Move to Washed Item
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, itemWashed{i}, true);

        % Grab Washed Item (assuming finger grip do something)

        % Move to just above Wash Basket
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, qWashedGuess{i}, true, itemName, i);

        if  (i <= noDinnerSet && i >= 4) %{itemJoint{i}(1, 2) < 0 %}
            % fprintf('i <= 6 && i >= 4 \n');
            % Reorientate Joints for Dining ware on right
            qCurrent = ur3.model.getpos();
            moveRobot.trapTraj(ur3, time, qCurrent, qRight, false, itemName, i);

        elseif (i <= 3 && i >= 1) %{if itemJoint{i}(1, 2) > 0 %} 
            % fprintf('i <= 3 && i >= 1 \n');
            % Reorientate Joints for Dining ware on right
            qCurrent = ur3.model.getpos();
            moveRobot.trapTraj(ur3, time, qCurrent, qLeft, false, itemName, i);
        end

        % Move to Just Above Item
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, qJointGuess{i}, false, itemName, i);

        % Move to Item
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, itemJoint{i}, false, itemName, i);

        % Let go of Item (assuming finger grip do something)

        % Move to Just Above Item
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, qJointGuess{i}, true);

        % Return to Default Pose
        qCurrent = ur3.model.getpos();
        moveRobot.trapTraj(ur3, time, qCurrent, q0, true);
    end
end

% ur3.model.teach(q0);
