%% Load in Environment
% Majority of Functions to be Put in separate classes.
clf;
clearvars;

% Load In Environment
Environment();
hold on;

% Declare how many pans & dinnerware to add
noDinnerSet = 6;
noPan = 8;

%% Get Poses
% Dining Table
[platePose, bowlPose, whiskyPose, knifePose, ...
    forkPose, spoonPose] = kitchenPoses.getDiningPose(noDinnerSet);

[plateWashed, bowlWashed, whiskyWashed, knifeWashed, ...
    forkWashed, spoonWashed] = kitchenPoses.getWashedPose(noDinnerSet);

% Pots & Pans
castPanPose = cell(1:2);
castSaucePose = cell(1:2);
saucePose = cell(1:2);
castPanPose{1} = transl(-0.75, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
castSaucePose{1} = transl(-0.5, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
saucePose{1} = transl(-0.25, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
saucePose{2} = transl(0.25, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
castSaucePose{2} = transl(0.5, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
castPanPose{2} = transl(0.75, -2.175, 1.3) * trotx(deg2rad(90)) * troty(pi) * trotz(0);
skilletPose{1} = transl(0.75, -1.6, 0.93) * trotx(0) * troty(0) * trotz(0);
wokPose{1} = transl(1.15, -1.6, 1.03) * trotx(0) * troty(0) * trotz(0);

washCastPanPose = cell(1:2);
washCastSaucePose = cell(1:2);
washSaucePose = cell(1:2);
washCastSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washCastPanPose{1} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washCastSaucePose{2} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washCastPanPose{2} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washSaucePose{2} = transl(-1.6, -1.25, 0.7) * trotz(deg2rad(-90)) * trotx(pi/2) * troty(pi);
washWokPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSkilletPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));

%% Load Models
plate = insertModel('Plate', noDinnerSet, plateWashed);
bowl = insertModel('Bowl', noDinnerSet, bowlWashed);
whisky = insertModel('Whisky', noDinnerSet, whiskyWashed);
knife = insertModel('Knife', noDinnerSet, knifeWashed);
fork = insertModel('Fork', noDinnerSet, forkWashed);
spoon = insertModel('Spoon', noDinnerSet, spoonWashed);

castPan= insertModel('Cast Iron Pan', 2, washCastPanPose);
castSauce = insertModel('Cast Iron Saucepan', 2, washCastSaucePose);
sauce = insertModel('Saucepan', 2, washSaucePose);
skillet = insertModel('Cast Iron Skillet', 1, washSkilletPose);
wok = insertModel('Wok', 1, washWokPose);

%% Load TM12, UR3 & Gripper
baseTr = transl([-0.7 2.5 0.63]) * trotz(deg2rad(180)); % Linear UR3 built into Table
ur3 = LinearUR3(baseTr);

ceilingTr = transl([-0.7 -1.2 2.1]) * troty(deg2rad(180)); % Linear tm12 built into Table
tm12 = LinearTM12(ceilingTr);

q0 = [0, 0, 0, 0, 0, 0, 0];

% Gripper
hold on
gripperR = GripperRight(tm12.model.fkine(tm12.model.getpos()).T);
gripperL = GripperLeft(tm12.model.fkine(tm12.model.getpos()).T);

% Gripper close joints
qGripper = [0,0,0] ;
gRight = [0,deg2rad(60),0];
gLeft = [0,deg2rad(-60),0];

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

%% Pots & Pans Waypoints & Joint Angles
% Joint Angle Guesses for IKCON & Waypoints for items
jJointGuess = cell(1, noPan);
jJointGuess{1} = [0, -deg2rad(160), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{2} = [0, -deg2rad(180), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{3} = [0, -deg2rad(200), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{4} = [-0.95, -deg2rad(160), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{5} = [-0.95, -deg2rad(180), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{6} = [-0.95, -deg2rad(200), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{7} = [-0.95, -deg2rad(200), 0, deg2rad(-60), 0, pi/2, 0];
jJointGuess{8} = [-1.4, -deg2rad(200), deg2rad(20), deg2rad(-70), 0, pi/2, 0];


jWashedGuess = cell(1, noPan);
for k = 1:numel(jWashedGuess)
    jWashedGuess{k} = [0, deg2rad(80), deg2rad(3), deg2rad(50), -deg2rad(70), deg2rad(-80), 0];
end

% Joint Angles at Pots & Pans
castSauceJointW = moveRobot.solveIK12(tm12, washCastSaucePose,  jJointGuess);
castPanJointW = moveRobot.solveIK12(tm12, washCastPanPose,  jJointGuess);
sauceJointW = moveRobot.solveIK12(tm12, washSaucePose,  jJointGuess);
wokJointW = moveRobot.solveIK12(tm12, washWokPose,  jJointGuess);
skilletJointW = moveRobot.solveIK12(tm12, washSkilletPose,  jJointGuess);

castSauceJoint = moveRobot.solveIK12(tm12, castSaucePose,  jJointGuess);
castPanJoint = moveRobot.solveIK12(tm12, castPanPose,  jJointGuess);
sauceJoint = moveRobot.solveIK12(tm12, saucePose,  jJointGuess);
wokJoint = moveRobot.solveIK12(tm12, wokPose,  jJointGuess);
skilletJoint = moveRobot.solveIK12(tm12, skilletPose,  jJointGuess);

%% Execute Motion
for k = 1:11
    % Loops through each Item to be Moved
    switch k
        case 1
            itemName = knife;
            itemJoint = knifeJoint;
            itemWashed = knifeWashedJoint;

        case 2
            itemName = castPan;
            itemJoint = castPanJoint;
            itemWashed = castPanJointW;
            % itemIndex = 1;

        case 3
            itemName = fork;
            itemJoint = forkJoint;
            itemWashed = forkWashedJoint;

        case 4
            itemName = castSauce;
            itemJoint = castSauceJoint;
            itemWashed = castSauceJointW;
            % itemIndex = 1;

        case 5
            itemName = spoon;
            itemJoint = spoonJoint;
            itemWashed = spoonWashedJoint;

        case 6
            itemName = sauce;
            itemJoint = sauceJoint;
            itemWashed = sauceJointW;

        case 7
            itemName = plate;
            itemJoint = plateJoint;
            itemWashed = plateWashedJoint;

        case 8
            itemName = skillet;
            itemJoint = skilletJoint;
            itemWashed = skilletJointW;

        case 9
            itemName = bowl;
            itemJoint = bowlJoint;
            itemWashed = bowlWashedJoint;

        case 10
            itemName = wok;
            itemJoint = wokJoint;
            itemWashed = wokJointW;

        case 11
            itemName = whisky;
            itemJoint = whiskyJoint;
            itemWashed = whiskyWashedJoint;       
    end

    if ismember(k, [1, 3, 5, 7, 9, 11])
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
    else
        for b = 1:numel(itemJoint)
            time = 1;
    
            % Move to just above Wash Basket
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, jWashedGuess{b}, false, gripperR, gripperL, qGripper,qGripper);
    
            % Move to Washed Item
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, itemWashed{b}, false, gripperR, gripperL,qGripper,qGripper);
    
            % Grab Washed Item (assuming finger grip do something)
            moveRobot.animateClaw(gripperR, gripperL,gRight, gLeft);
    
            % Move to just above Wash Basket
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, jWashedGuess{b}, false, gripperR, gripperL,gRight, gLeft, itemName, b);
    
            % Move to Just Above Item
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, jJointGuess{b}, false, gripperR, gripperL,gRight, gLeft, itemName, b);
    
            % Move to Item
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, itemJoint{b}, false, gripperR, gripperL, gRight, gLeft,itemName, b);
    
            % Let go of Item (assuming finger grip do something)
            moveRobot.animateClaw(gripperR, gripperL,qGripper, qGripper);
    
            % Move to Just Above Item
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, jJointGuess{b}, false, gripperR, gripperL, qGripper,qGripper);
    
            % Return to Default Pose
            qCurrent = tm12.model.getpos();
            moveRobot.trapTraj12(tm12, time, qCurrent, q0, false, gripperR, gripperL, qGripper, qGripper);
        end
    end
end



