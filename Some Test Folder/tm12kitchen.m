clf
clc
clearvars

% Load In Environment
Environment();
hold on;

% Declare how many pans & dinnerware to add
noDinnerSet = 6;
noPan = 8;

% Get Poses
panPose = kitchenPoses.getPanPose(noPan);
panWashed = kitchenPoses.getWashedPanPose(noPan);

% Create a Cell -> ___Pose = cell(1:x)
% ___Pose{1} = transl.......
% ___Pose{2} = transl.......
% ___Pose{x} = transl.......

% Do the same for wash___Pose
castPanPose = cell(1:2);
castSaucePose = cell(1:2);
saucePose = cell(1:2);

castPanPose{1} = transl(-0.75, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
castSaucePose{1} = transl(-0.5, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
saucePose{1} = transl(-0.25, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);

saucePose{2} = transl(0.25, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
castSaucePose{2} = transl(0.5, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);
castPanPose{2} = transl(0.75, -2.23, 1.3) * trotx(deg2rad(90)) * troty(0) * trotz(0);

skilletPose{1} = transl(0.75, -1.6, 0.93) * trotx(0) * troty(0) * trotz(0);
wokPose{1} = transl(1.15, -1.6, 1.03) * trotx(0) * troty(0) * trotz(0);

washCastPanPose = cell(1:2);
washCastSaucePose = cell(1:2);
washSaucePose = cell(1:2);

washCastSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washCastPanPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSaucePose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));

washCastSaucePose{2} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washCastPanPose{2} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSaucePose{2} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));

washWokPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));
washSkilletPose{1} = transl(-1.6, -1.25, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(-90));

% % Insert pans (Comment out the other if using this)
% 
% castPan= insertModel('Cast Iron Pan', 2, castPanPose);
% castSauce = insertModel('Cast Iron Saucepan', 2, castSaucePose);
% sauce = insertModel('Saucepan', 2, saucePose);
% 
% skillet = insertModel('Cast Iron Skillet', 1, skilletPose);
% wok = insertModel('Wok', 1, wokPose);

% Insert pans in washing machine (Comment out the other if using this)
castPan= insertModel('Cast Iron Pan', 2, washCastPanPose);
castSauce = insertModel('Cast Iron Saucepan', 2, washCastSaucePose);
sauce = insertModel('Saucepan', 2, washSaucePose);

skillet = insertModel('Cast Iron Skillet', 1, washSkilletPose);
wok = insertModel('Wok', 1, washWokPose);

%% LinearTM12
ceilingTr = transl([-0.6 -1.2 2.2]) * troty(deg2rad(180)); % Linear tm12 built into Table
tm12 = LinearTM12(ceilingTr);
q0 = [0, 0, 0, 0, 0, 0, 0];

%% Gripper
hold on
gripperR = GripperRight(tm12.model.fkine(tm12.model.getpos()).T);
gripperL = GripperLeft(tm12.model.fkine(tm12.model.getpos()).T);

%% Gripper close joints
qGripper = [0,0,0] ;
gRight = [0,deg2rad(60),0];
gLeft = [0,deg2rad(-60),0];

% tm12.model.teach(q0)

%% Kitchen Pans Waypoints & Joint Angles
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

%% Joint Angles at Kitchen Pans



panWashedJoint = moveRobot.solveIK(tm12, panWashed, jWashedGuess);
panJoint = moveRobot.solveIK(tm12, panPose,  jJointGuess);


% Move Robot
for k = 1:8
    % Loops through each Item to be Moved
    switch k
        case 1
            itemName = castPan;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 1;

        case 2
            itemName = castSauce;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 1;

        case 3
            itemName = sauce;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 1;

        case 4
            itemName = sauce;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 2;

        case 5
            itemName = castSauce;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 2;

        case 6
            itemName = castPan;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 2;

        case 7
            itemName = skillet;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 1;

        case 8
            itemName = wok;
            itemJoint = panJoint;
            itemWashed = panWashedJoint;
            itemIndex = 1;
    end

    time = 1;

    % Move to just above Wash Basket
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, jWashedGuess{k}, true, gripperR, gripperL, qGripper,qGripper);

    % Move to Washed Item
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, itemWashed{k}, true, gripperR, gripperL,qGripper,qGripper);

    % Grab Washed Item (assuming finger grip do something)

    % Move to just above Wash Basket
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, jWashedGuess{k}, true, gripperR, gripperL,gRight,gLeft, itemName, itemIndex);

    % if  (k <= noPan && k >= 4) %{itemJoint{i}(1, 2) < 0 %}
    %     % fprintf('i <= 6 && i >= 4 \n');
    %     % Reorientate Joints for Dining ware on right
    %     qCurrent = tm12.model.getpos();
    %     moveRobot.trapTraj(tm12, time, qCurrent, qRight, false, itemName, k);
    % 
    % elseif (k <= 3 && k >= 1) %{if itemJoint{i}(1, 2) > 0 %} 
    %     % fprintf('i <= 3 && i >= 1 \n');
    %     % Reorientate Joints for Dining ware on right
    %     qCurrent = tm12.model.getpos();
    %     moveRobot.trapTraj(tm12, time, qCurrent, qLeft, false, itemName, k);
    % end

    % Move to Just Above Item
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, jJointGuess{k}, false, gripperR, gripperL,gRight,gLeft, itemName, itemIndex);

    % Move to Item
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, itemJoint{k}, false, gripperR, gripperL, gRight,gLeft,itemName, itemIndex);

    % Let go of Item (assuming finger grip do something)

    % Move to Just Above Item
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, jJointGuess{k}, true, gripperR, gripperL, qGripper,qGripper);

    % Return to Default Pose
    qCurrent = tm12.model.getpos();
    moveRobot.trapTraj(tm12, time, qCurrent, q0, true, gripperR, gripperL, qGripper,qGripper);

end