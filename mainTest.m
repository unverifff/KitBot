%% TESTING FUNCTIONS & CLASSES
clf;
clearvars;

% Load In Environment
Environment();
hold on;

% Declare how many pans & dinnerware to add
noDinnerSet = 6;
noPan = 6;

% Get Poses
[platePose, bowlPose, whiskyPose, knifePose, ...
    forkPose, spoonPose] = kitchenPoses.getDiningPose(noDinnerSet);

[plateWashed, bowlWashed, whiskyWashed, knifeWashed, ...
    forkWashed, spoonWashed] = kitchenPoses.getWashedPose(noDinnerSet);

panPose = kitchenPoses.getWashedPanPose(noPan);

% Insert dinnerware at Table
insertModel('Plate', noDinnerSet, platePose);
insertModel('Bowl', noDinnerSet, bowlPose);
insertModel('Whisky', noDinnerSet, whiskyPose);
insertModel('Knife', noDinnerSet, knifePose);
insertModel('Fork', noDinnerSet, forkPose);
insertModel('Spoon', noDinnerSet, spoonPose);

% Insert dinnerware in Plastic Container
% insertModel('Plate', noDinnerSet, plateWashed);
% insertModel('Bowl', noDinnerSet, bowlWashed);
% insertModel('Whisky', noDinnerSet, whiskyWashed);
% insertModel('Knife', noDinnerSet, knifeWashed);
% insertModel('Fork', noDinnerSet, forkWashed);
% insertModel('Spoon', noDinnerSet, spoonWashed);

% Insert pans in Dishwashing Machine
for i = 1:noPan
    switch i
        case 1
            name = 'Wok';

        case 2
            name = 'Saucepan';

        case 3
            name = 'Pot';

        case 4
            name = 'Cast Iron Skillet';

        case 5
            name = 'Cast Iron Saucepan';

        case 6
            name = 'Cast Iron Pan';
    end

    insertModelAtPose(name, i, panPose);
end

