%% Loads in Poses of dining Utensils (chinaware & cutlery).
% Call in main script with:
%     [name1, name2, name3, name4, name5, name6] =
%         kitchenPoses.getDiningPose(numberOfPose);
%     [name1, name2, name3, name4, name5, name6] =
%         kitchenPoses.getWashedPose(numberOfPose);
%     name = kitchenPoses.getPanPose(numberOfPose);
%     name = kitchenPoses.getWashedPanPose(numberOfPose);
% To be used in main script in tandum with insertModel.m
% Ensure that numberOfPose <= 8, else modify below code to accomodate

classdef kitchenPoses 
    methods(Static)
        function [platePose, bowlPose, whiskyPose, knifePose, forkPose, spoonPose] = getDiningPose(numberOfPose)
            % Initiate Pose Cells
            platePose = cell(1, numberOfPose);
            bowlPose = cell(1, numberOfPose);
            whiskyPose = cell(1, numberOfPose);
            knifePose = cell(1, numberOfPose);
            forkPose = cell(1, numberOfPose);
            spoonPose = cell(1, numberOfPose);
        
            % Add More Cases if numberOfPose > 8
            for i = 1:numberOfPose
                switch i
                    case 1
                        x = 0;
                        y = 2.1;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 2
                        x = 0.75;
                        y = 2.1;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 3
                        x = -0.75;
                        y = 2.1;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 4
                        x = 0;
                        y = 2.9;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(90);
        
                    case 5
                        x = 0.75;
                        y = 2.9;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(90);
        
                    case 6
                        x = -0.75;
                        y = 2.9;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(90);
        
                    case 7
                        x = 1.1;
                        y = 2.5;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 8
                        x = -1.1;
                        y = 2.5;
                        z = 0.67;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(180);
                end
        
                platePose{i} = transl(x, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                bowlPose{i} = transl(x, y, z + 0.01) * trotx(rx) * troty(ry) * trotz(rz);
        
                if (i >= 4 && i <= 6)
                    whiskyPose{i} = transl(x - 0.175, y - 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    knifePose{i} = transl(x - 0.175, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                    forkPose{i} = transl(x + 0.175, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                    spoonPose{i} = transl(x - 0.225, y, z) * trotx(rx) * troty(ry) * trotz(rz);
        
                elseif (i == 7)
                    whiskyPose{i} = transl(x - 0.175, y + 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    knifePose{i} = transl(x, y + 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    forkPose{i} = transl(x, y - 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    spoonPose{i} = transl(x, y + 0.225, z) * trotx(rx) * troty(ry) * trotz(rz);
        
                elseif (i == 8)
                    whiskyPose{i} = transl(x + 0.175, y - 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    knifePose{i} = transl(x, y - 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    forkPose{i} = transl(x, y + 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    spoonPose{i} = transl(x, y - 0.225, z) * trotx(rx) * troty(ry) * trotz(rz);       
        
                else
                    whiskyPose{i} = transl(x + 0.175, y + 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                    knifePose{i} = transl(x + 0.175, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                    forkPose{i} = transl(x - 0.175, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                    spoonPose{i} = transl(x + 0.225, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                end
            end
        end

        function [plateWashedPose, bowlWashedPose, whiskyWashedPose, knifeWashedPose, forkWashedPose, spoonWashedPose] = getWashedPose(numberOfPose)
            % Initiate Pose Cells
            plateWashedPose = cell(1, numberOfPose);
            bowlWashedPose = cell(1, numberOfPose);
            whiskyWashedPose = cell(1, numberOfPose);
            knifeWashedPose = cell(1, numberOfPose);
            forkWashedPose = cell(1, numberOfPose);
            spoonWashedPose = cell(1, numberOfPose);

            % Add More Cases if numberOfPose > 8
            for i = 1:numberOfPose
                switch i
                    case 1
                        x = -1.15;
                        y = 2.42;
                        z = 0.68;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 2
                        x = -1.15;
                        y = 2.42;
                        z = 0.685;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 3
                        x = -1.15;
                        y = 2.42;
                        z = 0.69;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 4
                        x = -1.15;
                        y = 2.42;
                        z = 0.695;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 5
                        x = -1.15;
                        y = 2.42;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 6
                        x = -1.15;
                        y = 2.42;
                        z = 0.71;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 7
                        x = -1.15;
                        y = 2.42;
                        z = 0.715;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 8
                        x = -1.15;
                        y = 2.42;
                        z = 0.72;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
                end

                plateWashedPose{i} = transl(x, y, z) * trotx(rx) * troty(ry) * trotz(rz);
                bowlWashedPose{i} = transl(x - 0.05, y + 0.21, z) * trotx(rx) * troty(ry) * trotz(rz);
                whiskyWashedPose{i} = transl(x + 0.065, y + 0.175, z) * trotx(rx) * troty(ry) * trotz(rz);
                knifeWashedPose{i} = transl(x + 0.12, y + 0.165, z) * trotx(rx) * troty(ry) * trotz(rz - pi/2);
                forkWashedPose{i} = transl(x - 0.12, y + 0.165, z) * trotx(rx) * troty(ry) * trotz(rz - pi/2);
                spoonWashedPose{i} = transl(x + 0.12, y + 0.165, z) * trotx(rx) * troty(ry) * trotz(rz - pi/2);
            end
        end

        function panPose = getPanPose(numberOfPose)
            % Initiate Pose Cells
            panPose = cell(1, numberOfPose);
            
            % Add More Cases if numberOfPose > 8
            for i = 1:numberOfPose
                switch i
                    case 1
                        x = 1.15;
                        y = -1.6;
                        z = 1.03;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 2
                        x = 0.75;
                        y = -1.65;
                        z = 0.985;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 3
                        x = 0.6;
                        y = -0.4;
                        z = 0.65;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(180);
        
                    case 4
                        x = 1.75;
                        y = -2.23;
                        z = 1.3;
                        rx = deg2rad(90);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 5
                        x = 0.6;
                        y = -2.205;
                        z = 1.3;
                        rx = deg2rad(90);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 6
                        x = -0.6;
                        y = -2.205;
                        z = 1.3;
                        rx = deg2rad(90);
                        ry = deg2rad(0);
                        rz = deg2rad(0);
        
                    case 7
                        x = 0.25;
                        y = -0.35;
                        z = 1;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(90);
        
                    case 8
                        x = -1.6;
                        y = -1.25;
                        z = 1.2;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
                end

                panPose{i} = transl(x, y, z) * trotx(rx) * troty(ry) * trotz(rz);
            end
        end

        function washedPanPose = getWashedPanPose(numberOfPose)
            % Initiate Pose Cells
            washedPanPose = cell(1, numberOfPose);
            
            % Add More Cases if numberOfPose > 8
            for i = 1:numberOfPose
                switch i
                    case 1
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 2
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 3
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 4
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 5
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 6
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 7
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
        
                    case 8
                        x = -1.6;
                        y = -1.25;
                        z = 0.7;
                        rx = deg2rad(0);
                        ry = deg2rad(0);
                        rz = deg2rad(-90);
                end

                washedPanPose{i} = transl(x, y, z) * trotx(rx) * troty(ry) * trotz(rz);
            end
        end
    end
end