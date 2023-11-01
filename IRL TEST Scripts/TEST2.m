% clf  
% camlight
%             axis on
%             grid on
%             view(135, 30) % Rotate plane (z direction) by 135deg & look down at 30deg
%             axis([-4 4 -4 4 0 3]);
%             xticks(-5:0.5:5);
%             xlabel('x');
%             yticks(-5:0.5:5);
%             ylabel('y');
%             zticks(0:0.1:3);
%             zlabel('z');
%             daspect([1 1 1])
%             hold on
% 
% baseTr = transl([-0.65 2.5 0.63]) * trotz(deg2rad(180)); % Linear UR3 built into Table
% r = LinearUR3(baseTr);
% q = [0, 0, 0, 0, 0, 0, 0]
% r.model.teach(q);

% PlaceObject('Wok.PLY', [0, 0, 0]); % Pallet of Bricks
% PlaceObject('GripperV3Link0.ply', [0, 0, 0]);


%% Real UR3 Stuff
% Check if Robot has Raspberry Pi and Router with Ethernet
% Raspberry to 5V USB Power (red lights, sometimes yellow flashes)
% Connect Pi to Router and allow 2min boot time
% Static IP: 192.168.27.1
%     If using WIFI -> connection name: 'robotics_<some_random_uid>'
%     UID is found on PI
%     One Machie can connect to this WIFI at a time.
%     SSH into PI (using cmd ssh robotics@192.168.27.1)
%     Username & Password: 'admin'

% Turn on UR3
% Program Robot -> Run Program -> File -> Load -> UR_ROS_DRIVER.URP
% Press Play before sending commands.

% Initialise Connection
% If topic doesnt exist:
%     ROS computer fully booted
%     Computer is not on a separate network
%     Don't have firewall blocking traffic
% rosinit('192.168.27.1'); % If unsure, please ask a tutor

% Current Joint State from real robot
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% If fail see latest message -> if empty not connected properly
jointStateSubscriber.LatestMessage % Use if Empty Robot Not Properly Connected

% Create a Variable with joint names so commands are associated with
% particular joint.
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Send Joint Angles with a 'client' and 'goal'
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

% CHange for how long movement should take
durationSeconds = 10; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% Change for end goal
% nextJointState_123456 = currentJointState_123456 + [pi/8,0,0,0,0,pi/8];
nextJointState_123456 = [pi, deg2rad(-43.2), deg2rad(36), deg2rad(281), deg2rad(-86.4), deg2rad(21.6)];
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

% goal.Trajectory.Points = [startJointSend; endJointSend];
% goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
% sendGoal(client,goal);

%% The Plan
% Current Pos -> Pose of UR3 Loaded up
% Go to a Default Pose -> Pose of Home Position
% Go to Location 1 -> Pose of 'Wash Basket'
% Go to Location 2 -> Pose of 'Plate 1'
% Go to Location 1 -> Pose of 'Wash Basket'
% Go to Location 3 -> Pose of 'Plate 2'
% Return to Default
