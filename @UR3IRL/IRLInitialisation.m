%Initialisation
rosinit('192.168.27.1'); % If unsure, please ask a tutor

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

jointStateSubscriber.LatestMessage

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

