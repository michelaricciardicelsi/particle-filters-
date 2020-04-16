clear all;

% %{
path = [3.00    1.00;
        3.25    1.75;
        4.25    4.00;
        7.25    5.25;
        8.25    6.75;
        2.00   10.00;
        2.25   12.00;
        2.25   14.00;
        22.00   14.00;
        21.00   8.75;
        15.75   8.75;
        15.75   4.00;
        24.00   4.00;
        24.00   2.00];
%}

%{
path = [2.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
%}

robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];

robotRadius = 0.4;
global robot;
robot = ExampleHelperRobotSimulator('complexMap',2);
% robot = ExampleHelperRobotSimulator('simpleMap',2);
robot.enableLaser(true);
robot.LaserSensor.NumReadings = 5;
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

% rosinit
% enableROSInterface(robot,true);
% scanSub = rossubscriber('scan');
% [velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
% tftree = rostf;
% pause(1)

plot(path(:,1), path(:,2),'k--d')

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;

goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

dt = 0.05; % time step
controlRate = robotics.Rate(1/dt);

pf = robotics.ParticleFilter;

% initialize(pf, 5000, [6.5 6.5 0], eye(3), 'CircularVariables',[0 0 1]);
% initialize(pf, 5000, [0 13;0 13;0 2*pi], 'CircularVariables',[0 0 1]);
initialize(pf, 2000, [0 25.5;0 20;0 2*pi], 'CircularVariables',[0 0 1]);
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

h = plot(pf.Particles(:,1),pf.Particles(:,2),'.g');
set(h,'XDataSource','pf.Particles(:,1)')
set(h,'YDataSource','pf.Particles(:,2)')

% initialize(pf, 5000, [2 2 0], eye(3), 'CircularVariables',[0 0 1]);
% refreshdata(h);
% initialize(pf, 5000, [10 10 0], eye(3), 'CircularVariables',[0 0 1]);
% refreshdata(h);

% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @exampleHelperRobotStateTransition;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @exampleHelperRobotMeasurementLikelihood;

% measurement = robot.LaserSensor.getReading(robotCurrentPose);
% likelihood = exampleHelperRobotMeasurementLikelihood(pf, pf.Particles, measurement);

% robot.LaserSensor.getReading([5 1 0]);
% robot.LaserSensor.getReading([8 2 pi]);

%{
[v, omega] = controller(robot.getRobotPose);
drive(robot, v, omega);
z=[];
for i=1:1:10
    z(:,i) = getRangeData(robot);
    pause(0.1);
end
disp(z)
%}
correct_on = 0;
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    waitfor(controlRate); 
    
    [statePred, covPred] = predict(pf, dt, [v omega]);
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
%     [ranges, angles] = getRangeData(robot);
%     ranges(isnan(ranges)) = robot.LaserSensor.MaxRange;
    %modScan = lidarScan(ranges, angles);
    % disp(ranges')
    
%     if ~isempty(ranges)
%         [stateCorrected, covCorrected] = correct(pf, ranges);
%     else
%         stateCorrected = statePred;
%         covCorrected = covPred;
%     end
    
    if correct_on == 4
        [ranges, angles] = getRangeData(robot);
        ranges(isnan(ranges)) = robot.LaserSensor.MaxRange;
        [stateCorrected, covCorrected] = correct(pf, ranges);
        correct_on = 0; 
    end
    refreshdata(h);
    
%     scanMsg = receive(scanSub);
%     scan = lidarScan(scanMsg);
%     ranges = scan.Ranges;
%     ranges(isnan(ranges)) = robot.LaserSensor.MaxRange;
    
    % Re-compute the distance to the goal
    robotCurrentPose = robot.getRobotPose;
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    if (distanceToGoal < 2) && (correct_on < 3)
        correct_on = 3;
    end
    
    % waitfor(controlRate); 
    correct_on = correct_on + 1;
end

