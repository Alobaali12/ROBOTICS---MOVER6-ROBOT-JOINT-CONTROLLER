% Date 17/12/2024
% Authors: Ali Mohamed and Elion Selko
% Code Description: This script takes a user defined end effector translation and
% rotation defined in X, Y, Z, A, B, C which determines the pose of end effector frame with
% respect to robot base frame.
% The end effector moves linearly between the initial point and the
% final point.
% The URDF file is equipped with joint limits, which ik solver needs to satisfy
% whenever an Inverse Kinematics problem is solved, otherwise the code
% breaks and throws an error.
% The algorithm is also equipped with a customized inverse kinematics
% solver which the user can use if they uncomment lines [90-94] and [100-106]
% Additionally the user may establish a communication with ROS topics and
% publish joint conifgurations by uncommenting lines [40-46] and [129-135]
clear

clc
close all
lastwarn('');
rng default
% Position of end effector in meters
X = 0.445;
Y = 0;
Z = 0.4425;
% Pose of end effector in radians
A = 0;
B = 0;
C = 0;
N = 20; % Number of samples/ waypoints
%% Linear motion controller
% Establish connection with the desired topics
% rosinit
% pub = rospublisher('/joint_demands','std_msgs/Float32MultiArray');
% sub = rossubscriber("/joint_states", "sensor_msgs/JointState");
% Joint_Received = receive(sub,10);
% display(Joint_Received.Position);
% Initial_Joint_Config= struct('JointName', {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
'JointPosition', num2cell(Joint_Received.Position'));
% initial_Pos = getTransform(mover6, Initial_Joint_Config , 'link6');
mover6 = importrobot('CPMOVER6.urdf');
initial_Pos = getTransform(mover6, mover6.homeConfiguration, 'link6');
% Define an inverse kinematics object and the optimization weighting
% parameters
ik = inverseKinematics('RigidBodyTree', mover6);
weights = [0.25 0.25 0.25 1 1 1];
x_int = round(initial_Pos(1, 4),4);
y_int = round(initial_Pos(2, 4),4);
z_int = round(initial_Pos(3, 4),4);
t = linspace(0, 1, N); % parameter t for interpolation
% Define points along a straight line between initial and final position
x_co = (1-t) * x_int + t*X;
y_co = (1-t) * y_int + t*Y;
z_co = (1-t) * z_int + t*Z;
points_matrix = [x_co; y_co; z_co]; % Waypoint matrix 3 X N
% Extract current rotation transformation matrix of the end effector and
% apply the rotation transformation matrix from rotation A,B,C around Z, Y
% and X axis respectively
Rotation_TF = rotm2tform(tform2rotm(initial_Pos)*eul2rotm([A,B,C]));
% Initiatize and preallocate memory for Linear Transformation Matrix and HTM
Linear_TF = zeros(4, 4, N);
H_T_M = zeros(4, 4, N);
for i = 1:N
Linear_TF(:,:,i) = [1 0 0 points_matrix(1, i);
0 1 0 points_matrix(2, i);
0 0 1 points_matrix(3, i);
0 0 0 1];
H_T_M(:, :, i) = Linear_TF(:,:,i) * Rotation_TF; % homogeneous transformation matrix
end
initialguess = mover6.homeConfiguration;
% initialguess=Initial_Joint_Config;
Waypoints_Joints_Sol_M = zeros(N, 6); % Waypoints, joints solutions matrix
EndEffector_Realized_WayPoint=zeros(N,3); % Realized positions
% Set the opitmization terminal conditions for the solver fmincon as well as
% intiating a Multistart object for multiple solution searches
% options=optimoptions(@fmincon,'MaxFunctionEvaluations',1e6,'StepTolerance',...
% 1e-6,'ConstraintTolerance',1e-2);
% ms=MultiStart("FunctionTolerance",1e-5);
% u=[initialguess.JointPosition];
% weightsMS=[50 50 50 1 1 1];
for i = 1:N
current_HTM = H_T_M(:, :, i);
 [config, solnInfo] = ik('link6', current_HTM, weights, initialguess); % Solve IK
Waypoints_Joints_Sol_M(i,:) = [config.JointPosition];
% CostF=@(u)Cost(u,current_HTM,weightsMS,mover6);
% problem=createOptimProblem('fmincon','objective',CostF,'x0',u,...
% 'options',options,...
% 'ub',pi/2*ones(size([initialguess.JointPosition])),...
% 'lb',-pi/2*ones(size([initialguess.JointPosition])));
% [u,cost]=run(ms,problem,4);
% Waypoints_Joints_Sol_M(i,:) = u;
for j = 1:6
initialguess(j).JointPosition = Waypoints_Joints_Sol_M(i,j);
end
Tr(:,:,i) = getTransform(mover6,config,'link6');
EndEffector_Realized_WayPoint(i,:)=round(Tr(1:3,end,i)',4);
end
Angles = tform2eul(Tr);
[message, warningId] = lastwarn;
Difference=(Tr(1:3,end,end)-H_T_M(1:3,end,end));
if ~isempty(message)
error('Invalid pose: %s\nPlease Try another position in space !\n', message);
elseif (norm(Difference,2)>.05)
fprintf("Error in reached X: %.4f meters\nError in reached Y: %.4f meters\nError in " + ...
"reached Z: %.4f meters\n",Difference(1),Difference(2),Difference(3));

error('Invalid pose: %s\nPlease Try another position in space !\n', message);
end
fprintf("Error in reached X: %.4f meters\nError in reached Y: %.4f meters\nError in " + ...
"reached Z: %.4f meters\n",Difference(1),Difference(2),Difference(3));
% Publish all waypoints' robot joint angles as a single vector to
% '\joint_demands'
% Data=[];
% for n=1:N
% Data=[Data,Waypoints_Joints_Sol_M(n,:)];
% end
% msg = rosmessage(pub);
% msg.Data = Data;
% send(pub,msg);
%% Plotting
% Joint angle trajectory
figure;
hold on;
colours=[{'r'},{'g'},{'b'},{'y'},{'c'},{'k'}];
DisplayName=[];
jointsAm=6;
for jointIdx = 1:jointsAm
plot(Waypoints_Joints_Sol_M(:, jointIdx),[colours{jointIdx}], 'DisplayName', sprintf('Desired Joint %d',
jointIdx));
end
hold on
plot([1 N],[1.571 1.571],'r--','DisplayName','Joint Upper Limit')
plot([1 N],[-1.571 -1.571],'r--','DisplayName','Joint Lower Limit')
xticks(1:N)
hold off;
legend('show','Location','northwest');
xlabel('Waypoint Index');
ylabel('Joint Angle (rad)');
title('Joint Trajectories for Linear Motion');
grid on;
% The predicted linear trajectory and the realized trajectory from solving
% inverse kinematics
figure;
plot3([x_int, X], [y_int, Y], [z_int, Z], 'b-', 'LineWidth', 1.5); % Line
hold on;
plot3(x_co, y_co, z_co, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Endpoints
plot3(EndEffector_Realized_WayPoint(:,1),EndEffector_Realized_WayPoint(:,2),EndEffector_Realized_WayPoint(:
,3),'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Endpoints
grid on;
legend("Line","Desired Waypoints","Realized Waypoints")
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%% Define the objective function
function [J]=Cost(u,endEffector,weights,mover6)
Joint_State=struct('JointName', {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
'JointPosition', num2cell(u));
Transform=getTransform(mover6,Joint_State,'link6');
% J=InverseKinematics_mex(Transform,endEffector,weights); % User may choose
% to run an object file, building an object file can be done using MATLAB Coder
J=InverseKinematics(Transform,endEffector,weights);
end
function [J,c,ceq]=InverseKinematics(Transform,endEffector,weights)
AnglesEnd=tform2eul(endEffector);
TrEnd=tform2trvec(endEffector);
AnglesTransf=tform2eul(Transform);
TrTransf=tform2trvec(Transform);
J=sum(([TrEnd,AnglesEnd]-[TrTransf, AnglesTransf]).^2.*weights);
ceq=[];
A=[reshape(AnglesEnd,[],1)-pi/2;reshape(-AnglesEnd,[],1)-pi/2];
c = A;
end