%% Robot Initialization
%robot = NohBot();
rosshutdown();
robot = raspbot();
sendVelocity(robot,0,0)