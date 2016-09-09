%% Robot Initialization
rosshutdown();
robot = raspbot();
sendVelocity(robot,0,0)