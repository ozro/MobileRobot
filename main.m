robot = NohBot();
ref = RefCon(1, 1, 0.5);

sTime = tic;
while(true)
    [vel, angvel] = ref.computeControl(toc(sTime));
    [vl, vr] = robot.angVelToWheel(vel, angvel);
    robot.move(vl, vr);
end