function wait(est, t)
    dt = 0.1;
    beep
    for i= 1:t/dt
        est.processOdometryData();
        est.processRangeImage();
        pause(dt);
    end
    beep
    return
end

