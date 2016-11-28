function approachSail(system, image,est, offset)
    global laserRanges;    
    searching = true;
    
    while(searching)
        ranges = laserRanges;
        
        [xbar, ybar, th, xArray, yArray, allX, allY] = image.findLineCandidate(ranges, -1);

        figure(2)
        plot(0, 0, 'ok',-1*allY, allX, '.b', -1*yArray, xArray, '.r');
        title('Robot Frame Laser Range Positions');
        legend('Robot', 'Other Points', 'Sail');
        xlabel('y (m)');
        ylabel('x (m)');
        xlim([-1.3, 1.3])
        ylim([-1.3, 1.3])
    
        searching = abs(xbar) + abs(ybar) < 0.08;
        beep
        pause(1);
    end
    
    sgn = 1;
    if(xbar< 0)
        th = th - pi;
    end
    thoffset = th + 15 * pi/180;
    xbar = xbar - (offset)*cos(thoffset);
    ybar = ybar - (offset)*sin(thoffset);
    
    tarPos = est.fusePose.bToA() * [xbar;ybar;1;];
    
    rPose = est.fusePose.getPoseVec();
    rth = rPose(3);
    
    tarth = rth + th;
    
    if(numel(yArray) == 0)
        beep;
    else
        system.executeTrajectoryToAbsPose(tarPos(1), tarPos(2), tarth, sgn, 0.15, true);
    end
end
