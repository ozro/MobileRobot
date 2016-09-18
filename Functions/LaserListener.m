function LaserListener(~, latestMessage )
    global laserTime;
    global laserRanges;

    laserTime= latestMessage.Header.Stamp.Sec +(double(latestMessage.Header.Stamp.Nsec)/1000000000);
    laserRanges= latestMessage.Ranges;
end

