function EncoderListener(~, latestMessage)
    global encoderData;
    global encoderTime;
    x= latestMessage.Vector.X;
    y= latestMessage.Vector.Y;
    t= double(latestMessage.Header.Stamp.Sec) +(double(latestMessage.Header.Stamp.Nsec)/1000000000);
    enc = [x, y];
    
    encoderData = enc;
    encoderTime = t;
end
