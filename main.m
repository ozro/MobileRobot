robot = NohBot();

h = figure;
KPDriver(h);
vGain = 1;

p1 = [0,      0; 
      0, 1.2192];
p2 = [1.2192, 0; 
      0, 0];
LML = LineMapLocalizer(p1, p2,0.01, 0.001, 0.0005);
robotPose = pose(15*0.0254, 9*0.0254, pi/2);

robot.laserOn();

global laserRanges;

while(true)
    
    rangeImg = laserRanges;
    range = 1:10:360;
    rangeImg = rangeImg(range');
    pixels = size(rangeImg,1);
    goodOnes = rangeImg > 0.06 & rangeImg < 4.0;
    rangeImg = rangeImg(goodOnes);
    indices = (1:10:360)';
    indices = indices(goodOnes);    
    x = cosd(indices).*rangeImg;
    y = sind(indices).*rangeImg;
    modelPts = [x';y';ones(1,size(indices, 1))];
    
    [success, finalPose] = LML.refinePose(robotPose, modelPts, 50);
    robotPose = finalPose;
    robotPose.getPoseVec()
    
    KPDriver.drive(robot.rasp, vGain);
    pause(0.05);
end








% % pi = [0;0];
% % p1 = [-1;-1];
% % p2 = [1;1];
% % [rad2, po] = ClosestPOLS(pi,p1,p2);
% % rad2
% % po
% 
% % p1 = [-10, 0;
% %         0, 10];
% % p2 = [ 10,  0;
% %         0,-10];
% % mps = [-5,5,-2,2,0,0,0,0,0,0;
% %         0,0,0,0,1,-1,5,-5,2,0;
% %         1,1,1,1,1,1,1,1,1,1];
% % ps = pose(0,0.2,0);
% % LML = LineMapLocalizer(p1,p2,0.01,0.001,0.0005);
% % [e, J] = LML.getJacobian(ps,mps);
% % e
% % J
% 
% % Set up lines
% p1 = [-2 ; -2];
% p2 = [ 2 ; -2];
% p3 = [ 2 ; 2];
% p4 = [-2 ; 2];
% lines_p1 = [p1 p2 p3 p4];
% lines_p2 = [p2 p3 p4 p1];
% % Set up test points
% nPts = 10;
% x1 = -2.0*ones(1,nPts);
% x2 = linspace(-2.0,2.0,nPts);
% x3 = 2.0*ones(1,nPts);
% y1 = linspace(0.0,2.0,nPts);
% y2 = 2.0*ones(1,nPts);
% y3 = linspace(2.0,0,nPts);
% w = ones(1,3*nPts);
% x1pts = [x1 x2 x3];
% y1pts = [y1 y2 y3];
% w1pts = w;
% modelPts = [x1pts ; y1pts ; w1pts];
% % pick a pose
% dx = -0.05*rand();
% dy = -0.05*rand();
% dt = -0.05+0.2*rand();
% thePose = pose(0.0+dx,0.0+dy,0.0+dt);
% 
% robotPose = pose(15*0.0254,9*0.0254,pi()/2.0);
% LML = LineMapLocalizer(lines_p1,lines_p2,0.01,0.001,0.0005);
% finalPose = LML.refinePose(robotPose, modelPts, 60);
% 


