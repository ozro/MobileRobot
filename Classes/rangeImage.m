classdef rangeImage
    
    methods
        function [xPos, yPos, th, goodX, goodY, allX, allY] = findLineCandidate(~, rangeImg)
            xPos = 0;
            yPos = 0;
            th = 0;
            goodX = zeros(0, 0);
            goodY = zeros(0,0);

            pixels = size(rangeImg,1);
            goodOnes = rangeImg > 0.06 & rangeImg < 4.0;
            rangeImg = rangeImg(goodOnes);
            indices = linspace(2, pixels + 1, pixels)';
            indices = indices(goodOnes);
            thArray = (indices-1)*(pi/180);
            
            allX = cos(thArray).*rangeImg;
            allY = sin(thArray).*rangeImg;
            
            for i = 1:numel(rangeImg)
                r1 = rangeImg(i);
                th1 = thArray(i);
                
                
                filter = sqrt(r1^2 + rangeImg.^2 - 2*r1*cos(thArray - th1).*rangeImg) <= 0.08;
                goodRange = rangeImg(filter);
                goodTh = thArray(filter);
                
                if(numel(goodRange)>6)
                    ranges = goodRange;
                    ths = goodTh;
                    
                    x = ranges .* cos(ths);
                    y = ranges .* sin(ths);
                    
                    xbar = mean(x);
                    ybar = mean(y);
                    
                    px = x - xbar;
                    py = y - ybar;
                    
                    Ixx = px' * px;
                    Iyy = py' * py;
                    Ixy = -px' * py;
                    Inertia = [Ixx, Ixy;Ixy,Iyy]/numel(goodRange);
                    lambda = eig(Inertia);
                    lambda = sqrt(lambda) * 1000.0;
                    if(lambda(1) < 1.3)
                        xPos = xbar;
                        yPos = ybar;
                        th = atan2(2*Ixy, Iyy-Ixx)/2;
                        goodX = x;
                        goodY = y;
                        return;
                    end
                end
            end
        end
    end
end

