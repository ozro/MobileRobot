classdef rangeImage
    
    methods
        function [xPos, yPos, th, goodX, goodY, allX, allY] = findLineCandidate(~, rangeImg, preference)
            xPos = 0;
            yPos = 0;
            th = 0;
            solutions = [];
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
            
            for i = 1:size(rangeImg, 1)
                r1 = rangeImg(i);
                th1 = thArray(i);
                
                
                filter = sqrt(r1^2 + rangeImg.^2 - 2*r1*cos(thArray - th1).*rangeImg) <= 0.20;
                goodRange = rangeImg(filter);
                goodTh = thArray(filter);
                
                if(size(goodRange, 1)>6)
                    ranges = goodRange;
                    ths = goodTh;
                    
                    x = ranges .* cos(ths);
                    y = ranges .* sin(ths);
                    
                    xbar = mean(x);
                    ybar = mean(y);
                    
                    px = x - xbar;
                    py = y - ybar;
                    
                    d = sqrt((max(px)-min(px))^2 + (max(py)-min(py))^2);
                    if(d>15/100)
                        continue;
                    else
                        p = 'sail is too long';
                    end
                    
                    Ixx = px' * px;
                    Iyy = py' * py;
                    Ixy = -px' * py;
                    Inertia = [Ixx, Ixy;Ixy,Iyy]/size(goodRange,1);
                    lambda = eig(Inertia);
                    lambda = sqrt(lambda) * 1000.0;

                    if(lambda(1) < 2)
                        xPos = xbar;
                        yPos = ybar;
                        th = atan2(2*Ixy, Iyy-Ixx)/2;

                        solutions = [solutions; [xPos, yPos, th]];
                            
                        goodX = x;
                        goodY = y;
                        return;
                    else
                        p = 'lambda too large';
                    end
                else
                    p = 'range data too short';
                end
            end
            len = size(solutions, 1);
            if(len > 1)
                minDist = 50;
                for i = 1:len
                    dist = sqrt(solutions(i,1)^2 + solutions(i,2)^2);
                    if( dist < minDist)
                        minDist = dist;
                        xPos = solutions(i,1);
                        yPos = solutions(i,2);
                        th = solutions(i,3);
                        return
                    end
                end
            end
        end
    end
end

