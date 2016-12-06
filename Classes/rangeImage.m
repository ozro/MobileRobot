classdef rangeImage
    
    methods
        function [xPos, yPos, th, goodX, goodY, allX, allY] = findLineCandidate(~, rangeImg, ~)            
            xPos = 0;
            yPos = 0;
            th = 0;
            minDist = 1000;
            goodX = zeros(0, 0);
            goodY = zeros(0,0);
            
            pixels = size(rangeImg,1);
            goodOnes = rangeImg > 0.06 & rangeImg < 12 * 0.3048;
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
                        p = 'sail is too long';
                        continue;
                    else
                    end
                    
                    Ixx = px' * px;
                    Iyy = py' * py;
                    Ixy = -px' * py;
                    Inertia = [Ixx, Ixy;Ixy,Iyy]/size(goodRange,1);
                    lambda = eig(Inertia);
                    lambda = sqrt(lambda) * 1000.0;

                    if(lambda(1) < 2)
                        dist = sqrt(xbar^2 + ybar^2);
                        if(dist<minDist && xbar > 0)
                            xPos = xbar;
                            yPos = ybar;
                            th = atan2(2*Ixy, Iyy-Ixx)/2;
                            minDist = dist;
                            
                            goodX = x;
                            goodY = y;
                        end
                    else
                        p = 'lambda too large';
                    end
                else
                    p = 'range data too short';
                end
            end
        end
    end
end

