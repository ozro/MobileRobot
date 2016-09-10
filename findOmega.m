function omega = findOmega(V,bearing,dist)
    if(bearing == 0)
        omega =0;
        return;
    end
    theta = 2*bearing;
    
    r = sqrt(dist*dist/(2-2*cosd(theta)));
    curv = 1/r;
    omega = curv * V;
end