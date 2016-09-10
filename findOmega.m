function omega = findOmega(V,bearing,dist)
    if(bearing == 0)
        omega =0;
        return;
    end
    theta = 2*bearing;

    S = solve(dist*dist==2*R*R - 2*R*R*cosd(theta), R);
    R = abs(S(1));
    curv = 1/R;
    omega = curv * V;
end