function uref = trapezoidalV(t, aMax, vMax, dist, sgn)
    tRamp = vMax/aMax;
    tf = dist/vMax + tRamp;

    if(t < 0 || tf - t < 0)
        uref = 0;
        return;
    end
            
    if(t < tRamp)
        uref = aMax * t * sgn;
        return;
    end
    if(tf-t< tRamp)
        uref = -aMax * (t-tf) * sgn;
        return;
    end
    if(t>tRamp && t<(tf-tRamp))
        uref = vMax * sgn;
        return;
    end
end

