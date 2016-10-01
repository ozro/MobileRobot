classdef RefCon

    properties
        ks
        kv
        tPause
    end
    
    methods
        function obj = RefCon(ks, kv, tPause)
            obj.ks = ks;
            obj.kv = kv;
            obj.tPause = tPause;
        end
        function [vel, angVel] = computeControl(obj, timeNow)
            v = 0.2;
            sf = 1;
            kt = 2*pi/sf;
            kk = 15.1084;
            Tf = (obj.ks/obj.kv)*sf/v;
            
            t = (obj.kv/obj.ks) * timeNow - obj.tPause;
            if(t < 0)
                vel = 0;
                angVel = 0;
                return;
            elseif (t > Tf)
                vel = 0;
                angVel = 0;
                return
            end
            
            s = v * t;
            vel = obj.kv * v;
            angVel = kk/obj.ks*sin(kt*s)*obj.kv * v;
        end
        function duration = getTrajDuration(obj)
            sf = 1;
            v = 0.2;
            duration = obj.ks/obj.kv * sf/v + 2*obj.tPause;
        end
    end
    
end

