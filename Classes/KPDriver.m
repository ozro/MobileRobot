classdef KPDriver
    %robotKeypressDriver Creates a keyboard event handler and then lets the
 %user drive the robot with the arrow keys.

 properties(Constant)
 end

 properties(Access = private)
 fh=[];
 end

 properties(Access = public)
 end

 methods(Static = true)
     function drive(robot,vGain)
         % drive the robot
         Vmax = 0.02*vGain;
         dV = 0.006*vGain;
         key = pollKeyBoard();
         if(key ~= false)
             if(strcmp(key,'uparrow'))
                 disp('up');
                robot.move(Vmax,Vmax);
             elseif(strcmp(key,'downarrow'))
                 disp('down');
                robot.move(-Vmax,-Vmax);
             elseif(strcmp(key,'leftarrow'))
                 disp('left');
                 robot.move(Vmax,Vmax+dV);
             elseif(strcmp(key,'rightarrow'))
                 disp('right');
                robot.move(Vmax+dV,Vmax);
             elseif(strcmp(key,'s'))
                 disp('stop');
                 robot.move(0.0,0.0);
             end;
         end;
     end
 end

 methods(Access = private)

 end

 methods(Access = public)
     function obj = KPDriver(fh)
         % create a robotKeypressDriver for the figure handle
         % normally you call this with gcf for fh
         obj.fh = fh;
         set(fh,'KeyPressFcn',@KeyboardEventListener);
     end
 end

end

