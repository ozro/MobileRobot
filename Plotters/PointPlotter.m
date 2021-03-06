classdef PointPlotter
    properties
        pointX = 0;
        pointY = 0;
        
        fig;
        axisLim;
    end
    
    methods
        function obj = PointPlotter(xmin, xmax, ymin, ymax)
            obj.axisLim = [xmin, xmax, ymin, ymax];
            obj.fig = figure();
            PlotArrays(obj);
        end
        
        function obj = UpdatePoints(obj, x, y)
            obj.pointX = x;
            obj.pointY = y;
            PlotArrays(obj);
        end
        
        function PlotArrays(obj)
            figure(obj.fig);
            clf;
            plot(0,0, 'ks', -obj.pointY, obj.pointX, 'r*');
        end
    end 
end

