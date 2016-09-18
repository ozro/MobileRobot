classdef TimePlotter
    properties
        time = zeros(1,1);
        data = zeros(1,1);
        
        fig;
        axisLim;
    end
    
    methods
        function obj = TimePlotter(xmin, xmax, ymin, ymax)
            obj.axisLim = [xmin, xmax, ymin, ymax];
            obj.fig = figure();
            PlotArrays(obj);
        end
        
        function obj = Update(obj, t, y)
            obj.time = [obj.time, t];
            obj.data = [obj.data, y];
            PlotArrays(obj);
            %axis(obj.axisLim);
        end
        
        function PlotArrays(obj)
            figure(obj.fig);
            clf;
            plot(obj.time, obj.data);
        end
    end 
end

