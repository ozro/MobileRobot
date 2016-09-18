classdef DisplacementLinePlotter<handle
    properties
        timeArray = zeros(1,1)
        v = zeros(1,2)
        fig; % Handle for figure object
        
        axisLim;
    end
    methods
        function obj = DisplacementLinePlotter(xmin, xmax, ymin, ymax)
            obj.axisLim = [xmin, xmax, ymin, ymax];
            obj.fig = figure();
            PlotArrays(obj);
        end
        
        function obj = AddToArrays(obj, time, v)
            %Append new data to end of arrays
            obj.timeArray = cat(1, obj.timeArray, time);
            obj.v = cat(1, obj.v, v);
            PlotArrays(obj);
        end
        
        function obj = ClearArrays(obj)
            obj.timeArray = zeros(1,1);
            obj.v = zeros(1,1);
        end
        
        function PlotArrays(obj)
            figure(obj.fig);
            clf;
            plot(obj.timeArray, obj.v);
            %axis(obj.axisLim);
        end
    end
end