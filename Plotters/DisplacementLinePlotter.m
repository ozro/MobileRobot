classdef DisplacementLinePlotter<handle
    properties
        timeArray = zeros(1,1)
        leftArray = zeros(1,1)
        rghtArray = zeros(1,1)
        displacementArray = zeros(1,1);
        fig; % Handle for figure object
    end
    methods
        function obj = DisplacementLinePlotter()
            obj.fig = figure();
        end
        
        function obj = AddToArrays(obj, time, left,rght)
            %Append new data to end of arrays
            obj.timeArray = cat(1, obj.timeArray, time);
            obj.leftArray = cat(1, obj.leftArray, left);
            obj.rghtArray = cat(1, obj.rghtArray, rght);
            obj.displacementArray = (obj.leftArray+obj.rghtArray)/2;
            PlotArrays(obj);
        end
        
        function obj = ClearArrays(obj)
            obj.timeArray = zeros(1,1);
            obj.leftArray = zeros(1,1);
            obj.rghtArray = zeros(1,1);
        end
        
        function PlotArrays(obj)
            figure(obj.fig);
            clf;
            plot(obj.timeArray, obj.leftArray, obj.timeArray, obj.rghtArray);
        end
    end
end