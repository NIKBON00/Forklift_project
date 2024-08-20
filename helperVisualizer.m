classdef helperVisualizer < handle
    %helperVisualizer Helper class to display binary occupancy map,
    %   2-D lidar scan data, obstacles in the range of the sensor and
    %   a danger color signifying the extent of danger
    properties (Access = private)
        % Figure handle of the visualization window
        fig
        % Figure name of the visualization window
        figureName = 'Collision warning display';
        % Axes displaying binary occupancy map
        hAxes1
        % Axes displaying 2-D lidar data
        hAxes2
        % Axes displaying obstacles
        hAxes3
        % Axes displaying danger color
        hAxes4
    end
    methods
        function obj = helperVisualizer()
            % Initialize figure handle
            fh = findobj('type','figure','name',obj.figureName);
            if isempty(fh)
                obj.fig = figure('Name',obj.figureName,'visible','off');
            else
                obj.fig = fh;
                obj.fig.Visible = 'off';
            end
            hPanelTopLeft = uipanel(obj.fig,'Units','Normalized','Position',[0 9/20 11/20 11/20],'Title','Binary Occupancy Map');
            hPanelBotLeft = uipanel(obj.fig,'Units','Normalized','Position',[0 0 11/20 9/20],'Title','2-D Data');
            hPanelTopRight = uipanel(obj.fig,'Units','Normalized','Position',[11/20 3/10 9/20 7/10],'Title','Bird''s-Eye Plot');
            hPanel4BotRight = uipanel(obj.fig,'Units','Normalized','Position',[11/20 0 9/20 3/10],'Title','Warning Color');
            
            obj.hAxes1 = axes('Parent',hPanelTopLeft);
            obj.hAxes2 = axes('Parent',hPanelBotLeft);
            obj.hAxes3 = axes('Parent',hPanelTopRight);
            obj.hAxes4 = axes('Parent',hPanel4BotRight);
        end
        
        function hRobot = plotBinaryMap(obj, map, pose)
            % Display occupancy map with obstacles
            show(map,'Parent',obj.hAxes1);
            hRobot = helperPlotRobot(obj.hAxes1, pose);
            title(obj.hAxes1,'Warehouse Floor Plan');
        end
        
        function plotObstacleDisplay(obj)
            % Display AGV on axis3
            hold(obj.hAxes3,'on');
            legend(obj.hAxes3,'boxoff');
            scatter(obj.hAxes3,-0.3,0,180,'g','>','filled','DisplayName','AGV');
            plot(obj.hAxes3, [0 0], [0 -5], '--k','HandleVisibility','off')
            plot(obj.hAxes3, [0 0], [0 5], '--k','HandleVisibility','off')
            
            % Set limits of obj.hAxes3
            set(obj.hAxes3,'Color','w');
            ylim(obj.hAxes3,[-5 5]);
            xlim(obj.hAxes3,[-2 10]);
            hold(obj.hAxes3,'off');
            xlabel(obj.hAxes3,'X (meters)');
            ylabel(obj.hAxes3,'Y (meters)');
            title(obj.hAxes3, 'Display obstacles');
        end
        
        function updateMapDisplay(obj,hRobot,robotCurrentPose)

           
                 % Update robot pose on the map
                 helperUpdateMap(hRobot, robotCurrentPose);
                 hold(obj.hAxes1,'on');
                 plot(obj.hAxes1,robotCurrentPose(1),robotCurrentPose(2),'g.')
                 hold(obj.hAxes1,'off');
            
        end
        
        function plotLidarScan(obj,scan,currentOrientation)
            % Plot 2-D lidar scans
            plot(scan,'Parent',obj.hAxes2);
            helperPlotRobot(obj.hAxes2, [0 0 currentOrientation]);
            title(obj.hAxes2, "2-D lidar scan");
            xlabel(obj.hAxes2,'X (m)');
            ylabel(obj.hAxes2,'Y (m)');
            xlim(obj.hAxes2,[0 5]);
            ylim(obj.hAxes2,[-5 5]);
        end
        
        function sc = displayObstacles(obj,nearxy)
            % Display obstacle at the mentioned position
            hold(obj.hAxes3,'on');
            sc =scatter(obj.hAxes3,nearxy(1),nearxy(2),'filled','DisplayName','Obstacle');
            hold(obj.hAxes3,'off');
        end
        
        function circleDisplay(obj,c)
            % Update the danger level as a filled circle
            if ~isempty(c)
                scatter(obj.hAxes4,0.5,0.5,1000,c,'filled');
            else
                scatter(obj.hAxes4,[],[],[],'filled');
            end
            obj.hAxes4.YAxis.Visible = 'off';
            obj.hAxes4.XAxis.Visible = 'off';
            set(obj.hAxes4, 'Color', 'none');
        end
        
        function updateDisplay(obj)
            % Update the visibility of the visualization window
            obj.fig.Visible = 'on';
        end
        
        function ax3 = getDetectionAreaAxes(obj)
            % Return axes object of detection area
            ax3 = obj.hAxes3;
        end
        
        function closeDisplay(obj)
            % Close visualization window
            close(obj.fig);
        end
    end
end