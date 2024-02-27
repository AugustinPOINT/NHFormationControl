classdef AppDisplay < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure      matlab.ui.Figure
        mainAxis      matlab.graphics.axis.Axes
        plotAxis1     matlab.graphics.axis.Axes
        plotAxis2     matlab.graphics.axis.Axes
        middle
    end

    properties (Constant)
        marginX = 50;
        marginY = 50; %130
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [0 0 1680 1024];
            %app.UIFigure.Position = [0 0 1350 750];
            app.UIFigure.Name = 'Simulator Vxx';
            app.UIFigure.WindowState = 'maximized';
            app.middle = app.UIFigure.Position(3:4)/2 + [165 0];

            % Create mainAxis
            app.mainAxis = axes(app.UIFigure);
            title(app.mainAxis, 'Simulation');
            xlabel(app.mainAxis, 'X (m)');
            ylabel(app.mainAxis, 'Y (m)');
            %zlabel(app.mainAxis, 'Z (m)');
            app.mainAxis.Units = "pixels";
            app.mainAxis.Position = [app.marginX (app.UIFigure.Position(4)-app.middle(1)+2*app.marginX)/2 app.middle(1)-2*app.marginX app.middle(1)-2*app.marginX];
            app.mainAxis.XLim = [-20 20];
            app.mainAxis.YLim = [-20 20];
            hold(app.mainAxis, "on");
            grid(app.mainAxis, "on");
            %axis(app.mainAxis, "equal");

            % Create plotAxis1
            app.plotAxis1 = axes(app.UIFigure);
            title(app.plotAxis1, 'Interdistances');
            xlabel(app.plotAxis1, 'Time (s)');
            ylabel(app.plotAxis1, 'Data ()');
            %zlabel(app.plotAxis1, 'Z (m)');
            app.plotAxis1.Units = "pixels";
            app.plotAxis1.Position = [app.middle(1)+1/2*app.marginX app.middle(2)+app.marginX (app.UIFigure.Position(3)-app.middle(1))-app.marginX (app.UIFigure.Position(4)-app.middle(2))-2.25*app.marginX]; %[790 410 550 290];
            axis(app.plotAxis1, "manual");
            app.plotAxis1.XLim = [0 2.5];
            app.plotAxis1.YLim = [-10 10];
            hold(app.plotAxis1, "on");

            % Create plotAxis2
            app.plotAxis2 = axes(app.UIFigure);
            title(app.plotAxis2, 'Trajectories');
            xlabel(app.plotAxis2, 'Time (s)');
            ylabel(app.plotAxis2, 'Data ()');
            %zlabel(app.plotAxis2, 'Z (m)');
            app.plotAxis2.Units = "pixels";
            app.plotAxis2.Position = [app.middle(1)+1/2*app.marginX app.marginX (app.UIFigure.Position(3)-app.middle(1))-app.marginX app.middle(2)-7/4*app.marginX]; %[790 50 550 290];
            axis(app.plotAxis2, "auto y");
            app.plotAxis2.XLim = [-0.5 2.5];
            hold(app.plotAxis2, "on");

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = AppDisplay

            % Create UIFigure and components
            createComponents(app);

            % Register the app with App Designer
            registerApp(app, app.UIFigure);

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end

        % Clears the app axis
        function clearAll(app)
            
            cla(app.mainAxis);
            app.mainAxis.XLim = [-20 20];
            app.mainAxis.YLim = [-20 20];
            cla(app.plotAxis1);
            app.plotAxis1.XLim = [0 2.5];
            app.plotAxis1.YLim = [-10 10];
            cla(app.plotAxis1);
            axis(app.plotAxis2, "auto y");
            app.plotAxis2.XLim = [-0.5 2.5];
        end
    end
end
