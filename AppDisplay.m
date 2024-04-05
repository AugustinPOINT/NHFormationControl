classdef AppDisplay < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        mainAxis            matlab.graphics.axis.Axes
        distErrors          matlab.graphics.axis.Axes
        inputSignals        matlab.graphics.axis.Axes
        plotAxis2           matlab.graphics.axis.Axes
        plotSelection       matlab.ui.control.DropDown
        plotSelectionLabel  matlab.ui.control.Label
        middle
    end

    properties (Constant)
        marginX = 60;
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
            app.middle = app.UIFigure.Position(3:4)/2 + [175 0];

            % Create mainAxis
            app.mainAxis = axes(app.UIFigure);
            title(app.mainAxis, "Simulation", "FontSize", 20);
            xlabel(app.mainAxis, 'X (m)', 'FontSize', 20);
            ylabel(app.mainAxis, 'Y (m)', 'FontSize', 20);
            %zlabel(app.mainAxis, 'Z (m)');
            app.mainAxis.Units = "pixels";
            app.mainAxis.Position = [app.marginX (app.UIFigure.Position(4)-app.middle(1)+2*app.marginX)/2 app.middle(1)-2*app.marginX app.middle(1)-2*app.marginX];
            app.mainAxis.XLim = [-21 21];
            app.mainAxis.YLim = [-21 21];
            hold(app.mainAxis, "on");
            grid(app.mainAxis, "on");
            axis(app.mainAxis, "equal");

            % Create DropDownLabel
            app.plotSelectionLabel = uilabel(app.UIFigure);
            app.plotSelectionLabel.HorizontalAlignment = 'right';
            app.plotSelectionLabel.Position = [app.middle(1) app.UIFigure.Position(4)-3/2*app.marginX 65 22];
            app.plotSelectionLabel.Text = 'Plot type';

            % Create DropDown
            app.plotSelection = uidropdown(app.UIFigure, "Items", ["Input signals", "Errors"], "ValueChangedFcn", @(src,event) updatePlotType(app, src,event));
            app.plotSelection.Position = [app.middle(1)+70 app.UIFigure.Position(4)-3/2*app.marginX 100 22];

            % Create inputSignals
            app.inputSignals = axes(app.UIFigure);
            title(app.inputSignals, 'Input signals');
            xlabel(app.inputSignals, 'Time (s)');
            ylabel(app.inputSignals, 'Input signals ()');
            %zlabel(app.inputSignals, 'Z (m)');
            app.inputSignals.Units = "pixels";
            app.inputSignals.Position = [app.middle(1)+1/2*app.marginX app.middle(2)+1/2*app.marginX (app.UIFigure.Position(3)-app.middle(1))-app.marginX (app.UIFigure.Position(4)-app.middle(2))-2.25*app.marginX]; %[790 410 550 290];
            axis(app.inputSignals, "manual");
            app.inputSignals.XLim = [0 2.5];
            app.inputSignals.YLim = [-10 10];
            hold(app.inputSignals, "on");

            % Create distErrors
            app.distErrors = axes(app.UIFigure);
            title(app.distErrors, 'Interdistances');
            xlabel(app.distErrors, 'Time (s)');
            ylabel(app.distErrors, 'Data ()');
            %zlabel(app.distErrors, 'Z (m)');
            app.distErrors.Units = "pixels";
            app.distErrors.Position = [app.middle(1)+1/2*app.marginX app.middle(2)+1/2*app.marginX (app.UIFigure.Position(3)-app.middle(1))-app.marginX (app.UIFigure.Position(4)-app.middle(2))-2.25*app.marginX]; %[790 410 550 290];
            axis(app.distErrors, "manual");
            app.distErrors.XLim = [0 2.5];
            app.distErrors.YLim = [-10 10];
            hold(app.distErrors, "on");
            app.distErrors.Visible = 'off';

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

        function updatePlotType(app, src, event)
            plotType = src.Value;
            switch(plotType)
                case src.Items(1)
                    set([app.distErrors ; app.distErrors.Children], 'Visible','off');
                    set([app.inputSignals ; app.distErrors.Children], 'Visible','on');
                    % app.distErrors.Visible = 'off';
                    % app.inputSignals.Visible = 'on';
                case src.Items(2)
                    set([app.distErrors ; app.distErrors.Children], 'Visible','on');
                    set([app.inputSignals ; app.distErrors.Children], 'Visible','off');
                    % app.distErrors.Visible = 'on';
                    % app.inputSignals.Visible = 'off';
                otherwise
            end
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
            app.mainAxis.XLim = [-21 19];
            app.mainAxis.YLim = [-11 16];
            cla(app.distErrors);
            app.distErrors.XLim = [0 2.5];
            app.distErrors.YLim = [-10 10];
            cla(app.inputSignals);
            app.inputSignals.XLim = [0 2.5];
            app.inputSignals.YLim = [-10 10];
            cla(app.plotAxis2);
            axis(app.plotAxis2, "auto y");
            app.plotAxis2.XLim = [-0.5 2.5];
        end
    end
end
