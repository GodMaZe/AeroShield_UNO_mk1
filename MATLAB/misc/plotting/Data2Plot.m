classdef Data2Plot
    %DATA2PLOT Data2Plot represents an object containing the attributes to be used in the plotting
    %functions.

    properties (SetAccess = immutable)
        t;
        x;
        that;
        xhat;
        tdims;
        ntsamples;
        xdims;
        nxsamples;
        xhatdims;
        nxhatsamples;
        isxhat;
        isvalid;
    end

    properties
        tunits;
        xunits;
        title;
        subtitle;
        figure_name;
        show_zeroline;
        plottype;
        units_brackettype;
        grid;
        show_legend;
        figsize;
    end

    methods
        function obj = Data2Plot(t, x, that, xhat, plottype, tunits, xunits, title, figure_name, show_zeroline, units_brackettype, grid, subtitle, show_legend, figsize)
            arguments (Input)
                t;
                x;
                that = [];
                xhat = [];
                plottype = "plot"; % plot, stairs, scatter
                tunits = "s";
                xunits = "-";
                title = "X plot";
                figure_name = [];
                show_zeroline = false;
                units_brackettype = "s"; % r - round, s - square, c - curly
                grid = "all"; % all = minor + major, minor, none
                subtitle = "";
                show_legend = true;
                figsize = [0, 0, 17, 13.6];
            end
            obj.t = t;
            obj.x = x;
            obj.that = that;
            obj.xhat = xhat;
            obj.plottype = plottype;
            obj.tunits = tunits;
            obj.xunits = xunits;
            obj.title = title;
            obj.figure_name = figure_name;
            obj.show_zeroline = show_zeroline;
            obj.units_brackettype = units_brackettype;
            obj.grid = grid;
            obj.subtitle = subtitle;
            obj.show_legend = show_legend;
            obj.figsize = figsize;

            obj.tdims = size(t, 1);
            obj.ntsamples = size(t, 2);
            obj.xdims = size(x, 1);
            obj.nxsamples = size(x, 2);
            obj.isxhat = ~isempty(xhat);
            obj.xhatdims = size(xhat, 1);
            obj.nxhatsamples = size(xhat, 2);

            obj.isvalid = obj.checkdims();
        end

        function [f, ax] = plotx(obj, i, xlabel_, ylabel_, filename)
            arguments (Input)
                obj;
                i = -1;
                xlabel_ = "t";
                ylabel_ = "x";
                filename = [];
            end
            arguments (Output)
                f;
                ax;
            end

            if ~obj.isvalid
                error('Invalid dimensions for plotting.');
            end

            [f, ax] = obj.getfigureaxes(i);
            pltfcn = obj.plotcustom();

            if obj.xdims > 1
                hold on;
                for i=1:obj.xdims
                    if obj.tdims > 1
                        pltfcn(ax, obj.t(i, :), obj.x(i, :), "LineWidth", 1, "DisplayName", "x" + num2str(i));
                    else
                        pltfcn(ax, obj.t, obj.x(i, :), "LineWidth", 1, "DisplayName", "x" + num2str(i));
                    end
                end
                hold off;
            else
                if obj.plottype == "scatter"
                    pltfcn(ax, obj.t, obj.x, 120, "k", ".");
                else
                    pltfcn(ax, obj.t, obj.x, "LineWidth", 1, "DisplayName", "x");
                end
            end

            obj.updatefigure(f, ax, xlabel_, ylabel_);
            saveplot2file(f, filename);
        end

        function [fig, ax1, ax2] = plotoutnerror(obj, i, nskipsamples, filename, is_measurement)
        %PLOTOUTNERROR Plot function for plotting the measured state (x) and estimated
        %state (xhat) in time (t). Including the
        %error squared of the inconsistencies within the measured and estimated data.
        arguments (Input)
            obj;
            i = -1; % Figure id
            nskipsamples = 0;
            filename = [];
            is_measurement = false;
        end
        arguments (Output)
            fig;
            ax1;
            ax2;
        end

        if ~obj.isvalid
            error('Invalid dimensions for plotting.');
        end
        
        if nskipsamples < 0
            error("The number of samples to skip cannot be negative.");
        end
        
        if ~isscalar(nskipsamples)
            error("The number of samples to skip needs to be of a scalar value.");
        end
        
        
        SKIP_STEPS = nskipsamples + 1;
        select_mask = SKIP_STEPS:obj.ntsamples;
        canrmse = obj.isxhat;
        
        fig = obj.getcleanfigure(i);

        if canrmse
            tiledlayout(fig, 3, 1, "TileSpacing", "compact", "Padding", "tight");
            ax1 = nexttile([2 1]);
        else
            ax1 = axes(fig);
        end
        
        hold on;
        
        plotfcn = obj.plotcustom();
        
        
        that = obj.that;
        if isempty(obj.that)
            that = obj.t;
        end

        if is_measurement
            plotfcn(ax1, obj.t, obj.x, 'DisplayName', 'y');
            plotfcn(ax1, that, obj.xhat, 'DisplayName', 'ysim');
        else
            plotfcn(ax1, obj.t, obj.x, 'DisplayName', 'x');
            plotfcn(ax1, that, obj.xhat, 'DisplayName', 'xhat');
        end
        
        hold off;

        obj.updatefigure(i, ax1);

        if ~canrmse
            ax2 = [];
            return;
        end
        e_x = (obj.x(select_mask) - obj.xhat(select_mask)).^2;
        
        RMSE_X = rad2deg(sqrt(mean(e_x)));
        
        ax2 = nexttile;
        hold on;
        errorbar(ax2, obj.t(select_mask), zeros(size(obj.t)), e_x, '.');
        hold off;

        if is_measurement
            title(ax2, "Measurement and simulation output: difference squared");
            ylabel(ax2, "$\Delta y^{2}\ " + obj.getxunits() + "^2$", "Interpreter", "latex");
        else
            title(ax2, "Simulated and observed state x: difference squared");
            ylabel(ax2, "$\Delta x^{2}\ " + obj.getxunits() + "^2$", "Interpreter", "latex");
        end
        subtitle(ax1, "RMSE: $" + num2str(RMSE_X) + "\ " + obj.getxunits() + "$", "Interpreter", "latex");
        xlabel(ax2, "$t\ " + obj.gettunits() + "$", "Interpreter", "latex");
        
        grid minor;
        grid on;
        
        saveplot2file(fig, filename);
        end
        function isvalid = checkdims(obj)
            isvalid = true;
            
            if obj.tdims > 1
                isvalid = obj.xdims == obj.tdims;
            end

            isvalid = isvalid && obj.nxsamples == obj.ntsamples;

            if obj.isxhat
                isvalid = isvalid && obj.nxsamples == obj.nxhatsamples && obj.xdims == obj.xhatdims;
            end
        end
    end

    methods (Access = private)
        function unit = gettunits(obj)
            unit = obj.getunitsinbracket(obj.tunits);
        end

        function unit = getxunits(obj)
            unit = obj.getunitsinbracket(obj.xunits);
        end

        function unit = getunitsinbracket(obj, u)
            switch obj.units_brackettype
                case "s"
                    unit = "\left[" + u + "\right]";
                case "c"
                    unit = "\left\{" + u + "\}right]";
                case "r"
                    unit = "\left(" + u + ")right]";
                otherwise
                    unit = u;
            end
        end

        function fig = getfigure(obj, i)
            arguments (Input)
                obj;
                i = -1;
            end
            arguments (Output)
                fig;
            end

            if i < 0
                fig = figure;
            else
                fig = figure(i);
            end

            fig.Units = "centimeters";
            if ~isempty(obj.figsize)
                    fig.Position = obj.figsize;
            end
            if ~isempty(obj.figure_name)
                fig.Name = obj.figure_name;
                fig.NumberTitle = 'off';
            end
        end

        function fig = getcleanfigure(obj, i)
            arguments (Input)
                obj;
                i = -1;
            end
            arguments (Output)
                fig;
            end
            fig = obj.getfigure(i);
            clf(fig);
        end
        function [fig, ax] = getfigureaxes(obj, i)
            arguments (Input)
                obj;
                i = -1;
            end
            arguments (Output)
                fig;
                ax;
            end
            
            fig = obj.getfigure(i);
            ax = axes(fig);
        end

        function updatefigure(obj, fig, ax, xlabel_, ylabel_)
            arguments (Input)
                obj;
                fig;
                ax = [];
                xlabel_ = "t";
                ylabel_ = "x";
            end
            if isempty(ax)
                ax = axes(fig);
            end

            if obj.show_zeroline
                hold on;
                yline(ax, 0, 'k--', 'DisplayName', '0-line');
                hold off;
            end


            xlabel(ax, "$" + xlabel_ + "\ " + obj.gettunits() + "$", "Interpreter", "latex");
            ylabel(ax, "$" + ylabel_ + "\ " + obj.getxunits() + "$", "Interpreter", "latex");
            
            if ~isempty(obj.title)
                title(ax, obj.title);
            end
            if ~isempty(obj.subtitle)
                subtitle(ax, obj.subtitle);
            end
            
            obj.gridupdate(ax);
            obj.legendupdate(ax);
        end

        function gridupdate(obj, ax)
            arguments (Input)
                obj;
                ax;
            end
            switch obj.grid
                case "none"
                    grid(ax, "off");
                case "all"
                    grid(ax, "off");
                    grid(ax, "minor");
                    grid(ax, "on");
                case "minor"
                    grid(ax, "off");
                    grid(ax, "minor");
                otherwise
                    grid(ax, "off");
                    grid(ax, "on");
            end
        end

        function legendupdate(obj, ax)
            arguments (Input)
                obj;
                ax;
            end
            if obj.show_legend
                legend(ax, "show", "Location", "best", "AutoUpdate","on");
            else
                legend(ax, "hide");
            end
        end

        function plotfcn = plotcustom(obj)
            switch obj.plottype
                case "plot"
                    plotfcn = @plot;
                case "stairs"
                    plotfcn = @stairs;
                case "scatter"
                    plotfcn = @scatter;
                otherwise
                    error('Unsupported plot type: %s (only supported types are: plot, scatter, stairs)', obj.plottype);
            end
        end


    end
end