classdef DataLogger
    %DATALOGGER Logs the data into a file and handles console prints.
    %   Easier manipulation of the data and beginner-friendly approach to
    %   logging the measurement data.

    properties (SetAccess = private)
        data = [];
        varnames string;
        dhandle;
        Ts double;
        dtcolidx double = -1;
        dtcolidxfound logical = false;
        count double = 0;
    end

    methods
        function obj = DataLogger(dhandle, varnames, Ts, dtcolidx)
            %LOGGER Construct an instance of this class
            %   Containing all the necessary instances for file writing,
            %   variable names and the predefined sampling period for this
            %   measurement run.
            %   dhandle - Data file handle. (dfile_handle)
            %   varnames - Names of the variables in the data row, by
            %   column. (t,tp,y1,...,step)
            %   Ts - The defined sampling period per the run.
            %   dtcolidx - The column index where to find the real delta
            %   time between the measurements. (defaults to -1, finds the
            %   column automatically by searching for column named "dt")
            arguments (Input)
                dhandle;
                varnames;
                Ts;
                dtcolidx = -1;
            end

            obj.dhandle = dhandle;
            obj.varnames = varnames;
            obj.Ts = Ts;
            obj.data = zeros(0, length(varnames)); % Initialize data array

            if dtcolidx >= 1
                obj.dtcolidx = dtcolidx;
                obj.dtcolidxfound = true;
            else
                obj.dtcolidx = obj.finddtcolidx(); % Automatically find dt column index
                obj.dtcolidxfound = ~isempty(obj.dtcolidx);
            end
        end

        function obj = clear(obj)
            obj.data = [];
        end

        function obj = record(obj, data, verbose)
            %RECORD Appends the measurement data into the dataset.
            %   data - The measurement data per one loop.
            %   verbose - Should the recorded data be printed into the
            %   console.
            arguments (Input)
                obj;
                data;
                verbose = false;
            end
            arguments (Output)
                obj
            end
            obj.data = [obj.data; data];
            obj.count = obj.count + 1;
            dt = 0;
            if obj.dtcolidxfound
                dt = data(obj.dtcolidx);
            end
            writenum2file(obj.dhandle, data, verbose, obj.Ts, dt);
        end

        function [data] = get(obj, varname, range)
            %GET Method used for getting the column by the provided name.
            arguments (Input)
                obj;
                varname;
                range = 0;
            end

            arguments (Output)
                data;
            end

            idx = obj.finddtcolidx(varname);
            if isempty(idx)
                throw(MException("DataLogger:get_colidx_failed", "No column with the provided names exists. Column names: " + join(obj.varnames, ",") + " | Provided varname: " + varname));
            else
                if range < 1
                    data = obj.data(:, idx);
                else
                    data = obj.data(range, idx);
                end
            end
        end

        function tabdata = totable(obj)
            %TOTABLE Function returns the recorded data in a table format
            %with the provided column names. (varnames)
            arguments (Input)
                obj;
            end
            arguments (Output)
                tabdata;
            end;

            tabdata = array2table(obj.data, 'VariableNames', cellstr(obj.varnames));
        end
    end

    methods (Access = private)
        function i = finddtcolidx(obj, dtmatch)
            arguments (Input)
                obj;
                dtmatch = "dt";
            end
            arguments (Output)
                i;
            end
            i = find(obj.varnames.matches(dtmatch, "IgnoreCase", true) == 1, 1);
        end
    end
end