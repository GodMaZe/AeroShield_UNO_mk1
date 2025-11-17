classdef AeroData
    %AERODATA Object holding the same data as the one being sent by
    %Arduino.
    %   This object should have the same structure as the one defined
    %   within Arduino code, to match the sent bytes via serial comms.
    properties (SetAccess = private)
        nbytes uint32 = 24;
        packetsize uint32 = 0;
        isnormalized logical = false;
    end
    properties
        time double = 0; % long
        output double = 0.0; % float
        control double = 0.0; % float
        potentiometer double = 0.0; % float
        controltime double = 0; % long
        dt double = 0; % long
    end
    
    methods
        function obj = AeroData()
            %AERODATA Construct an instance of this class
            %   Create AeroData object from bytes received via serial
            %   comms.
            % Arduino type lengths:
            % int, float, long = 4 bytes
            % double, long long = 8 bytes
            % short = 2 bytes
            % char, bool = 1 byte
            obj.time = 0;
            obj.output = 0;
            obj.control = 0;
            obj.potentiometer = 0;
            obj.controltime = 0;
            obj.dt = 0;
            obj.packetsize = obj.size();
        end

        function sz = size(obj)
            sz = obj.nbytes;
        end

        function obj = reset(obj)
            obj.time = 0;
            obj.output = 0;
            obj.control = 0;
            obj.potentiometer = 0;
            obj.controltime = 0;
            obj.dt = 0;
        end
        
        function obj = parse(obj,bytes,shouldnormalize)
            %PARSE Parse the incoming serial communication into an object
            %like structure.
            %   This object should represent the same structure within the
            %   Arduino device.
            if nargin <= 1
                throw(MException("AeroData:parse_nargin_failed", "Not enougth input parameters."))
            end

            if numel(bytes) ~= obj.packetsize
                throw(MException("AeroData:parse_nbytes_failed", "The number of bytes to be parsed is not equal to the number of bytes, defined within the AeroData class. Awaited number of bytes: " ...
                    + num2str(obj.packetsize) + " | Received bytes size: " + num2str(numel(bytes))));
            end

            bytes = uint8(bytes);
            
            % Construct the packet's body
            obj.time = double(typecast(bytes(1:4), "uint32"));
            obj.output = double(typecast(bytes(5:8), "single"));
            obj.control = double(typecast(bytes(9:12), "single"));
            obj.potentiometer = double(typecast(bytes(13:16), "single"));
            obj.controltime = double(typecast(bytes(17:20), "uint32"));
            obj.dt = double(typecast(bytes(21:24), "uint32"));

            obj.isnormalized = false;

            if nargin > 2 && ~shouldnormalize
                return;
            end

            obj = obj.normalize();
        end

        function obj = normalize(obj)
            if obj.isnormalized
                return;
            end

            obj.time = obj.time/1e6;
            obj.controltime = obj.controltime/1e6;
            obj.dt = obj.dt/1e6;

            obj.isnormalized = true;
        end

        function s = tostring(obj)
            s = sprintf("%s ", num2str(obj.time), num2str(obj.output), num2str(obj.control), num2str(obj.potentiometer), num2str(obj.controltime), num2str(obj.dt));
        end
    end
end

