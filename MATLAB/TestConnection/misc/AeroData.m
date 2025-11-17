classdef AeroData
    %AERODATA Object holding the same data as the one being sent by
    %Arduino.
    %   This object should have the same structure as the one defined
    %   within Arduino code, to match the sent bytes via serial comms.
    properties (SetAccess = private)
        preamblesize uint32 = 4;
        postamblesize uint32 = 2;
        nbytes uint32 = 24;
        packetsize uint32 = 0;
        isnormalized logical = false;
    end
    properties
        preamble struct = struct("start_byte", 0, "packet_id", 0, "overhead_byte", 0, "nbytes_payload", 0);
        postamble struct = struct("crc", 0, "stop_byte", 0);
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
            sz = obj.preamblesize + obj.nbytes + obj.postamblesize;
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
            persistent oldbytes;
            if nargin <= 1
                throw(MException("AeroData:parse_nargin_failed", "Not enougth input parameters."))
            end

            if numel(bytes) ~= obj.packetsize
                throw(MException("AeroData:parse_nbytes_failed", "The number of bytes to be parsed is not equal to the number of bytes, defined within the AeroData class. Awaited number of bytes: " ...
                    + num2str(obj.packetsize) + " | Received bytes size: " + num2str(numel(bytes))));
            end
            bytesorg = bytes;
            bytes = uint8(bytes);
            old_time = obj.time;
            
            % Construct the packet's preamble
            obj.preamble.start_byte = bytes(1);
            obj.preamble.packet_id = bytes(2);
            obj.preamble.overhead_byte = bytes(3);
            obj.preamble.nbytes_payload = bytes(4);

            % Construct the packet's body
            obj.time = double(typecast(bytes(5:8), "uint32"));
            obj.output = double(typecast(bytes(9:12), "single"));
            obj.control = double(typecast(bytes(13:16), "single"));
            obj.potentiometer = double(typecast(bytes(17:20), "single"));
            obj.controltime = double(typecast(bytes(21:24), "uint32"));
            obj.dt = double(typecast(bytes(25:28), "uint32"));

            % Construct the packet's postamble
            obj.postamble.crc = bytes(29);
            obj.postamble.stop_byte = bytes(30);

            obj.isnormalized = false;
            
            if old_time > obj.time
                disp("Bytes org:");
                disp(bytesorg);
                disp("Bytes uint8:");
                disp(bytes);
                disp("Old bytes:");
                disp(oldbytes);
                disp("Old bytes uint8:");
                disp(uint8(oldbytes));
                fprintf("Old time: %f | New time: %f\n", old_time, obj.time);
                % throw(MException("AeroData:timesync", "Error in the monotonically ascending time."));
            end
            
            oldbytes = bytesorg;

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

