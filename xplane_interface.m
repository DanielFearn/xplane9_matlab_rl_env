% X-Plane net data outputs ticked:
% 3
% 17
% 18
% 20
% recommended data rate: 40 per second

% X-Plane "Net Connections" page:
%   "Advanced" -> "IP for Data Output"
%       127.0.0.1   49003   <ticked>
%
%   "UDP Port"
%       49000
%       49001
%       49002



classdef xplane_interface
    
    properties
        u
        LOCAL_PORT = 49003;
        REMOTE_PORT = 49000;

        pitch
        roll
        heading
        p
        q
        r
        ias
        altitude
        longitude
        latitude
    end

    methods
        function obj = xplane_interface()
            obj.u = udpport("IPV4", LocalPort=obj.LOCAL_PORT, LocalHost="127.0.0.1");
        end

        function setDataref(obj, dataref, value)
            packet = zeros(1,509);
            sendValue  = single(value);
            packet(1:9+length(dataref)) = ['DREF' 0 '****' dataref];  
            packet(6:6+4-1)   = typecast(sendValue,'uint8');
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);
        end

        function setData(obj, index, vals)
            fullVals = ones(1,8)*-999;
            for i=1:length(vals)
                fullVals(i) = vals(i);
            end
            sendVals = single(fullVals);
            packet = ['DATA' 0 typecast(int32(index), 'uint8') typecast(sendVals, 'uint8')];
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);
        end

        function sendCommand(obj, cmd)

            packet = zeros(1,509);
            packet(1:5+length(cmd)) = ['CMND' 0 cmd];  
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);
        end

        function obj = getLatestData(obj)
            obj.u.flush(); % clear all received data
            numBytes = 0;
            while (numBytes == 0) % wait for new data
                numBytes = obj.u.NumBytesAvailable();
            end
            data = obj.u.read(numBytes);

            if (char(data(1:4)) ~= 'DATA') % check its a valid packet
                return
            end
            
            data = uint8(data);
            vals = [];
            for j=0:((numBytes-5)/4 - 1)
                index = j*4+6;
                val = typecast(data(index:index+3), "single");
                vals(j+1) = val;
            end

            % now decode values into properties
            index = 1;
            obj.ias = vals(index+1); % knots indicated
            index = 10;
            obj.p = vals(index+1);
            obj.q = vals(index+2);
            obj.r = vals(index+3);
            index = 19;
            obj.pitch = vals(index+1);
            obj.roll = vals(index+2);
            obj.heading = vals(index+3);
            
            index = 28;
            obj.latitude = vals(index+1);
            obj.longitude = vals(index+2);
            obj.altitude = vals(index+4);


        end
        
        function pauseSim(obj)
            obj.setDataref('sim/time/sim_speed', 0);
        end

        function advanceTimestep(obj, timestep, sim_speed)
            obj.setDataref('sim/time/sim_speed', sim_speed);
            pause(timestep/sim_speed);
            obj.setDataref('sim/time/sim_speed', 0);
        end

        function playSim(obj, speed)
            obj.setDataref('sim/time/sim_speed', speed);
        end
    

        function setApproach(obj, type_index, rnwy_index)
            types = [5001 5002 5003]; % takeoff, approach, far away approach
            type = types(type_index); 
            airport = 'EGLL';
            runway = rnwy_index;

            packet = ['PAPT' 0 airport 0 0 0 0 typecast(int32(type), 'uint8') typecast(int32(runway), 'uint8') typecast(int32(0), 'uint8')]
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);

        end

        function failEquipment(obj, equip_id)
            packet = ['FAIL' 0 convertStringsToChars(string(equip_id))];
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);

        end

        function recoverEquipment(obj, equip_id)
            packet = ['RECO' 0 convertStringsToChars(string(equip_id))];
            write(obj.u, packet, "uint8", "127.0.0.1", obj.REMOTE_PORT);
        end

    end
end
