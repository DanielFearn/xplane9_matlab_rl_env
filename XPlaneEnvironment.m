% for state augmentation, see commented out lines and swap them for the non
% commented lines


classdef XPlaneEnvironment < rl.env.MATLABEnvironment

    properties
        plane = xplane_interface();
        airport = "ELGG" % London Heathrow

        State = zeros(10, 1)
        % altitude
        % ias
        % pitch
        % roll
        % heading
        % Q
        % P
        % R angular velocities
        % long
        % lat

        previous_normed_state = zeros(10,1);

        LastAction = zeros(3,1)
        % elevator, aileron, rudder
        elevator = 0;
        aileron = 0;
        rudder = 0;
        
        TimeStep = 0.1;
        Time = 0;

        L = [];

        control_auth = 0.01;
        
        % runway threshold
        target_hdng = 88.69;
        target_lat = 51.48;
        target_long = -0.489;
        target_alt = 0.277;

        last_distance = 0;

        system_ids = [69, 70, 71, 72, 73, 74, 122, 123];

    end

    methods

        function this = XPlaneEnvironment(ActionInfo)
            
            % ObservationInfo(1) = rlNumericSpec([24 1]); augmented
            ObservationInfo(1) = rlNumericSpec([11 1]);
            ObservationInfo(1).Name = 'states';

            els = {
                [-1;-1;-1],[-1;0;-1],[-1;1;-1],...
                [0;-1;-1],[0;0;-1],[0;1;-1],...
                [1;-1;-1],[1;0;-1],[1;1;-1],...
                [-1;-1;0],[-1;0;0],[-1;1;0],...
                [0;-1;0],[0;0;0],[0;1;0],...
                [1;-1;0],[1;0;0],[1;1;0],...
                [-1;-1;1],[-1;0;1],[-1;1;1],...
                [0;-1;1],[0;0;1],[0;1;1],...
                [1;-1;1],[1;0;1],[1;1;1]
            }'; % possible combinations for elevator + aileron + rudder
            

            ActionInfo(1) = rlFiniteSetSpec(els);
            ActionInfo(1).Name = 'surfaces';


            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);


            this.plane.pauseSim();
            this.plane = this.plane.getLatestData();

            this.updateState();
            %this.Log = [this.Log state];

        end

        function [nextobs, reward, isdone, loggedSignals] = step(this, Action)
            loggedSignals = [];

            action = Action(:) .* this.control_auth;
            this.elevator = min(max(this.elevator + action(1), -1), 1);
            this.aileron = min(max(this.aileron + action(2), -1), 1);
            this.rudder = min(max(this.rudder + action(3), -1), 1);

            this.LastAction = action;

            this.plane.setData(8, [this.elevator, this.aileron, this.rudder]);
    
            %this.plane.advanceTimestep(this.TimeStep, 8)
            this.plane.playSim(2);
            pause(this.TimeStep/2);

            prev_long = this.plane.longitude;
            this.plane = this.plane.getLatestData();

            this.updateState();

            this.Time = this.Time + this.TimeStep;
            
            alt = this.plane.altitude;
            roll = this.plane.roll;
            pitch = this.plane.pitch;
            hdng = this.plane.heading;
            long = this.plane.longitude;
            lat = this.plane.latitude;

            %glideslope = (1000) / (-0.57 - this.target_long);

            %glide_alt = glideslope * (long - this.target_long);
            %alt_error = alt - glide_alt;

            

            % each distance is normalised to max ~1
            distance =  sqrt(((alt - this.target_alt)/1000)^2 + (abs(long - this.target_long)/0.08)^2 + (abs(lat - this.target_lat)/0.129 )^2); % longitude vs latitude at latitude
            if (this.last_distance == 0)
                this.last_distance = distance;
            end
            reward = -(distance - this.last_distance) * 100;
            
            this.last_distance = distance;

            %reward = reward + 1/6 * (abs(alt_error) < 100);

            
            isdone = false;
            flag = 0;
            
            % out of bounds
            if (abs(roll) > 30 || abs(pitch) > 30 || alt < 10)
                isdone = true;
                fprintf("out of bounds");
                flag = -1;
            end

            % has crashed and been reset
            if abs(long - prev_long) > 0.01
                isdone = true;
                fprintf("crashed and reset");
                flag = -1;
            end

            % hasnt ended for another reason, and is close to threshold
            if distance < 0.1 && isdone == false
                reward = reward + 500;
                isdone = true;
                fprintf("success");
                flag = 1;
            end

            if isdone
                this.plane.pauseSim();
            end
            
            % nextobs = [this.normalised_state()' flag Action(:)' this.previous_normed_state']'; augmented
            nextobs = [this.normalised_state()' flag]';
    
            % this.previous_normed_state = this.normalised_state(); augmented

        end

        function obs = reset(this)
            this.plane.setApproach(2, 0);
            this.plane.setData(62, 100); % refill fuel
            pause(this.TimeStep);
            
            % initial input
            this.plane.setData(8, rand(1, 3)*2 - 1);
            this.plane.advanceTimestep(this.TimeStep*2, 1);
            this.plane.setData(8, [0, 0, 0]);
            this.elevator = 0;
            this.aileron = 0;
            this.rudder = 0;
            this.last_distance = 0;
            this.plane.advanceTimestep(2, 1);

            this.Time = 0;

            if rand > 0.5
                system_to_fail = this.system_ids(randi(length(this.system_ids), 1));
                this.plane.failEquipment(system_to_fail);
                if rand > 0.1
                    second_system_to_fail = this.system_ids(randi(length(this.system_ids), 1));
                    this.plane.failEquipment(second_system_to_fail);
                end
            end
            
            this.plane = this.plane.getLatestData();

            this.updateState();

            this.previous_normed_state = zeros(10, 1);

            % obs = [this.normalised_state()' 0 zeros(1,3) this.previous_normed_state']'; augmented

            obs = [this.normalised_state()' 0]; 

        end

        function norm_state = normalised_state(this) 
            norm_state = this.State(:) ./ [1000 100 90 90 90 1 1 1 0.08 0.129]';
        end

        function ans = updateState(this)
            this.State = [...
                this.plane.altitude ...
                this.plane.ias ...
                this.plane.pitch ...
                this.plane.roll ...
                this.plane.heading ...
                this.plane.p ...
                this.plane.q ...
                this.plane.r ...
                (this.plane.latitude - this.target_lat)...
                (this.plane.longitude - this.target_long)...
                ];
        end


    end



end