classdef Wheel_Force < matlab.System
    % This is the force calculator for landing gear
    %
    % 

    % Public, tunable properties
    properties
        drone_mass = 4.25; % Wheel-mass


    end

    % Pre-computed constants or internal states
    properties (Access = private)
        wheel_grav;

    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.wheel_grav = 9.81 * obj.drone_mass; %
        end

        function impact_Force = stepImpl(obj,drone_vel,vert_dist)

            % obj is the drone respectively
            %drone_vel is the speed at which the drone is moving
            %vert_dist is the distance the landing gear can be compressed
            impact_Force = 1/2*obj.drone_mass*drone_vel^2/vert_dist + obj.wheel_grav;

        end

        function resetImpl(obj)
            obj.wheel_m = 4.25;
            % Initialize / reset internal properties
        end
    end
end