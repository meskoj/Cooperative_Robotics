function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

    switch mission.phase
        case 1
            %% Computation of vehicle distance from seafloor on Z-axis
            uvms.xdot.altitudeControl = 0.2 * (2 - uvms.altitude);
            uvms.xdot.altitudeControl = Saturate(uvms.xdot.altitudeControl, 0.8);
    
            %% Computation of horizontal attitude
            ang = ReducedVersorLemma([0;0;1], uvms.vTw(1:3, 3));
            uvms.horizontalMisalignmentVector = ang(1:2);
            if abs(uvms.horizontalMisalignmentVector(1)) < deg2rad(1)
                mis_x = 0;
            else
                mis_x = sign(uvms.horizontalMisalignmentVector(1)) * (abs(uvms.horizontalMisalignmentVector(1)) - deg2rad(1));
            end
            if abs(uvms.horizontalMisalignmentVector(2)) < deg2rad(1)
                mis_y = 0;
            else
                mis_y = sign(uvms.horizontalMisalignmentVector(2)) * (abs(uvms.horizontalMisalignmentVector(2)) - deg2rad(1));
            end
            uvms.xdot.horizontalAttitude = 0.7 * [mis_x; mis_y];
            uvms.xdot.horizontalAttitude = Saturate(uvms.xdot.horizontalAttitude, 1);

            %% Computation of direction to goal
            versorToGoal = uvms.vTnodule(1:3,4) / norm(uvms.vTnodule(1:3,4));
            rotVec = ReducedVersorLemma([1;0;0], versorToGoal);
            uvms.theta_z = rotVec(3);
            uvms.xdot.headingControl = 0.7 * uvms.theta_z;
            uvms.xdot.headingControl = Saturate(uvms.xdot.headingControl, 0.8);
    
            %% Computation of position error
            [~, lin] = CartError(uvms.wTbodyGoal , uvms.wTv);
            uvms.xdot.vehiclePosition = 0.5 * lin;
            uvms.xdot.vehiclePosition = Saturate(uvms.xdot.vehiclePosition, 0.5);
            
        case 2
            %% Computation of vehicle distance from seafloor on Z-axis
            uvms.xdot.altitudeControl = 0.8 * -uvms.altitude;
            uvms.xdot.altitudeControl = Saturate(uvms.xdot.altitudeControl, 1);
    
            %% Computation of position error
            [~, lin] = CartError(uvms.wTbodyGoal , uvms.wTv);
            uvms.xdot.vehiclePosition = 0.5 * [lin(1:2);0];
            uvms.xdot.vehiclePosition = Saturate(uvms.xdot.vehiclePosition, 0.5);

            %% Computation of direction to goal
            versorToGoal = uvms.vTnodule(1:3,4) / norm(uvms.vTnodule(1:3,4));
            rotVec = ReducedVersorLemma([1;0;0], versorToGoal);
            uvms.theta_z = rotVec(3);
            uvms.xdot.headingControl = 0.7 * uvms.theta_z;
            uvms.xdot.headingControl = Saturate(uvms.xdot.headingControl, 0.8);
    
            %% Computation of horizontal attitude
            ang = ReducedVersorLemma([0;0;1], uvms.vTw(1:3, 3));
            uvms.horizontalMisalignmentVector = ang(1:2);
            if abs(uvms.horizontalMisalignmentVector(1)) < deg2rad(1)
                mis_x = 0;
            else
                mis_x = sign(uvms.horizontalMisalignmentVector(1)) * (abs(uvms.horizontalMisalignmentVector(1)) - deg2rad(1));
            end
            if abs(uvms.horizontalMisalignmentVector(2)) < deg2rad(1)
                mis_y = 0;
            else
                mis_y = sign(uvms.horizontalMisalignmentVector(2)) * (abs(uvms.horizontalMisalignmentVector(2)) - deg2rad(1));
            end
            uvms.xdot.horizontalAttitude = 0.7 * [mis_x; mis_y];
            uvms.xdot.horizontalAttitude = Saturate(uvms.xdot.horizontalAttitude, 1);
    
        case 3    
            %% Computation of arm control velocity
            [ang, lin] = CartError(uvms.wTg, uvms.wTt);
            ang = uvms.vTw(1:3,1:3) * ang;
            lin = uvms.vTw(1:3,1:3) * lin;
            uvms.xdot.armControl = 0.7 * [ang;lin];
    
            %% No movement
            uvms.xdot.noMovement = zeros(6,1);

    end
end
