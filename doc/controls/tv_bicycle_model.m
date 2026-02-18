% Reference Implementation of the TV Bicycle Model Algorithm
% NOTE: This does not use the same convention as the VCU for motor mapping.

% https://www.youtube.com/watch?v=coHGv3G2JOU

% Inputs to System
% Constants
wheelBase = 1;
propGain = 0;

% Dyanmic Inputs
steeringAng = 0;
velocity = 0;
throttle = 0.7;

% IMU derived measurement
measuredYawRate = 0;

% Compute expected yaw rate from steering
targetYawRate = expectedYawRate(steeringAng, wheelBase, velocity);

% Compute motor commands using torque vectoring
motorCmd = torquevectoring(throttle, measuredYawRate, targetYawRate, propGain);



function motorDistribution = torquevectoring(throttlePosition, measuredYawRate, targetYawRate, propGain)
%TORQUEVECTORINGCONTROLLERSAFE Proportional torque vectoring controller
% Inputs:
%   throttlePosition  - Driver throttle input [0..1]
%   measuredYawRate   - Vehicle measured yaw rate [rad/s]
%   targetYawRate     - Desired yaw rate [rad/s]
%   propGain          - Proportional gain for yaw correction
% Outputs:
%   motorDistribution - 1x4 vector of motor commands [0..1]
%                       Wheel order:
%                       1 - Front Right, 2 - Front Left
%                       3 - Rear Right, 4 - Rear Left

motorDistribution = zeros(1,4);

% Compute yaw correction
yawCorrection = (measuredYawRate - targetYawRate) * propGain;

%% Torque distribution logic
if yawCorrection > 0
    % Oversteer condition
    if targetYawRate > 0
        % Right-hand turn
        motorDistribution(1) = throttlePosition + yawCorrection;
        motorDistribution(2) = throttlePosition;
        motorDistribution(3) = throttlePosition;
        motorDistribution(4) = throttlePosition - yawCorrection;
    else
        % Left-hand turn
        motorDistribution(1) = throttlePosition;
        motorDistribution(2) = throttlePosition + yawCorrection;
        motorDistribution(3) = throttlePosition - yawCorrection;
        motorDistribution(4) = throttlePosition;
    end
else
    % Understeer condition
    if targetYawRate > 0
        % Right-hand turn
        motorDistribution(1) = throttlePosition;
        motorDistribution(2) = throttlePosition + yawCorrection; % yawCorrection < 0
        motorDistribution(3) = throttlePosition - yawCorrection;
        motorDistribution(4) = throttlePosition;
    else
        % Left-hand turn
        motorDistribution(1) = throttlePosition + yawCorrection;
        motorDistribution(2) = throttlePosition;
        motorDistribution(3) = throttlePosition;
        motorDistribution(4) = throttlePosition - yawCorrection;
    end
end

%% C-style clamps to enforce 0-100%
for i = 1:4
    motorDistribution(i) = max(min(motorDistribution(i), 1.0), 0.0);
end

% Optional: preserve average throttle like a virtual differential
avgThrottle = throttlePosition;
currentAvg = mean(motorDistribution);
if currentAvg > 0
    scale = avgThrottle / currentAvg;
    motorDistribution = motorDistribution * scale;
    % Re-clamp after scaling
    for i = 1:4
        motorDistribution(i) = max(min(motorDistribution(i), 1.0), 0.0);
    end
end

end



function r_target = expectedYawRate(steeringAngle, wheelbase, velocity)
% EXPECTEDYAWRATE
% Simple bicycle model: r = vx / L * tan(delta)
% Inputs:
%   steeringAngle - steering angle (rad)
%   wheelbase     - wheelbase (m)
%   velocity      - longitudinal velocity (m/s)
% Output:
%   r_target      - expected yaw rate (rad/s)

r_target = (velocity / wheelbase) * tan(steeringAngle);

end