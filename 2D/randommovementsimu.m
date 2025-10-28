% Base 2D Robotic Soccer Simulation with Ball Physics (Ball Starts Moving)
clear; clc; close all;

% Field dimensions (meters)
fieldLength = 100;
fieldWidth = 60;
goalWidth = 10;

% Robot properties
robotRadius = 1;
ballRadius = 0.5;
robotSpeed = 2; % m/step

% Ball properties
ballVel = [randn()*2, randn()*2]; % Initial random velocity (small drift)
ballAccel = [0, 0]; % Acceleration
friction = 0.98; % Velocity decay per step
kickSpeed = 10; % Speed when kicked

% Initial positions (x, y)
gkPos = [5, fieldWidth/2];
def1Pos = [20, 20];
def2Pos = [20, 40];
str1Pos = [80, 25];
str2Pos = [80, 35];
ballPos = [50, 30];

% Simulation parameters
numSteps = 100;
dt = 0.1;

% Figure setup
figure;
axis([0 fieldLength 0 fieldWidth]);
hold on;
title('Robotic Soccer: Base Simulation with Moving Ball');
xlabel('X (m)'); ylabel('Y (m)');
grid on;

% Draw goals
rectangle('Position', [0, fieldWidth/2 - goalWidth/2, 2, goalWidth], 'FaceColor', 'red');
rectangle('Position', [fieldLength-2, fieldWidth/2 - goalWidth/2, 2, goalWidth], 'FaceColor', 'blue');

% Plot initial positions (robots first, ball last for visibility)
gkPlot = plot(gkPos(1), gkPos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
def1Plot = plot(def1Pos(1), def1Pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
def2Plot = plot(def2Pos(1), def2Pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
str1Plot = plot(str1Pos(1), str1Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
str2Plot = plot(str2Pos(1), str2Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
ballPlot = plot(ballPos(1), ballPos(2), 'm*', 'MarkerSize', 10, 'MarkerFaceColor', 'm'); % Purple star

% Add legend for clarity
legend('GK', 'Def1', 'Def2', 'Str1', 'Str2', 'Ball');

% Simulation loop
for t = 1:numSteps
    % Update ball physics (now with initial movement)
    ballVel = ballVel * friction; % Apply friction
    ballPos = ballPos + ballVel * dt;
    ballPos = max([0,0], min([fieldLength, fieldWidth], ballPos)); % Keep on field
    
    % Debug: Print ball position
    fprintf('Step %d: Ball at (%.2f, %.2f), Vel (%.2f, %.2f)\n', t, ballPos(1), ballPos(2), ballVel(1), ballVel(2));
    
    % Check for kicks (closest robot within 2m kicks)
    robotPositions = [gkPos; def1Pos; def2Pos; str1Pos; str2Pos];
    dists = vecnorm(robotPositions - ballPos, 2, 2);
    [minDist, idx] = min(dists);
    if minDist < 2
        % Kick towards target
        if idx == 1 % GK: Kick towards center
            target = [50, fieldWidth/2];
        elseif idx <= 3 % Defenders: Kick towards own goal
            target = [0, fieldWidth/2];
        else % Strikers: Kick towards opponent's goal
            target = [fieldLength, fieldWidth/2];
        end
        dir = (target - ballPos) / norm(target - ballPos);
        ballVel = kickSpeed * dir;
    end
    
    % Robot movements (same as before)
    % GK
    distGK = norm(ballPos - gkPos);
    if distGK < 10
        dirGK = (ballPos - gkPos) / distGK;
        gkPos = gkPos + robotSpeed * dt * dirGK;
    else
        gkPos(1) = max(0, min(10, gkPos(1) + randn()*0.5));
    end
    
    % Defenders
    for i = 1:2
        if i == 1
            pos = def1Pos;
        else
            pos = def2Pos;
        end
        dist = norm(ballPos - pos);
        if dist < 15
            dir = (ballPos - pos) / dist;
            pos = pos + robotSpeed * dt * dir;
        else
            pos = pos + [randn()*0.5, randn()*0.5];
        end
        if i == 1
            def1Pos = pos;
        else
            def2Pos = pos;
        end
    end
    
    % Strikers
    for i = 1:2
        if i == 1
            pos = str1Pos;
        else
            pos = str2Pos;
        end
        dist = norm(ballPos - pos);
        if dist < 20
            dir = (ballPos - pos) / dist;
            pos = pos + robotSpeed * dt * dir;
        else
            goalDir = ([fieldLength, fieldWidth/2] - pos) / norm([fieldLength, fieldWidth/2] - pos);
            pos = pos + robotSpeed * dt * goalDir;
        end
        if i == 1
            str1Pos = pos;
        else
            str2Pos = pos;
        end
    end
    
    % Update plots (ball last, fixed Y-data for str1)
    set(gkPlot, 'XData', gkPos(1), 'YData', gkPos(2));
    set(def1Plot, 'XData', def1Pos(1), 'YData', def1Pos(2));
    set(def2Plot, 'XData', def2Pos(1), 'YData', def2Pos(2));
    set(str1Plot, 'XData', str1Pos(1), 'YData', str1Pos(2));
    set(str2Plot, 'XData', str2Pos(1), 'YData', str2Pos(2));
    set(ballPlot, 'XData', ballPos(1), 'YData', ballPos(2));
    
    pause(0.05);
end