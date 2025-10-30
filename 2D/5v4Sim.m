% Base 2D Robotic Soccer Simulation with Ball Physics (Ball Starts Moving)
clear; clc; close all;

% Field dimensions (meters*10)
fieldLength = 90;
fieldWidth = 60;
goalWidth = 26;

% Robot properties
robotRadius = 3;
ballRadius = 0.7;
robotSpeed = 2; % m/step

% Ball properties
ballVel = [randn()*2, randn()*2]; % Initial random velocity (small drift)
ballAccel = [0, 0]; % Acceleration
friction = 0.98; % Velocity decay per step
kickSpeed = 10; % Speed when kicked

%% Initial positions (x, y)
gkPos = [5, fieldWidth/2];
def1Pos = [20, fieldWidth/2];
def2Pos = [20, 40];
str1Pos = [30, 25];
str2Pos = [30, 35];

% Team 2
T2_gkPos = [85, fieldWidth/2];
T2_defPos = [70, fieldWidth/2];
T2_str1Pos = [60, 15];
T2_str2Pos = [60, 45];
ballPos = [50, 30];
%%


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
def1Plot = plot(def1Pos(1), def1Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
def2Plot = plot(def2Pos(1), def2Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
str1Plot = plot(str1Pos(1), str1Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
str2Plot = plot(str2Pos(1), str2Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');

T2_gkPlot = plot(T2_gkPos(1), T2_gkPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'magenta', 'MarkerFaceColor', 'magenta');
T2_defPlot = plot(T2_defPos(1), T2_defPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
T2_str1Plot = plot(T2_str1Pos(1), T2_str1Pos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
T2_str2Plot = plot(T2_str2Pos(1), T2_str2Pos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');

ballPlot = plot(ballPos(1), ballPos(2), 'm*', 'MarkerSize', 10, 'MarkerFaceColor', 'm'); % Purple star

% legend for clarity
legend('T1 GK', 'T1 Def1', 'T1 Def2', 'T1 Str1', 'T1 Str2', ...
       'T2 GK', 'T2 Def', 'T2 Str1', 'T2 Str2', 'Ball');

% Simulation loop
for t = 1:numSteps
    % Update ball physics (now with initial movement)
    ballVel = ballVel * friction; % Apply friction
    ballPos = ballPos + ballVel * dt;
    ballPos = max([0,0], min([fieldLength, fieldWidth], ballPos)); % Keep on field
    
    % Debug: Print ball position
    fprintf('Step %d: Ball at (%.2f, %.2f), Vel (%.2f, %.2f)\n', t, ballPos(1), ballPos(2), ballVel(1), ballVel(2));
    
    %% Check for kicks (closest robot within 2m kicks)
    % Create a list of ALL robot positions
    robotPositions = [gkPos; def1Pos; def2Pos; str1Pos; str2Pos; ... % Team 1
                      T2_gkPos; T2_defPos; T2_str1Pos; T2_str2Pos]; % Team 2
                  
    dists = vecnorm(robotPositions - ballPos, 2, 2);
    [minDist, idx] = min(dists);

    if minDist < (robotRadius + ballRadius) % Use radii for better collision
        
        % --- Kicking Logic ---
        if idx <= 5  % Team 1 (Red) kicked
            
            if idx == 1 % GK: Kick towards center
                target = [fieldLength/2, fieldWidth/2];
            else % Defenders or Strikers: Kick towards opponent's goal
                target = [fieldLength, fieldWidth/2]; % Kick away from own goal
            end
            
        else % Team 2 (Blue) kicked
            
            if idx == 6 % T2 GK: Kick towards center
                target = [fieldLength/2, fieldWidth/2];
            else % T2 Defender or Strikers: Kick towards opponent's goal
                target = [0, fieldWidth/2]; % Kick away from own goal
            end
            
        end
        
        dir = (target - ballPos) / norm(target - ballPos);
        ballVel = kickSpeed * dir;
    end
    
    %% Robot movements (same as before)
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

    %% Team 2 Robot movements (NEW)
    
    % T2 GK
    distGK_T2 = norm(ballPos - T2_gkPos);
    if distGK_T2 < 15 % GK has a larger range
        dirGK_T2 = (ballPos - T2_gkPos) / distGK_T2;
        T2_gkPos = T2_gkPos + robotSpeed * dt * dirGK_T2;
    else
        % Stay in goal box (80 to 90)
        T2_gkPos(1) = max(fieldLength-10, min(fieldLength, T2_gkPos(1) + randn()*0.5));
    end
    % Keep GK near goal line
    T2_gkPos(1) = max(fieldLength-15, T2_gkPos(1)); % Don't go past 15m line

    % T2 Defender
    % This defender will 'sweep', staying between the ball and the goal
    targetDefPos_T2 = [T2_defPos(1), ballPos(2)]; % Follow ball's Y
    dirDef_T2 = (targetDefPos_T2 - T2_defPos) / norm(targetDefPos_T2 - T2_defPos + eps);
    T2_defPos = T2_defPos + robotSpeed * dt * dirDef_T2;
    % Keep defender on their side
    T2_defPos(1) = max(fieldLength/2, min(fieldLength-10, T2_defPos(1))); % Stay on right half
    
    % T2 Strikers
    for i = 1:2
        if i == 1
            pos = T2_str1Pos;
        else
            pos = T2_str2Pos;
        end
        
        dist = norm(ballPos - pos);
        if dist < 20
            % Chase ball if close
            dir = (ballPos - pos) / dist;
            pos = pos + robotSpeed * dt * dir;
        else
            % Move towards opponent goal
            goalDir = ([0, fieldWidth/2] - pos) / norm([0, fieldWidth/2] - pos);
            pos = pos + robotSpeed * dt * goalDir;
        end
        
        if i == 1
            T2_str1Pos = pos;
        else
            T2_str2Pos = pos;
        end
    end
    
    %% Update plots (ball last, fixed Y-data for str1)
    set(gkPlot, 'XData', gkPos(1), 'YData', gkPos(2));
    set(def1Plot, 'XData', def1Pos(1), 'YData', def1Pos(2));
    set(def2Plot, 'XData', def2Pos(1), 'YData', def2Pos(2));
    set(str1Plot, 'XData', str1Pos(1), 'YData', str1Pos(2));
    set(str2Plot, 'XData', str2Pos(1), 'YData', str2Pos(2));
    
    % new lines
    set(T2_gkPlot, 'XData', T2_gkPos(1), 'YData', T2_gkPos(2));
    set(T2_defPlot, 'XData', T2_defPos(1), 'YData', T2_defPos(2));
    set(T2_str1Plot, 'XData', T2_str1Pos(1), 'YData', T2_str1Pos(2));
    set(T2_str2Plot, 'XData', T2_str2Pos(1), 'YData', T2_str2Pos(2));
    
    set(ballPlot, 'XData', ballPos(1), 'YData', ballPos(2));
    
    pause(0.05);
end