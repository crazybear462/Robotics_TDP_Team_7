% Base 2D Robotic Soccer Simulation with Ball Physics (Ball Starts Moving)
clear; clc; close all;

% Field dimensions (meters*10)
fieldLength = 90;
fieldWidth = 60;
goalWidth = 26;

% Robot properties
robotRadius = 3;
ballRadius = 0.7;
robotSpeed = 2; % decimeter/step

% Ball properties
ballVel = [randn()*2, randn()*2]; % Initial random velocity (small drift)
ballAccel = [0, 0]; % Acceleration
friction = 0.98; % Velocity decay per step
kickSpeed = 2; % Speed when kicked (previous 10)
dribble = 0.5;

%% Initial positions (x, y)
gkPos = [5, fieldWidth/2];
def1Pos = [20, fieldWidth/2];
str1Pos = [30, 25];
str2Pos = [30, 35];

% Team 2
T2_gkPos = [85, fieldWidth/2];
T2_defPos = [70, fieldWidth/2];
T2_str1Pos = [50, 30];
T2_str2Pos = [60, 45];
ballPos = [45, 30];
%%


% Simulation parameters
numSteps = 200;
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
str1Plot = plot(str1Pos(1), str1Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
str2Plot = plot(str2Pos(1), str2Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');

T2_gkPlot = plot(T2_gkPos(1), T2_gkPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'magenta', 'MarkerFaceColor', 'magenta');
T2_defPlot = plot(T2_defPos(1), T2_defPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
T2_str1Plot = plot(T2_str1Pos(1), T2_str1Pos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
T2_str2Plot = plot(T2_str2Pos(1), T2_str2Pos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');

ballPlot = plot(ballPos(1), ballPos(2), 'm*', 'MarkerSize', 10, 'MarkerFaceColor', 'm'); % Purple star

% legend for clarity
legend('T1 GK', 'T1 Def1', 'T1 Str1', 'T1 Str2', ...
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
    % Create two lists of ALL robot positions
    
    team1_Pos = [gkPos; def1Pos; str1Pos; str2Pos];
    team2_Pos = [T2_gkPos; T2_defPos; T2_str1Pos; T2_str2Pos]; %Team 2 Positions

    T1_dist = vecnorm(team1_Pos - ballPos, 2, 2);
    [minT1_dist, minT1_idx] = min(T1_dist);

    T2_dist = vecnorm(team2_Pos - ballPos, 2, 2);
    [minT2_dist, minT2_idx] = min(T2_dist);

    if minT1_dist < (robotRadius + ballRadius) || minT2_dist < (robotRadius + ballRadius)% Use radii for better collision
        
        % --- Kicking Logic ---
        if minT1_dist < minT2_dist %idx <= 4  % Team 1 (Red) kicked            
            if minT1_idx == 1 % GK: Kick towards center
                target = [fieldLength/2, fieldWidth/2];
            else % Defenders or Strikers: Kick towards opponent's goal
                target = [fieldLength, fieldWidth/2]; % Kick away from own goal
            end
            
        else % Team 2 (Blue) kicked
            if minT2_idx == 1 % T2 GK: Kick towards center
                target = [fieldLength/2, fieldWidth/2];
            else % T2 Defender or Strikers: Kick towards opponent's goal
                target = [0, fieldWidth/2]; % Kick away from own goal
            end
            
        end       
        dir = (target - ballPos) / norm(target - ballPos);
        ballVel = kickSpeed * dir;
    end
    
    %% Team 1 Robot movements (same as before)
    % GK
    gkPos = move_T1_Goalkeeper(ballPos, gkPos, robotSpeed, dt);
    
    % Defender
    def1Pos = move_T1_Defender(ballPos, def1Pos, robotSpeed, dt);
    
    % Strikers
    str1Pos = move_T1_Striker(ballPos, str1Pos, robotSpeed, dt, fieldLength, fieldWidth);
    str2Pos = move_T1_Striker(ballPos, str2Pos, robotSpeed, dt, fieldLength, fieldWidth);
            
    %% Team 2 Robot movements (NEW)
    
    % T2 GK
    T2_gkPos = move_T2_gk(ballPos, T2_gkPos, robotSpeed, dt, fieldLength);

    % T2 Defender
    % This defender will 'sweep', staying between the ball and the goal
    T2_defPos = move_T2_def(ballPos, T2_defPos, robotSpeed, dt, fieldLength);

    % T2 Strikers
    T2_str1Pos = move_T2_str(ballPos, T2_str1Pos, robotSpeed, dt, fieldWidth);
    T2_str2Pos = move_T2_str(ballPos, T2_str2Pos, robotSpeed, dt, fieldWidth);
        
    %% Update plots (ball last, fixed Y-data for str1)
    set(gkPlot, 'XData', gkPos(1), 'YData', gkPos(2));
    set(def1Plot, 'XData', def1Pos(1), 'YData', def1Pos(2));
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

%% Functions
%GK Team 1 AI
function new_gkPos = move_T1_Goalkeeper(ballPos, current_gkPos, robotSpeed, dt)
    distGK = norm(ballPos - current_gkPos);
    if distGK < 10
        dirGK = (ballPos - current_gkPos)/distGK;
        new_gkPos = current_gkPos + robotSpeed * dt * dirGK;
    else
        new_gkPos = current_gkPos; % Start with the old (x,y)
        new_gkPos(1) = max(0, min(10, current_gkPos(1) + randn()*0.5));
    end
end

%Def Team 1 AI
function new_defPos = move_T1_Defender(ballPos, current_defPos, robotSpeed, dt)
    dist = norm(ballPos - current_defPos);
    if dist < 15
        dirDf = (ballPos - current_defPos)/dist;
        new_defPos = current_defPos + robotSpeed * dt * dirDf;
    else
        
        new_defPos = current_defPos + [randn()*0.5, randn()*0.5];
    end
end

%ST Team 1 AI
function new_strPos = move_T1_Striker(ballPos, current_strPos, robotSpeed, dt, fieldLength, fieldWidth)
    dist = norm(ballPos - current_strPos);
    if dist < 20
        dirST = (ballPos - current_strPos)/dist;
        new_strPos = current_strPos + robotSpeed * dt * dirST;
    else
        dirST = ([fieldLength, fieldWidth/2] - current_strPos)/norm([fieldLength, fieldWidth/2] - current_strPos);
        new_strPos = current_strPos + robotSpeed * dt * dirST;
    end
end

%GK Team 2 AI
function new_T2gk = move_T2_gk(ballPos, current_T2gkPos, robotSpeed, dt, fieldLength)
    dist = norm(ballPos - current_T2gkPos);
    if dist < 15
        dir = (ballPos - current_T2gkPos)/dist;
        new_T2gk = current_T2gkPos + robotSpeed * dir * dt;
    else
        new_T2gk = current_T2gkPos;
        new_T2gk(1) = max(fieldLength-10, min(fieldLength, current_T2gkPos(1) + randn()*0.5));
    end
    new_T2gk(1) = max(fieldLength-15, new_T2gk(1));
end

%DEF Team 2 AI
function new_T2def = move_T2_def(ballPos, current_T2defPos, robotSpeed, dt, fieldLength)
    target = [current_T2defPos(1), ballPos(2)]; % robot to follow the ball y coordinate
    dir = (target - current_T2defPos)/norm(target - current_T2defPos + eps);
    new_T2def = current_T2defPos + robotSpeed * dir * dt; % The robot mirrors the balls position
    new_T2def(1) = max(fieldLength/2, min(fieldLength-10, new_T2def(1))); % Robot can't cross halfway line 
end

%ST Team 2 AI
function new_T2st = move_T2_str(ballPos, current_T2strPos, robotSpeed, dt, fieldWidth)
    dist = norm(ballPos - current_T2strPos);
    if dist < 20
        dir = (ballPos - current_T2strPos)/dist;
        new_T2st = current_T2strPos + dir * robotSpeed * dt;
    else
        goaldir = ([20, fieldWidth/2] - current_T2strPos) / norm([20, fieldWidth/2] - current_T2strPos);
        new_T2st = current_T2strPos + robotSpeed * dt * goaldir;
    end
end
