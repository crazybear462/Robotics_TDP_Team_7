% Enhanced Mixed Simulation with Scoring, Collisions, and Events
clear; clc; close all;

% Field and properties (same as before)
fieldLength = 100; fieldWidth = 60; goalWidth = 10;
robotRadius = 1; ballRadius = 0.5; robotSpeed = 2;
ballVel = [randn()*2, randn()*2]; friction = 0.98; kickSpeed = 10;
gkPos = [5, fieldWidth/2]; def1Pos = [20, 20]; def2Pos = [20, 40]; str1Pos = [80, 25]; str2Pos = [80, 35]; ballPos = [50, 30];
numSteps = 100; dt = 0.1;
possession = 'striker';

% New variables for enhancements
scoreStriker = 0; scoreDefender = 0;
possessionIdx = 0; % Index of robot with possession

figure; axis([0 fieldLength 0 fieldWidth]); hold on; title('Robotic Soccer: Enhanced Mixed Simulation - Score: Strikers 0, Defenders 0'); xlabel('X (m)'); ylabel('Y (m)'); grid on;
rectangle('Position', [0, fieldWidth/2 - goalWidth/2, 2, goalWidth], 'FaceColor', 'red');
rectangle('Position', [fieldLength-2, fieldWidth/2 - goalWidth/2, 2, goalWidth], 'FaceColor', 'blue');
gkPlot = plot(gkPos(1), gkPos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
def1Plot = plot(def1Pos(1), def1Pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
def2Plot = plot(def2Pos(1), def2Pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
str1Plot = plot(str1Pos(1), str1Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
str2Plot = plot(str2Pos(1), str2Pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
ballPlot = plot(ballPos(1), ballPos(2), 'm*', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
legend('GK', 'Def1', 'Def2', 'Str1', 'Str2', 'Ball');

for t = 1:numSteps
    % Check for goals
    if ballPos(1) <= 2 && ballPos(2) >= fieldWidth/2 - goalWidth/2 && ballPos(2) <= fieldWidth/2 + goalWidth/2
        scoreStriker = scoreStriker + 1;
        ballPos = [50, 30]; ballVel = [randn()*2, randn()*2]; % Reset ball
        title(sprintf('Robotic Soccer: Enhanced Mixed Simulation - Score: Strikers %d, Defenders %d', scoreStriker, scoreDefender));
    elseif ballPos(1) >= fieldLength - 2 && ballPos(2) >= fieldWidth/2 - goalWidth/2 && ballPos(2) <= fieldWidth/2 + goalWidth/2
        scoreDefender = scoreDefender + 1;
        ballPos = [50, 30]; ballVel = [randn()*2, randn()*2]; % Reset ball
        title(sprintf('Robotic Soccer: Enhanced Mixed Simulation - Score: Strikers %d, Defenders %d', scoreStriker, scoreDefender));
    end
    
    % End if a team scores 3
    if scoreStriker >= 3 || scoreDefender >= 3
        break;
    end
    
    % Ball physics
    ballVel = ballVel * friction;
    ballPos = ballPos + ballVel * dt;
    ballPos = max([0,0], min([fieldLength, fieldWidth], ballPos));
    
    fprintf('Step %d: Ball at (%.2f, %.2f), Vel (%.2f, %.2f) - Score: S%d D%d\n', t, ballPos(1), ballPos(2), ballVel(1), ballVel(2), scoreStriker, scoreDefender);
    
    % Determine possession
    robotPositions = [gkPos; def1Pos; def2Pos; str1Pos; str2Pos];
    dists = vecnorm(robotPositions - ballPos, 2, 2);
    [minDist, idx] = min(dists);
    possessionIdx = idx;
    if idx <= 3
        possession = 'defender';
    else
        possession = 'striker';
    end
    
    % Kicks
    if strcmp(possession, 'striker')
        target = [fieldLength, fieldWidth/2];
    else
        target = [0, fieldWidth/2];
    end
    if minDist < 2
        dir = (target - ballPos) / norm(target - ballPos);
        ballVel = kickSpeed * dir;
    end
    
    % Random events (every 20 steps)
    if mod(t, 20) == 0
        eventRobot = randi(5); % Random robot
        if rand() > 0.5
            % Boost speed
            if eventRobot == 1
                gkPos = gkPos + [randn()*2, randn()*2];
            elseif eventRobot <= 3
                if eventRobot == 2
                    def1Pos = def1Pos + [randn()*2, randn()*2];
                else
                    def2Pos = def2Pos + [randn()*2, randn()*2];
                end
            else
                if eventRobot == 4
                    str1Pos = str1Pos + [randn()*2, randn()*2];
                else
                    str2Pos = str2Pos + [randn()*2, randn()*2];
                end
            end
        else
            % Slow down (penalty)
            robotSpeed = robotSpeed * 0.5;
        end
    end
    
    % Robot movements (with collision avoidance)
    allPos = [gkPos; def1Pos; def2Pos; str1Pos; str2Pos];
    for r = 1:5
        pos = allPos(r, :);
        for c = 1:5
            if c ~= r
                distToOther = norm(pos - allPos(c, :));
                if distToOther < 2 * robotRadius
                    repelDir = (pos - allPos(c, :)) / distToOther;
                    pos = pos + 0.5 * repelDir; % Repel
                end
            end
        end
        allPos(r, :) = pos;
    end
    gkPos = allPos(1, :); def1Pos = allPos(2, :); def2Pos = allPos(3, :); str1Pos = allPos(4, :); str2Pos = allPos(5, :);
    
    % Apply possession-based movements (similar to before, but with collisions)
    if strcmp(possession, 'striker')
        % Strikers attack, defenders defend
        distGK = norm(ballPos - gkPos);
        if distGK < 10
            dirGK = (ballPos - gkPos) / distGK;
            gkPos = gkPos + robotSpeed * dt * dirGK;
        else
            gkPos(1) = max(0, min(10, gkPos(1) + randn()*0.5));
        end
        
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
    else
        % Defenders attack, strikers defend
        gkPos(1) = max(0, min(10, gkPos(1) + randn()*0.2));
        
        for i = 1:2
            if i == 1
                pos = def1Pos;
            else
                pos = def2Pos;
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
                def1Pos = pos;
            else
                def2Pos = pos;
            end
        end
        
        for i = 1:2
            if i == 1
                pos = str1Pos;
            else
                pos = str2Pos;
            end
            dist = norm(ballPos - pos);
            if dist < 15
                dir = (ballPos - pos) / dist;
                pos = pos + robotSpeed * dt * dir;
            else
                pos = pos + [randn()*0.5, randn()*0.5];
            end
            if i == 1
                str1Pos = pos;
            else
                str2Pos = pos;
            end
        end
    end
    
    % Highlight possession (temporary color change)
    set(gkPlot, 'MarkerFaceColor', 'green');
    set(def1Plot, 'MarkerFaceColor', 'blue');
    set(def2Plot, 'MarkerFaceColor', 'blue');
    set(str1Plot, 'MarkerFaceColor', 'red');
    set(str2Plot, 'MarkerFaceColor', 'red');
    if possessionIdx == 1
        set(gkPlot, 'MarkerFaceColor', 'yellow');
    elseif possessionIdx == 2
        set(def1Plot, 'MarkerFaceColor', 'yellow');
    elseif possessionIdx == 3
        set(def2Plot, 'MarkerFaceColor', 'yellow');
    elseif possessionIdx == 4
        set(str1Plot, 'MarkerFaceColor', 'yellow');
    elseif possessionIdx == 5
        set(str2Plot, 'MarkerFaceColor', 'yellow');
    end
    
    % Update plots
    set(gkPlot, 'XData', gkPos(1), 'YData', gkPos(2));
    set(def1Plot, 'XData', def1Pos(1), 'YData', def1Pos(2));
    set(def2Plot, 'XData', def2Pos(1), 'YData', def2Pos(2));
    set(str1Plot, 'XData', str1Pos(1), 'YData', str1Pos(2));
    set(str2Plot, 'XData', str2Pos(1), 'YData', str2Pos(2));
    set(ballPlot, 'XData', ballPos(1), 'YData', ballPos(2));
    
    pause(0.05);
end

disp('Simulation ended. Final Score:');
fprintf('Strikers: %d, Defenders: %d\n', scoreStriker, scoreDefender);