% Attack Simulation with Ball Physics (Purple Ball, Moving)
clear; clc; close all;

% Same field, robot, and ball properties as Base
fieldLength = 100; fieldWidth = 60; goalWidth = 10;
robotRadius = 1; ballRadius = 0.5; robotSpeed = 2;
ballVel = [randn()*2, randn()*2]; friction = 0.98; kickSpeed = 10;
gkPos = [5, fieldWidth/2]; def1Pos = [20, 20]; def2Pos = [20, 40]; str1Pos = [80, 25]; str2Pos = [80, 35]; ballPos = [50, 30];
numSteps = 100; dt = 0.1;

figure; axis([0 fieldLength 0 fieldWidth]); hold on; title('Robotic Soccer: Attack Simulation'); xlabel('X (m)'); ylabel('Y (m)'); grid on;
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
    ballVel = ballVel * friction;
    ballPos = ballPos + ballVel * dt;
    ballPos = max([0,0], min([fieldLength, fieldWidth], ballPos));
    
    fprintf('Step %d: Ball at (%.2f, %.2f), Vel (%.2f, %.2f)\n', t, ballPos(1), ballPos(2), ballVel(1), ballVel(2));
    
    robotPositions = [gkPos; def1Pos; def2Pos; str1Pos; str2Pos];
    dists = vecnorm(robotPositions - ballPos, 2, 2);
    [minDist, idx] = min(dists);
    if minDist < 2
        if idx == 1
            target = [50, fieldWidth/2];
        elseif idx <= 3
            target = [0, fieldWidth/2];
        else
            target = [fieldLength, fieldWidth/2];
        end
        dir = (target - ballPos) / norm(target - ballPos);
        ballVel = kickSpeed * dir;
    end
    
    % GK stays near goal
    gkPos(1) = max(0, min(10, gkPos(1) + randn()*0.2));
    
    % Defenders intercept
    for i = 1:2
        if i == 1
            pos = def1Pos;
        else
            pos = def2Pos;
        end
        dir = (ballPos - pos) / norm(ballPos - pos);
        pos = pos + robotSpeed * dt * dir;
        if i == 1
            def1Pos = pos;
        else
            def2Pos = pos;
        end
    end
    
    % Strikers towards goal
    goalPos = [fieldLength, fieldWidth/2];
    for i = 1:2
        if i == 1
            pos = str1Pos;
        else
            pos = str2Pos;
        end
        distToBall = norm(ballPos - pos);
        if distToBall < 5
            dir = (goalPos - pos) / norm(goalPos - pos);
            pos = pos + robotSpeed * dt * dir;
        else
            dir = (ballPos - pos) / distToBall;
            pos = pos + robotSpeed * dt * dir;
        end
        if i == 1
            str1Pos = pos;
        else
            str2Pos = pos;
        end
    end
    
    set(gkPlot, 'XData', gkPos(1), 'YData', gkPos(2));
    set(def1Plot, 'XData', def1Pos(1), 'YData', def1Pos(2));
    set(def2Plot, 'XData', def2Pos(1), 'YData', def2Pos(2));
    set(str1Plot, 'XData', str1Pos(1), 'YData', str1Pos(2));
    set(str2Plot, 'XData', str2Pos(1), 'YData', str2Pos(2));
    set(ballPlot, 'XData', ballPos(1), 'YData', ballPos(2));
    
    pause(0.05);
end