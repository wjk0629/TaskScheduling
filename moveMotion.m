function [robotSet,robotPaths] = moveMotion(robotSet,robotPaths,gridMap)
% 출발 시뮬레이션 (step의 경과에 따라 그려줌)
% Conflict가 있으면 경로 수정
workingRobotIdx = robotSet((robotSet(:,4)>0),1);%Identify working robots
for i = 1:length(workingRobotIdx)
    timeRobotMove = timeRobotMove + 1;
    thisRobotIdx = workingRobotIdx(i);
    thisRobotPath = robotPaths{thisRobotIdx};
    thisRobotXY = robotSet(thisRobotIdx, 2:3);
    thisRobotBundle = robotBundles{thisRobotIdx};
    %thisRobotWaitingCell = [];
    timeInExecution = timeInExecution + size(robotBundlesRecord{thisRobotIdx},1);
    % 속도,각도 추가한 부분
    thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
    thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
    
    if isequal(thisRobotPath(1,:),thisRobotXY)
        if size(thisRobotPath,1) > 1
            myNextCell = thisRobotPath(2,:);
        else
            myNextCell = thisRobotPath(1,:);
        end
        myCrtCell = thisRobotPath(1,:);
    elseif isempty(thisRobotPath)
        myNextCell = thisRobotXY;
        myCrtCell = thisRobotXY;
    else
        myNextCell = thisRobotPath(1,:);
        myCrtCell = thisRobotPath(1,:);
    end
    conflict = false;
    makeDetour = false;
    % 내 다음 위치에 다른 로봇이 있는지 확인한다.
    for j=1:nRobot
        if robotSet(j,4)==0
            yourNextCell = [robotSet(j,2),robotSet(j,3)];
            yourCrtCell = [robotSet(j,2),robotSet(j,3)];
        else
            yourNextPath = robotPaths{j};
            if isequal(yourNextPath(1,:),robotSet(j,2:3))
                if size(yourNextPath,1) > 1
                    yourNextCell = yourNextPath(2,:);
                else
                    yourNextCell = yourNextPath(1,:);
                end
                yourCrtCell = yourNextPath(1,:);
            elseif isempty(yourNextPath)
                yourNextCell = [robotSet(j,2),robotSet(j,3)];
                yourCrtCell = [robotSet(j,2),robotSet(j,3)];
            else
                yourNextCell = yourNextPath(1,:);
                yourCrtCell = yourNextPath(1,:);
            end
        end
        % If there is at least one robot that has the same next cell,
        if (isequal(myNextCell, yourNextCell) || isequal(myNextCell, yourCrtCell) || isequal(myCrtCell, yourNextCell)) && (j ~= thisRobotIdx)
            conflict = true;
            if (j > thisRobotIdx)
                makeDetour = true;% If its ID is larger than me, I should not move
            end
        end
    end
    % If conflict is false, just the same with the current version
    if (~conflict) || (conflict && ~makeDetour)
        robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
        thisRobotPath(1,:) = [];
        robotPaths{thisRobotIdx} = thisRobotPath;
    end
    % If conflict is true and deferMove is true,
    if conflict && makeDetour
        for k=1:nRobot
            if robotSet(k,4)==0
                gridMap(robotSet(k,2),robotSet(k,3)) = 1;
            else
                yourNextPath = robotPaths{k};
                if isequal(yourNextPath(1,:),robotSet(k,2:3))
                    if size(yourNextPath,1) > 1
                        yourNextCell = yourNextPath(2,:);
                    else
                        yourNextCell = yourNextPath(1,:);
                    end
                    yourCrtCell = yourNextPath(1,:);
                elseif isempty(yourNextPath)
                    yourNextCell = [robotSet(k,2),robotSet(k,3)];
                    yourCrtCell = [robotSet(k,2),robotSet(k,3)];
                else
                    yourNextCell = yourNextPath(1,:);
                    yourCrtCell = yourNextPath(1,:);
                end
                gridMap(yourNextCell(1),yourNextCell(2)) = 1;
                gridMap(yourCrtCell(1),yourCrtCell(2)) = 1;
            end
        end
        gridMap(thisRobotXY(1), thisRobotXY(2)) = 0;
        % If conflict is true and deferMove is false, I am the robot with the highest priority.
        
        % rerun the A* planner with the current location of mine to the task
        startXY = thisRobotXY;
        if isempty(thisRobotBundle)
            goalXY = warehouseXY;
        else
            goalXY  = thisRobotBundle(1,3:4);
        end
        gridMap(goalXY(1), goalXY(2)) = 0;%goal location should be free
        thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
        modWhen = 1;
        robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
        if showFig == 2
            delete(hPath(thisRobotIdx))
            hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'co', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
        end
        thisRobotPath(1,:) = [];
        robotPaths{thisRobotIdx} = thisRobotPath;
    end
end