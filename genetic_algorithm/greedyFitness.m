function [robotTaskSeq,output_origin] = greedyFitness(nRobot,numTaskPerStep,tMax,taskSet,gridMap,bSize)
load('mapGeneration');
showFig = 2;
% plot 설명 : hTaskcall = call 들어오면 노란색
%           : hTask = 지정되면 로봇의 색
tStep = 0;
xSize = length(gridMap(1,:));
ySize = length(gridMap(:,1));
taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numBundledTasks = 0;
timeInQueueTasks = 0;
timeInBundle = 0;
timeInExecution = 0;

map_sheet = 'Map';
task_sheet = 'Task';
station_sheet = 'Station';

% gridMap = readmatrix('Map.xlsx');
taskXY = readmatrix('Map.xlsx','Sheet',task_sheet,'Range','A3:D30');
stationXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','B3:C4');
palletXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','F3:H4');

nStation = length(stationXY(:,1));
nTask = length(taskXY(:,1));

robotX = (1:nRobot)'+1;
robotY = (ones(1,nRobot))';
robotXY = [robotX, robotY];
robotSet = [(1:nRobot)' robotXY zeros([nRobot 3])];
for i =1:nRobot
    robotSet(i,6) = pi/2;
end
% robotSet info : (:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = working state
%                 (:,5) = allocated task
%                 (:,6) = angle
for i=1:nRobot
    robotBundles{i} = [];
    robotBundlesRecord{i} = [];
    robotPaths{i} = [];
    robotTaskSeq{i,2} = [];
end
taskSetTime = [];
for i=1:length(taskSet(:,1))
    taskSetTime = [taskSetTime 30];
end
gridMap_save = gridMap;
%rackXY = taskID2XY(1:itemEndID, itemEndID, xSize, ySize);
taskQueue = [];
taskQueue_all = [];
taskAll = [];
hPath = zeros([1, nRobot]);

if showFig > 0
    hFig = figure; hold on;
    set(hFig, 'Position', [20, 70, xSize*35+30, ySize*35+30]);
    [X,Y]=ind2sub([xSize, ySize], 1:xSize*ySize);
    gcolor = [0.8 0.8 0.8];
    plot(X,Y,'color', gcolor, 'marker', 'square', 'linestyle', 'none', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor',gcolor, 'MarkerFaceColor',gcolor);
    ocolor = [1 1 1];
    for x = 1:xSize
        for y = 1:ySize
            if gridMap(x,y) == 0
                plot(x,y,'color', ocolor, 'marker', 'square', 'linestyle', 'none', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor',ocolor, 'MarkerFaceColor',ocolor);
            end
        end
    end
    axis off
    for i=1:nRobot
        hcolor = 1/nRobot*ones(1,3);
        hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
        hVelo(i,:) = quiver(robotSet(i,2), robotSet(i,3),0,0,1);
    end
    plot(taskXY(:,2), taskXY(:,3), 'bs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    plot(stationXY(:,1), stationXY(:,2), 'ks', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end
newTaskXYs = [];
thisRobotTaskSeq2Idx = ones(1,nRobot);


while tStep < tMax
    %% Task arrival process 2 (load task before) 타스크 콜 들어오는 부분
    for i = 1:length(taskSet(:,1))
        if taskSet(i,1) == tStep
            newTaskXYs = [newTaskXYs; taskSet(i,:)];
        end
    end
    taskIdx = 1:size(newTaskXYs,1);
    if ~isempty(newTaskXYs)
        for i = 1:length(taskXY(:,1))
            for j = 1:length(newTaskXYs(:,1))
                if newTaskXYs(j,3:5) == taskXY(i,1:3)
                    taskXY(i,4) = taskXY(i,4)-1;
                    if showFig == 2
                        hTaskCall(taskXY(i,1)) = plot(taskXY(i,2), taskXY(i,3), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                    end
                end
            end
        end
        taskXY_now = newTaskXYs;
        taskQueue = [taskQueue; taskXY_now];
        
        if length(taskQueue(:,1)) > numTaskPerStep
            taskQueue_all = [taskQueue_all; taskQueue(end-numTaskPerStep+1:end,:)];
        else
            taskQueue_all = [taskQueue_all; taskQueue];
        end
        taskAll = [taskAll; (taskNum+taskIdx)' tStep.*ones(length(taskIdx),1) -100.*ones(length(taskIdx),1) zeros(length(taskIdx),1)];
        taskNum = taskNum+length(taskIdx);
    end
    newTaskXYs = [];
    %taskQueue = [];
    %% 시간 초과된거는 지워주자
    if ~isempty(taskQueue)
        timelimit_over = find((taskSetTime(taskQueue(:,2))' <= taskQueue(:,6)) == 1); % taskSetTime에 지정한 숫자가 지나면 call이 꺼진다
        taskAll(taskQueue(timelimit_over,2),3) = -1; % 꺼진거는 taskAll에 -1로 기록해준다
        taskQueue(timelimit_over,:) = []; %% 시간 초과된거 지워준다
    end
    
        
    
    % (implementation) -1. Greedy allocation: first come first served
    idleRobotIdx = robotSet((robotSet(:,4)==0),1);%Identify available robots
    for i = 1:length(idleRobotIdx)
        thisRobotIdx = idleRobotIdx(i);
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotAngle = robotSet(thisRobotIdx, 6);
        thisRobotBundle = robotBundles{thisRobotIdx};
        thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1};
        thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2};
        timeInBundle = timeInBundle + size(thisRobotBundle,1);
        if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize
            bFill = bSize - size(thisRobotBundle,1);
            if size(taskQueue,1) >= bFill
                thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
                thisTaskIdx1 = taskQueue(1:bFill,2);
                thisTaskIdx2 = taskQueue(1:bFill,3);
                thisRobotTaskSeq1 = [thisRobotTaskSeq1; thisTaskIdx1];
                thisRobotTaskSeq2 = [thisRobotTaskSeq2; thisTaskIdx2];
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
                numBundledTasks = numBundledTasks + bFill;
                taskQueue(1:bFill,:) = []; %removing from the queue
            else
                thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
                thisTaskIdx1 = taskQueue(1:end,2);
                thisTaskIdx2 = taskQueue(1:end,3);
                thisRobotTaskSeq1 = [thisRobotTaskSeq1; thisTaskIdx1];
                thisRobotTaskSeq2 = [thisRobotTaskSeq2; thisTaskIdx2];
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
                numBundledTasks = numBundledTasks + size(taskQueue,1);
                taskQueue = []; %removing from the queue
            end
            robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
            robotTaskSeq{thisRobotIdx,1} = thisRobotTaskSeq1;
            robotTaskSeq{thisRobotIdx,2} = thisRobotTaskSeq2;
        end
        if (size(thisRobotBundle,1) >= bSize)
            numAssignedTasks = numAssignedTasks + bSize;
            robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
            robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
            robotSet(thisRobotIdx,5) = thisTaskIdx2(1);
            % A* path planning
            startXY = thisRobotXY;
            goalXY  = thisRobotBundle(1, 4:5);
            thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
            modWhen = -1;
            robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
            if showFig == 2
                for dr = 1:length(thisRobotBundle(:,1))
                    delete(hTaskCall(thisRobotBundle(dr, 3)));
                    hTask(thisRobotBundle(dr,2)) = plot(thisRobotBundle(dr,4), thisRobotBundle(dr,5), 'rs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','r', 'MarkerFaceColor',1/nRobot*ones(1,3)*thisRobotIdx);
                end
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end
        end
    end

    modWhen = 0;
    workingRobotIdx = robotSet((robotSet(:,4)>0),1);%Identify working robots
    for i = 1:length(workingRobotIdx)
        timeRobotMove = timeRobotMove + 1;
        thisRobotIdx = workingRobotIdx(i);
        thisRobotPath = robotPaths{thisRobotIdx};
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotBundle = robotBundles{thisRobotIdx};
        thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1};
        thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2};
        thisRobotWaitingCell = [];
        timeInExecution = timeInExecution + size(robotBundlesRecord{thisRobotIdx},1);
        
        % 속도,각도 추가한 부분
        thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
        thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
        
        
        % See the next path thisRobotPath(1,:)
        
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
        % Check other robots' second-top cell in robotPaths{:}
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
        %%%%%%%%%%%%%%
        % If conflict is false, just the same with the current version
        if (~conflict) || (conflict && ~makeDetour)
            robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
            thisRobotWaitingCell = thisRobotPath(1,:);
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
            %thisRobotStationXY = stationXY(thisRobotIdx,:);
            if isempty(thisRobotBundle)
                goalXY = thisRobotStationXY;
            else
                goalXY  = thisRobotBundle(1,4:5);
            end
            gridMap(goalXY(1), goalXY(2)) = 0;%goal location should be free
            thisRobotWaitingCell = thisRobotXY;
            thisRobotPath_temp = flipud(astar(startXY, goalXY, ~gridMap)); %%%% astar
            if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                modWhen = 1;
                robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
                if showFig == 2
                    delete(hPath(thisRobotIdx))
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'co', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
                end
                thisRobotPath_temp = thisRobotPath;
                thisRobotWaitingCell = thisRobotPath(1,:);
                thisRobotPath(1,:) = [];
                robotPaths{thisRobotIdx} = thisRobotPath;
            else
                thisRobotPath = thisRobotPath_temp;
                modWhen = 1;
                robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
                if showFig == 2
                    delete(hPath(thisRobotIdx))
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'co', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
                end
                thisRobotPath_temp = thisRobotPath;
                thisRobotPath(1,:) = [];
                robotPaths{thisRobotIdx} = thisRobotPath;
            end
        end
        
        
        % visulizaing
        if showFig > 0
            title({['Step ' num2str(tStep)];''})
            delete(hRobot)
            delete(hVelo(thisRobotIdx,:))
            % plot direction by 'quiver'
            % but can plot that robotSet index 'angle' (:,6)
            if isempty(thisRobotPath)
                hVelo(thisRobotIdx,:) = quiver(robotSet(thisRobotIdx,2), robotSet(thisRobotIdx,3), 0,0,'r-','linewidth',0.001);
            else
                % for adjust AngleVelocity
                hVelo(thisRobotIdx,:) = quiver(robotSet(thisRobotIdx,2), robotSet(thisRobotIdx,3),thisVeloInfo(1),thisVeloInfo(2),'r-','linewidth',5);
            end
            for i=1:nRobot
                hcolor = 1/nRobot*ones(1,3);
                hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
            end
        end
        
        
        if isempty(thisRobotPath)%if the current task is done, now need to see the bundle for the rest of tasks in the bundle
            if robotSet(thisRobotIdx,4)==2
                thisRobotTaskIdx = thisRobotBundle(1,2);
                thisRobotBundle(1,:) = [];%arrived so remove from bundle
                robotBundles{thisRobotIdx} = thisRobotBundle;
                robotTaskSeq{thisRobotIdx,1} = thisRobotTaskSeq1;
                robotTaskSeq{thisRobotIdx,2} = thisRobotTaskSeq2;
            end
            if isempty(thisRobotBundle)%if bundle is empty
                if robotSet(thisRobotIdx,4) == 2
                    robotSet(thisRobotIdx,4) = 1;%mark this robot going to packing station
                    taskXY(robotSet(thisRobotIdx,5),4) = 0;
                    robotSet(thisRobotIdx,5) = 0;
                    %plan path to the station
                    startXY = thisRobotXY;
                    % 일단 station 두개 번갈아 설정하도록 해놓음 나중에 바꿔도됨
                    if mod(tStep,2) == 0
                        thisRobotStationXY = stationXY(1,:);
                        thisRobotStationXY_temp = 1;
                    else
                        thisRobotStationXY = stationXY(2,:);
                        thisRobotStationXY_temp = 2;
                    end
                    goalXY  = thisRobotStationXY; %% 자 이제 이송해야할 지역으로 가자!
                    thisRobotWaitingCell = thisRobotXY;
                    thisRobotPath_temp = pickupPath(startXY,goalXY,robotToStation,2,thisRobotStationXY_temp);
                    if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                        thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                        modWhen = 2;
                        robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    else
                        thisRobotPath = thisRobotPath_temp;
                        modWhen = 2;
                        robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    end
                    if showFig == 2 % 로봇 마크는 로봇이 도착했으니 지워줘야함
                        delete(hTask(thisRobotIdx))
                    end
                elseif robotSet(thisRobotIdx,4) == 1
                    robotSet(thisRobotIdx,4) = 0; %mark this robot idle
                    % At this time, the tasks are completed
                    doneTasksInThisBundle = robotBundlesRecord{thisRobotIdx};
                    for k=1:size(doneTasksInThisBundle,1)
                        if taskAll(doneTasksInThisBundle(k,2), 3) ~= -100
                            disp(' 중 복 된ㄷ ㅏ ! ! !')
                        end
                        taskAll(doneTasksInThisBundle(k,2), 3) = tStep;
                        taskAll(doneTasksInThisBundle(k,2), 4) = thisRobotIdx;
                    end
                end
            else%if there is at least one task remaining in the bundle
                thisRobotTaskIdx = thisRobotBundle(1,2);
                startXY = thisRobotXY;
                goalXY  = thisRobotBundle(1, 4:5);
                thisRobotWaitingCell = thisRobotXY;
                thisRobotPath = pickupPath(startXY,goalXY,robotToTask,1,thisRobotBundle(1, 3));
                thisRobotPath_temp = thisRobotPath;
                %modWhen = 3;
                if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                    thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                else
                    thisRobotPath = thisRobotPath_temp;
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                end
                if showFig == 2 % 로봇 마크는 로봇이 도착했으니 지워줘야함
                    delete(hTask(thisRobotBundle(1,2)))
                end
            end
        end
        gridMap = gridMap_save;
    end
    
    timeRobotWait = timeRobotWait + sum((robotSet(:,4)==0));
    if showFig > 0
        drawnow
    end
    %% Increment counter
    % call 켜진 애들은 1초씩 세준다..
    if ~isempty(taskQueue)
        taskQueue(:,6) = taskQueue(:,6) + 1;
    end
    % 시간의 경과    
    tStep = tStep + 1;
end
numCompletedTasks = sum((taskAll(:,3) >= 0));
tp = taskAll((taskAll(:,3) >= 0),:);
tp2 = taskAll((taskAll(:,3) < 0),:);
completionTimeDoneTasks = sum(tp(:,3)-tp(:,2));
taskAll((taskAll(:,3) < 0),3) = tStep;
completionTimeAllTasks = sum(taskAll(:,3)-taskAll(:,2));
tp2(:,3) = tStep;
timeInQueueTasks = timeInQueueTasks + sum(tp2(:,3)-tp2(:,2));
numAllTasks = taskNum;
output_origin = [numCompletedTasks; timeRobotMove/(numCompletedTasks); timeInBundle/numBundledTasks; completionTimeAllTasks/numAllTasks; completionTimeDoneTasks/numCompletedTasks; timeInQueueTasks/numAllTasks; (timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks;];
end
