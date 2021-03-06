%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Changjoo Nam (cjnam@kist.re.kr)

% This script runs coordination of multiple robots in a warehouse setup.
% Pick up tasks arrive in an online fashion following a Poisson distribution.
% The robots pick the items from shelves and return the item to a station located at the bottom of the warehouse.

% This script is developed with MATLAB R2018b version. May not properly work with other versions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
clear;close all
showFig = 2;
nRobot = 3;
numTaskPerStep = 4;
probArrival = 1; % 일은 무조건 오게 1로 설정했으나, 변경 가능
alpha = 20;
xSize = 22;
ySize = 22;
itemEndID = 200; %xSize*ySize > itemEndID + ySize*2
tMax = 500;
tStep = 0;
bSize = 1;
taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numBundledTasks = 0;
timeCompletedTasks = 0;
timeInQueueTasks = 0;
timeInBundle = 0;
timeInExecution = 0;
timeWaiting = 0;
numCollison = 0;

map_sheet = 'Map';
task_sheet = 'Task';
station_sheet = 'Station';

gridMap = readmatrix('Map.xlsx');
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
end
taskSetTime = [];
for i=1:nTask
    taskSetTime = [taskSetTime 30];
end
gridMap_save = gridMap;
rackXY = taskID2XY(1:itemEndID, itemEndID, xSize, ySize);
taskQueue = [];
taskQueue_all = [];
taskAll = [];
collisonFrame = [];
hPath = zeros([1, nRobot]);
totalSchedule = zeros(tMax,nTask);

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
        hcolor = [0.3 0.3 0.3];
        hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
        hVelo(i,:) = quiver(robotSet(i,2), robotSet(i,3),0,0,1);
    end
    plot(taskXY(:,2), taskXY(:,3), 'bs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    plot(stationXY(:,1), stationXY(:,2), 'ks', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end

while tStep < tMax
    %% Task arrival process 1 (fixed task & call)
    %     for i = 1:nTask
    %         %if mod(tStep,taskSetTime(i))==0 && taskXY(i,4) == 0
    %         if taskSetTime(i)==50 && taskXY(i,4) == 0
    %             TaskCall = taskXY(i,2:3);
    %             taskXY(i,4) = -1;
    %             if ~isempty(TaskCall)
    %                 taskIdx = 1:size(TaskCall,1);
    %                 taskQueue = [taskQueue; tStep.*ones(length(taskIdx),1) (taskNum+taskIdx)' taskXY(i,1:4)];
    %                 taskAll = [taskAll; (taskNum+taskIdx)' tStep.*ones(length(taskIdx),1) -100.*ones(length(taskIdx),1)];
    %                 taskNum = taskNum + length(taskIdx);
    %             end
    %         else
    %             timeWaiting = timeWaiting + 1;
    %             taskSetTime(i) = taskSetTime(i) + 1;
    %         end
    %     end
    if mod(tStep,alpha)==0
        newTaskXYs = task_arrival_XY_jk(taskXY, numTaskPerStep, probArrival);
        taskIdx = 1:size(newTaskXYs,1);
        if ~isempty(newTaskXYs)
            for i = 1:length(taskXY(:,1))
                for j = 1:length(newTaskXYs(:,1))
                    if newTaskXYs(j,1:3) == taskXY(i,1:3)
                        taskXY(i,4) = -1;
                        if showFig == 2
                            hTaskCall(taskXY(i,1)) = plot(taskXY(i,2), taskXY(i,3), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                        end
                    end
                end
            end
            taskXY_now = newTaskXYs;
            taskQueue = [taskQueue; tStep.*ones(length(taskIdx),1) (taskNum+taskIdx)' taskXY_now];
            if length(taskQueue(:,1)) > numTaskPerStep
                taskQueue_all = [taskQueue_all; taskQueue(end-numTaskPerStep+1:end,:)];
            else
                taskQueue_all = [taskQueue_all; taskQueue];
            end
            taskAll = [taskAll; (taskNum+taskIdx)' tStep.*ones(length(taskIdx),1) -100.*ones(length(taskIdx),1)];
            taskNum = taskNum+length(taskIdx);
        end
    end
    
    
    %% Task allocation overview
    % Identify all available robots (m)
    % Choose x tasks from the queue where x is the optimal bundle size. x should be smaller than or equal to p, which is the capacity of the item-transporting tray.
    % Compute Manhattan distances from the robots to the tasks (m by x matrix)
    % Allocate the robots to the tasks
    %   Allocation methods (0-worst, 2-best)
    %   -1. Greedy (first come first served)
    %   0. Run an ILP based on the costs computed considering only the distance
    %   1. Plan paths collaboratively considering the conflicts among the robots included in the allocation
    %   2. Plan paths collaboratively considering the conflicts among the robots included in the allocation and other working robots
    %     Comment: 2 is not possible. 1 is the best one that I can do but 0 is
    %     still fine. But I need to admit that the allocation would not be optimal.
    %     1 still could be suboptimal since it does not consider dynamic
    %     obstacles when computing the allocation.
    % Generate paths (A*)
    
    %     % (implementation) -1. Greedy allocation: first come first served
    %     idleRobotIdx = robotSet((robotSet(:,4)==0),1);%Identify available robots
    %     for i = 1:length(idleRobotIdx)
    %         thisRobotIdx = idleRobotIdx(i);
    %         thisRobotXY = robotSet(thisRobotIdx, 2:3);
    %         thisRobotAngle = robotSet(thisRobotIdx, 6);
    %         thisRobotBundle = robotBundles{thisRobotIdx};
    %         timeInBundle = timeInBundle + size(thisRobotBundle,1);
    %         if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize
    %             bFill = bSize - size(thisRobotBundle,1);
    %             if size(taskQueue,1) >= bFill
    %                 thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
    %                 thisTaskIdx = taskQueue(1:bFill,3);
    %                 timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
    %                 numBundledTasks = numBundledTasks + bFill;
    %                 taskQueue(1:bFill,:) = []; %removing from the queue
    %             else
    %                 thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
    %                 thisTaskIdx = taskQueue(taskQueue,3);
    %                 timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
    %                 numBundledTasks = numBundledTasks + size(taskQueue,1);
    %                 taskQueue = []; %removing from the queue
    %             end
    %             robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
    %         end
    %         if (size(thisRobotBundle,1) >= bSize)
    %             numAssignedTasks = numAssignedTasks + bSize;
    %             robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
    %             robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
    %             robotSet(thisRobotIdx,5) = thisTaskIdx;
    %             % A* path planning
    %             startXY = thisRobotXY;
    %             goalXY  = thisRobotBundle(1, 4:5);
    %             thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
    %             modWhen = -1;
    %             robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
    %             if showFig == 2
    %                 hTask(thisRobotBundle(1,2)) = plot(thisRobotBundle(1,4), thisRobotBundle(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
    %                 hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
    %             end
    %         end
    %     end
    
    % (implementation) -1. Greedy allocation: first come first served
    idleRobotIdx = robotSet((robotSet(:,4)==0),1);%Identify available robots
    for i = 1:length(idleRobotIdx)
        thisRobotIdx = idleRobotIdx(i);
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotAngle = robotSet(thisRobotIdx, 6);
        thisRobotBundle = robotBundles{thisRobotIdx};
        timeInBundle = timeInBundle + size(thisRobotBundle,1);
        if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize
            bFill = bSize - size(thisRobotBundle,1);
            if size(taskQueue,1) >= bFill
                thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
                thisTaskIdx = taskQueue(1:bFill,3);
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
                numBundledTasks = numBundledTasks + bFill;
                taskQueue(1:bFill,:) = []; %removing from the queue
            else
                thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
                thisTaskIdx = taskQueue(taskQueue,3);
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
                numBundledTasks = numBundledTasks + size(taskQueue,1);
                taskQueue = []; %removing from the queue
            end
            robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
        end
        if (size(thisRobotBundle,1) >= bSize)
            numAssignedTasks = numAssignedTasks + bSize;
            robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
            robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
            robotSet(thisRobotIdx,5) = thisTaskIdx(1);
            % A* path planning
            startXY = thisRobotXY;
            goalXY  = thisRobotBundle(1, 4:5);
            thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
            modWhen = -1;
            robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
            if showFig == 2
                for i = 1:length(thisRobotBundle(:,1))
                    delete(hTaskCall(thisRobotBundle(i, 3)));
                end
                hTask(thisRobotBundle(1,2)) = plot(thisRobotBundle(1,4), thisRobotBundle(1,5), 'rs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','r', 'MarkerFaceColor',[0.3 0.3 0.3]*thisRobotIdx);
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end
        end
    end
    
    
    
    
    
    %% Execution + conflict resolution overview
    % Run the robots to reach their assigned racks along the computed paths
    % Once a robot arrives to the first rack, it stays one step at the rack for loading.
    % Then it moves to the next rack.
    % Once all items are loaded, the robot goes to the packing station and stays one step for unloading.
    % If there is no waiting tasks in the queue, the robots goes to the center of the warehouse until new tasks arrive.
    % Locally resolve conflicts while the robots are running: basically, stop and wait to the robots encountered pass. No proactive detour for avoiding dynamic obstacles (robots)
    %  -> This would be the hardest part. If D*-lite solves this problem, that would be great. If not,
    %     Check the cell at the forward direction
    %     If it is occupied, wait in this time step.
    modWhen = 0;
    workingRobotIdx = robotSet((robotSet(:,4)>0),1);%Identify working robots
    for i = 1:length(workingRobotIdx)
        timeRobotMove = timeRobotMove + 1;
        thisRobotIdx = workingRobotIdx(i);
        thisRobotPath = robotPaths{thisRobotIdx};
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotBundle = robotBundles{thisRobotIdx};
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
            thisRobotStationXY = stationXY(thisRobotIdx,:);
            if isempty(thisRobotBundle)
                goalXY = thisRobotStationXY;
            else
                goalXY  = thisRobotBundle(1,4:5);
            end
            gridMap(goalXY(1), goalXY(2)) = 0;%goal location should be free
            thisRobotWaitingCell = thisRobotXY;
            thisRobotPath_temp = flipud(astar(startXY, goalXY, ~gridMap));
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
                hcolor = [0.3 0.3 0.3];
                hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
            end
        end
        
        if isempty(thisRobotPath)%if the current task is done, now need to see the bundle for the rest of tasks in the bundle
            if robotSet(thisRobotIdx,4)==2
                thisRobotTaskIdx = thisRobotBundle(1,2);
                thisRobotBundle(1,:) = [];%arrived so remove from bundle
                robotBundles{thisRobotIdx} = thisRobotBundle;
            end
            if showFig == 2
                delete(hPath(thisRobotIdx))
                if robotSet(thisRobotIdx,4)==2
                    delete(hTask(thisRobotTaskIdx))
                    
                end
            end
            if isempty(thisRobotBundle)%if bundle is empty
                if robotSet(thisRobotIdx,4) == 2
                    robotSet(thisRobotIdx,4) = 1;%mark this robot going to packing station
                    taskXY(robotSet(thisRobotIdx,5),4) = 0;
                    taskSetTime(robotSet(thisRobotIdx,5)) = 0;
                    robotSet(thisRobotIdx,5) = 0;
                    %plan path to the station
                    startXY = thisRobotXY;
                    % 일단 station 두개 번갈아 설정하도록 해놓음 나중에 바꿔도됨
                    if mod(tStep,2) == 0
                        thisRobotStationXY = stationXY(1,:);
                    else
                        thisRobotStationXY = stationXY(2,:);
                    end
                    goalXY  = thisRobotStationXY;
                    thisRobotWaitingCell = thisRobotXY;
                    thisRobotPath_temp = flipud(astar(startXY, goalXY, ~gridMap));
                    if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                        thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                        modWhen = 2;
                        robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                        if showFig == 2
                            hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                        end
                    else
                        thisRobotPath = thisRobotPath_temp;
                        modWhen = 2;
                        robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                        if showFig == 2
                            hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                        end
                    end
                    
                    
                    %                     modWhen = 2;
                    %                     robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    %                     if showFig == 2
                    %                         hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    %                     end
                elseif robotSet(thisRobotIdx,4) == 1
                    robotSet(thisRobotIdx,4) = 0; %mark this robot idle
                    % At this time, the tasks are completed
                    doneTasksInThisBundle = robotBundlesRecord{thisRobotIdx};
                    for k=1:size(doneTasksInThisBundle,1)
                        taskAll(doneTasksInThisBundle(k,2), 3) = tStep;
                    end
                end
            else%if there is at least one task remaining in the bundle
                thisRobotTaskIdx = thisRobotBundle(1,2);
                startXY = thisRobotXY;
                goalXY  = thisRobotBundle(1, 4:5);
                thisRobotWaitingCell = thisRobotXY;
                thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
                thisRobotPath_temp = thisRobotPath;
                %modWhen = 3;
                if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                    thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    if showFig == 2
                        hTask(thisRobotTaskIdx) = plot(thisRobotBundle(1,4), thisRobotBundle(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                else
                    thisRobotPath = thisRobotPath_temp;
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    if showFig == 2
                        hTask(thisRobotTaskIdx) = plot(thisRobotBundle(1,4), thisRobotBundle(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                end
                
                
                
                %                 robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                %                 if showFig == 2
                %                     hTask(thisRobotTaskIdx) = plot(thisRobotBundle(1,4), thisRobotBundle(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                %                     hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                %                 end
            end
        end
        gridMap = gridMap_save;
    end
    timeRobotWait = timeRobotWait + sum((robotSet(:,4)==0));
    if showFig > 0
        drawnow
    end
    %% Increment counter
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

fprintf('%d robots, %d bundle size, %d tasks done\n', nRobot, bSize, numCompletedTasks)
fprintf('Execution time per task: %.4f\n', timeRobotMove/(numCompletedTasks))
fprintf('Bundling time per task: %.4f\n', timeInBundle/numBundledTasks)
fprintf('Timespan per task (all): %.4f\n', completionTimeAllTasks/numAllTasks)
fprintf('Timespan per task (done): %.4f\n', completionTimeDoneTasks/numCompletedTasks)
fprintf('Queue residing time: %.4f\n', timeInQueueTasks/numAllTasks)
(timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks
