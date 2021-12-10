function [taskSet,robotTaskSeq,output_origin] = tasksetting2(nRobot,numTaskPerStep,alpha,gridMap,tMax,bSize)

probArrival = 1; % 일은 무조건 오게 1로 설정했으나, 변경 가능

xSize = length(gridMap(:,1));
ySize = length(gridMap(1,:));
%itemEndID = 200; %xSize*ySize > itemEndID + ySize*2

tStep = 0;

taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numBundledTasks = 0;
timeInQueueTasks = 0;
timeInBundle = 0;
timeInExecution = 0;


map_sheet = 'Map';
station_sheet = 'Station';
pallet_sheet = 'Pallet';
warehouse_sheet = 'Warehouse';

gridMap = readmatrix('Map_2.xlsx');
stationXY = readmatrix('Map_2.xlsx','Sheet',station_sheet,'Range','A3:D8');
palletXY = readmatrix('Map_2.xlsx','Sheet',pallet_sheet,'Range','A3:D4');
warehouseXY = readmatrix('Map_2.xlsx','Sheet',warehouse_sheet,'Range','A3:D4');

nTask = length(stationXY(:,1));
nPallet = length(palletXY(:,1));
nWarehouse = length(warehouseXY(:,1));

robotX = (1:nRobot)'+1;
robotY = (ones(1,nRobot))'+1;
robotXY = [robotX, robotY];
robotSet = [(1:nRobot)' robotXY zeros([nRobot 3])];
% robotSet info : (:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = working state
%                 (:,5) = allocated task
%                 (:,6) = angle
for i =1:nRobot
    robotSet(i,6) = pi/2;
end

for i=1:nRobot
    robotBundles{i} = [];
    robotBundlesRecord{i} = [];
    robotPaths{i} = [];
    robotTaskSeq{i,2} = [];
end

taskSetTime = [];
for i=1:nTask
    taskSetTime = [taskSetTime 30];
end

gridMap_save = gridMap;
taskQueue = [];
taskQueue_all = [];
taskAll = [];
collisonFrame = [];
hPath = zeros([1, nRobot]);
totalSchedule = zeros(tMax,nTask);

while tStep < tMax
    %% 로봇이 놀고 있는지 확인합니다.
    idleRobotIdx = robotSet((robotSet(:,4)==0),1); %놀고 있는 로봇이 있으면
    % Pallet로 보내거나 완료된 Station에 가야한다
    
    for i = 1:length(idleRobotIdx)
        thisRobotIdx = idleRobotIdx(i); % 컨트롤 로봇 지정
        thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
        thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle
        thisRobotBundle = robotBundles{thisRobotIdx}; % 로봇의 capacity 상태
        thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1}; % 로봇의 일의 순서1
        thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2}; % 로봇의 일의 순서2
        timeInBundle = timeInBundle + size(thisRobotBundle,1);
        % 임무 할당 Logic
        if ~isempty(taskQueue)
            % 임무가 없는데 로봇 capacity가 있으면 Pallet로 보내야한다.
            if size(thisRobotBundle,1) < bSize
                bFill = bSize - size(thisRobotBundle,1);
                % A* path planning
                startXY = thisRobotXY;
                % pallet 중 더 로봇이 안찾아온 곳을 goal로 정한다.
                pallet_check = find(min(palletXY(:,4))==palletXY(:,4));
                if length(pallet_check(1)) == 1
                    goalXY  = palletXY(pallet_check(1),2:3);
                    palletXY(pallet_check(1),4) = palletXY(pallet_check(1),4) + 1; 
                else
                    goalXY = palletXY(1,2:3);
                    palletXY(1,1) = palletXY(1,1) + 1;
                end
                % 로봇의 Path를 업데이트하고, 상태를 1로 바꾼다.
                thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
                thisRobotStatus = 1;
                robotPaths{thisRobotIdx} = thisRobotPath;
                robotSet(thisRobotIdx,4) = thisRobotStatus;
            % capacity가 꽉차있으면 Warehouse로 보내야한다.    
            elseif size(thisRobotBundle,1) >= bSize
                numAssignedTasks = numAssignedTasks + bSize;
                robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
                robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
                robotSet(thisRobotIdx,5) = thisTaskIdx2(1);
                % A* path planning
                startXY = thisRobotXY;
                % warehouse 중 더 물건이 안 쌓인 곳을 goal로 정한다.
                warehouse_check = find(min(warehouseXY(:,4))==warehouseXY(:,4));
                if length(warehouse_check) == 1
                    goalXY  = warehouseXY(warehouse_check(1),2:3);
                    warehouseXY(warehouse_check(1),4) = warehouseXY(warehouse_check(1),4) + 1; 
                else
                    goalXY = warehouseXY(1,2:3);
                    warehouseXY(1,1) = warehouseXY(1,1) + 1;
                end
                thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
                robotPaths{thisRobotIdx} = thisRobotPath;
            end
        end
        
        if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize
            bFill = bSize - size(thisRobotBundle,1);
            if size(taskQueue,1) >= bFill
                thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
%                 thisTaskIdx1 = taskQueue(1:bFill,2);
%                 thisTaskIdx2 = taskQueue(1:bFill,3);
%                 thisRobotTaskSeq1 = [thisRobotTaskSeq1; thisTaskIdx1];
%                 thisRobotTaskSeq2 = [thisRobotTaskSeq2; thisTaskIdx2];
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
                numBundledTasks = numBundledTasks + bFill;
                taskQueue(1:bFill,:) = []; %removing from the queue
            else
                thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
%                 thisTaskIdx1 = taskQueue(taskQueue,2);
%                 thisTaskIdx2 = taskQueue(taskQueue,3);
%                 thisRobotTaskSeq1 = [thisRobotTaskSeq1; thisTaskIdx1];
%                 thisRobotTaskSeq2 = [thisRobotTaskSeq2; thisTaskIdx2];
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
                numBundledTasks = numBundledTasks + size(taskQueue,1);
                taskQueue = []; %removing from the queue
            end
            robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
            robotTaskSeq{thisRobotIdx,1} = thisRobotTaskSeq1;
            robotTaskSeq{thisRobotIdx,2} = thisRobotTaskSeq2;
        end
        
        % 만약 로봇이 Station에 갈 만큼 Pallet_fill을 모으지 않았고
        % Call하고 있는 Station이 있다면
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
            robotPaths{thisRobotIdx} = thisRobotPath; 
            
       end
    end
            
    
    
    
    %% Task arrival process 1 (fixed task & call)
    if mod(tStep,alpha)==0
        newstationXYs = task_arrival_XY_jk(stationXY, numTaskPerStep, probArrival);
        taskIdx = 1:size(newstationXYs,1);
        if ~isempty(newstationXYs)
            for i = 1:length(stationXY(:,1))
                for j = 1:length(newstationXYs(:,1))
                    if newstationXYs(j,1:3) == stationXY(i,1:3)
                        stationXY(i,4) = -1;
                    end
                end
            end
            stationXY_now = newstationXYs;
            taskQueue = [taskQueue; tStep.*ones(length(taskIdx),1) (taskNum+taskIdx)' stationXY_now];
            if length(taskQueue(:,1)) > numTaskPerStep
                taskQueue_all = [taskQueue_all; taskQueue(end-numTaskPerStep+1:end,:)];
            else
                taskQueue_all = [taskQueue_all; taskQueue];
            end
            taskAll = [taskAll; (taskNum+taskIdx)' tStep.*ones(length(taskIdx),1) -100.*ones(length(taskIdx),1)];
            taskNum = taskNum+length(taskIdx);
        end
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
                thisTaskIdx1 = taskQueue(taskQueue,2);
                thisTaskIdx2 = taskQueue(taskQueue,3);
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
                thisRobotPath_temp = thisRobotPath;
                thisRobotWaitingCell = thisRobotPath(1,:);
                thisRobotPath(1,:) = [];
                robotPaths{thisRobotIdx} = thisRobotPath;
            else
                thisRobotPath = thisRobotPath_temp;
                modWhen = 1;
                robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
                thisRobotPath_temp = thisRobotPath;
                thisRobotPath(1,:) = [];
                robotPaths{thisRobotIdx} = thisRobotPath;
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
                    stationXY(robotSet(thisRobotIdx,5),4) = 0;
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
                    else
                        thisRobotPath = thisRobotPath_temp;
                        modWhen = 2;
                        robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    end
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
                else
                    thisRobotPath = thisRobotPath_temp;
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                end
            end
        end
        gridMap = gridMap_save;
    end
    timeRobotWait = timeRobotWait + sum((robotSet(:,4)==0));
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
taskSet = taskQueue_all;
output_origin = [numCompletedTasks; timeRobotMove/(numCompletedTasks); timeInBundle/numBundledTasks; completionTimeAllTasks/numAllTasks; completionTimeDoneTasks/numCompletedTasks; timeInQueueTasks/numAllTasks; (timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks;];
end
