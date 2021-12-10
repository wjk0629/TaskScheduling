%% Initialization
clc;clear;close all
iteration = 10;
output_set = [];
output_real = [];
showFig = 0; % 0���� �ϸ� test�� 2�� �ϸ� plot���� �Ѵ�. 
nRobot = 3;
numTaskPerStep = 4;
probArrival = 1; % ���� ������ ���� 1�� ����������, ���� ����
alpha = 40;
xSize = 22;
ySize = 22;
%itemEndID = 200; %xSize*ySize > itemEndID + ySize*2
tMax = 300;
tStep = 0;
bSize = 1;
timeWaiting = 0;
notgood = false;
notgoodIdx = 1;

map_sheet = 'Map';
task_sheet = 'Task';
station_sheet = 'Station';

gridMap = readmatrix('Map.xlsx');
taskXY = readmatrix('Map.xlsx','Sheet',task_sheet,'Range','A3:D30');
stationXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','B3:C4');
palletXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','F3:H4');

nStation = length(stationXY(:,1));
nTask = length(taskXY(:,1));

% task set�� ���� ���ΰ�??
[taskSet,robotTaskSeq,output_origin] = tasksetting(nRobot,numTaskPerStep,alpha,tMax,gridMap,bSize);
robotTaskSeq_origin = robotTaskSeq;
% �̹� �ִ� task set, greedy �۾� ������ �� ���ΰ�?
% load('taskset1');
% [robotTaskSeq,output_origin] = greedyFitness(nRobot,numTaskPerStep,tMax,taskSet,gridMap);
% robotTaskSeq_origin = robotTaskSeq;

individualTaskCount = zeros(1,nRobot); % �۾� ������ �ٲ��ֱ� ���Ͽ� ���� �ε���

fprintf('*********************   test   start   *********************\n\n');
fprintf('greedy algorithm���� ���� task_origin ������ ������ �����ϴ�.\n');
for i = 1:nRobot
    fprintf('%d�� �κ� : ',i);disp([robotTaskSeq_origin{i,1}]');%fprintf('\n');
end
fprintf('************************************************************\n\n');


for iter = 1:iteration
taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numBundledTasks = 0;
timeCompletedTasks = 0;
timeInQueueTasks = 0;
timeInBundle = 0;
timeInExecution = 0;
numCollison = 0;

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
for i=1:length(taskSet(:,1))
    taskSetTime = [taskSetTime 30];
end
gridMap_save = gridMap;
%rackXY = taskID2XY(1:itemEndID, itemEndID, xSize, ySize);
taskQueue = [];
taskQueue_all = [];
taskAll = [];
collisonFrame = [];
hPath = zeros([1, nRobot]);
totalSchedule = zeros(tMax,nTask);     %%%%% ���߿� ������ֱ�

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
        hcolor =  1/nRobot*ones(1,3);
        hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
        hVelo(i,:) = quiver(robotSet(i,2), robotSet(i,3),0,0,1);
    end
    plot(taskXY(:,2), taskXY(:,3), 'bs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    plot(stationXY(:,1), stationXY(:,2), 'ks', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end

newTaskXYs = [];

robotTaskSeq_before = robotTaskSeq;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% for hill-climb search
thisRobotTaskSeq2Idx = ones(1,nRobot); % ������ �۾����� �� ���� �۾��� �ϱ����� �߰��� �ε���

if (mod(iter,nRobot) + 1) == nRobot
    changeTaskIdx = 1;
else
    changeTaskIdx = mod(iter,nRobot)+2;
end
for j = 1:nRobot
    individual{j} = robotTaskSeq{:,1};
    individual_index(j) = length(individual{j});
end
individual_index2 = min(individual_index);

if iter ~= 1
    for j = 1:individual_index2
        if taskSet(find(robotTaskSeq{(mod(iter,nRobot)+1),1}(j)==taskSet(:,2)),1) == individualTaskCount(mod(iter,nRobot)+1)*alpha
            robotTask_temp1= robotTaskSeq{(mod(iter,nRobot)+1),1}(j);
            robotTaskSeq{(mod(iter,nRobot)+1),1}(j) = robotTaskSeq{changeTaskIdx,1}(j);
            robotTaskSeq{changeTaskIdx,1}(j) = robotTask_temp1;
            robotTask_temp2= robotTaskSeq{(mod(iter,nRobot)+1),2}(j);
            robotTaskSeq{(mod(iter,nRobot)+1),2}(j) = robotTaskSeq{changeTaskIdx,2}(j);
            robotTaskSeq{changeTaskIdx,2}(j) = robotTask_temp2;
            individualTaskCount(mod(iter,nRobot)+1) = individualTaskCount(mod(iter,nRobot)+1) + 1;
            fprintf('%d robot�� %d��° �۾��� %d robot�� %d��° �۾��� �ٲ�����ϴ�.\n', mod(iter,nRobot)+1, j, changeTaskIdx, j)
            fprintf('���� �۾� ������ ������ �����ϴ�.\n');
            for k = 1:nRobot
                fprintf('%d�� �κ� : ',k);disp([robotTaskSeq{k,1}]');%fprintf('\n');
            end
            break;
        else
            %individualTaskCount(mod(iter,3)+1) = individualTaskCount(mod(iter,3)+1) + 1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while tStep < tMax
    %% Task arrival process 2 (load task before)
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
                    taskXY(i,4) = -1;
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
    taskQueue = [];
    %% �ð� �ʰ��ȰŴ� ��������
    if ~isempty(taskQueue)
        timelimit_over = find((taskSetTime(taskQueue(:,2))' <= taskQueue(:,6)) == 1); % taskSetTime�� ������ ���ڰ� ������ call�� ������
        taskAll(taskQueue(timelimit_over,2),3) = -1; % �����Ŵ� taskAll�� -1�� ������ش�
        taskQueue(timelimit_over,:) = []; %% �ð� �ʰ��Ȱ� �����ش�
    end

    
    
    
    %% Task allocation overview
    idleRobotIdx = robotSet((robotSet(:,4)==0),1);%Identify available robots
    for i = 1:length(idleRobotIdx)
        thisRobotIdx = idleRobotIdx(i);
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotAngle = robotSet(thisRobotIdx, 6);
        thisRobotBundle = robotBundles{thisRobotIdx};
        timeInBundle = timeInBundle + size(thisRobotBundle,1);
        
        
        thisRobotStationXYIdx = 0;
        if thisRobotStationXYIdx == 0
            thisRobotStationXY = stationXY(1,:);
            thisRobotStationXYIdx = 1;
        else
            thisRobotStationXY = stationXY(2,:);
            thisRobotStationXYIdx = 0;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %�ϴ� ���� �۾��� �ʿ��ϴ�
        %task seq�� �ҷ��ͼ� �� �� �ð��� �´°� ���� �۾����� �����Ϸ��� �ߴ�.
        %�׷��� task�� call time(alpha), capacity�� ������ ������ �ҷ��ð��̴�
        %������ �ϴ� �̰ɷ� ����.
        thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1};
        thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2};
        thisRobotTaskSeq2Lth = length(thisRobotTaskSeq2);
        % origin�� �۾� ���̸� �������� �Ȱ��� �������ִµ�,
        % else���� �̺��� ȿ���� ���� ����� ��츦 �ٷ��.
        if thisRobotTaskSeq2Lth >= thisRobotTaskSeq2Idx(thisRobotIdx)
            thisRobotTaskTemp1 = find(robotTaskSeq{thisRobotIdx,1}(thisRobotTaskSeq2Idx(thisRobotIdx))==taskSet(:,2));
            thisRobotTaskTemp2 = taskSet(thisRobotTaskTemp1,1); % ���� �Ҵ���� ���� call �Ǵ� �ð�
            % ���� �Ҵ� ���� �� ���� ���� ��ġ���� ���� �ð�
            thisRobotPredictedTime = length(flipud(astar(thisRobotXY, taskSet(thisRobotTaskTemp1(1),4:5), ~gridMap)));
%             for j = 1:length(thisRobotTaskTemp2)
%                 %���� (���� �Ҵ���� ���� ���� �ð�+���� �ð�)�� 
%                 %(���� �ð� - ����ġ �ð�(�����ǰ�� 50))���� ũ�ٸ� ������.
%                 %�� �� �ð��� ���� �ð� + ����ġ �ð�(������ ��� 100)���� ������ ������.
%                 if thisRobotTaskTemp2(j) + thisRobotPredictedTime >= tStep - alpha && thisRobotTaskTemp2(j) + thisRobotPredictedTime < tStep + 2*alpha
                    thisRobotTask = taskSet(thisRobotTaskTemp1(1),:);
                    thisRobotTaskSeq2Idx(thisRobotIdx) = thisRobotTaskSeq2Idx(thisRobotIdx)+1;
%                 else
%                     disp('error check');
%                 end
%             end
            taskQueue = [taskQueue; thisRobotTask];
        else
            thisRobotTaskTemp1 = thisRobotTask(2)+1;
            while 1
                indexindexindex = nRobot;
                for j = 1:nRobot
                    if isempty(find(thisRobotTaskTemp1==robotTaskSeq{j,1}))
                        indexindexindex = indexindexindex -1;
                    end
                end
                if indexindexindex == 0
                    thisRobotTask = taskSet(thisRobotTaskTemp1,:);
                    robotTaskSeq{thisRobotIdx,1} = [robotTaskSeq{thisRobotIdx,1}; taskSet(thisRobotTaskTemp1,2)];
                    robotTaskSeq{thisRobotIdx,2} = [robotTaskSeq{thisRobotIdx,2}; taskSet(thisRobotTaskTemp1,3)];
                    taskQueue = [taskQueue; thisRobotTask];
                    thisRobotTaskSeq2Idx(thisRobotIdx) = thisRobotTaskSeq2Idx(thisRobotIdx)+1;
                    break;
                else
                    thisRobotTaskTemp1 = thisRobotTaskTemp1 + 1;
                        if thisRobotTaskTemp1 > length(taskSet)
                            break;
                        end
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize
            bFill = bSize - size(thisRobotBundle,1);
            if size(taskQueue,1) >= bFill
                %                 for j = 1:bFill
                %                     thisRobotPredictedTime(j) = length(flipud(astar(thisRobotXY, taskQueue(j,4:5), ~gridMap))); %������������ �Ÿ�(=�ð�)
                %                     thisRobotPredictedTime(j) = thisRobotPredictedTime(j) + length(flipud(astar(taskQueue(j,4:5),thisRobotStationXY,~gridMap))); % station������ �Ÿ�(=�ð�)�� ������ ��
                %                 end
                thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
                thisTaskIdx = taskQueue(1:bFill,3);
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
                numBundledTasks = numBundledTasks + bFill;
                taskQueue(1:bFill,:) = []; %removing from the queue
            else
                %                 thisRobotPredictedTime = length(flipud(astar(thisRobotXY, taskQueue(j,4:5), ~gridMap))); %������������ �Ÿ�(=�ð�)
                %                 thisRobotPredictedTime = thisRobotPredictedTime + length(flipud(astar(taskQueue(j,4:5),thisRobotStationXY,~gridMap))); % station������ �Ÿ�(=�ð�)�� ������ ��
                thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
                thisTaskIdx = taskQueue(taskQueue,3);
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
                numBundledTasks = numBundledTasks + size(taskQueue,1);
                taskQueue = []; %removing from the queue
            end
            robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % �������� ���������� ���� �κ��� path�� astar�� ���Ѵ�.
        if (size(thisRobotBundle,1) >= bSize)
            numAssignedTasks = numAssignedTasks + bSize;
            robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
            robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
            robotSet(thisRobotIdx,5) = thisTaskIdx(1);
            % A* path planning
            startXY = thisRobotXY;
            goalXY  = thisRobotBundle(1, 4:5);
            %goalXY = taskXY(robotTaskSeq{thisRobotIdx}(i),2:3);
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
        % �̶� ����� path���� �浹 ���ɼ��� ����Ͽ� ����������Ѵ�. (stop & go ��)
        
    end
    %% global conflict 
    robotPaths = pathConflictCheck(robotPaths);
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
        
        % �ӵ�,���� �߰��� �κ�
        thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
        thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
        
        
        % See the next path thisRobotPath(1,:)
        if isequal(thisRobotPath(1,:),thisRobotXY)
            if size(thisRobotPath,1) > 1
                myNextCell = thisRobotPath(1,:);
            else
                myNextCell = thisRobotPath(1,:);
            end
            myCrtCell = thisRobotPath(1,:);
        elseif isempty(thisRobotPath)
            myNextCell = thisRobotXY;
            myCrtCell = thisRobotXY;
        else
            %myNextCell = thisRobotPath(1,:);
            %%myCrtCell = thisRobotPath(1,:);
            myNextCell = thisRobotPath(1,:);
            myCrtCell = thisRobotXY;
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
                        yourNextCell = yourNextPath(1,:);
                    else
                        yourNextCell = yourNextPath(1,:);
                    end
                    yourCrtCell = yourNextPath(1,:);
                elseif isempty(yourNextPath)
                    yourNextCell = [robotSet(j,2),robotSet(j,3)];
                    yourCrtCell = [robotSet(j,2),robotSet(j,3)];
                else
                    %yourNextCell = yourNextPath(1,:);
                    %yourCrtCell = yourNextPath(1,:);
                    yourNextCell = yourNextPath(1,:);
                    yourCrtCell = [robotSet(j,2),robotSet(j,3)];
                end
            end
            % If there is at least one robot that has the same next cell,
            if (isequal(myNextCell, yourNextCell) || (isequal(myNextCell, yourCrtCell) || isequal(myCrtCell, yourNextCell)) && (j ~= thisRobotIdx))
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
            if mod(tStep,2) == 0
                thisRobotStationXY = stationXY(1,:);
            else
                thisRobotStationXY = stationXY(2,:);
            end
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
                hcolor =  1/nRobot*ones(1,3);
                hRobot(i) = plot(robotSet(i,2), robotSet(i,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*i);
            end
        end
        
        if isempty(thisRobotPath)%if the current task is done, now need to see the bundle for the rest of tasks in the bundle
            if robotSet(thisRobotIdx,4)==2
                thisRobotTaskIdx = thisRobotBundle(1,2);
                %%%%% ���� �����ؼ� ���� 2�� 5��
                if taskSet(find(taskSet(:,2)==thisRobotTaskIdx),1) <= tStep % ���� �κ��� ������ �ð��� ���� ������ ���̶�� ���� ���ö����� ��ٸ���.
                    thisRobotBundle(1,:) = [];%arrived so remove from bundle
                end
                %thisRobotBundle(1,:) = [];%arrived so remove from bundle
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
                    % �ϴ� station �ΰ� ������ �����ϵ��� �س��� ���߿� �ٲ㵵��
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
                elseif robotSet(thisRobotIdx,4) == 1
                    robotSet(thisRobotIdx,4) = 0; %mark this robot idle
                    % At this time, the tasks are completed
                    doneTasksInThisBundle = robotBundlesRecord{thisRobotIdx};
                    for k=1:size(doneTasksInThisBundle,1)
                        if taskAll(doneTasksInThisBundle(k,2), 3) ~= -100
                            disp(' �� �� �Ȥ� �� ! ! !')
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
            end
        end
        gridMap = gridMap_save;
    end
    timeRobotWait = timeRobotWait + sum((robotSet(:,4)==0));
    if showFig > 0
        drawnow
    end
    %% Increment counter
    % call ���� �ֵ��� 1�ʾ� ���ش�..
    if ~isempty(taskQueue)
        taskQueue(:,6) = taskQueue(:,6) + 1;
    end
    % �ð��� ���    
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
fprintf('** %d iteration �� %d �� �����߽��ϴ�. **\n', iteration, iter)
tStep = 0; 
output = [numCompletedTasks; timeRobotMove/(numCompletedTasks); completionTimeDoneTasks/numAllTasks; completionTimeAllTasks/numAllTasks; completionTimeDoneTasks/numCompletedTasks; timeInQueueTasks/numAllTasks; (timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks;];
if iter == 1
    if output(1) < output_origin(1)
        robotTaskSeq = robotTaskSeq_origin;
        notgood = true;
        fprintf('����� �� �������Ƿ� ���� ����\n\n')
        output_set = [output_set output_origin];
    elseif output(7) > output_origin(7)        
        robotTaskSeq = robotTaskSeq_origin;
        notgood = true;
        fprintf('����� �� �������Ƿ� ���� ����\n\n')
        output_set = [output_set output_origin];
    else
        notgood = false;
        notgoodIdx = iter;
        fprintf('����� ���������Ƿ� �ٲ۴�\n\n')
        output_set = [output_set output];
    end
elseif iter > 1
    if output(1) < output_set(1,notgoodIdx)
        robotTaskSeq = robotTaskSeq_before;
        notgood = true;
        %notgoodIdx = notgoodIdx + 1;
        fprintf('����� �� �������Ƿ� ���� ����\n\n')
        output_set = [output_set output_set(:,(iter-1))];
    elseif output(7) > output_set(7,notgoodIdx)
        robotTaskSeq = robotTaskSeq_before;
        notgood = true;
        %notgoodIdx = notgoodIdx + 1;
        fprintf('����� �� �������Ƿ� ���� ����\n\n')
        output_set = [output_set output_set(:,(iter-1))];
    else
        notgood = false;
        notgoodIdx = iter;
        fprintf('����� ���������Ƿ� �ٲ۴�\n\n')
        output_set = [output_set output];
    end
end
output_real = [output_real output];
fprintf('************************************************************\n\n')

% output ���� :
% 1. fprintf('%d robots, %d bundle size, %d tasks done\n', nRobot, bSize, numCompletedTasks)
% 2. fprintf('Execution time per task: %.4f\n', timeRobotMove/(numCompletedTasks))
% 3. fprintf('Bundling time per task: %.4f\n', timeInBundle/numBundledTasks)
% 4. fprintf('Timespan per task (all): %.4f\n', completionTimeAllTasks/numAllTasks)
% 5. fprintf('Timespan per task (done): %.4f\n', completionTimeDoneTasks/numCompletedTasks)
% 6. fprintf('Queue residing time: %.4f\n', timeInQueueTasks/numAllTasks)
% 7. (timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks
end
