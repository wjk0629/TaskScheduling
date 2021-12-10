%% Initialization
clear;close all;clc;
showFig = 2;
nRobot = 3;
numTaskPerStep = 4;
probArrival = 1; % 일은 무조건 오게 1로 설정했으나, 변경 가능
alpha = 20;
tMax = 500;
tStep = 0;
bSize = 1;
taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numTaskdTasks = 0;
timeCompletedTasks = 0;
timeInQueueTasks = 0;
timeInTask = 0;
timeInExecution = 0;
timeWaiting = 0;
numCollison = 0;


% Map setting
[gridMap,stationXY,palletXY,warehouseXY] = mapLoading('Map_2.xlsx');
% map origin save
gridMap_save = gridMap;
% size
xSize = length(gridMap(:,1));
ySize = length(gridMap(1,:));
% number
nStation = length(stationXY(:,1));
nPallet = length(palletXY(:,1));
nWarehouse = length(warehouseXY(:,1));


% robot information setting
[palletLUtime,warehouseLUtime,stationLUtime, ...
    robotSet,robotTasks,robotTasksRecord,robotPaths,robotDistance, ...
    taskQueue,taskQueue_all] = initialSetting(nRobot,nStation);


% from pallet to station path 기록
[PathTable_PalletToStation,PathTable_PalletToStation_Length] = pathMatrixByAstar(nPallet,nStation,palletXY,stationXY,gridMap);
% from station to warehouse path 기록
[PathTable_StationToWarehouse,PathTable_StationToWarehouse_Length] = pathMatrixByAstar(nStation,nWarehouse,stationXY,warehouseXY,gridMap);
% from warehouse to station path 기록
[PathTable_WarehouseToStation,PathTable_WarehouseToStation_Length] = pathMatrixByAstar(nWarehouse,nStation,warehouseXY,stationXY,gridMap);
% from station to station path 기록
[PathTable_StationToStation,PathTable_StationToStation_Length] = pathMatrixByAstar(nStation,nStation,stationXY,stationXY,gridMap);

%% initial visualizing 
if showFig > 0
    hPath = zeros([1, nRobot]);
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
    plot(palletXY(:,2), palletXY(:,3), 'ms', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','m', 'MarkerFaceColor','m')
    plot(stationXY(:,2), stationXY(:,3), 'bs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    plot(warehouseXY(:,2), warehouseXY(:,3), 'ks', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end


%% loop start
while tStep < tMax
    % 로봇 idx generation
    RobotIdxfree = [];
    RobotIdxpallet = [];
    RobotIdxstation = [];
    RobotIdxwarehouse = [];
    PathLengthCompare_Pallet = [];
    PathLengthCompare_Station = [];
    PathLengthCompare_Warehouse = [];
    PathLengthCompare_Now = [];
    % 사전 작업
    for i = 1:nRobot
        if robotSet(i,4) == 0 % 대기중인 로봇 index
            RobotIdxfree = [RobotIdxfree; i];
        elseif robotSet(i,4) == 1 % pallet로 가고있는 로봇 index
            RobotIdxpallet = [RobotIdxpallet; i];
        elseif robotSet(i,4) == 2 % pallet을 끝내고 station으로 가야있는 로봇 index
            RobotIdxstation = [RobotIdxstation; i];
        elseif robotSet(i,4) == 3 % station을 끝낸 로봇 index
            RobotIdxwarehouse = [RobotIdxwarehouse; i];
        end
        
        % 로봇 to Pallet Path generation
        for j = 1:nPallet
            startXY = robotSet(i,2:3);
            goalXY = palletXY(j,2:3);
            PathNow = flipud(astar(startXY, goalXY, ~gridMap));
            PathTable_RobotToPallet{i,j} = PathNow;
            PathTable_RobotToPallet_Length(i,j) = length(PathNow);
        end
        % 비어 있는 pallet을 조사
        palletFree = palletXY((palletXY(:,4)==0),1);
        % 로봇이 더 방문하지 않은 Pallet은 어디인가?
        pallet_check = find(min(palletXY(:,5))==palletXY(:,5));
        if length(pallet_check) == 1
            palletNowIdx  = palletXY(pallet_check(1),1);
        else
            palletNowIdx  = palletXY(1,1);
        end
        PathLengthCompare_Pallet = [PathLengthCompare_Pallet; PathTable_RobotToPallet_Length(i,palletNowIdx)];
        
        % 로봇 to Station Path generation
        for j = 1:nStation
            startXY = robotSet(i,2:3);
            goalXY = stationXY(j,2:3);
            PathNow = flipud(astar(startXY, goalXY, ~gridMap));
            PathTable_RobotToStation{i,j} = PathNow;
            PathTable_RobotToStation_Length(i,j) = length(PathNow);
        end
        % 작업 완료된 station을 조사
        stationReady = stationXY((stationXY(:,5)==1),1);
        if ~isempty(stationReady)
            if length(stationReady) ==1
                stationNowIdx = stationReady;
            else
                stationNowIdx = stationReady(1);
            end
            PathLengthCompare_Station = [PathLengthCompare_Station; PathTable_RobotToStation_Length(i,stationNowIdx)];
        end
    end
    for i = 1:nStation
        % station에서 작업을 진행 중이라면, remain time을 time step 만큼 줄여준다.
        if stationXY(i,5) > 1
            stationXY(i,5) = stationXY(i,5) - 1;
        end
    end
    
    
    % 출발 시나리오1. to pallet
    %[palletXY,robotTasks,thisRobotIdx,thisVeloInfo,thisRobotPath,thisRobotAngle] = toPalletFunc(RobotIdxfree,palletXY,robotSet,robotTasks,robotPaths,palletLUtime,PathTable_RobotToPallet,PathLengthCompare_Pallet,gridMap,showFig);
    for i = 1:length(RobotIdxfree)
        % 가장 우선인게 완료된작업을 warehouse에 가져다주는것.
        sFree1 = stationXY((stationXY(:,4)==0),1);
        sFree2 = stationXY((stationXY(:,5)==1),1);
        sFree3 = stationXY((stationXY(:,5)==0),1);
        stationReady = stationXY(intersect(sFree1,sFree2),1);
        stationFree = stationXY(intersect(sFree1,sFree3),1);
        % 빈 pallet과 가장 가까우면서 놀고 있는 로봇은 누구인가? ** 그리디 알고리즘이 반영된 부분
        % 비어 있는 pallet을 조사
        palletFree = palletXY((palletXY(:,4)==0),1);
        
        if isempty(stationReady)
            stationNowIdx = 0;
            if isempty(palletFree)
                palletNowIdx = 0;
            elseif length(palletFree) == 1
                palletNowIdx = palletXY(palletFree,1);
                palletXY(palletNowIdx,5) = palletXY(palletNowIdx,5) + 1;
            else
                pallet_check = find(min(palletXY(palletFree,5))==palletXY(palletFree,5));
                if length(pallet_check) == 1
                    palletNowIdx  = palletXY(pallet_check,1);
                    palletXY(pallet_check,5) = palletXY(pallet_check,5) + 1;
                else
                    palletNowIdx  = palletXY(1,1);
                    palletXY(1,5) = palletXY(1,5) + 1;
                end
            end
        else % 이 경우 완료된 작업이 있다는 뜻이므로 팔렛과 상관없이 완료된거 부터 이송한다.
            % 이게 맞을까? 완료된 작업은 어짜피 다음 팔렛을 필요로 한다.
            % 그러므로 완료된 작업이 있든 말든 로봇은 팔렛으로 가는게 맞다?
            % 추후 고민해보고 수정
            if length(stationReady) ==1
                stationNowIdx = stationReady;
            else
                stationNowIdx = stationReady(1);
            end
        end
        % 만약 작업 완료한 station이 없을때    
        if stationNowIdx == 0
            if palletNowIdx == 0
                % 빈 pallet 없으면 제자리 대기한다.
                standRobotIdx = robotSet((robotSet(:,4)==0),1);
                for j = 1:length(standRobotIdx)
                    thisRobotIdx = standRobotIdx(j);
                    thisRobotXY = robotSet(thisRobotIdx, 2:3);
                    goalXY = thisRobotXY;
                    PathNow = flipud(astar(thisRobotXY, goalXY, ~gridMap));
                    thisRobotPath = PathNow;
                    robotPaths{thisRobotIdx} = thisRobotPath;
                    
                    % visualizing 하자.
                    if showFig == 2
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                end
            else % 빈 pallet이 있으면 로봇이 출발
                % 만약 여러군데가 비어있다면 로봇이 더 방문하지 않은 Pallet은 어디인가?
                % 가장 가까운 로봇은?
                if length(RobotIdxfree) == 1
                    PathLengthCompare_Now = PathLengthCompare_Pallet(RobotIdxfree);
                else
                    PathLengthCompare_Now = PathLengthCompare_Pallet((robotSet(:,4)==0),1);
                end
                pallet_select = find(min(PathLengthCompare_Now)==PathLengthCompare_Pallet);
                if length(pallet_select)==1
                    thisRobotIdx = pallet_select;
                elseif length(pallet_select)>1
                    thisRobotIdx = pallet_select(1);
                end
                thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
                thisRobotTask = robotTasks{thisRobotIdx}; % 로봇의 capacity 상태 (아직 수정안됨)
                thisRobotDes = robotSet(thisRobotIdx,5);
                thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle (아직 수정안됨)
                %thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1}; % 로봇의 일의 순서1 (아직 수정안됨)
                %thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2}; % 로봇의 일의 순서2 (아직 수정안됨)
                %timeInTask = timeInTask + size(thisRobotTask,1);
                % 로봇의 Path를 업데이트하고, 상태를 적용해준다.
                thisRobotPath = PathTable_RobotToPallet{thisRobotIdx,palletNowIdx};
                % 로봇의 Task를 업데이트한다.
                thisRobotTask = palletXY(palletNowIdx,:);
                % 팔렛을 싣는 시간을 path에 추가해줌
                for LUtime = 1:palletLUtime
                    thisRobotPath = [thisRobotPath; thisRobotPath(end,:)];
                end
                thisRobotStatus = 0.5;
                robotPaths{thisRobotIdx} = thisRobotPath;
                robotSet(thisRobotIdx,4) = thisRobotStatus;
                robotSet(thisRobotIdx,5) = palletNowIdx;
                
                palletXY(palletNowIdx,4) = thisRobotIdx; % pallet에 몇번 로봇이 가고있는지 기록
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotTasksRecord{thisRobotIdx} = thisRobotTask;
                thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
                thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
                
                % visualizing 하자.
                if showFig == 2
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        else
            % 만약 여러 작업이 완료되었다면?
            % 완료된 station에서 가장 가까운 로봇은?
            
            % robot이 warehouse에서 station으로 가는 경우 ( free를 search )
            if length(RobotIdxfree) == 1
                PathLengthCompare_Now = PathLengthCompare_Station(RobotIdxfree);
            else
                PathLengthCompare_Now = PathLengthCompare_Station((robotSet(:,4)==0),1);
            end
            station_select = find(min(PathLengthCompare_Now)==PathLengthCompare_Station);
            if ~isempty(station_select)
                if length(station_select)==1
                    thisRobotIdx = station_select;
                elseif length(station_select)>1
                    thisRobotIdx = station_select(station_select==RobotIdxfree); % 여기서 error 나는지 check해야함
                end
                %PathLengthCompare_Now = PathLengthCompare_Pallet(RobotIdxfree);
                %PathLengthCompare_Now = PathLengthCompare_Now(find((robotSet(:,4)==0)));
                %thisRobotIdx = RobotIdxpallet(i);
                thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
                if robotSet(thisRobotIdx,5) > 100
                    thisRobotDes = robotSet(thisRobotIdx,5) - 100;
                elseif robotSet(thisRobotIdx,5) >10 && robotSet(thisRobotIdx,5)  < 100
                    thisRobotDes = robotSet(thisRobotIdx,5) - 10;
                else
                    thisRobotDes = robotSet(thisRobotIdx,5);
                end
                thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle (아직 수정안됨)
                
                % 로봇의 Path를 업데이트하고, 상태를 적용해준다.
                thisRobotPath = PathTable_WarehouseToStation{thisRobotDes,stationNowIdx};
                % 로봇의 Task를 업데이트한다.
                thisRobotTask = stationXY(stationNowIdx,:);
                
                thisRobotStatus = 1.5;
                robotPaths{thisRobotIdx} = thisRobotPath;
                robotSet(thisRobotIdx,4) = thisRobotStatus;
                robotSet(thisRobotIdx,5) = stationNowIdx + 10;
                
                stationXY(stationNowIdx,4) = thisRobotIdx; % station에 몇번 로봇이 가고있는지 기록
                
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotTasksRecord{thisRobotIdx} = thisRobotTask;
                
                thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
                thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
                
                % visualizing 하자.
                if showFig == 2
                    hStation(thisRobotTaskIdx) = plot(thisRobotTask(:,3), thisRobotTask(:,3), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    % 출발 시나리오2. from pallet to station
    for i = 1:length(RobotIdxpallet)
        % pallet에 도착한 로봇은 어느 station으로 갈 것인가?
        % 비어 있는 station을 조사
        sFree1 = stationXY((stationXY(:,4)==0),1);
        sFree2 = stationXY((stationXY(:,5)==1),1);
        sFree3 = stationXY((stationXY(:,5)==0),1);
        stationReady = stationXY(intersect(sFree1,sFree2),1);
        stationFree = stationXY(intersect(sFree1,sFree3),1);
        % 만약 station이 다 작업 중이라면 더 빨리 끝나는(비어지는) station은 어디인가?
        if isempty(stationReady)
            if isempty(stationFree)
                stationNowIdx = 0;
            % station이 한개만 비어져 있으면 무조건 거기로 가야겠지
            elseif length(stationFree) == 1
                stationNowIdx = stationXY(stationFree,1);
                % 비어있는 station 중에서는 어느 곳이 제일 가까운가?
            else
                % 일단 비어있는 곳 순서대로 보내자.robotSet((robotSet(:,4)==0),1)
                station_check = stationFree;
                % 비어있는 곳이 한곳이면 그곳으로 보내야겠지
                if length(station_check) == 1
                    stationNowIdx  = stationXY(station_check,1);
                else
                    for station_empty = 1:length(station_check)
                        stationNowIdx  = stationXY(station_check(station_empty,1));
                    end
                    stationNowIdx = stationNowIdx(1);
                end
            end
        else % 이 경우 완료된 작업이 있다는 뜻이므로 팔렛과 상관없이 완료된거 부터 이송한다.
            % 물론 로봇의 capacity가 최대가 아닐때
            if length(stationReady) ==1
                stationNowIdx = stationReady;
            else
                stationNowIdx = stationReady(1);
            end
        end
        if stationNowIdx == 0 || robotSet(RobotIdxpallet(i),7) == 1
            % 빈 station 없으면 제자리 대기한다.
            % --> 곧 빌 station으로 간다 추가해야함.
            standRobotIdx = robotSet((robotSet(:,4)==1),1);
            for j = 1:length(standRobotIdx)
                thisRobotIdx = standRobotIdx(j);
                thisRobotXY = robotSet(thisRobotIdx, 2:3);
                goalXY = thisRobotXY;
                PathNow = flipud(astar(thisRobotXY, goalXY, ~gridMap));
                thisRobotPath = PathNow;
                robotPaths{thisRobotIdx} = thisRobotPath;
                % visualizing 하자.
                if showFig == 2
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        else % 빈 station이 있으면 로봇이 출발
            % 가장 가까운 로봇은?
            %PathLengthCompare_Now = PathLengthCompare_Pallet(RobotIdxfree);
            %PathLengthCompare_Now = PathLengthCompare_Now(find((robotSet(:,4)==0)));
            thisRobotIdx = RobotIdxpallet(i);
            thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
            thisRobotDes = robotSet(thisRobotIdx,5);
            thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle (아직 수정안됨)
           
            % 로봇의 Path를 업데이트하고, 상태를 적용해준다.
            thisRobotPath = PathTable_PalletToStation{thisRobotDes,stationNowIdx};
            % 로봇의 Task를 업데이트한다.
            thisRobotTask = stationXY(stationNowIdx,:);
            
            thisRobotStatus = 1.5;
            robotPaths{thisRobotIdx} = thisRobotPath;
            robotSet(thisRobotIdx,4) = thisRobotStatus;
            robotSet(thisRobotIdx,5) = stationNowIdx + 10;
            
            stationXY(stationNowIdx,4) = thisRobotIdx; % station에 몇번 로봇이 가고있는지 기록
            
            robotTasks{thisRobotIdx} = thisRobotTask;
            robotTasksRecord{thisRobotIdx} = thisRobotTask;
            
            thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
            thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
            
            % visualizing 하자.
            if showFig == 2
                hStation(thisRobotTaskIdx) = plot(thisRobotTask(:,2), thisRobotTask(:,3), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 출발 시나리오3. from station to station? to warehouse?
    for i = 1:length(RobotIdxstation)
        % 빈 warehouse과 가장 가까우면서 놀고 있는 로봇은 누구인가?
        % 가장 우선인게 완료된작업을 warehouse에 가져다주는것.
        sFree1 = stationXY((stationXY(:,4)==0),1);
        sFree2 = stationXY((stationXY(:,5)==1),1);
        stationReady = stationXY(intersect(sFree1,sFree2),1);
        % 비어 있는 warehouse을 조사
        warehouseFree = warehouseXY((warehouseXY(:,4)==0),1);
        % 만약 여러군데가 비어있다면 로봇이 더 방문하지 않은 warehouse은 어디인가?
        % 물론 그 전에 완료된 작업이 없고 
        if isempty(stationReady)
            stationNowIdx = 0;
            if isempty(warehouseFree)
                warehouseNowIdx = 0;
            elseif length(warehouseFree) == 1
                warehouseNowIdx = warehouseXY(warehouseFree,1);
                warehouseXY(warehouseNowIdx,5) = warehouseXY(warehouseNowIdx,5) + 1;
            else
                warehouse_check = find(min(warehouseXY(warehouseFree,5))==warehouseXY(warehouseFree,5));
                if length(warehouse_check) == 1
                    warehouseNowIdx  = warehouseXY(warehouse_check,1);
                    warehouseXY(warehouse_check,5) = warehouseXY(warehouse_check,5) + 1;
                else
                    warehouseNowIdx  = warehouseXY(1,1);
                    warehouseXY(1,5) = warehouseXY(1,5) + 1;
                end
            end
        else
            % 이 경우 완료된 작업이 있다는 뜻이므로 
            % 그와 동시에 로봇이 짐을 들수 있는 상태면 들렸다가 warehouse에 간다.
            if robotSet(thisRobotIdx,7) == 0
                if length(stationReady) ==1
                    stationNowIdx = stationReady;
                else
                    stationNowIdx = stationReady(1);
                end
            else
                stationNowIdx = 0;
            end
        end        
        % 만약 작업 완료한 station이 없을때    
        if stationNowIdx == 0
            if warehouseNowIdx == 0
                % 빈 warehouse 없으면 제자리 대기한다.
                standRobotIdx = robotSet((robotSet(:,4)==2),1);
                for j = 1:length(standRobotIdx)
                    thisRobotIdx = standRobotIdx(j);
                    thisRobotXY = robotSet(thisRobotIdx, 2:3);
                    goalXY = thisRobotXY;
                    PathNow = flipud(astar(thisRobotXY, goalXY, ~gridMap));
                    thisRobotPath = PathNow;
                    robotPaths{thisRobotIdx} = thisRobotPath;
                    
                    % visualizing 하자.
                    if showFig == 2
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                end
            else % 빈 warehouse이 있으면 로봇이 출발
                thisRobotIdx = RobotIdxstation(i);
                thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
                thisRobotTask = robotTasks{thisRobotIdx}; % 로봇의 capacity 상태 (아직 수정안됨)
                thisRobotDes = robotSet(thisRobotIdx,5);
                thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle (아직 수정안됨)
                %thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1}; % 로봇의 일의 순서1 (아직 수정안됨)
                %thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2}; % 로봇의 일의 순서2 (아직 수정안됨)
                %timeInTask = timeInTask + size(thisRobotTask,1);
                % 로봇의 Path를 업데이트하고, 상태를 적용해준다.
                thisRobotPath = PathTable_StationToWarehouse{thisRobotDes-10,warehouseNowIdx};
                % 로봇의 Task를 업데이트한다.
                thisRobotTask = warehouseXY(warehouseNowIdx,:);
                % warehouse에서 unload 시간을 path에 추가해줌
                for LUtime = 1:warehouseLUtime
                    thisRobotPath = [thisRobotPath; thisRobotPath(end,:)];
                end
                thisRobotStatus = 2.5;
                robotPaths{thisRobotIdx} = thisRobotPath;
                robotSet(thisRobotIdx,4) = thisRobotStatus;
                robotSet(thisRobotIdx,5) = warehouseNowIdx + 100;
                
                warehouseXY(warehouseNowIdx,4) = thisRobotIdx; % warehouse에 몇번 로봇이 가고있는지 기록
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotTasksRecord{thisRobotIdx} = thisRobotTask;
                thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
                thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
                % visualizing 하자.
                if showFig == 2
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        else
            % 만약 여러 작업이 완료되었다면?
            % 완료된 station에서 가장 가까운 로봇은?
            % robot이 station에서 station으로 가는 경우 ( free를 search )
            if length(RobotIdxstation) == 1
                PathLengthCompare_Now = PathLengthCompare_Station(RobotIdxstation);
            else
                PathLengthCompare_Now = PathLengthCompare_Station((robotSet(:,4)==2),1);
            end
            station_select = find(min(PathLengthCompare_Now)==PathLengthCompare_Station);
            if ~isempty(station_select)
                if length(station_select)==1
                    thisRobotIdx = station_select;
                elseif length(station_select)>1
                    thisRobotIdx = station_select(station_select==RobotIdxstation); % 여기서 error 나는지 check해야함
                end
                %PathLengthCompare_Now = PathLengthCompare_Pallet(RobotIdxfree);
                %PathLengthCompare_Now = PathLengthCompare_Now(find((robotSet(:,4)==0)));
                %thisRobotIdx = RobotIdxpallet(i);
                thisRobotXY = robotSet(thisRobotIdx, 2:3); % 로봇 xy 위치
                if robotSet(thisRobotIdx,5) > 100
                    thisRobotDes = robotSet(thisRobotIdx,5) - 100;
                elseif robotSet(thisRobotIdx,5) >10 && robotSet(thisRobotIdx,5)  < 100
                    thisRobotDes = robotSet(thisRobotIdx,5) - 10;
                else
                    thisRobotDes = robotSet(thisRobotIdx,5);
                end
                thisRobotAngle = robotSet(thisRobotIdx, 6); % 로봇 angle (아직 수정안됨)
                
                % 로봇의 Path를 업데이트하고, 상태를 적용해준다.
                thisRobotPath = PathTable_StationToStation{thisRobotDes,stationNowIdx};
                % 로봇의 Task를 업데이트한다.
                thisRobotTask = stationXY(stationNowIdx,:);
                
                thisRobotStatus = 1.5;
                robotPaths{thisRobotIdx} = thisRobotPath;
                robotSet(thisRobotIdx,4) = thisRobotStatus;
                robotSet(thisRobotIdx,5) = stationNowIdx + 10;
                
                stationXY(stationNowIdx,4) = thisRobotIdx; % station에 몇번 로봇이 가고있는지 기록
                
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotTasksRecord{thisRobotIdx} = thisRobotTask;
                
                thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
                thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
                
                % visualizing 하자.
                if showFig == 2
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        end
    end
    
    % conflict check
    robotPaths = pathConflictCheck(robotPaths);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 출발 시뮬레이션 (step의 경과에 따라 그려줌)
    workingRobotIdx = robotSet((rem(robotSet(:,4),1)>0),1);%Identify working robots
    for i = 1:length(workingRobotIdx)
        timeRobotMove = timeRobotMove + 1;
        thisRobotIdx = workingRobotIdx(i);
        thisRobotPath = robotPaths{thisRobotIdx};
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotTask = robotTasks{thisRobotIdx};
        %thisRobotWaitingCell = [];
        timeInExecution = timeInExecution + size(robotTasksRecord{thisRobotIdx},1);
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
            if rem(robotSet(:,4),1) == 0
                yourNextCell = [robotSet(j,2),robotSet(j,3)];
                yourCrtCell = [robotSet(j,2),robotSet(j,3)];
            else
                yourNextPath = robotPaths{j};
                if isempty(yourNextPath)
                    yourNextPath = robotSet(j,2:3);
                end
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
            %if length(thisRobotPath(:,1)) > 1
            thisRobotPath(1,:) = [];
            %end
            robotPaths{thisRobotIdx} = thisRobotPath;
        end
        
        
        % If conflict is true and deferMove is true,
        if conflict && makeDetour
            for k=1:nRobot
                if robotSet(k,4)==0
                    gridMap(robotSet(k,2),robotSet(k,3)) = 1;
                else
                    yourNextPath = robotPaths{k};
                    if isempty(yourNextPath)
                        yourNextPath = robotSet(k,2:3);
                    end
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
            if isempty(thisRobotTask)
                goalXY = warehouseXY;
            else
                goalXY  = thisRobotTask(1,2:3);
            end
            gridMap(goalXY(1), goalXY(2)) = 0;%goal location should be free
            thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
            modWhen = 1;
            robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
            if showFig == 2
                delete(hPath(thisRobotIdx))
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'co', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
            end
            if length(thisRobotPath(:,1)) > 1
                thisRobotPath(1,:) = [];
            end
            robotPaths{thisRobotIdx} = thisRobotPath;
        end
        
        % 로봇이 지금 하고 있는 일이 끝나면, 상태를 업데이트 해준다.
        %if the current task is done, now need to see the Task for the rest of tasks in the Task
        if isempty(thisRobotPath)
            % pallet에 도착한 경우
            if robotSet(thisRobotIdx,4)==0.5
                thisRobotTaskIdx = thisRobotTask(1,2);
                thisRobotTask(1,:) = [];%arrived so remove from Task
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotSet(thisRobotIdx,4) = robotSet(thisRobotIdx,4)+0.5;
                palletXY((palletXY(:,4)==thisRobotIdx),4) = 0;
            % station에 도착한 경우    
            elseif robotSet(thisRobotIdx,4)==1.5
                thisRobotTaskIdx = thisRobotTask(1,2);
                thisRobotTask(1,:) = [];%arrived so remove from Task
                robotTasks{thisRobotIdx} = thisRobotTask;
                robotSet(thisRobotIdx,4) = robotSet(thisRobotIdx,4)+0.5;
                stationRobotarriv = (stationXY(:,4)==thisRobotIdx);
                stationXY(stationRobotarriv,4) = 0;
                % station 도착한 후 빈 station에 pallet 전달
                if stationXY(stationRobotarriv,5) == 0
                    if robotSet(thisRobotIdx,7) == 0 
                        stationXY(stationRobotarriv,5) = stationLUtime(stationRobotarriv);
                        %robotSet(thisRobotIdx,7) = robotSet(thisRobotIdx,7) + 1; % bundle 수 추가 
                    else
                        stationXY(stationRobotarriv,5) = stationLUtime(stationRobotarriv);
                        %robotSet(thisRobotIdx,7) = robotSet(thisRobotIdx,7) + 1; % bundle 수 추가
                    end
                % station 도착한 후 완료한 bundle 가지고 간다
                elseif stationXY(stationRobotarriv,5) == 1
                    if robotSet(thisRobotIdx,7) == 0
                        stationXY(stationRobotarriv,5) = 0;
                        robotSet(thisRobotIdx,7) = robotSet(thisRobotIdx,7) + 1;
                    else
                        stationXY(stationRobotarriv,5) = 0;
                        robotSet(thisRobotIdx,7) = robotSet(thisRobotIdx,7) + 1;
                        
                    end
                else
                    disp('not good');
                end
            % warehouse에 도착한 경우
            elseif robotSet(thisRobotIdx,4)==2.5
                thisRobotTaskIdx = thisRobotTask(1,2);
                thisRobotTask(1,:) = [];%arrived so remove from Task
                robotTasks{thisRobotIdx} = thisRobotTask;
                %robotSet(thisRobotIdx,4) = robotSet(thisRobotIdx,4)+0.5;
                robotSet(thisRobotIdx,4) = 0;
                robotSet(thisRobotIdx,7) = 0;
                warehouseXY((warehouseXY(:,4)==thisRobotIdx),4) = 0;
            end
            
            
            if showFig == 2
                delete(hPath(thisRobotIdx))
                if robotSet(thisRobotIdx,4)==2
                    delete(hStation(thisRobotTask-10))
                end
            end
            
            if isempty(thisRobotTask)%if Task is empty
                if robotSet(thisRobotIdx,4) == 4
                    robotSet(thisRobotIdx,4) = 1;%mark this robot going to packing station
                    stationXY(robotSet(thisRobotIdx,5),4) = 0;
                    taskSetTime(robotSet(thisRobotIdx,5)) = 0;
                    robotSet(thisRobotIdx,5) = 0;
                    %plan path to the station
                    startXY = thisRobotXY;
                    % 일단 station 두개 번갈아 설정하도록 해놓음 나중에 바꿔도됨
                    if mod(tStep,2) == 0
                        thisRobotwarehouseXY = warehouseXY(1,2:3);
                    else
                        thisRobotwarehouseXY = warehouseXY(2,2:3);
                    end
                    goalXY  = thisRobotwarehouseXY;
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
                    %                 robotSet(thisRobotIdx,4) = 0; %mark this robot idle
                    %                 % At this time, the tasks are completed
                    %                 doneTasksInThisTask = robotTasksRecord{thisRobotIdx};
                    %                 for k=1:size(doneTasksInThisTask,1)
                    %                     taskAll(doneTasksInThisTask(k,2), 3) = tStep;
                    %                 end
                end
            else%if there is at least one task remaining in the Task
                thisRobotTaskIdx = thisRobotTask(1,2);
                startXY = thisRobotXY;
                goalXY  = thisRobotTask(1,2:3);
                thisRobotWaitingCell = thisRobotXY;
                thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
                thisRobotPath_temp = thisRobotPath;
                %modWhen = 3;
                if isempty(thisRobotPath_temp) && ~isempty(thisRobotWaitingCell)
                    thisRobotPath = [thisRobotWaitingCell; thisRobotWaitingCell; thisRobotPath];
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    if showFig == 2
                        %hTask(thisRobotTaskIdx) = plot(thisRobotTask(1,4), thisRobotTask(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                else
                    thisRobotPath = thisRobotPath_temp;
                    modWhen = 3;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    if showFig == 2
                        %hTask(thisRobotTaskIdx) = plot(thisRobotTask(1,4), thisRobotTask(1,5), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end
                end
            end
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
        for nRobotPlot=1:nRobot
            hcolor = [0.3 0.3 0.3];
            hRobot(nRobotPlot) = plot(robotSet(nRobotPlot,2), robotSet(nRobotPlot,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor',hcolor*nRobotPlot);
        end
    end
    
    
    gridMap = gridMap_save;
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

fprintf('%d robots, %d Task size, %d tasks done\n', nRobot, bSize, numCompletedTasks)
fprintf('Execution time per task: %.4f\n', timeRobotMove/(numCompletedTasks))
fprintf('Bundling time per task: %.4f\n', timeInTask/numTaskdTasks)
fprintf('Timespan per task (all): %.4f\n', completionTimeAllTasks/numAllTasks)
fprintf('Timespan per task (done): %.4f\n', completionTimeDoneTasks/numCompletedTasks)
fprintf('Queue residing time: %.4f\n', timeInQueueTasks/numAllTasks)
(timeInQueueTasks + timeInTask + timeInExecution)/numAllTasks
