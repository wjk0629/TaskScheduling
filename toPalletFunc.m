function [palletXY,robotTasks,thisRobotIdx,thisVeloInfo,thisRobotPath,thisRobotAngle] = toPalletFunc(RobotIdxfree,palletXY,stationXY,robotSet,robotTasks,robotPaths,palletLUtime,PathTable_RobotToPallet,PathLengthCompare_Pallet,gridMap,showFig)

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
        if length(stationReady) ==1
            stationNowIdx = stationReady;
        else
            stationNowIdx = stationReady(1);
        end
    end
    % 만약 작업 완료한 statio이 없을때
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
end