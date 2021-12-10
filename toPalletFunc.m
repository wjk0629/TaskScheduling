function [palletXY,robotTasks,thisRobotIdx,thisVeloInfo,thisRobotPath,thisRobotAngle] = toPalletFunc(RobotIdxfree,palletXY,stationXY,robotSet,robotTasks,robotPaths,palletLUtime,PathTable_RobotToPallet,PathLengthCompare_Pallet,gridMap,showFig)

% ��� �ó�����1. to pallet
%[palletXY,robotTasks,thisRobotIdx,thisVeloInfo,thisRobotPath,thisRobotAngle] = toPalletFunc(RobotIdxfree,palletXY,robotSet,robotTasks,robotPaths,palletLUtime,PathTable_RobotToPallet,PathLengthCompare_Pallet,gridMap,showFig);
for i = 1:length(RobotIdxfree)
    % ���� �켱�ΰ� �Ϸ���۾��� warehouse�� �������ִ°�.
    sFree1 = stationXY((stationXY(:,4)==0),1);
    sFree2 = stationXY((stationXY(:,5)==1),1);
    sFree3 = stationXY((stationXY(:,5)==0),1);
    stationReady = stationXY(intersect(sFree1,sFree2),1);
    stationFree = stationXY(intersect(sFree1,sFree3),1);
    % �� pallet�� ���� �����鼭 ��� �ִ� �κ��� �����ΰ�? ** �׸��� �˰����� �ݿ��� �κ�
    % ��� �ִ� pallet�� ����
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
    else % �� ��� �Ϸ�� �۾��� �ִٴ� ���̹Ƿ� �ȷ��� ������� �Ϸ�Ȱ� ���� �̼��Ѵ�.
        if length(stationReady) ==1
            stationNowIdx = stationReady;
        else
            stationNowIdx = stationReady(1);
        end
    end
    % ���� �۾� �Ϸ��� statio�� ������
    if stationNowIdx == 0
        if palletNowIdx == 0
            % �� pallet ������ ���ڸ� ����Ѵ�.
            standRobotIdx = robotSet((robotSet(:,4)==0),1);
            for j = 1:length(standRobotIdx)
                thisRobotIdx = standRobotIdx(j);
                thisRobotXY = robotSet(thisRobotIdx, 2:3);
                goalXY = thisRobotXY;
                PathNow = flipud(astar(thisRobotXY, goalXY, ~gridMap));
                thisRobotPath = PathNow;
                robotPaths{thisRobotIdx} = thisRobotPath;
                
                % visualizing ����.
                if showFig == 2
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end
            end
        else % �� pallet�� ������ �κ��� ���
            % ���� ���������� ����ִٸ� �κ��� �� �湮���� ���� Pallet�� ����ΰ�?
            % ���� ����� �κ���?
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
            thisRobotXY = robotSet(thisRobotIdx, 2:3); % �κ� xy ��ġ
            thisRobotTask = robotTasks{thisRobotIdx}; % �κ��� capacity ���� (���� �����ȵ�)
            thisRobotDes = robotSet(thisRobotIdx,5);
            thisRobotAngle = robotSet(thisRobotIdx, 6); % �κ� angle (���� �����ȵ�)
            %thisRobotTaskSeq1 = robotTaskSeq{thisRobotIdx,1}; % �κ��� ���� ����1 (���� �����ȵ�)
            %thisRobotTaskSeq2 = robotTaskSeq{thisRobotIdx,2}; % �κ��� ���� ����2 (���� �����ȵ�)
            %timeInTask = timeInTask + size(thisRobotTask,1);
            % �κ��� Path�� ������Ʈ�ϰ�, ���¸� �������ش�.
            thisRobotPath = PathTable_RobotToPallet{thisRobotIdx,palletNowIdx};
            % �κ��� Task�� ������Ʈ�Ѵ�.
            thisRobotTask = palletXY(palletNowIdx,:);
            % �ȷ��� �ƴ� �ð��� path�� �߰�����
            for LUtime = 1:palletLUtime
                thisRobotPath = [thisRobotPath; thisRobotPath(end,:)];
            end
            thisRobotStatus = 0.5;
            robotPaths{thisRobotIdx} = thisRobotPath;
            robotSet(thisRobotIdx,4) = thisRobotStatus;
            robotSet(thisRobotIdx,5) = palletNowIdx;
            
            palletXY(palletNowIdx,4) = thisRobotIdx; % pallet�� ��� �κ��� �����ִ��� ���
            robotTasks{thisRobotIdx} = thisRobotTask;
            robotTasksRecord{thisRobotIdx} = thisRobotTask;
            thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
            thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
            
            % visualizing ����.
            if showFig == 2
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end
        end
    else
        % ���� ���� �۾��� �Ϸ�Ǿ��ٸ�?
        % �Ϸ�� station���� ���� ����� �κ���?
        
        % robot�� warehouse���� station���� ���� ��� ( free�� search )
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
                thisRobotIdx = station_select(station_select==RobotIdxfree); % ���⼭ error ������ check�ؾ���
            end
            %PathLengthCompare_Now = PathLengthCompare_Pallet(RobotIdxfree);
            %PathLengthCompare_Now = PathLengthCompare_Now(find((robotSet(:,4)==0)));
            %thisRobotIdx = RobotIdxpallet(i);
            thisRobotXY = robotSet(thisRobotIdx, 2:3); % �κ� xy ��ġ
            if robotSet(thisRobotIdx,5) > 100
                thisRobotDes = robotSet(thisRobotIdx,5) - 100;
            elseif robotSet(thisRobotIdx,5) >10 && robotSet(thisRobotIdx,5)  < 100
                thisRobotDes = robotSet(thisRobotIdx,5) - 10;
            else
                thisRobotDes = robotSet(thisRobotIdx,5);
            end
            thisRobotAngle = robotSet(thisRobotIdx, 6); % �κ� angle (���� �����ȵ�)
            
            % �κ��� Path�� ������Ʈ�ϰ�, ���¸� �������ش�.
            thisRobotPath = PathTable_WarehouseToStation{thisRobotDes,stationNowIdx};
            % �κ��� Task�� ������Ʈ�Ѵ�.
            thisRobotTask = stationXY(stationNowIdx,:);
            
            thisRobotStatus = 1.5;
            robotPaths{thisRobotIdx} = thisRobotPath;
            robotSet(thisRobotIdx,4) = thisRobotStatus;
            robotSet(thisRobotIdx,5) = stationNowIdx + 10;
            
            stationXY(stationNowIdx,4) = thisRobotIdx; % station�� ��� �κ��� �����ִ��� ���
            
            robotTasks{thisRobotIdx} = thisRobotTask;
            robotTasksRecord{thisRobotIdx} = thisRobotTask;
            
            thisVeloInfo = [thisRobotPath(1,1)-robotSet(thisRobotIdx,2) thisRobotPath(1,2)-robotSet(thisRobotIdx,3)];
            thisRobotAngle = atan2(thisVeloInfo(2),thisVeloInfo(1));
            
            % visualizing ����.
            if showFig == 2
                hStation(thisRobotTaskIdx) = plot(thisRobotTask(:,3), thisRobotTask(:,3), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end
        end
    end
end
end