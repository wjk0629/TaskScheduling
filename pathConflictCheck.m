function [new_robotPaths] = pathConflictCheck(robotPaths)

fakeIndex = 0.1;
nRobot = length(robotPaths);

for i = 1:nRobot
    Path_length(i) = length(robotPaths{i}(:,1));
end

Path_longest = max(Path_length);

% 길이를 가장 긴 path에 맞춰준다.
for i = 1:nRobot
    for j = 1:Path_longest-Path_length(i)
        if Path_length(i) < Path_longest
            robotPaths{i} = [robotPaths{i}; fakeIndex fakeIndex]; % 0으로 설정해서 실제 맵에 0 좌표가 있으면 안됨..
        end
    end
end

conflictInfo = [];
conflictInfo2 = [];
% 중복되는 cell이 있는지 검사(있으면 충돌인것)
for i = 1:nRobot
    for j = 1:nRobot
        egoRobotIdx = i;
        egoRobotPath = robotPaths{i};
        otherRobotIdx = j;
        otherRobotPath = robotPaths{j};
        
        for k = 1:Path_longest
            % 충돌 case 1. cell이 동일 시간에 중복될 때
            if egoRobotPath(k,:) == otherRobotPath(k,:) & egoRobotPath(k,:) ~= fakeIndex & i~=j
                conflictInfo = [conflictInfo; egoRobotPath(k,:),i,j,k];
            end
            % 충돌 case 2. egocellbefore = othercellnow
            % && othercellbefore = egocellnow 일때
            if k > 1
                if egoRobotPath(k,:) == otherRobotPath(k-1,:) & otherRobotPath(k,:) == egoRobotPath(k-1,:) ...
                        & egoRobotPath(k,:) ~= fakeIndex & i~=j
                    conflictInfo2 = [conflictInfo2; egoRobotPath(k,:),i,j,k; otherRobotPath(k-1,:),i,j,k-1];
                end
            end
        end
    end
end
% conflictInfo(:,1:2) : x,y location , (:,3:4) : ego & other , (:,5) : step

% 충돌하는 cell의 정보를 알았으니, 이를 분석하여 어느 cell에서 어느 robot이 기다릴지 도출

%case 1일때
if ~isempty(conflictInfo)
    %for i = 1:length(conflictInfo(:,1))/2
    conflictInfo = conflictInfo(conflictInfo(:,5)==min(conflictInfo(:,5)),:);
        time_conflict = conflictInfo(length(conflictInfo(:,1))/2,5);
        % 여기서 ego의 path를 바꿔주는데, 나중에 input으로 지금까지 이동한 거리 받아서 더 짧은애가 기다리는게 좋을듯 하다
        egoRobotIdx = conflictInfo(length(conflictInfo(:,1))/2,3);
        egoRobotPath = robotPaths{egoRobotIdx};
        otherRobotIdx = conflictInfo(length(conflictInfo(:,1))/2,4);
        otherRobotPath = robotPaths{otherRobotIdx};
        for j = 1:Path_longest-1
            egoRobotVelo(j,:) = egoRobotPath(j+1,:) - egoRobotPath(j,:);
            otherRobotVelo(j,:) = otherRobotPath(j+1,:) - otherRobotPath(j,:);
        end
        conflictIndex = 0;
        while 1
            if time_conflict-conflictIndex-1 < 1
                break;
            end
            index_ego1 = egoRobotVelo(time_conflict-conflictIndex,:);
            index_ego2 = egoRobotVelo(time_conflict-conflictIndex-1,:);
            index_ego = index_ego1 - index_ego2;
            index_other1 = otherRobotVelo(time_conflict-conflictIndex,:);
            index_other2 = otherRobotVelo(time_conflict-conflictIndex-1,:);
            index_other = index_other1 - index_other2;
            if (index_ego(1) == 0 && index_ego(2) == 0) && time_conflict+conflictIndex < Path_longest-1
                conflictIndex = conflictIndex + 1;
            else
                if conflictIndex == 0
                    break;
                else
                conflictCell = robotPaths{egoRobotIdx}(time_conflict,:);
                waitingCell = robotPaths{egoRobotIdx}(time_conflict-conflictIndex-1,:);
                waitingIdx = find(((conflictCell(:,1) == otherRobotPath(:,1)) & (conflictCell(:,2) == otherRobotPath(:,2))) == 1);
                for j = 1:waitingIdx
                    robotPaths{egoRobotIdx} = [robotPaths{egoRobotIdx}(1:conflictIndex-1,:); cat(1,waitingCell,robotPaths{egoRobotIdx}(conflictIndex:end,:))];
                end    
                break;
                end
            end
        end
    %end
%case 2일때
elseif ~isempty(conflictInfo2)
    for i = 1:length(conflictInfo2(:,1))/4
        time_conflict = conflictInfo2(4*i-2,5);
        % 여기서 ego의 path를 바꿔주는데, 나중에 input으로 지금까지 이동한 거리 받아서 더 짧은애가 기다리는게 좋을듯 하다
        egoRobotIdx = conflictInfo2(4*i-3,3);
        egoRobotPath = robotPaths{egoRobotIdx};
        otherRobotIdx = conflictInfo2(4*i-3,4);
        otherRobotPath = robotPaths{otherRobotIdx};
        for j = 1:Path_longest-1
            egoRobotVelo(j,:) = egoRobotPath(j+1,:) - egoRobotPath(j,:);
            otherRobotVelo(j,:) = otherRobotPath(j+1,:) - otherRobotPath(j,:);
        end
        conflictIndex = 0;
        while 1
            if time_conflict-conflictIndex-1 < 1
                break;
            end
            index_ego1 = egoRobotVelo(time_conflict-conflictIndex,:);
            index_ego2 = egoRobotVelo(time_conflict-conflictIndex-1,:);
            index_ego = index_ego1 - index_ego2;
            index_other1 = otherRobotVelo(time_conflict-conflictIndex,:);
            index_other2 = otherRobotVelo(time_conflict-conflictIndex-1,:);
            index_other = index_other1 - index_other2;
            if (index_ego(1) == 0 && index_ego(2) == 0) && time_conflict+conflictIndex < Path_longest-1
                conflictIndex = conflictIndex + 1;
            else
                if conflictIndex == 0
                    break;
                else
                conflictCell = robotPaths{egoRobotIdx}(time_conflict,:);
                waitingCell = robotPaths{egoRobotIdx}(time_conflict-conflictIndex-1,:);
                waitingIdx = find(((conflictCell(:,1) == otherRobotPath(:,1)) & (conflictCell(:,2) == otherRobotPath(:,2))) == 1);
                for j = 1:waitingIdx
                    robotPaths{egoRobotIdx} = [robotPaths{egoRobotIdx}(1:conflictIndex-1,:); cat(1,waitingCell,robotPaths{egoRobotIdx}(conflictIndex:end,:))];
                end    
                break;
                end
            end
        end
    end
else
    % there are no conflict
end

% 이제 path의 길이를 맞추려고 추가한 것들을 없애준다.
for i = 1:nRobot
    for j = 1:length(robotPaths{i}(:,1))
        if robotPaths{i}(end,1) == fakeIndex && robotPaths{i}(end,2) == fakeIndex
            robotPaths{i}(end,:) = [];
        end
    end
end
new_robotPaths = robotPaths;
end