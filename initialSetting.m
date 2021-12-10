function [palletLUtime,warehouseLUtime,stationLUtime, ...
    robotSet,robotTasks,robotTasksRecord,robotPaths,robotDistance, ...
    taskQueue,taskQueue_all] = initialSetting(nRobot,nStation)
% initial을 위해 선언해주는 함수이다. 

% robotSet info : (:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = working state  ---> 놀면 0, pallet 가면 0.5, 
%                                             station 가면 2, warehouse 가면 3
%                 (:,5) = allocated task ---> 0이 아닐때 가고 있는 node의 번호
%                                            (0x(일의 자리):pallet,1x(십의 자리):station,10x(백의 자리):warehouse)
%                 (:,6) = angle
%                 (:,7) = number of bundle now 
% pallet load time 
palletLUtime = 5;
% warehouse unload time
warehouseLUtime = 6;
% station time setting
% station에서 얼마나 걸리는지는 여기서 설정할 것.
stationLUtime = [];
% for i=1:nStation
%     stationLUtime = [stationLUtime 30];
% end
stationLUtime = [30 30 40 50 60 70];

robotX = (1:nRobot)'+1;
robotY = (ones(1,nRobot))'+1;
robotXY = [robotX, robotY];
robotSet = [(1:nRobot)' robotXY zeros([nRobot 4])];
for i =1:nRobot
    robotSet(i,6) = pi/2;
end
for i=1:nRobot
    robotTasks{i} = [];
    robotTasksRecord{i} = [];
    robotPaths{i} = [];
    robotDistance{i} = [];
end

taskQueue = [];
taskQueue_all = [];
