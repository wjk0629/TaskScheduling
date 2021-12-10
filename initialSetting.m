function [palletLUtime,warehouseLUtime,stationLUtime, ...
    robotSet,robotTasks,robotTasksRecord,robotPaths,robotDistance, ...
    taskQueue,taskQueue_all] = initialSetting(nRobot,nStation)
% initial�� ���� �������ִ� �Լ��̴�. 

% robotSet info : (:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = working state  ---> ��� 0, pallet ���� 0.5, 
%                                             station ���� 2, warehouse ���� 3
%                 (:,5) = allocated task ---> 0�� �ƴҶ� ���� �ִ� node�� ��ȣ
%                                            (0x(���� �ڸ�):pallet,1x(���� �ڸ�):station,10x(���� �ڸ�):warehouse)
%                 (:,6) = angle
%                 (:,7) = number of bundle now 
% pallet load time 
palletLUtime = 5;
% warehouse unload time
warehouseLUtime = 6;
% station time setting
% station���� �󸶳� �ɸ������� ���⼭ ������ ��.
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
