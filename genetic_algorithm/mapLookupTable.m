map_sheet = 'Map';
task_sheet = 'Task';
station_sheet = 'Station';

gridMap = readmatrix('Map.xlsx');
taskXY = readmatrix('Map.xlsx','Sheet',task_sheet,'Range','A3:D30');
stationXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','B3:C4');
palletXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','F3:H4');

xSize = length(gridMap(1,:));
ySize = length(gridMap(:,1));

%% robot To task
for t = 1:length(taskXY(:,1))
    for x = 1:xSize
        for y = 1:ySize
            if gridMap(x,y) == 0
                startXY = [x,y];
                goalXY = taskXY(t,2:3);
                robotToTask{x,y,t} = flipud(astar(startXY, goalXY, ~gridMap));
            else
                robotToTask{x,y} = 0;
            end
        end
    end
end

%% robot To Station
for t = 1:length(stationXY(:,1))
    for x = 1:xSize
        for y = 1:ySize
            if gridMap(x,y) == 0
                startXY = [x,y];
                goalXY = stationXY(t,1:2);
                robotToStation{x,y,t} = flipud(astar(startXY, goalXY, ~gridMap));
            else
                robotToStation{x,y} = 0;
            end
        end
    end
end

%% taskToStation 
for t = 1:length(taskXY(:,1))
    for s = 1:length(stationXY(:,1))
        startXY = taskXY(t,2:3);
        goalXY = stationXY(s,1:2);
        taskToStation{t,s} = flipud(astar(startXY, goalXY, ~gridMap));
    end
end

%% stationToTask 
for s = 1:length(stationXY(:,1))
    for t = 1:length(taskXY(:,1))
        startXY = stationXY(s,1:2);
        goalXY = taskXY(t,2:3);
        stationToTask{s,t} = flipud(astar(startXY, goalXY, ~gridMap));
    end
end

save('mapGeneration');