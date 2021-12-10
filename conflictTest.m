% case 1-1
robotPaths{1} = [6 -1; 6 0; 6 1; 6 2; 5 2; 4 2; 3 2; 2 2; 2 3]; 
robotPaths{2} = [1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1;]; % 13°³
robotPaths{3} = [2 -3; 2 -2; 2 -1; 2 0; 2 1; 2 2; 3 2; 4 2; 5 2; 6 2; 6 3]; 

% case 2-1
robotPaths{1} = [5 2; 4 2; 3 2; 2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 3 7]; 
robotPaths{2} = [-3 7;-2 7;-1 7;0 7; 1 7; 2 7; 2 6; 2 5; 2 4; 2 3; 2 2; 1 2; 1 3; 1 4;];
robotPaths{3} = [1 1; 1 1; 1 1]; 


% case 2-2
robotPaths{1} = [7 1; 7 2; 6 2; 5 2; 4 2; 3 2; 2 2; 2 3]; 
robotPaths{2} = [2 1; 2 2; 3 2; 4 2; 5 2; 6 2; 7 2; 7 3];
robotPaths{3} = [1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1; 1 1]; 


for j = 1:length(robotPaths{1}(:,1))-1
            egoRobotVelo(j,:) = robotPaths{1}(j+1,:) - robotPaths{1}(j,:);
            otherRobotVelo(j,:) = robotPaths{2}(j+1,:) - robotPaths{2}(j,:);
            
end