function [robotSet_string] = robotStatusString(robotSet,robotSet_string)
switch nargin
    case 2
        nRobot = length(robotSet(:,1));
    case 1
        nRobot = length(robotSet(:,1));
        robotSet_string = {};
        for i = 1:nRobot
            robotSet_string = [robotSet_string; {0}];
        end
    otherwise
        disp('입력 인수를 알맞게 넣으세요. (robotSet or robotSet,robotSet_string)');
end


status1 = ("Going Pallet "); 
status2 = ("Going Station ");
status3 = ("Going Warehouse ");
status4 = ("Loading Pallet "); 
status5 = ("UnLoading Pallet "); 
status6 = ("Loading Bundle ");
status7 = ("UnLoading Bundle ");
status8 = ("Stand-by ");


for i = 1:nRobot
    if robotSet(i,4)==0
        robotSet_string{i} = status8;
    elseif robotSet(i,4)==0.5
        robotSet_string{i} = status1;
    elseif robotSet(i,4)==1
        robotSet_string{i} = status4;
    elseif robotSet(i,4)==1.5
        robotSet_string{i} = status2;
    elseif robotSet(i,4)==2
        robotSet_string{i} = status5;
    elseif robotSet(i,4)==2.5
        robotSet_string{i} = status3;
    end
end


