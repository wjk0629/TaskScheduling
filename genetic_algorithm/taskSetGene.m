function [robotTaskSeq_new] = taskSetGene(taskSet,robotTaskSeq_set)
% robotTaskSeq´Â cell Çü
robotTaskSeq_set = robotTaskSeq_set(:,1);

for i = 1:length(robotTaskSeq_set(:,1))
    robotTaskSeq_temp = robotTaskSeq_set{i,:}';
    robotTaskSeq_temp2 = [];
    for j = 1:length(robotTaskSeq_temp)
        robotTaskSeq_temp2 = [robotTaskSeq_temp2, taskSet(robotTaskSeq_temp(j)==taskSet(:,2),3)];
    end
    robotTaskSeq_second{i,1} = robotTaskSeq_temp2';
end


robotTaskSeq_new = [robotTaskSeq_set, robotTaskSeq_second];

end