function taskXYs = task_arrival_XY_jk(taskXY,numTask, probArrival)
taskXYs = [];
selectedTaskQue = [];
selectedTaskIdx = 1:length(taskXY(:,1));
selectedTaskIdx_temp = selectedTaskIdx;
for i=1:length(taskXY(:,1))
    if taskXY(i,4) == -1
        selectedTaskIdx(i) = 0;
    else
        selectedTaskIdx(i) = i;
    end
end
loopIdx = 0;
for i=1:numTask
    while loopIdx < i
        if probArrival >= rand
            selectedTask = randi(length(taskXY(:,1)));
            if selectedTaskIdx(selectedTask) ~= 0
                taskXYs = [taskXYs; taskXY(selectedTask,1:3), -1];
                selectedTaskIdx(selectedTask) = 0;
                loopIdx = loopIdx + 1;
            end
            
            if sum(selectedTaskIdx,'all') == 0
                    break;
            end
                
        end
    end
end
end