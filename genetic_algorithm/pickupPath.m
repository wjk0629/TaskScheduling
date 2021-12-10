function [path] = pickupPath(startXY,goalXY,lookupTable,first_index,second_index)
% robotToTask
if first_index == 1
    % second_index는 task의 고유번호
    x = startXY(1);
    y = startXY(2);
    path_cell = lookupTable(x,y,second_index);
    path = path_cell{1};
% robotToStation    
elseif first_index == 2
    % second_index는 station의 번호
    x = startXY(1);
    y = startXY(2);
    path_cell = lookupTable(x,y,second_index);
    path = path_cell{1};    
% taskToStation 
elseif first_index == 3
    x = startXY(1);
    y = startXY(2);
    path_cell = lookupTable(x,y,second_index);
    path = path_cell{1};      
end


