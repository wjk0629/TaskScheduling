function [PathTable,PathTable_Length] = pathMatrixByAstar(nSize,mSize,nXY,mXY,gridMap)
% 고정된 Node들 사이의 path는 미리 만들어 준다.
for i = 1:nSize
    for j = 1:mSize
        startXY = nXY(i,2:3);
        goalXY = mXY(j,2:3);
        PathNow = flipud(astar(startXY, goalXY, ~gridMap));
        PathTable{i,j} = PathNow;
        PathTable_Length(i,j) = length(PathNow);
    end
end