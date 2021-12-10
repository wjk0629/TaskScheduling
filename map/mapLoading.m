function [gridMap,stationXY,palletXY,warehouseXY] = mapLoading(mapfilename)
% Excel Map 로드하는 함수이다. 변경사항 있으면 반영해줄 것.
map_sheet = 'Map';
station_sheet = 'Station';
pallet_sheet = 'Pallet';
warehouse_sheet = 'Warehouse';

gridMap = readmatrix(mapfilename);
stationXY = readmatrix(mapfilename,'Sheet',station_sheet,'Range','A3:E8');
palletXY = readmatrix(mapfilename,'Sheet',pallet_sheet,'Range','A3:E4');
warehouseXY = readmatrix(mapfilename,'Sheet',warehouse_sheet,'Range','A3:E4');

% 아래와 같이 설정하였으니 참고할것.

% stationXY info :(:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = allocated robot number (0 -> empty)
%                 (:,5) = remain task time (0 -> not in work)

% palletXY info : (:,1) = number
%                 (:,2) = x location
%                 (:,3) = y location
%                 (:,4) = allocated robot number (0 -> empty)
%                 (:,5) = number of robot visits 

% warehouseXY info : (:,1) = number
%                    (:,2) = x location
%                    (:,3) = y location
%                    (:,4) = allocated robot number (0 -> empty)
%                    (:,5) = number of robot visits 

end