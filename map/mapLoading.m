function [gridMap,stationXY,palletXY,warehouseXY] = mapLoading(mapfilename)
% Excel Map �ε��ϴ� �Լ��̴�. ������� ������ �ݿ����� ��.
map_sheet = 'Map';
station_sheet = 'Station';
pallet_sheet = 'Pallet';
warehouse_sheet = 'Warehouse';

gridMap = readmatrix(mapfilename);
stationXY = readmatrix(mapfilename,'Sheet',station_sheet,'Range','A3:E8');
palletXY = readmatrix(mapfilename,'Sheet',pallet_sheet,'Range','A3:E4');
warehouseXY = readmatrix(mapfilename,'Sheet',warehouse_sheet,'Range','A3:E4');

% �Ʒ��� ���� �����Ͽ����� �����Ұ�.

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