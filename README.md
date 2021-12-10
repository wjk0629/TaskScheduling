# TaskScheduling
공장 환경이라 가정.
환경은 맵 폴더에서 커스텀 할 수 있다.
AGV는 정해진 시간 동안 많은 짐을 이송하는 것을 목표로 한다.
짐의 수와 생기는 순서는 정해져있으며 사용자가 적절하게 지정한다.
basic, comsenvironment 등은 greedy 알고리즘을 기반으로 한다.
greedy 알고리즘은 가장 가까운 거리의 목표를 찾아 움직인다.
hillclimb은 본인이 학술지에서 제안한 방법으로 더 빠른 작업 순서를 찾는다.
genetic은 유전알고리즘을 사용하여 더 빠른 작업 순서를 찾는다. 
다만 로딩 속도가 느리기에 withtable을 사용하면 lookup table로 저장한 거리를 사용하기에
더 빠르다.


투고 학술지 : https://www.dbpia.co.kr/pdf/pdfView.do?nodeId=NODE10609258
레퍼런스 : https://github.com/KistCloudRobot/multirobot

assumed that it is a factory environment. 
The environment can be customized in the map folder. 
AGV aims to transport a lot of luggage for a set time. 
The number of luggage and the order in which they occur are determined and appropriately designated by the user. 
Mfile:Basic, comsenvironment, etc. are based on the greedy algorithm. 
The greedy algorithm moves in search of the closest distance goal. 
Hillclimb finds a faster order of work in the way he proposed in my conference paper. 
Genetic uses genetic algorithms to find faster work order. 
because of the loading speed is slow in genetic, using the 'Mfile:withtable' is faster to use the distance saved as a lookup table.
