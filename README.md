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



assumed that it is a factory environment. 
The environment can be customized in the map folder. 
AGV aims to transport a lot of luggage for a set time. 
The number of luggage and the order in which they occur are determined and appropriately designated by the user. 
Mfile:Basic, comsenvironment, etc. are based on the greedy algorithm. 
The greedy algorithm moves in search of the closest distance goal. 
Hillclimb finds a faster order of work in the way he proposed in my conference paper. 
Genetic uses genetic algorithms to find faster work order. 
because of the loading speed is slow in genetic, using the 'Mfile:withtable' is faster to use the distance saved as a lookup table.


투고 학술지 : https://www.dbpia.co.kr/pdf/pdfView.do?nodeId=NODE10609258

레퍼런스 : https://github.com/KistCloudRobot/multirobot




![공장환경그리드맵ver2](https://user-images.githubusercontent.com/51067104/145567676-a97d50f7-cf74-4fdb-9491-c8a083213774.PNG)


![task set](https://user-images.githubusercontent.com/51067104/145567654-aca7acda-ed58-4161-9945-ea7695bdb7b5.PNG)


![taskscheduling](https://user-images.githubusercontent.com/51067104/145568224-0ae243c3-43ac-4cf6-8b14-c44e789a7b0b.gif)




![GA결과_Greedy_3R_2per10T_300t_1b_50pop_200iter](https://user-images.githubusercontent.com/51067104/145567287-1305ff07-1e94-4415-b508-4dd2b30f4057.png)
GA test, 3 robot, total 300s, 2 per 10 task, 50 population, 200 iteration, first population : greedy's output



![GA결과_Random_3R_2per10T_300t_1b_150pop_435iter](https://user-images.githubusercontent.com/51067104/145567399-15cc5a40-5b71-47ae-b58c-c0d6de11ae0b.png)
GA tset, 3 robot, total 300s, 2 per 10 task, 150 population, 435 iteration, first population : random
