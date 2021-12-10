%% Initialization
clc;clear;close all

map_sheet = 'Map';
task_sheet = 'Task';
station_sheet = 'Station';

gridMap = readmatrix('Map.xlsx');
taskXY = readmatrix('Map.xlsx','Sheet',task_sheet,'Range','A3:D30');
stationXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','B3:C4');
palletXY = readmatrix('Map.xlsx','Sheet',station_sheet,'Range','F3:H4');

pop_size = 101; % 원하는 수 + 1해야됨 나중에 수정할 것
nRobot = 3;
numTaskPerStep = 8;
alpha = 30; % task가 오는 텀
tMax = 250;
bSize = 1;


Pcromo = 0.25;
Pmutant = 0.2;
iteration = 400;
iteration_count = 0;

load('mapGeneration');
Forplotdata_x = [];
Forplotdata_y = [];
robotTaskSeq_best_now = [];


% make taskSet
[taskSet,robotTaskSeq_origin,output_origin] = tasksetting_withtable(nRobot,numTaskPerStep,alpha,tMax,gridMap,bSize);

%% 1. random initial
robotTaskSeq_initial = robotTaskSeq_origin;
[output_origin,output_set,robotTaskSeq_set] = geneticInitial_withtable(taskSet,robotTaskSeq_initial,output_origin,pop_size,gridMap,taskXY,stationXY,palletXY);
output_focus_now = output_set(1,:);

iteration_count = iteration_count + 1;
Forplotdata_x = [Forplotdata_x, iteration_count];
Forplotdata_y = [Forplotdata_y, max(output_focus_now)];
%% 2. use greedy for initial
% output_set = [];
% for i = 1:1:pop_size-1
%     for j = nRobot:-1:1
%         origin_length(nRobot-j+1) = length(robotTaskSeq_origin{nRobot-j+1});
%     end
%     origin_sum = sum(origin_length);
%     if origin_sum < length(taskSet(:,1))
%         origin_index = origin_sum + 1;
%         origin_index2 = randperm(nRobot,1);
%         robotTaskSeq_origin{origin_index2} = [robotTaskSeq_origin{origin_index2}; origin_index]; 
%     end
%     output_set = [output_set, output_origin];
% end
% for i = 1:1:pop_size-1
%     for j = nRobot:-1:1
%         robotTaskSeq_set{i*nRobot-j+1,1} = robotTaskSeq_origin{nRobot-j+1};
%     end
% end
% robotTaskSeq_set = taskSetGene(taskSet,robotTaskSeq_set);
% iteration_count = iteration_count + 1;
% Forplotdata_x = [Forplotdata_x, iteration_count];
% Forplotdata_y = [Forplotdata_y, max(output_focus_now)];

%% start iteration
while iteration_count < iteration
    
    
    %% random selection
    % total fit (무엇을 고려할까? 일단 test는 작을수록 좋은것이므로 역수 취함)
    fitness_bundle = sum(output_set(1,:));
    fitness_time = sum(output_set(7,:));
    fitness_mytest = 1./output_set(4,:);
    fitness_test = sum(fitness_mytest);

    
    % 복제 수량의 확률
    output_proba_bTemp = output_set(1,:)/fitness_bundle;
    output_proba_tTemp = output_set(7,:)/fitness_time;
    output_proba_ttTemp = fitness_mytest/fitness_test;
    
    % !!! 무엇을 쓸까?
    
    output_proba_now = output_proba_bTemp;

    %% Remainder stochastic sampling with replacement
    output_proba = output_proba_now.*(pop_size-1); % 복제 수량의 기대치
    output_proba = round(output_proba,6); % 전처리
    output_proba2 = floor(output_proba); % 정수부 만큼 내림
    robotTaskSeq_select = [];
    output_set_roullet_index = [];
    for i = 1:length(output_proba)
        if output_proba2(i) >= 1 % 정수부 숫자만큼
            for j = 1:output_proba2(i) % 고놈은 가져온다
                robotTaskSeq_select = [robotTaskSeq_select; robotTaskSeq_set(nRobot*i-(nRobot-1):nRobot*i,:)];
                robotTaskSeq_select_index(i) = i; % 몇번이 선택되어 다음에 살아남았는지 표시
            end
        else
            robotTaskSeq_select_index(i) = 0; % 몇번이 룰렛으로 넘어갈건지 표시
            output_set_roullet_index = [output_set_roullet_index, i];
        end
    end
    if ~isempty(output_set_roullet_index) % 모두 정수부일때(모두 같을때) 방지용 if문
        output_set_roullet = output_set(:,output_set_roullet_index);
        % 이제 남은걸로 Random Selection과 동일하게 룰렛 진행
        output_proba_index = 0;
        output_proba_index2 = 0;
        output_proba_index3 = 0;
        fitness_bundle = sum(output_set_roullet(1,:));
        fitness_time = sum(output_set_roullet(7,:));
        fitness_mytest = 1./output_set_roullet(4,:);
        fitness_test = sum(fitness_mytest);
        % probability of selection
        output_proba_bTemp = output_set_roullet(1,:)/fitness_bundle;
        output_proba_tTemp = output_set_roullet(7,:)/fitness_time;
        output_proba_ttTemp = fitness_mytest/fitness_test;

        for i = 1:length(output_set_roullet(1,:))
            output_proba_bundle(1,i) = output_proba_index + output_proba_bTemp(1,i);
            output_proba_index = output_proba_index + output_proba_bTemp(1,i);
            output_proba_time(1,i) = output_proba_index2 + output_proba_tTemp(1,i);
            output_proba_index2 = output_proba_index2 + output_proba_tTemp(1,i);
            output_proba_test(1,i) = output_proba_index3 + output_proba_ttTemp(1,i);
            output_proba_index3 = output_proba_index3 + output_proba_ttTemp(1,i);
        end
        
        % 뭐 골라줄까? 뒷부분 바꿔주기
        output_proba_now = [0, output_proba_bundle];
        % random number generation
        randnum_gene = rand(1,length(output_set_roullet(1,:)));
        % selection
        robotTaskSeq_roullet = [];
        for i = 1:length(output_set_roullet(1,:))
            robotTaskSeq_roullet = [robotTaskSeq_roullet; robotTaskSeq_set(nRobot*output_set_roullet_index(i)-(nRobot-1):nRobot*output_set_roullet_index(i),:)];
        end
        for i = 1:length(output_set_roullet(1,:))
            for j = 1:length(output_set_roullet(1,:))
                if randnum_gene(i) > output_proba_now(j) && randnum_gene(i) <= output_proba_now(j+1)
                    robotTaskSeq_select = [robotTaskSeq_select; robotTaskSeq_roullet(nRobot*j-(nRobot-1):nRobot*j,:)];
                    robotTaskSeq_select_roullet_index(i) = j; % 몇번이 선택되어 다음에 살아남았는지 표시
                end
            end
        end
    end
   
    %% Roullet Selection
%     output_proba_index = 0;
%     output_proba_index2 = 0;
%     output_proba_index3 = 0;
%     for i = 1:pop_size-1
%         output_proba_bundle(1,i) = output_proba_index + output_proba_bTemp(1,i);
%         output_proba_index = output_proba_index + output_proba_bTemp(1,i);
%         output_proba_time(1,i) = output_proba_index2 + output_proba_tTemp(1,i);
%         output_proba_index2 = output_proba_index2 + output_proba_tTemp(1,i);
%         output_proba_test(1,i) = output_proba_index3 + output_proba_ttTemp(1,i);
%         output_proba_index3 = output_proba_index3 + output_proba_ttTemp(1,i);
%     end
%     output_proba_test = [0, output_proba_test];
%     % random number generation
%     randnum_gene = rand(1,length(output_set(1,:)));
%     % selection
%     robotTaskSeq_select = [];
%     for i = 1:pop_size-1
%         for j = 1:pop_size-1
%             if randnum_gene(i) > output_proba_test(j) && randnum_gene(i) <= output_proba_test(j+1)
%                 robotTaskSeq_select = [robotTaskSeq_select; robotTaskSeq_set(nRobot*j-(nRobot-1):nRobot*j,:)];
%                 robotTaskSeq_select_index(i) = j; % 몇번이 선택되어 다음에 살아남았는지 표시
%             end
%         end
%     end
    
    
    %% crossover
    % chromosome 선택 ( 짝수로만 선택하게 함, 두개가 같아도 다시 선택 )
    chromosome_index = 1;
    chromosome_count = 0;
    while mod(length(chromosome_index),2) ~= 0
        randnum_gene = rand(1,length(output_set(1,:)));
        chromosome_index = [];
        for i = 1:pop_size-1
            if randnum_gene(i) < Pcromo
                chromosome_index = [chromosome_index, i];
            end
        end
    end
    if isempty(chromosome_index)
        disp("warning! there is no crossover (random chromosome not selected)");
    else
        % chromosome Pair 결정, crossover
        chromosome_pair_index = randperm(length(chromosome_index)); % pair 정하는것은 랜덤하게
        for i = 1:length(chromosome_index)/2
            % 어떻게 할지 방법이 무궁무진하네요.
            % 일단 내가 정한것은 randperm(nRobot)하여 로봇 번호 한개 정하고 두 작업순서 바꾼다
            % 그럼 각 작업순서에서 다른 로봇이 겹치는거 없게 정리해준다.
            robotTaskSeq_chro_selected1 = {};
            robotTaskSeq_chro_selected2 = {};
            for j = nRobot:-1:1
                robotTaskSeq_chro_selected1{nRobot-j+1} = robotTaskSeq_select{chromosome_index(chromosome_pair_index(2*i-1))*nRobot-j+1, 1};
                robotTaskSeq_chro_selected2{nRobot-j+1} = robotTaskSeq_select{chromosome_index(chromosome_pair_index(2*i))*nRobot-j+1, 1};
            end
            robotTaskSeq_chro_selected1 = robotTaskSeq_chro_selected1';
            robotTaskSeq_chro_selected2 = robotTaskSeq_chro_selected2';
            chromosome_pair_index2 = randperm(nRobot,1); % pair로 정해진 쌍의 몇번 로봇을 서로 바꿀지도 랜덤하게
            
            if  isequal(robotTaskSeq_chro_selected1{chromosome_pair_index2},robotTaskSeq_chro_selected2{chromosome_pair_index2}) == 1
                fprintf("%d robot of (%d chromo and %d chromo) were selected but pair is same(no crossover).\n",chromosome_pair_index2,chromosome_pair_index(2*i-1),chromosome_pair_index(2*i));
                chromosome_count = chromosome_count + 1;
                if chromosome_count == length(chromosome_index)/2
                    chromosome_index = [];
                    break;
                end
            else
                % 바꿔주는 과정
                robotTaskSeq_chro_temp = robotTaskSeq_chro_selected1;
                robotTaskSeq_chro_temp2 = robotTaskSeq_chro_selected2;
                robotTaskSeq_chro_temp(chromosome_pair_index2,:) = robotTaskSeq_chro_selected2(chromosome_pair_index2,:);
                robotTaskSeq_chro_temp2(chromosome_pair_index2,:) = robotTaskSeq_chro_selected1(chromosome_pair_index2,:);
                
                
                [C1,ia1,ib1] = intersect(robotTaskSeq_chro_selected1{chromosome_pair_index2}',robotTaskSeq_chro_selected2{chromosome_pair_index2}');
                
                chromosome_arrange1 = 1:length(taskSet(:,1));
                chromosome_arrange1(robotTaskSeq_chro_temp{chromosome_pair_index2}') = [];
                chromosome_arrange2 = 1:length(taskSet(:,1));
                chromosome_arrange2(robotTaskSeq_chro_temp2{chromosome_pair_index2}') = [];
                
                for j = 1:nRobot
                    
                    if j ~= chromosome_pair_index2
                        [C2,ia2,ib2] = intersect(robotTaskSeq_chro_temp{j}',robotTaskSeq_chro_temp{chromosome_pair_index2}');
                        robotTaskSeq_chro_intersect1 = robotTaskSeq_chro_temp{j}';
                        robotTaskSeq_chro_intersect1(ia2) = [];
                        for k = 1:length(robotTaskSeq_chro_intersect1)
                            chromosome_arrange1(find(chromosome_arrange1==robotTaskSeq_chro_intersect1(k))) = [];
                        end
                    end
                    
                    if j ~= chromosome_pair_index2
                        [C3,ia3,ib3] = intersect(robotTaskSeq_chro_temp2{j}',robotTaskSeq_chro_temp2{chromosome_pair_index2}');
                        robotTaskSeq_chro_intersect2 = robotTaskSeq_chro_temp2{j}';
                        robotTaskSeq_chro_intersect2(ia3) = [];
                        for k = 1:length(robotTaskSeq_chro_intersect2)
                            chromosome_arrange2(find(chromosome_arrange2==robotTaskSeq_chro_intersect2(k))) = [];
                        end
                    end
                end
                chromosome_trash_index = randperm(nRobot-1,1);
                for j = 1:nRobot
                    if j ~= chromosome_pair_index2
                        [C2,ia2,ib2] = intersect(robotTaskSeq_chro_temp{j}',robotTaskSeq_chro_temp{chromosome_pair_index2}');
                        if length(C2) <= length(chromosome_arrange1)
                            robotTaskSeq_chro_temp{j}(ia2) = chromosome_arrange1(1:length(C2))';
                            chromosome_arrange1(1:length(C2)) = [];
                            if ~isempty(chromosome_arrange1) && j == nRobot
                                robotTaskSeq_chro_temp{chromosome_trash_index} = [robotTaskSeq_chro_temp{chromosome_trash_index}; chromosome_arrange1'];
                            end
                        elseif length(C2) > length(chromosome_arrange1)
                            robotTaskSeq_chro_temp{j}(ia2(1:length(chromosome_arrange1))) = chromosome_arrange1';
                            robotTaskSeq_chro_temp{j}(ia2(length(chromosome_arrange1)+1:end)) = [];
                        end
                    end
                    if j ~= chromosome_pair_index2
                        [C3,ia3,ib3] = intersect(robotTaskSeq_chro_temp2{j}',robotTaskSeq_chro_temp2{chromosome_pair_index2}');
                        if length(C3) <= length(chromosome_arrange2)
                            robotTaskSeq_chro_temp2{j}(ia3) = chromosome_arrange2(1:length(C3))';
                            chromosome_arrange2(1:length(C3)) = [];
                            if ~isempty(chromosome_arrange2) && j == nRobot
                                robotTaskSeq_chro_temp2{chromosome_trash_index} = [robotTaskSeq_chro_temp2{chromosome_trash_index}; chromosome_arrange2'];
                            end
                        elseif length(C3) > length(chromosome_arrange2)
                            robotTaskSeq_chro_temp2{j}(ia3(1:length(chromosome_arrange2))) = chromosome_arrange2';
                            robotTaskSeq_chro_temp2{j}(ia3(length(chromosome_arrange2)+1:end)) = [];
                        end
                    end
                end
                
                robotTaskSeq_cross = robotTaskSeq_select;
                
                for j = nRobot:-1:1
                    robotTaskSeq_cross{chromosome_index(chromosome_pair_index(2*i-1))*nRobot-j+1, 1} = robotTaskSeq_chro_temp{nRobot-j+1};
                    robotTaskSeq_cross{chromosome_index(chromosome_pair_index(2*i))*nRobot-j+1, 1} = robotTaskSeq_chro_temp2{nRobot-j+1};
                end
            end
        end
    end
    
    
    
    %% mutation
    mutation_num = 2; % mutation하는 로봇 수
    randnum_gene = rand(1,length(output_set(1,:))); % Pm에 따라 pop_size 중 골라줌
    mutation_index = [];
    for i = 1:pop_size-1
        if randnum_gene(i) < Pmutant
            mutation_index = [mutation_index, i];
        end
    end
    if isempty(mutation_index)
        disp("warning! there is no mutation (random mutation not selected)");
    end
    mutation_pair_index = randperm(nRobot,mutation_num); % 고른 것중에 서로 교환할 로봇 골라줌
    for i = 1:length(mutation_index)
        if isempty(chromosome_index)
            for j = nRobot:-1:1
                robotTaskSeq_muta_selected{nRobot-j+1} = robotTaskSeq_select{mutation_index(i)*nRobot-j+1, 1};
                robotTaskSeq_cross = robotTaskSeq_select;
            end
        else
            for j = nRobot:-1:1
                robotTaskSeq_muta_selected{nRobot-j+1} = robotTaskSeq_cross{mutation_index(i)*nRobot-j+1, 1};
            end
        end
    
        % 바꿔주는 과정은 3개로 나뉘는데 이것도 랜덤하게 선택하자
        mutation_method = randperm(3,1);
        robotTaskSeq_muta_temp = robotTaskSeq_muta_selected'; % origin 저장
        robotTaskSeq_muta_temp1 = robotTaskSeq_muta_temp(mutation_pair_index(1),:);
        robotTaskSeq_muta_temp2 = robotTaskSeq_muta_temp(mutation_pair_index(2),:); 
        
        if mutation_method == 1
            % 과정1. one-to-one mutation 선택된 pair의 작업을 랜덤하게 한개씩 교환
            mutation_changed_task_index = randperm(length(robotTaskSeq_muta_temp1{1}),1); % 몇번째 task 인가
            mutation_changed_task = robotTaskSeq_muta_temp1{1}(mutation_changed_task_index); % 바뀐 task
            mutation_changed_task_index2 = randperm(length(robotTaskSeq_muta_temp2{1}),1); % 몇번째 task 인가
            mutation_changed_task2 = robotTaskSeq_muta_temp2{1}(mutation_changed_task_index2); % 바뀐 task
            robotTaskSeq_muta_selected{mutation_pair_index(1)}(mutation_changed_task_index) = mutation_changed_task2;
            robotTaskSeq_muta_selected{mutation_pair_index(2)}(mutation_changed_task_index2) = mutation_changed_task;
        elseif mutation_method == 2
            % 과정2. one-to-two mutation 선택된 pair의 작업 중 한 로봇의 2개, 다른 로봇의 1개를 교환 (길이 변경됨)
            mutation_changed_task_index = randperm(length(robotTaskSeq_muta_temp1{1}),1); % 몇번째 task 인가
            if mutation_changed_task_index ~= 1
                mutation_changed_task_index =  [mutation_changed_task_index-1, mutation_changed_task_index];
            else
                mutation_changed_task_index =  [mutation_changed_task_index, mutation_changed_task_index+1];
            end
            mutation_changed_task = robotTaskSeq_muta_temp1{1}(mutation_changed_task_index)'; % 바뀐 task (2개)
            mutation_changed_task_index2 = randperm(length(robotTaskSeq_muta_temp2{1}),1); % 몇번째 task 인가
            mutation_changed_task2 = robotTaskSeq_muta_temp2{1}(mutation_changed_task_index2); % 바뀐 task
            robotTaskSeq_muta_selected{mutation_pair_index(1)}(mutation_changed_task_index) = [];
            robotTaskSeq_muta_selected{mutation_pair_index(1)} = [robotTaskSeq_muta_temp1{1}(1:mutation_changed_task_index(1)-1); mutation_changed_task2'; robotTaskSeq_muta_temp1{1}(mutation_changed_task_index(2)+1:end)];
            robotTaskSeq_muta_selected{mutation_pair_index(2)}(mutation_changed_task_index2) = [];
            robotTaskSeq_muta_selected{mutation_pair_index(2)} = [robotTaskSeq_muta_temp2{1}(1:mutation_changed_task_index2-1); mutation_changed_task'; robotTaskSeq_muta_temp2{1}(mutation_changed_task_index2+1:end)];
        elseif mutation_method == 3
            % 과정3. migration mutation 선택된 pair의 작업 중 한 로봇의 1개를 다른 로봇에게 전달 (길이 변경됨)
            mutation_changed_task_index = randperm(length(robotTaskSeq_muta_temp1{1}),1); % 몇번째 task 인가
            mutation_changed_task = robotTaskSeq_muta_temp1{1}(mutation_changed_task_index)'; % 건너갈 task 
            mutation_changed_task_index2 = randperm(length(robotTaskSeq_muta_temp2{1}),1); % 몇번째 task 뒤에 건너오는가
            mutation_changed_task2 = robotTaskSeq_muta_temp2{1}(mutation_changed_task_index2); % 건너오는 task 바로 앞 task
            robotTaskSeq_muta_selected{mutation_pair_index(1)}(mutation_changed_task_index) = [];
            robotTaskSeq_muta_selected{mutation_pair_index(2)} = [robotTaskSeq_muta_temp2{1}(1:mutation_changed_task_index2-1); mutation_changed_task; robotTaskSeq_muta_temp2{1}(mutation_changed_task_index2:end)];
        end
        robotTaskSeq_mutation = robotTaskSeq_cross;
        for j = nRobot:-1:1
            robotTaskSeq_mutation{mutation_index(i)*nRobot-j+1,1} = robotTaskSeq_muta_selected{nRobot-j+1};
        end
    end
 
    %% evaluation
    % robotTaskSeq_set = taskSetGene(taskSet,robotTaskSeq_cross);
    % [output_set] = geneticFitness(taskSet,robotTaskSeq_set,pop_size,gridMap,taskXY,stationXY);
    if isempty(mutation_index)
        robotTaskSeq_set = taskSetGene(taskSet,robotTaskSeq_select);
        [output_set] = geneticFitness_withtable(nRobot,numTaskPerStep,tMax,taskSet,robotTaskSeq_set,pop_size,gridMap,taskXY,stationXY);
    else
        robotTaskSeq_set = taskSetGene(taskSet,robotTaskSeq_mutation);
        [output_set] = geneticFitness_withtable(nRobot,numTaskPerStep,tMax,taskSet,robotTaskSeq_set,pop_size,gridMap,taskXY,stationXY);
    end
    
    % 한 iteration 중 가장 좋은 놈 저장
    robotTaskSeq_best_now = [robotTaskSeq_best_now; robotTaskSeq_set(nRobot*find(max(output_focus_now)==output_focus_now)-(nRobot-1):nRobot*find(max(output_focus_now)==output_focus_now),:)];
  
    % 모든 iteration 중 가장 좋은 놈 저장
    if max(output_focus_now) < max(Forplotdata_y)
        robotTaskSeq_best_total = robotTaskSeq_set(nRobot*find(max(output_focus_now)==output_focus_now)-(nRobot-1):nRobot*find(max(output_focus_now)==output_focus_now),:);
    end
    fprintf(" %d iter was completed !!\n\n",iteration_count);
    iteration_count = iteration_count + 1;
    Forplotdata_x = [Forplotdata_x, iteration_count];
    Forplotdata_y = [Forplotdata_y, max(output_focus_now)];
    
end