%% Implementation of all Algorithms
clc

myObj = benchmark_comparisonsOTR;

%Initialize Parameters

rng(42)
s = rng;
%Total number of instances
K = 200;

%Number of roads
M = 3;
    
%Number of people
N = 120;

%Capacities of all roads
C = [20, 24, 120];
    
%Cost incurred in using road
S = [20, 24, 130];

%Arrival Rate
lambda_tot = [5.25, 5.25, 5.25, 5.25, 5.25];

%Generate random number for Theta customer types
theta = 3;
key_vals = {1, 2, 3, 4};
urgency_types = {1, 9, 20, 400};
mapping = containers.Map(key_vals, urgency_types);

%Set Batch size
%batch_size = 5;

%Set Lookahead size
%num_lookahead = 5;

greedy_comp = [];
greedy_comp_dist = [];
k_greedy_comp = [];
k_greedy_comp_dist = [];
k_greedy_comp_sc = [];
k_greedy_comp_dist_sc = [];
k_rolling_comp = [];
k_rolling_comp_dist = [];
comp_ratio_vals = [];
comp_ratio_vals_dist = [];
comp_ratio_vals_test = [];
comp_ratio_vals_new = [];
comp_ratio_vals_new_dist = [];
comp_ratio_vals_new_test = [];
total_spt_classic = [];
total_spt_cplx = [];
total_spt_classic_alpha = [];
total_spt_cplx_alpha = [];
num_violations_arr_frac = [];
comp_ratio_vals_dist_frac = [];
comp_ratio_vals_frac = [];
total_support_frac_alpha = [];
total_support_frac_classic = [];
num_violations_arr_frac_cplx = [];
total_support_frac_cplx = [];
total_support_frac_alpha_cplx = [];
comp_ratio_vals_dist_frac_cplx = [];
comp_ratio_vals_frac_cplx = [];

rng(42)
s = rng;

OPT_vec = [];

%Here customers have a poisson arrival rate with mean lambda_
lambda_ = lambda_tot(1);

%Store properties of instance, i.e., time of arrival, urgency
urgency_glob = [];
time_arrival_glob = [];

%Iterate over instances to determine OPT for all instances
for num_instances = 1:K
    
    %Find arrival times
    T_arrival_ = myObj.T_calc(N, lambda_);
    time_arrival_glob = [time_arrival_glob; T_arrival_];
    
    %Find urgencies
    U = myObj.U_calc(N, key_vals, urgency_types);
    urgency_glob = [urgency_glob; U];
    
    %Append to vector of objective values for each instance
    obj_value = myObj.computeOPT(N, M, time_arrival_glob, num_instances, S, U, C);
    OPT_vec = [OPT_vec, obj_value];
    sprintf('OPT: %d',num_instances)
    
end

disp('OPT COMPUTED');

% Greedy Allocation without Optimization problem

comp_ratio_greedy = [];
for instance = K/2:K
    obj_val = myObj.greedyCompute(N, M, C, S, time_arrival_glob, urgency_glob, instance);
    comp_ratio_greedy = [comp_ratio_greedy, obj_val/OPT_vec(instance)];
end

greedy_comp = [greedy_comp; max(comp_ratio_greedy)];
greedy_comp_dist = [greedy_comp_dist; comp_ratio_greedy];

disp('GREEDY COMPUTED');

%Scenario Approach TI
%Training Step
reverse_mapping = containers.Map(urgency_types, key_vals);
[alpha_val, p] = myObj.scenarioTI_Train(K, N, M, S, C, urgency_glob, time_arrival_glob, theta, OPT_vec, reverse_mapping);
comp_ratio_vals_test = [comp_ratio_vals_test; alpha_val]; %These are really the training values

disp('SCENARIO TI TRAINED');

%Online Implementation of Scenario TI

urgency_glob_ = urgency_glob(1:K, :);
comp_ratio_scenario = [];
rng(42);
for instance = K/2:K
    cost_alg = myObj.onlineScenarioTI(S, instance, p, urgency_glob_, N, reverse_mapping);
    comp_ratio_scenario = [comp_ratio_scenario, cost_alg/OPT_vec(instance)];
end

comp_ratio_vals = [comp_ratio_vals, max(comp_ratio_scenario)];
comp_ratio_vals_dist = [comp_ratio_vals_dist; comp_ratio_scenario];

disp('SCENARIO TI TESTED');

%Scenario Approach TD
%Training Step
[alpha_val, p1, p2, p3, p4, p5] = myObj.scenarioTD_Train(K, N, M, S, C, urgency_glob, time_arrival_glob, theta, OPT_vec, reverse_mapping);
comp_ratio_vals_new_test = [comp_ratio_vals_new_test; alpha_val]; %These are really the training values

disp('SCENARIO TD TRAINED');

%Online Implementation of Scenario TD
urgency_glob_ = urgency_glob(1:K, :);
comp_ratio_scenario_new = [];
rng(42);
for instance = K/2:K
    cost_alg = myObj.onlineScenarioTD(S, instance, p1, p2, p3, p4, p5, urgency_glob_, N, reverse_mapping, time_arrival_glob);
    comp_ratio_scenario_new = [comp_ratio_scenario_new, cost_alg/OPT_vec(instance)];
end

comp_ratio_vals_new = [comp_ratio_vals_new, max(comp_ratio_scenario_new)];
comp_ratio_vals_new_dist = [comp_ratio_vals_new_dist; comp_ratio_scenario_new];

disp('SCENARIO TD TESTED');

for num_iter = 1:5
    
    %Set Batch size
    batch_size = 2*num_iter;
    
    %Set Lookahead size
    num_lookahead = 2*num_iter;
    
    %k-Greedy Allocation with given batching size
    
%     k_comp_ratio_greedy = [];
%     for instance = K/2:K
%         obj_val = myObj.kGreedy(batch_size, N, M, C, S, time_arrival_glob, urgency_glob, instance);
%         k_comp_ratio_greedy = [k_comp_ratio_greedy, obj_val/OPT_vec(instance)];
%         
%         sprintf('K-GREEDY: %d, k_val = %d', instance, batch_size)
%     end
%     
%     k_greedy_comp = [k_greedy_comp; max(k_comp_ratio_greedy)];
%     k_greedy_comp_dist = [k_greedy_comp_dist; k_comp_ratio_greedy];

    %k-Rolling Batching
    k_comp_ratio_rolling = [];
    for instance = K/2:K
        obj_val = myObj.kRollingBatching(num_lookahead, N, M, C, S, time_arrival_glob, urgency_glob, instance, lambda_, mapping);
        k_comp_ratio_rolling = [k_comp_ratio_rolling, obj_val/OPT_vec(instance)];
        
        sprintf('K-Rolling: %d, k_val = %d', instance, num_lookahead)
        
    end
    
    k_rolling_comp = [k_rolling_comp; max(k_comp_ratio_rolling)];
    k_rolling_comp_dist = [k_rolling_comp_dist; k_comp_ratio_rolling];
    
    %k-scenario generation with specified lookahead
    k_comp_ratio_greedy_sc = [];
    for instance = K/2:K
        obj_val = myObj.kScenario(num_lookahead, N, M, C, S, time_arrival_glob, urgency_glob, instance, lambda_, mapping);
        k_comp_ratio_greedy_sc = [k_comp_ratio_greedy_sc, obj_val/OPT_vec(instance)];
        
        sprintf('K-SAMPLING: %d, k_val = %d', instance, num_lookahead)
        
    end
    
    k_greedy_comp_sc = [k_greedy_comp_sc; max(k_comp_ratio_greedy_sc)];
    k_greedy_comp_dist_sc = [k_greedy_comp_dist_sc; k_comp_ratio_greedy_sc];
    
    
end 

%% Plot Data
subplot(3,2,1);
histogram(greedy_comp_dist(1, :), 10, 'FaceAlpha',0)
hold on
histogram(comp_ratio_vals_dist(1, :), 10, 'facecolor', [0.3 0.5 0.2], 'FaceAlpha',0, 'EdgeColor', [0.3 0.5 0.2])
histogram(comp_ratio_vals_new_dist(1, :), 10, 'facecolor', [0.7 0.1 0.1], 'FaceAlpha',0, 'EdgeColor', [0.7 0.1 0.1])
histogram(k_rolling_comp_dist(1, :), 10, 'facecolor', [0.2 0.2 0.6], 'FaceAlpha',0, 'EdgeColor', [0.2 0.2 0.6])
histogram(k_greedy_comp_dist_sc(1, :), 10, 'facecolor', [0.1 0.8 0.1], 'FaceAlpha',0, 'EdgeColor', [0.1 0.8 0.1])
hold off
legend('Greedy', 'Scenario TI', 'Scenario TD', 'k-Greedy', 'k-Scenario')
xlabel('Competitive Ratio')
ylabel('Count')
title('k = 2')

subplot(3,2,2);
histogram(greedy_comp_dist(1, :), 10, 'FaceAlpha',0.2)
hold on
histogram(comp_ratio_vals_dist(1, :), 10, 'facecolor', [0.3 0.5 0.2], 'FaceAlpha',0.4)
histogram(comp_ratio_vals_new_dist(1, :), 10, 'facecolor', [0.7 0.1 0.1], 'FaceAlpha',0.4)
histogram(k_rolling_comp_dist(2, :), 10, 'facecolor', [0.2 0.2 0.6], 'FaceAlpha',0.4)
histogram(k_greedy_comp_dist_sc(2, :), 10, 'facecolor', [0.1 0.8 0.1], 'FaceAlpha',0.4)
hold off
legend('Greedy', 'Scenario TI', 'Scenario TD', 'k-Greedy', 'k-Scenario')
xlabel('Competitive Ratio')
ylabel('Count')
title('k = 4')

subplot(3,2,3);
histogram(greedy_comp_dist(1, :), 10, 'FaceAlpha',0.2)
hold on
histogram(comp_ratio_vals_dist(1, :), 10, 'facecolor', [0.3 0.5 0.2], 'FaceAlpha',0.4)
histogram(comp_ratio_vals_new_dist(1, :), 10, 'facecolor', [0.7 0.1 0.1], 'FaceAlpha',0.4)
histogram(k_rolling_comp_dist(3, :), 10, 'facecolor', [0.2 0.2 0.6], 'FaceAlpha',0.4)
histogram(k_greedy_comp_dist_sc(3, :), 10, 'facecolor', [0.1 0.8 0.1], 'FaceAlpha',0.4)
hold off
legend('Greedy', 'Scenario TI', 'Scenario TD', 'k-Greedy', 'k-Scenario')
xlabel('Competitive Ratio')
ylabel('Count')
title('k = 6')

subplot(3,2,4);
histogram(greedy_comp_dist(1, :), 10, 'FaceAlpha',0.2)
hold on
histogram(comp_ratio_vals_dist(1, :), 10, 'facecolor', [0.3 0.5 0.2], 'FaceAlpha',0.4)
histogram(comp_ratio_vals_new_dist(1, :), 10, 'facecolor', [0.7 0.1 0.1], 'FaceAlpha',0.4)
histogram(k_rolling_comp_dist(4, :), 10, 'facecolor', [0.2 0.2 0.6], 'FaceAlpha',0.4)
histogram(k_greedy_comp_dist_sc(4, :), 10, 'facecolor', [0.1 0.8 0.1], 'FaceAlpha',0.4)
hold off
legend('Greedy', 'Scenario TI', 'Scenario TD', 'k-Greedy', 'k-Scenario')
xlabel('Competitive Ratio')
ylabel('Count')
title('k = 8')

subplot(3,2,5);
histogram(greedy_comp_dist(1, :), 10, 'FaceAlpha',0.2)
hold on
histogram(comp_ratio_vals_dist(1, :), 10, 'facecolor', [0.3 0.5 0.2], 'FaceAlpha',0.4)
histogram(comp_ratio_vals_new_dist(1, :), 10, 'facecolor', [0.7 0.1 0.1], 'FaceAlpha',0.4)
histogram(k_rolling_comp_dist(5, :), 10, 'facecolor', [0.2 0.2 0.6], 'FaceAlpha',0.4)
histogram(k_greedy_comp_dist_sc(5, :), 10, 'facecolor', [0.1 0.8 0.1], 'FaceAlpha',0.4)
hold off
legend('Greedy', 'Scenario TI', 'Scenario TD', 'k-Greedy', 'k-Scenario')
xlabel('Competitive Ratio')
ylabel('Count')
title('k = 10')

set(gca,'view',[90 90])
    