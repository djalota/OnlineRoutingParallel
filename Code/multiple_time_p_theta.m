%% Compute OPT for all instances and store urgencies and time of arrivals

%clear all
%clc
rng(42)
s = rng;
%Total number of instances
K = 200;

%Number of roads
M = 3;
    
%Number of people
N = 100;

%Capacities of all roads
C = [20, 24, 100];
    
%Cost incurred in using road
S = [20, 24, 130];

%Here customers have a poisson arrival rate with mean lambda_
%lambda_tot = [5.25, 3; 5.25, 5.5; 5.25, 8; 5.25, 10.5; 5.25, 13];
%lambda_tot = [3, 3, 3, 3, 3; 3, 5.5, 3, 5.5, 3; 3, 8, 3, 8, 3; 3, 10.5, 3, 10.5, 3; 3, 8, 3, 5.5, 3; 3, 8, 3, 10.5, 3;];
%lambda_tot = [3, 5.5, 3, 5.5, 3; 3, 8, 3, 8, 3; 3, 10.5, 3, 10.5, 3];
%lambda_tot = [3, 10.5, 3, 10.5, 3];
lambda_tot = [5.25, 5.25, 5.25, 5.25, 5.25; 5.25, 7.125, 5.25, 7.125, 5.25; 5.25, 9, 5.25, 9, 5.25; 5.25, 12, 5.25, 12, 5.25; 5.25, 12, 5.25, 9, 5.25; 5.25, 9, 5.25, 12, 5.25];
%lambda_tot = [3.375, 3.375, 3.375, 3.375, 3.375; 3.375, 7.125, 3.375, 7.125, 3.375; 3.375, 9, 3.375, 9, 3.375; 3.375, 12, 3.375, 12, 3.375; 3.375, 12, 3.375, 9, 3.375; 3.375, 9, 3.375, 12, 3.375];


%lambda_tot = [5.25, 9, 5.25, 12, 5.25];

%Generate random number for Theta customer types
theta = 3;
key_vals = {1, 2, 3, 4};
urgency_types = {1, 9, 20, 400};
mapping = containers.Map(key_vals, urgency_types);


greedy_comp = [];
greedy_comp_dist = [];
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

for num_iter = 1:1
    
    rng(42)
    s = rng;
    
    OPT_vec = [];
    
    %Here customers have a poisson arrival rate with mean lambda_
    lambda_ = lambda_tot(num_iter, :);
    
    %Store properties of instance, i.e., time of arrival, urgency
    urgency_glob = [];
    time_arrival_glob = [];

    %Iterate over instances to determine OPT for all instances
    for num_instances = 1:K

        %Set of all time intervals at which people arrive into the system

        T_log = 0;
        T_arrival = [];
        for num_people = 1:N
            if T_log<=14
                inter_arrival_time = -log(rand/lambda_(1))/lambda_(1);%-log(rand)
            elseif T_log<= 24
                inter_arrival_time = -log(rand/lambda_(2))/lambda_(2);
            elseif T_log<= 38
                inter_arrival_time = -log(rand/lambda_(3))/lambda_(3);
            elseif T_log<=48
                inter_arrival_time = -log(rand/lambda_(4))/lambda_(4);
            else 
                inter_arrival_time = -log(rand/lambda_(5))/lambda_(5);%-log(rand)
            end
            T_log = T_log+inter_arrival_time;
            T_arrival = [T_arrival, T_log];
        end

        time_arrival_glob = [time_arrival_glob; T_arrival];
        %T_arrival = sort(round((10)*rand(N,1), 1),'ascend');

        %Current Number of people on each road
        N1 = 0; N2 = 0;

        %Urgencies of customers
        %Map the customer types to the urgency values
        U = [];
        for num_people = 1:N
            prob_val = rand;
            if prob_val<0.32
                U = [U, mapping(1)];
            elseif prob_val<0.71
                U = [U, mapping(2)];
            else
                U = [U, mapping(3)];
            end
            %chosen_num = randi([1, theta]);
            %U = [U, mapping(chosen_num)];
        end

        urgency_glob = [urgency_glob; U];
        %U = [1, 3, 2, 2, 1, 2, 1, 3, 1, 3];

        Z = [];
        for time_arrival = 1:N
            capacity_mat = zeros(M, N);
            for links = 1:M
                for person = 1:N
                    if time_arrival_glob(num_instances, time_arrival) >= time_arrival_glob(num_instances, person) && time_arrival_glob(num_instances, time_arrival) <= time_arrival_glob(num_instances, person)+S(links)
                        capacity_mat(links, person) = capacity_mat(links, person) + 1;

                    end

                end
            end
            Z = cat(3, Z, capacity_mat);
        end

        size_z = size(Z);

        %Solve the optimization problem to compute OPT for given instance
        cvx_begin
        variable x(N, M);

        %Objective function
        minimize( sum((x*S').*U') ); %
        subject to
        %Sum of routing probabilities for each agent must add up to one over all
        %routes
        for i2 = 1:N
            sum(x(i2, :)) == 1;
        end

        %Capacity constraint is met at each point a customer arrives
        for cap_const = 1:size_z(3)
            for link_val = 1:M
                Z(link_val, :, cap_const)*x(:, link_val) <= C(link_val);
            end
        end

        %Non-negativity constraint for the routing probabilities
        for i5=1:N
            for j5 = 1:M
                x(i5, j5) >= 0;
            end
        end %x >= zeros(N, M, length(T_all));
        cvx_end

        %                 if mod(cvx_optval, 1) >= 0.01 && mod(cvx_optval, 1) <= 0.99
        %                     break;
        %                 end
        %Append to vector of objective values for each instance
        OPT_vec = [OPT_vec, cvx_optval];

    end

    % Greedy Allocation without Optimization problem
    comp_ratio_greedy = [];
    for instance = K/2:K

        %Initialize Fraction of roads used
        frac_roads = zeros(N, M);
        obj_val = 0;
        %Store all x values
        x_glob = [];
        U = urgency_glob(instance, :);
        T_arrival = time_arrival_glob(instance, :);
        allocated = zeros(1, N);
        for customer = 1:N
            for j3 = 1:M
                if frac_roads(customer, j3) < 1-0.001 && allocated(customer) == 0
                    obj_val = obj_val + U(customer)*S(j3);
                    for t3 = 1:N %iterate over all time periods

                        %Check if person arrives in appropriate time interval
                        if T_arrival(t3) >= T_arrival(customer) && T_arrival(t3) <= T_arrival(customer)+S(j3)
                            frac_roads(t3, j3) = frac_roads(t3, j3) + 1/C(j3);
                        end
                        allocated(customer) = 1;
                    end
                end
            end

            %Update x values
            x_glob = [x_glob; x];

        end
        
%         if obj_val < OPT_vec(instance)
%             break
%         end

        comp_ratio_greedy = [comp_ratio_greedy, obj_val/OPT_vec(instance)];

    end

    greedy_comp = [greedy_comp; max(comp_ratio_greedy)];
    greedy_comp_dist = [greedy_comp_dist; comp_ratio_greedy];

    % Solve scenario optimization problem

    %Create mapping from urgency to customer type
    reverse_mapping = containers.Map(urgency_types, key_vals);

    for K_val = K/2

        %Restrict to the time and urgency set
        time_arrival_glob_ = time_arrival_glob(1:K_val, :);
        urgency_glob_ = urgency_glob(1:K_val, :);

        Z = [];
        %Capacity constraint is met at each point a customer arrives
        for instance = 1:K_val
            for time_arrival = 1:N
                capacity_mat = zeros(M, theta);
                for links = 1:M
                    for person = 1:N
                        if time_arrival_glob_(instance, time_arrival) >= time_arrival_glob_(instance, person) && time_arrival_glob_(instance, time_arrival) <= time_arrival_glob_(instance, person)+S(links)
                            capacity_mat(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat(links, reverse_mapping(urgency_glob_(instance, person))) + 1;

                        end

                    end
                end
                Z = cat(3, Z, capacity_mat);
            end
        end

        size_z = size(Z);

        %Solve optimization problem
        cvx_begin

        %Initialize probability of routing person of type theta to each link
        variable p(theta, M);
        variable alpha_val;
        %Objective function - competitive ratio
        minimize(alpha_val);

        subject to

        %Non-negativity constraint for the probabilities
        p >= zeros(theta, M);

        %Sum of routing probabilities for each type must add up to one over all
        %routes
        for types = 1:theta
            sum(p(types, :)) == 1;
        end

        %Capacity constraint is met at each point a customer arrives
        for cap_const = 1:size_z(3)
            for link_val = 1:M
                Z(link_val, :, cap_const)*p(:, link_val) <= C(link_val);
            end
        end


        %Competitive ratio constraint
        for instance = 1:K_val
            %Find cost of the algorithm
            cost_alg = 0;
            for person_val = 1:N
                for link_val = 1:M
                    cost_alg = cost_alg + p(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
            %Add in the constraint
            cost_alg <= alpha_val*OPT_vec(instance);
        end

        cvx_end
        
        comp_ratio_vals_test = [comp_ratio_vals_test; alpha_val];
        
        
        %Check support constraints
        num_spt_classic_alpha = 0;
        spt_indx = [];
        for instance = 1:K_val
            %Find cost of the algorithm
            cost_alg = 0;
            for person_val = 1:N
                for link_val = 1:M
                    cost_alg = cost_alg + p(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
            %Add in the constraint
            if cost_alg >= alpha_val*OPT_vec(instance) - 0.0001
                num_spt_classic_alpha = num_spt_classic_alpha + 1;
                spt_indx = [spt_indx; instance];
            end
        end
        total_spt_classic_alpha = [total_spt_classic_alpha; num_spt_classic_alpha];
        
        num_spt_classic = 0;
        for cap_const = 1:size_z(3)
            for link_val = 1:M
                if Z(link_val, :, cap_const)*p(:, link_val) >= C(link_val) - 0.00001
                    if ~ismember(ceil(cap_const/(100)), spt_indx)
                        num_spt_classic = num_spt_classic + 1;
                        spt_indx = [spt_indx; ceil(cap_const/(100))];
                    end
                end
            end
        end
        total_spt_classic = [total_spt_classic; num_spt_classic];

        %Online implementation of scenario
        urgency_glob_ = urgency_glob(1:K, :);
        comp_ratio_scenario = [];
        for instance = K/2:K
            %Find cost of the algorithm
            cost_alg = 0;
            for person_val = 1:N
                prob_val = rand;
                if prob_val<p(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
                %for link_val = 1:M
                %    cost_alg = cost_alg + p(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                %end
            end
            comp_ratio_scenario = [comp_ratio_scenario, cost_alg/OPT_vec(instance)];
        end

        comp_ratio_vals = [comp_ratio_vals, max(comp_ratio_scenario)];
        comp_ratio_vals_dist = [comp_ratio_vals_dist; comp_ratio_scenario];
        
        %Online implementation of scenario fractional
        %urgency_glob_ = urgency_glob(1:K, :);
        spt_indx_frac = [];
        comp_ratio_scenario_frac = [];
        num_spt_frac_alpha = 0;
        for instance = K/2:K
            %Find cost of the algorithm
            cost_alg = 0;
            for person_val = 1:N
                for link_val = 1:M
                    cost_alg = cost_alg + p(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
            if cost_alg >= alpha_val*OPT_vec(instance) % 0.0001
                num_spt_frac_alpha = num_spt_frac_alpha + 1;
                spt_indx_frac = [spt_indx_frac; instance];
            end
            comp_ratio_scenario_frac = [comp_ratio_scenario_frac, cost_alg/OPT_vec(instance)];
        end
        
        total_support_frac_alpha = [total_support_frac_alpha; num_spt_frac_alpha];
        
        comp_ratio_vals_frac = [comp_ratio_vals_frac, max(comp_ratio_scenario_frac)];
        comp_ratio_vals_dist_frac = [comp_ratio_vals_dist_frac; comp_ratio_scenario_frac];
        
        time_arrival_glob_ = time_arrival_glob(1:K, :);
        
        Z_ = [];
        for instance = K/2:K
            for time_arrival = 1:N
                capacity_mat = zeros(M, theta);
                for links = 1:M
                    for person = 1:N
                        if time_arrival_glob_(instance, time_arrival) >= time_arrival_glob_(instance, person) && time_arrival_glob_(instance, time_arrival) <= time_arrival_glob_(instance, person)+S(links)
                            capacity_mat(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                            
                        end
                        
                    end
                end
                Z_ = cat(3, Z_, capacity_mat);
            end
        end
        
        num_violations_frac = 0;
        
        size_z_ = size(Z_);
        for cap_const = 1:size_z_(3)
            for link_val = 1:M
                if Z_(link_val, :, cap_const)*p(:, link_val) > C(link_val)
                    if ~ismember(ceil(cap_const/(100)), spt_indx_frac)
                        num_violations_frac = num_violations_frac+1;
                        spt_indx_frac = [spt_indx_frac; ceil(cap_const/(100))];
                    end
                end
            end
        end
        total_support_frac_classic = [total_support_frac_classic; num_violations_frac];
        num_violations_arr_frac = [num_violations_arr_frac, num_violations_frac];
    end

    % Solve scenario optimization with new parametrization

    Z1 = [];
    Z2 = [];
    Z3 = [];
    Z4 = [];
    Z5 = [];
    %Capacity constraint is met at each point a customer arrives
    for instance = 1:K_val
        for time_arrival = 1:N
            capacity_mat1 = zeros(M, theta);
            capacity_mat2 = zeros(M, theta);
            capacity_mat3 = zeros(M, theta);
            capacity_mat4 = zeros(M, theta);
            capacity_mat5 = zeros(M, theta);
            for links = 1:M
                for person = 1:N
                    if time_arrival_glob_(instance, time_arrival) >= time_arrival_glob_(instance, person) && time_arrival_glob_(instance, time_arrival) <= time_arrival_glob_(instance, person)+S(links)
                        if time_arrival_glob_(instance, time_arrival) <= 14
                            capacity_mat1(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat1(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 24
                            capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 38
                            capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 48
                            capacity_mat4(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat4(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        else
                            capacity_mat5(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat5(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        end
                    end

                end
            end
            Z1 = cat(3, Z1, capacity_mat1);
            Z2 = cat(3, Z2, capacity_mat2);
            Z3 = cat(3, Z3, capacity_mat3);
            Z4 = cat(3, Z4, capacity_mat4);
            Z5 = cat(3, Z5, capacity_mat5);
        end
    end

    %Solve optimization problem
    cvx_begin

    %Initialize probability of routing person of type theta to each link
    variable p1(theta, M);
    variable p2(theta, M);
    variable p3(theta, M);
    variable p4(theta, M);
    variable p5(theta, M);
    variable alpha_val;
    %Objective function - competitive ratio
    minimize(alpha_val);

    subject to

    %Non-negativity constraint for the probabilities
    p1 >= zeros(theta, M);
    p2 >= zeros(theta, M);
    p3 >= zeros(theta, M);
    p4 >= zeros(theta, M);
    p5 >= zeros(theta, M);

    %Sum of routing probabilities for each type must add up to one over all
    %routes
    for types = 1:theta
        sum(p1(types, :)) == 1;
        sum(p2(types, :)) == 1;
        sum(p3(types, :)) == 1;
        sum(p4(types, :)) == 1;
        sum(p5(types, :)) == 1;
    end

    %Competitive ratio constraint
    for instance = 1:K_val
        %Find cost of the algorithm
        cost_alg = 0;
        for person_val = 1:N
            for link_val = 1:M
                if time_arrival_glob_(instance, person_val) <= 14
                    cost_alg = cost_alg + p1(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 24
                    cost_alg = cost_alg + p2(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 38
                    cost_alg = cost_alg + p3(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 48
                    cost_alg = cost_alg + p4(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + p5(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
        end
        %Add in the constraint
        cost_alg <= alpha_val*OPT_vec(instance);
    end

    %Capacity constraint is met at each point a customer arrives
    for cap_const = 1:size_z(3)
        for link_val = 1:M
            Z1(link_val, :, cap_const)*p1(:, link_val) + Z2(link_val, :, cap_const)*p2(:, link_val) + Z3(link_val, :, cap_const)*p3(:, link_val) + Z4(link_val, :, cap_const)*p4(:, link_val) + Z5(link_val, :, cap_const)*p5(:, link_val) <= C(link_val);
        end
    end

    cvx_end
    
    comp_ratio_vals_new_test = [comp_ratio_vals_new_test; alpha_val];
    
    %Compute num support
    num_spt_complex_alpha = 0;
    spt_index2 = [];
    %Competitive ratio constraint
    for instance = 1:K_val
        %Find cost of the algorithm
        cost_alg = 0;
        for person_val = 1:N
            for link_val = 1:M
                if time_arrival_glob_(instance, person_val) <= 14
                    cost_alg = cost_alg + p1(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 24
                    cost_alg = cost_alg + p2(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 38
                    cost_alg = cost_alg + p3(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 48
                    cost_alg = cost_alg + p4(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + p5(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
        end
        %Add in the constraint
        if cost_alg >= alpha_val*OPT_vec(instance) - 0.0001
            num_spt_complex_alpha = num_spt_complex_alpha + 1;
            spt_index2 = [spt_index2; instance];
        end
    end
    total_spt_cplx_alpha = [total_spt_cplx_alpha; num_spt_complex_alpha];
    
    num_spt_complex = 0;
    for cap_const = 1:size_z(3)
        for link_val = 1:M
            if Z1(link_val, :, cap_const)*p1(:, link_val) + Z2(link_val, :, cap_const)*p2(:, link_val) + Z3(link_val, :, cap_const)*p3(:, link_val) + Z4(link_val, :, cap_const)*p4(:, link_val) + Z5(link_val, :, cap_const)*p5(:, link_val) >= C(link_val)-0.00001
%                 disp(cap_const);
%                 disp(spt_index2);
                if ~ismember(ceil(cap_const/(100)), spt_index2)
                    num_spt_complex = num_spt_complex + 1;
                    spt_index2 = [spt_index2; ceil(cap_const/(100))];
                end
                
            end
        end
    end
    
    total_spt_cplx = [total_spt_cplx; num_spt_complex];

    %Online implementation of scenario
    urgency_glob_ = urgency_glob(1:K, :);
    comp_ratio_scenario_new = [];
    for instance = K/2:K
        %Find cost of the algorithm
        cost_alg = 0;
        for person_val = 1:N
            prob_val = rand;
            if time_arrival_glob(instance, time_arrival) <= 14
                if prob_val<p1(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p1(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p1(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
            elseif time_arrival_glob(instance, time_arrival) <= 24
                if prob_val<p2(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p2(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p2(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
            elseif time_arrival_glob(instance, time_arrival) <= 38
                if prob_val<p3(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p3(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p3(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
            elseif time_arrival_glob(instance, time_arrival) <= 48
                if prob_val<p4(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p4(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p4(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
            else
                if prob_val<p5(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                    cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                elseif prob_val<p5(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p5(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                    cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                end
            end
            %for link_val = 1:M
            %    cost_alg = cost_alg + p(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
            %end
        end
        comp_ratio_scenario_new = [comp_ratio_scenario_new, cost_alg/OPT_vec(instance)];
    end

    comp_ratio_vals_new = [comp_ratio_vals_new, max(comp_ratio_scenario_new)];
    comp_ratio_vals_new_dist = [comp_ratio_vals_new_dist; comp_ratio_scenario_new];
    
    %Online implementation of scenario fractional
    %urgency_glob_ = urgency_glob(1:K, :);
    spt_indx_frac_cplx = [];
    comp_ratio_scenario_frac_cplx = [];
    num_spt_frac_alpha_cplx = 0;
    for instance = K/2:K
        %Find cost of the algorithm
        cost_alg = 0;
        for person_val = 1:N
            for link_val = 1:M
                if time_arrival_glob_(instance, person_val) <= 14
                    cost_alg = cost_alg + p1(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 24
                    cost_alg = cost_alg + p2(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 38
                    cost_alg = cost_alg + p3(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                elseif time_arrival_glob_(instance, person_val) <= 48
                    cost_alg = cost_alg + p4(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                else
                    cost_alg = cost_alg + p5(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                end
            end
        end
        if cost_alg > alpha_val*OPT_vec(instance) % 0.0001
            num_spt_frac_alpha_cplx = num_spt_frac_alpha_cplx + 1;
            spt_indx_frac_cplx = [spt_indx_frac_cplx; instance];
        end
        comp_ratio_scenario_frac_cplx = [comp_ratio_scenario_frac_cplx, cost_alg/OPT_vec(instance)];
    end
    
    total_support_frac_alpha_cplx = [total_support_frac_alpha_cplx; num_spt_frac_alpha_cplx];
    
    comp_ratio_vals_frac_cplx = [comp_ratio_vals_frac_cplx, max(comp_ratio_scenario_frac_cplx)];
    comp_ratio_vals_dist_frac_cplx = [comp_ratio_vals_dist_frac_cplx; comp_ratio_scenario_frac_cplx];
    
    time_arrival_glob_ = time_arrival_glob(1:K, :);
    
    Z1_ = [];
    Z2_ = [];
    Z3_ = [];
    Z4_ = [];
    Z5_ = [];
    for instance = K/2:K
        for time_arrival = 1:N
            capacity_mat1 = zeros(M, theta);
            capacity_mat2 = zeros(M, theta);
            capacity_mat3 = zeros(M, theta);
            capacity_mat4 = zeros(M, theta);
            capacity_mat5 = zeros(M, theta);
            for links = 1:M
                for person = 1:N
                    if time_arrival_glob_(instance, time_arrival) >= time_arrival_glob_(instance, person) && time_arrival_glob_(instance, time_arrival) <= time_arrival_glob_(instance, person)+S(links)
                        if time_arrival_glob_(instance, time_arrival) <= 14
                            capacity_mat1(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat1(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 24
                            capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 38
                            capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        elseif time_arrival_glob_(instance, time_arrival) <= 48
                            capacity_mat4(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat4(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        else
                            capacity_mat5(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat5(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                        end
                    end
                    
                end
            end
            Z1_ = cat(3, Z1_, capacity_mat1);
            Z2_ = cat(3, Z2_, capacity_mat2);
            Z3_ = cat(3, Z3_, capacity_mat3);
            Z4_ = cat(3, Z4_, capacity_mat4);
            Z5_ = cat(3, Z5_, capacity_mat5);
        end
    end
    
    num_violations_frac_cplx = 0;
    
    size_z_ = size(Z1_);
    for cap_const = 1:size_z_(3)
        for link_val = 1:M
            if Z1_(link_val, :, cap_const)*p1(:, link_val) + Z2_(link_val, :, cap_const)*p2(:, link_val) + Z3_(link_val, :, cap_const)*p3(:, link_val) + Z4_(link_val, :, cap_const)*p4(:, link_val) + Z5_(link_val, :, cap_const)*p5(:, link_val) > C(link_val)
                if ~ismember(ceil(cap_const/(100)), spt_indx_frac_cplx)
                    num_violations_frac_cplx = num_violations_frac_cplx+1;
                    spt_indx_frac_cplx = [spt_indx_frac_cplx; ceil(cap_const/(100))];
                end
            end
        end
    end
    total_support_frac_cplx = [total_support_frac_cplx; num_violations_frac_cplx];
    num_violations_arr_frac_cplx = [num_violations_arr_frac_cplx, num_violations_frac_cplx];
    
end

y = [greedy_comp'; comp_ratio_vals; comp_ratio_vals_new]';
h = bar(1:6, y);
set(h, {'DisplayName'}, {'Greedy','Scenario', 'Scenario Time'}')
legend() 
xlabel('Arrival Rate')
ylabel('Competitive Ratio')
ylim([1 2.6])

%%
subplot(3,2,1);
h1 = boxplot([greedy_comp_dist(1, :)', comp_ratio_vals_dist(1, :)', comp_ratio_vals_new_dist(1, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Constant Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(1)+total_spt_classic_alpha(1),g,0.1);
%sol1 = fzero(fun,[0, 1]);
set(h1(6,2),'YData',[comp_ratio_vals_test(1), comp_ratio_vals_test(1)])

%fun = @(g)func2(100, total_spt_cplx(1)+total_spt_cplx_alpha(1),g,0.1);
%sol1a = fzero(fun,[0, 1]);
set(h1(6,3),'YData',[comp_ratio_vals_new_test(1), comp_ratio_vals_new_test(1)])


% percent_violate1 = (total_spt_classic(1)+total_spt_classic_alpha(1))/(K/2)*100;
% percent_violate1_cplx = (total_spt_cplx(1)+total_spt_cplx_alpha(1))/(K/2)*100;
% 
% [percent_violate1_bd1, percent_violate1_bd2] = epsLU(total_spt_classic(1)+total_spt_classic_alpha(1),K/2,0.1);
% [percent_violate1_bd_cplx1, percent_violate1_bd_cplx2] = epsLU(total_spt_cplx(1)+total_spt_cplx_alpha(1),K/2,0.1);

subplot(3,2,2);
h2 = boxplot([greedy_comp_dist(2, :)', comp_ratio_vals_dist(2, :)', comp_ratio_vals_new_dist(2, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Low Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(2)+total_spt_classic_alpha(2),g,0.1);
%sol2 = fzero(fun,[0, 1]);
set(h2(6,2),'YData',[comp_ratio_vals_test(2), comp_ratio_vals_test(2)])

%fun = @(g)func2(100, total_spt_cplx(2)+total_spt_cplx_alpha(2),g,0.1);
%sol1b = fzero(fun,[0, 1]);
set(h2(6,3),'YData',[comp_ratio_vals_new_test(2), comp_ratio_vals_new_test(2)])

subplot(3,2,3);
h3 = boxplot([greedy_comp_dist(3, :)', comp_ratio_vals_dist(3, :)', comp_ratio_vals_new_dist(3, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Medium Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(3)+total_spt_classic_alpha(3),g,0.1);
%sol3 = fzero(fun,[0, 1]);
set(h3(6,2),'YData',[comp_ratio_vals_test(3), comp_ratio_vals_test(3)])

%fun = @(g)func2(100, total_spt_cplx(3)+total_spt_cplx_alpha(3),g,0.1);
%sol1c = fzero(fun,[0, 1]);
set(h3(6,3),'YData',[comp_ratio_vals_new_test(3), comp_ratio_vals_new_test(3)])


subplot(3,2,4);
h4 = boxplot([greedy_comp_dist(4, :)', comp_ratio_vals_dist(4, :)', comp_ratio_vals_new_dist(4, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('High Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(4)+total_spt_classic_alpha(4),g,0.1);
%sol4 = fzero(fun,[0, 1]);
set(h4(6,2),'YData',[comp_ratio_vals_test(4), comp_ratio_vals_test(4)])

%fun = @(g)func2(100, total_spt_cplx(4)+total_spt_cplx_alpha(4),g,0.1);
%sol1d = fzero(fun,[0, 1]);
set(h4(6,3),'YData',[comp_ratio_vals_new_test(4), comp_ratio_vals_new_test(4)])


subplot(3,2,5);
h5 = boxplot([greedy_comp_dist(5, :)', comp_ratio_vals_dist(5, :)', comp_ratio_vals_new_dist(5, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Afternoon Peak < Morning Peak')
%fun = @(g)func2(100, total_spt_classic(5)+total_spt_classic_alpha(5),g,0.1);
%sol5 = fzero(fun,[0, 1]);
set(h5(6,2),'YData',[comp_ratio_vals_test(5), comp_ratio_vals_test(5)])

%fun = @(g)func2(100, total_spt_cplx(5)+total_spt_cplx_alpha(5),g,0.1);
%sol1e = fzero(fun,[0, 1]);
set(h5(6,3),'YData',[comp_ratio_vals_new_test(5), comp_ratio_vals_new_test(5)])


subplot(3,2,6);
h6 = boxplot([greedy_comp_dist(6, :)', comp_ratio_vals_dist(6, :)', comp_ratio_vals_new_dist(6, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Afternoon Peak > Morning Peak')
%fun = @(g)func2(100, total_spt_classic(6)+total_spt_classic_alpha(6),g,0.1);
%sol6 = fzero(fun,[0, 1]);
set(h6(6,2),'YData',[comp_ratio_vals_test(6), comp_ratio_vals_test(6)])

%fun = @(g)func2(100, total_spt_cplx(6)+total_spt_cplx_alpha(6),g,0.1);
%sol1f = fzero(fun,[0, 1]);
set(h6(6,3),'YData',[comp_ratio_vals_new_test(6), comp_ratio_vals_new_test(6)])

%% Fractional
subplot(3,2,1);
h1 = boxplot([greedy_comp_dist(1, :)', comp_ratio_vals_dist_frac(1, :)', comp_ratio_vals_dist_frac_cplx(1, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Constant Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(1)+total_spt_classic_alpha(1),g,0.1);
%sol1 = fzero(fun,[0, 1]);
set(h1(6,2),'YData',[comp_ratio_vals_test(1), comp_ratio_vals_test(1)])

%fun = @(g)func2(100, total_spt_cplx(1)+total_spt_cplx_alpha(1),g,0.1);
%sol1a = fzero(fun,[0, 1]);
set(h1(6,3),'YData',[comp_ratio_vals_new_test(1), comp_ratio_vals_new_test(1)])


% percent_violate1 = (total_spt_classic(1)+total_spt_classic_alpha(1))/(K/2)*100;
% percent_violate1_cplx = (total_spt_cplx(1)+total_spt_cplx_alpha(1))/(K/2)*100;
% 
% [percent_violate1_bd1, percent_violate1_bd2] = epsLU(total_spt_classic(1)+total_spt_classic_alpha(1),K/2,0.1);
% [percent_violate1_bd_cplx1, percent_violate1_bd_cplx2] = epsLU(total_spt_cplx(1)+total_spt_cplx_alpha(1),K/2,0.1);

subplot(3,2,2);
h2 = boxplot([greedy_comp_dist(2, :)', comp_ratio_vals_dist_frac(2, :)', comp_ratio_vals_dist_frac_cplx(2, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Low Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(2)+total_spt_classic_alpha(2),g,0.1);
%sol2 = fzero(fun,[0, 1]);
set(h2(6,2),'YData',[comp_ratio_vals_test(2), comp_ratio_vals_test(2)])

%fun = @(g)func2(100, total_spt_cplx(2)+total_spt_cplx_alpha(2),g,0.1);
%sol1b = fzero(fun,[0, 1]);
set(h2(6,3),'YData',[comp_ratio_vals_new_test(2), comp_ratio_vals_new_test(2)])

subplot(3,2,3);
h3 = boxplot([greedy_comp_dist(3, :)', comp_ratio_vals_dist_frac(3, :)', comp_ratio_vals_dist_frac_cplx(3, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Medium Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(3)+total_spt_classic_alpha(3),g,0.1);
%sol3 = fzero(fun,[0, 1]);
set(h3(6,2),'YData',[comp_ratio_vals_test(3), comp_ratio_vals_test(3)])

%fun = @(g)func2(100, total_spt_cplx(3)+total_spt_cplx_alpha(3),g,0.1);
%sol1c = fzero(fun,[0, 1]);
set(h3(6,3),'YData',[comp_ratio_vals_new_test(3), comp_ratio_vals_new_test(3)])


subplot(3,2,4);
h4 = boxplot([greedy_comp_dist(4, :)', comp_ratio_vals_dist_frac(4, :)', comp_ratio_vals_dist_frac_cplx(4, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('High Peak Arrival Rate')
%fun = @(g)func2(100, total_spt_classic(4)+total_spt_classic_alpha(4),g,0.1);
%sol4 = fzero(fun,[0, 1]);
set(h4(6,2),'YData',[comp_ratio_vals_test(4), comp_ratio_vals_test(4)])

%fun = @(g)func2(100, total_spt_cplx(4)+total_spt_cplx_alpha(4),g,0.1);
%sol1d = fzero(fun,[0, 1]);
set(h4(6,3),'YData',[comp_ratio_vals_new_test(4), comp_ratio_vals_new_test(4)])


subplot(3,2,5);
h5 = boxplot([greedy_comp_dist(5, :)', comp_ratio_vals_dist_frac(5, :)', comp_ratio_vals_dist_frac_cplx(5, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Afternoon Peak < Morning Peak')
%fun = @(g)func2(100, total_spt_classic(5)+total_spt_classic_alpha(5),g,0.1);
%sol5 = fzero(fun,[0, 1]);
set(h5(6,2),'YData',[comp_ratio_vals_test(5), comp_ratio_vals_test(5)])

%fun = @(g)func2(100, total_spt_cplx(5)+total_spt_cplx_alpha(5),g,0.1);
%sol1e = fzero(fun,[0, 1]);
set(h5(6,3),'YData',[comp_ratio_vals_new_test(5), comp_ratio_vals_new_test(5)])


subplot(3,2,6);
h6 = boxplot([greedy_comp_dist(6, :)', comp_ratio_vals_dist_frac(6, :)', comp_ratio_vals_dist_frac_cplx(6, :)'], 'Labels',{'Greedy','Scenario', 'Scenario Time'});
ylabel('Competitive Ratio')
title('Afternoon Peak > Morning Peak')
%fun = @(g)func2(100, total_spt_classic(6)+total_spt_classic_alpha(6),g,0.1);
%sol6 = fzero(fun,[0, 1]);
set(h6(6,2),'YData',[comp_ratio_vals_test(6), comp_ratio_vals_test(6)])

%fun = @(g)func2(100, total_spt_cplx(6)+total_spt_cplx_alpha(6),g,0.1);
%sol1f = fzero(fun,[0, 1]);
set(h6(6,3),'YData',[comp_ratio_vals_new_test(6), comp_ratio_vals_new_test(6)])

%%
percent_violate = [];
percent_violate_cplx = [];
percent_violate_frac = [];
percent_violate_cplx_frac = [];
percent_violate_lb = [];
percent_violate_ub = [];
percent_violate_cplx_lb = [];
percent_violate_cplx_ub = [];
for a = 1:6
    percent_violate = [percent_violate; (total_spt_classic(a)+total_spt_classic_alpha(a))/(K/2)];
    percent_violate_cplx = [percent_violate_cplx; (total_spt_cplx(a)+total_spt_cplx_alpha(a))/(K/2)];
    percent_violate_frac = [percent_violate_frac; (total_support_frac_classic(a)+total_support_frac_alpha(a))/(K/2)];
    percent_violate_cplx_frac = [percent_violate_cplx_frac; (total_support_frac_cplx(a)+total_support_frac_alpha_cplx(a))/(K/2)];
    [percent_violate2_bd1, percent_violate2_bd2] = epsLU(total_spt_classic(a)+total_spt_classic_alpha(a),K/2,0.0001);
    [percent_violate2_bd_cplx1, percent_violate2_bd_cplx2] = epsLU(total_spt_cplx(a)+total_spt_cplx_alpha(a),K/2,0.0001);
    
    percent_violate_lb = [percent_violate_lb; percent_violate2_bd1];
    percent_violate_ub = [percent_violate_ub; percent_violate2_bd2];
    percent_violate_cplx_lb = [percent_violate_cplx_lb; percent_violate2_bd_cplx1];
    percent_violate_cplx_ub = [percent_violate_cplx_ub; percent_violate2_bd_cplx2];
end

subplot(1,2,1);
y = [percent_violate_lb'; percent_violate_frac'; percent_violate_ub']';
h = bar(1:6, y);
set(h, {'DisplayName'}, {'Theory LB','DATA', 'Theory UB'}')
legend() 
xlabel('Instances')
ylabel('Percent Violation')
title('Scenario')

subplot(1,2,2);
y = [percent_violate_cplx_lb'; percent_violate_cplx_frac'; percent_violate_cplx_ub']';
h = bar(1:6, y);
set(h, {'DisplayName'}, {'Theory LB','DATA', 'Theory UB'}')
legend() 
xlabel('Instances')
ylabel('Percent Violation')
title('Scenario Time')

%% Combined Bar plot of violations

y = [percent_violate_lb'; percent_violate_frac'; percent_violate_ub'; percent_violate_cplx_lb'; percent_violate_cplx_frac'; percent_violate_cplx_ub']';
h = bar(1:6, y);
set(h, {'DisplayName'}, {'Sc: Theory LB','Sc: DATA', 'Sc: Theory UB', 'Sc Time: Theory LB','Sc Time: DATA', 'Sc Time: Theory UB'}')
legend() 
xlabel('Instances')
ylabel('Percent Violation')
ylim([0, 0.6])
%title('Scenario')

%% Histogram
subplot(3,2,1);
histogram(comp_ratio_vals_dist(1, :))
hold on
histogram(comp_ratio_vals_new_dist(1, :))
histogram(greedy_comp_dist(1, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('Constant Arrival Rate')

subplot(3,2,2);
histogram(comp_ratio_vals_dist(2, :))
hold on
histogram(comp_ratio_vals_new_dist(2, :))
histogram(greedy_comp_dist(2, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('Low Peak Arrival Rate')

subplot(3,2,3);
histogram(comp_ratio_vals_dist(3, :))
hold on
histogram(comp_ratio_vals_new_dist(3, :))
histogram(greedy_comp_dist(3, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('Medium Peak Arrival Rate')

subplot(3,2,4);
histogram(comp_ratio_vals_dist(4, :))
hold on
histogram(comp_ratio_vals_new_dist(4, :))
histogram(greedy_comp_dist(4, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('High Peak Arrival Rate')

subplot(3,2,5);
histogram(comp_ratio_vals_dist(5, :))
hold on
histogram(comp_ratio_vals_new_dist(5, :))
histogram(greedy_comp_dist(5, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('Afternoon Peak < Morning Peak')

subplot(3,2,6);
histogram(comp_ratio_vals_dist(6, :))
hold on
histogram(comp_ratio_vals_new_dist(6, :))
histogram(greedy_comp_dist(6, :))
hold off
legend('Scenario', 'Scenario Time', 'Greedy')
xlabel('Competitive Ratio')
ylabel('Count')
title('Afternoon Peak > Morning Peak')
% %% Arrival Rate Profiles High
% 
% X = [0, 14, 24, 38, 48, 60];
% 
% subplot(3,2,1);
% Y1 = [2, 2, 2, 2, 2, 2];
% stairs(X, Y1)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
% subplot(3,2,2);
% Y2 = [2, 2.5, 2, 2.5, 2, 2];
% stairs(X, Y2)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
% subplot(3,2,3);
% Y3 = [2, 3, 2, 3, 2, 2];
% stairs(X, Y3)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
% subplot(3,2,4);
% Y4 = [2, 3.5, 2, 3.5, 2, 2];
% stairs(X, Y4)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
% subplot(3,2,5);
% Y5 = [2, 3.5, 2, 3, 2, 2];
% stairs(X, Y5)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
% subplot(3,2,6);
% Y6 = [2, 3, 2, 3.5, 2, 2];
% stairs(X, Y6)
% ylim([1.5, 4])
% xlabel('Time')
% ylabel('Arrival Rate')
% 
%% Arrival Rate Profiles Low

X = [0, 14, 24, 38, 48, 60];

subplot(3,2,1);
Y1 = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5];
stairs(X, Y1)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

subplot(3,2,2);
Y2 = [1.5, 2.5, 1.5, 2.5, 1.5, 1.5];
stairs(X, Y2)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

subplot(3,2,3);
Y3 = [1.5, 3, 1.5, 3, 1.5, 1.5];
stairs(X, Y3)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

subplot(3,2,4);
Y4 = [1.5, 3.5, 1.5, 3.5, 1.5, 1.5];
stairs(X, Y4)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

subplot(3,2,5);
Y5 = [1.5, 3.5, 1.5, 3, 1.5, 1.5];
stairs(X, Y5)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

subplot(3,2,6);
Y6 = [1.5, 3, 1.5, 3.5, 1.5, 1.5];
stairs(X, Y6)
ylim([1, 4])
xlabel('Time')
ylabel('Arrival Rate')

% function [ sum ] = func2(n, k, x, beta)
% 
% sum = nchoosek(n, k) * x^(n-k);
% 
% for i = k:n-1
%     sum = sum - beta/(2*n)*nchoosek(i, k)*x^(i-k);
% end
% 
% for j = n+1:4*n
%     sum = sum - beta/(6*n)*nchoosek(i, k)*x^(i-k);
% end
% end

function [epsL, epsU] = epsLU(k,N,bet)
alphaL = betaincinv(bet,k,N-k+1);
alphaU = 1-betaincinv(bet,N-k+1,k);
m1 = [k:1:N];
aux1 = sum(triu(log(ones(N-k+1,1)*m1),1),2);
aux2 = sum(triu(log(ones(N-k+1,1)*(m1-k)),1),2);
coeffs1 = aux2-aux1;
m2 = [N+1:1:4*N];
aux3 = sum(tril(log(ones(3*N,1)*m2)),2);
aux4 = sum(tril(log(ones(3*N,1)*(m2-k))),2);
coeffs2 = aux3-aux4;
t1 = 1-alphaL;
t2 = 1;

poly1 = 1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1 - (N-m1')*log(t1)))...
    -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t1)));
poly2 = 1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1 - (N-m1')*log(t2)))...
    -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t2)));
if ((poly1*poly2) > 0)
    epsL = 0;
else
    while t2-t1 > 1e-10
        t = (t1+t2)/2;
        polyt = 1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1 - (N-m1')*log(t)))...
            -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t)));
        if polyt > 0
            t1=t;
        else
            t2=t;
        end
    end
    epsL = 1-t2;
end
t1 = 0;
t2 = 1-alphaU;
poly1 = 1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1 - (N-m1')*log(t1)))...
    -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t1)));
poly2 = 1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1 - (N-m1')*log(t2)))...
    -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t2)));
if ((poly1*poly2) > 0)
    epsL = 0;
else
    while t2-t1 > 1e-10
        t = (t1+t2)/2;
        polyt =1+bet/(2*N)-bet/(2*N)*sum(exp(coeffs1-(N-m1')*log(t)))...
            -bet/(6*N)*sum(exp(coeffs2 + (m2'-N)*log(t)));
        if polyt > 0
            t2=t;
        else
            t1=t;
        end
    end
    epsU = 1-t1;
end
end



