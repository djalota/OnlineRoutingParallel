%% Important Functions

classdef benchmark_comparisonsOTR
   methods
      %Set of all time intervals at which people arrive into the system
      function [T_arrivals] = T_calc(obj, N, lambda_)
          
          T_log = 0;
          T_arrival = [];
          for num_people = 1:N
              inter_arrival_time = -log(rand/lambda_)/lambda_;
              T_log = T_log+inter_arrival_time;
              T_arrival = [T_arrival, T_log];
          end
          
          T_arrivals = T_arrival;
      end
      
      %Function to calculate urgencies for a given instance
      
      function [U] = U_calc(obj, N, key_vals, urgency_types)
          
          mapping = containers.Map(key_vals, urgency_types);
          
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
          
      end
      
      %Compute optimal objective values
      function [obj_] = computeOPT(obj, N, M, time_arrival_glob, num_instances, S, U, C)
          
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
          cvx_begin quiet
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
          
          obj_ = cvx_optval;
          
      end
      
      % Greedy Allocation without Optimization problem
      function [obj_val] = greedyCompute(obj, N, M, C, S, time_arrival_glob, urgency_glob, instance)
          
          %Initialize Fraction of roads used
          frac_roads = zeros(N, M);
          obj_val = 0;
          %Store all x values
          %x_glob = [];
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
              %x_glob = [x_glob; x];
              
          end
      end
      
      %k-Greedy Allocation with given batching size
      
      function [obj_val] = kGreedy(obj, batch_size, N, M, C, S, time_arrival_glob, urgency_glob, instance)
          
          obj_val = 0;
          %Store all x values
          x_glob = [];
          U = urgency_glob(instance, :);
          T_arrival = time_arrival_glob(instance, :);
          allocated = zeros(1, N);
          
          %Capacity Remaining
          C_rem = repmat(C, N, 1);
          num_batches = N/batch_size;
          
          for batch = 1:num_batches
              %disp(batch)
              initial_person = (batch_size)*batch-batch_size+1;
              final_person = (batch_size)*batch;
              
              %Find batch utilities
              U_batch = U(initial_person:final_person);
              
              Z = [];
              for time_arrival = initial_person:final_person
                  capacity_mat = zeros(M, batch_size);
                  for links = 1:M
                      idx_batch = 1;
                      for person = initial_person:final_person
                          
                          if time_arrival_glob(instance, time_arrival) >= time_arrival_glob(instance, person) && time_arrival_glob(instance, time_arrival) <= time_arrival_glob(instance, person)+S(links)
                              capacity_mat(links, idx_batch) = capacity_mat(links, idx_batch) + 1;
                              
                          end
                          idx_batch = idx_batch +1;
                          
                      end
                  end
                  Z = cat(3, Z, capacity_mat);
              end
              
              size_z = size(Z);
              
              
              
              %Solve optimization problem to compute optimal greedy
              %allocation
              cvx_begin quiet
              variable x(batch_size, M);
              
              %Objective function
              minimize( sum((x*S').*U_batch') ); %
              subject to
              %Sum of routing probabilities for each agent must add up to one over all
              %routes
              for i2 = 1:batch_size
                  sum(x(i2, :)) >= 1;
              end
              
              %Capacity constraint is met at each point a customer arrives
              for cap_const = 1:size_z(3)
                  for link_val = 1:M
                      Z(link_val, :, cap_const)*x(:, link_val) <= C_rem(initial_person+cap_const-1, link_val);
                  end
              end
              
              %Non-negativity constraint for the routing probabilities
              for i5=1:batch_size
                  for j5 = 1:M
                      x(i5, j5) >= 0;
                  end
              end
              cvx_end
              
%               if batch == 58
%                   disp(C_rem(initial_person+cap_const-1, :))
%                   disp(x)
%                   disp(cvx_status)
%               end
              
              obj_val = obj_val + cvx_optval;
              
              %Update remaining capacities based on optimal allocation
              if final_person<N
                  idx_batch = 0;
                  for batch_user = initial_person:final_person
                      idx_batch = idx_batch + 1;
                      for j3 = 1:M
                          for t3 = final_person+1:N %iterate over all remaing time periods
                              
                              %Check if person arrives in appropriate time interval
                              if T_arrival(t3) >= T_arrival(batch_user) && T_arrival(t3) <= T_arrival(batch_user)+S(j3)
                                  C_rem(t3, j3) = C_rem(t3, j3) - x(idx_batch, j3);
                              end
                              
                          end
                      end
                  end
              end
              
          end
          
      end
      
      %k-Lookahead Scenario Algorithm
      function [obj_val] = kScenario(obj, num_lookahead, N, M, C, S, time_arrival_glob, urgency_glob, instance, lambda_, mapping)
          
          %Initialize Fraction of roads used
          frac_roads = zeros(N, M);
          obj_val = 0;
          %Store all x values
          x_glob = [];
          U = urgency_glob(instance, :);
          T_arrival = time_arrival_glob(instance, :);
          allocated = zeros(1, N);
          
          %Capacity Remaining
          C_rem = repmat(C, N, 1);

          for customer_val = 1:N
              
              if customer_val >= N-num_lookahead + 1
                  num_lookahead = num_lookahead - 1;
              end
              
              U_customer = U(customer_val);
              T_customer = time_arrival_glob(instance, customer_val);
              
              %Generate data for next loookahead customers
              T_log_l = T_customer;
              T_arrival_l = [];
              for num_people = 1:num_lookahead
                  inter_arrival_time_l = -log(rand/lambda_)/lambda_;
                  T_log_l = T_log_l+inter_arrival_time_l;
                  T_arrival_l = [T_arrival_l, T_log_l];
              end
              
              %time_arrival_glob_l = [time_arrival_glob_l; T_arrival_l];
              
              %Generate Urgencies of lookahead customers
              %Map the customer types to the urgency values
              U_l = [];
              for num_people_l = 1:num_lookahead
                  prob_val_l = rand;
                  if prob_val_l<0.32
                      U_l = [U_l, mapping(1)];
                  elseif prob_val_l<0.71
                      U_l = [U_l, mapping(2)];
                  else
                      U_l = [U_l, mapping(3)];
                  end
                  %chosen_num = randi([1, theta]);
                  %U = [U, mapping(chosen_num)];
              end
              
              %urgency_glob = [urgency_glob; U];
              %Generate batch utilities
              U_batch = [U_customer, U_l];
              T_arrival_l = [T_customer, T_arrival_l];
              
              Z = [];
              for time_arrival = 1:num_lookahead+1
                  capacity_mat = zeros(M, num_lookahead+1);
                  for links = 1:M
                      %idx_batch = 1;
                      for person = 1:num_lookahead+1
                          
                          if T_arrival_l(time_arrival) >= T_arrival_l(person) && T_arrival_l(time_arrival) <= T_arrival_l(person)+S(links)
                              capacity_mat(links, person) = capacity_mat(links, person) + 1;
                              
                          end
                          %idx_batch = idx_batch +1;
                          
                      end
                  end
                  Z = cat(3, Z, capacity_mat);
              end
              
              size_z = size(Z);
              
              if customer_val<N
                  %Solve optimization problem to compute optimal greedy
                  %allocation
                  cvx_begin quiet
                  variable x(num_lookahead+1, M);
                  
                  %Objective function
                  minimize( sum((x*S').*U_batch') ); %
                  subject to
                  %Sum of routing probabilities for each agent must add up to one over all
                  %routes
                  for i2 = 1:num_lookahead+1
                      sum(x(i2, :)) == 1;
                  end
                  
                  %Capacity constraint is met at each point a customer arrives
                  for cap_const = 1:size_z(3)
                      for link_val = 1:M
                          Z(link_val, :, cap_const)*x(:, link_val) <= C_rem(customer_val+cap_const-1, link_val);
                      end
                  end
                  
                  %Non-negativity constraint for the routing probabilities
                  for i5=1:num_lookahead+1
                      for j5 = 1:M
                          x(i5, j5) >= 0;
                      end
                  end
                  cvx_end
                  
                  obj_val = obj_val + U_customer*(x(1,:)*S');
                  
                  %Update remaining capacities based on optimal allocation
                  if customer_val<N
                      
                      for j3 = 1:M
                          for t3 = customer_val+1:N %iterate over all remaing time periods
                              
                              %Check if person arrives in appropriate time interval
                              if T_arrival(t3) >= T_arrival(customer_val) && T_arrival(t3) <= T_arrival(customer_val)+S(j3)
                                  C_rem(t3, j3) = C_rem(t3, j3) - x(1, j3);
                              end
                          end
                      end
                  end
              else
                  %Route last person greedily
                  allocated_last = 0;
                  for j3 = 1:M
                      if C_rem(customer_val, j3) > 0.001 && allocated_last == 0
                          obj_val = obj_val + U_customer*S(j3);
                          allocated_last = 1;
                      end
                  end
              end
              
          end
          
      end
      
      %Solve k-Rolling Batching
      %k-Lookahead Scenario Algorithm
      function [obj_val] = kRollingBatching(obj, num_lookahead, N, M, C, S, time_arrival_glob, urgency_glob, instance, lambda_, mapping)
          
          %Initialize Fraction of roads used
          frac_roads = zeros(N, M);
          obj_val = 0;
          %Store all x values
          x_glob = [];
          U = urgency_glob(instance, :);
          T_arrival = time_arrival_glob(instance, :);
          allocated = zeros(1, N);
          
          %Capacity Remaining
          C_rem = repmat(C, N, 1);

          for customer_val = 1:N
              
              if customer_val >= N-num_lookahead + 1
                  num_lookahead = num_lookahead - 1;
              end
              
              U_customer = U(customer_val);
              T_customer = time_arrival_glob(instance, customer_val);
              
              %Generate data for next loookahead customers
              T_log_l = T_customer;
              T_arrival_l = [];
              for num_people = 1:num_lookahead
                  inter_arrival_time_l = -log(rand/lambda_)/lambda_;
                  T_log_l = T_log_l+inter_arrival_time_l;
                  T_arrival_l = [T_arrival_l, T_log_l];
              end
              
              %time_arrival_glob_l = [time_arrival_glob_l; T_arrival_l];
              
              %Generate Urgencies of lookahead customers
              %Map the customer types to the urgency values
              U_l = [];
              for num_people_l = 1:num_lookahead
                  prob_val_l = rand;
                  if prob_val_l<0.32
                      U_l = [U_l, mapping(1)];
                  elseif prob_val_l<0.71
                      U_l = [U_l, mapping(2)];
                  else
                      U_l = [U_l, mapping(3)];
                  end
                  %chosen_num = randi([1, theta]);
                  %U = [U, mapping(chosen_num)];
              end
              
              %urgency_glob = [urgency_glob; U];
              %Generate batch utilities
              
              U_batch = [U_customer, U(customer_val+1:customer_val+num_lookahead)];
              T_arrival_l = [T_customer, time_arrival_glob(instance, customer_val+1:customer_val+num_lookahead)];
              
              Z = [];
              for time_arrival = 1:num_lookahead+1
                  capacity_mat = zeros(M, num_lookahead+1);
                  for links = 1:M
                      %idx_batch = 1;
                      for person = 1:num_lookahead+1
                          
                          if T_arrival_l(time_arrival) >= T_arrival_l(person) && T_arrival_l(time_arrival) <= T_arrival_l(person)+S(links)
                              capacity_mat(links, person) = capacity_mat(links, person) + 1;
                              
                          end
                          %idx_batch = idx_batch +1;
                          
                      end
                  end
                  Z = cat(3, Z, capacity_mat);
              end
              
              size_z = size(Z);
              
              if customer_val<N
                  %Solve optimization problem to compute optimal greedy
                  %allocation
                  cvx_begin quiet
                  variable x(num_lookahead+1, M);
                  
                  %Objective function
                  minimize( sum((x*S').*U_batch') ); %
                  subject to
                  %Sum of routing probabilities for each agent must add up to one over all
                  %routes
                  for i2 = 1:num_lookahead+1
                      sum(x(i2, :)) >= 1;
                  end
                  
                  %Capacity constraint is met at each point a customer arrives
                  for cap_const = 1:size_z(3)
                      for link_val = 1:M
                          Z(link_val, :, cap_const)*x(:, link_val) <= C_rem(customer_val+cap_const-1, link_val);
                      end
                  end
                  
                  %Non-negativity constraint for the routing probabilities
                  for i5=1:num_lookahead+1
                      for j5 = 1:M
                          x(i5, j5) >= 0;
                      end
                  end
                  cvx_end
                  
                  obj_val = obj_val + U_customer*(x(1,:)*S');
                  
                  %Update remaining capacities based on optimal allocation
                  if customer_val<N
                      
                      for j3 = 1:M
                          for t3 = customer_val+1:N %iterate over all remaing time periods
                              
                              %Check if person arrives in appropriate time interval
                              if T_arrival(t3) >= T_arrival(customer_val) && T_arrival(t3) <= T_arrival(customer_val)+S(j3)
                                  C_rem(t3, j3) = C_rem(t3, j3) - x(1, j3);
                              end
                          end
                      end
                  end
              else
                  %Route last person greedily
                  allocated_last = 0;
                  for j3 = 1:M
                      if C_rem(customer_val, j3) > 0.001 && allocated_last == 0
                          obj_val = obj_val + U_customer*S(j3);
                          allocated_last = 1;
                      end
                  end
              end
              
          end
          
      end
      
      % Solve scenario optimization problem
      function [opt_alpha, p_val] = scenarioTI_Train(obj, K, N, M, S, C, urgency_glob, time_arrival_glob, theta, OPT_vec, reverse_mapping)
          
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
              
              opt_alpha = alpha_val;
              p_val = p;
              
          end
          
      end
      
      %Online implementation of scenario
      function [cost_alg] = onlineScenarioTI(obj, S, instance, p, urgency_glob_, N, reverse_mapping)
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
          end
          
      end
      
      % Solve scenario optimization with new parametrization
      function [opt_alpha, p1_, p2_, p3_, p4_, p5_] = scenarioTD_Train(obj, K, N, M, S, C, urgency_glob, time_arrival_glob, theta, OPT_vec, reverse_mapping)
          
          for K_val = K/2
              
              %Restrict to the time and urgency set
              time_arrival_glob_ = time_arrival_glob(1:K_val, :);
              urgency_glob_ = urgency_glob(1:K_val, :);
              
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
                                  elseif time_arrival_glob_(instance, time_arrival) <= 28
                                      capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat2(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                                  elseif time_arrival_glob_(instance, time_arrival) <= 42
                                      capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) = capacity_mat3(links, reverse_mapping(urgency_glob_(instance, person))) + 1;
                                  elseif time_arrival_glob_(instance, time_arrival) <= 56
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
              
              size_z = size(Z1);
              
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
                          elseif time_arrival_glob_(instance, person_val) <= 28
                              cost_alg = cost_alg + p2(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                          elseif time_arrival_glob_(instance, person_val) <= 42
                              cost_alg = cost_alg + p3(reverse_mapping(urgency_glob_(instance, person_val)), link_val)*S(link_val)*urgency_glob_(instance, person_val);
                          elseif time_arrival_glob_(instance, person_val) <= 56
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
              
              opt_alpha = alpha_val;
              p1_ = p1;
              p2_ = p2;
              p3_ = p3;
              p4_ = p4;
              p5_ = p5;
              
          end
          
      end
      
      %Online implementation of scenario TD
      function [cost_alg] = onlineScenarioTD(obj, S, instance, p1, p2, p3, p4, p5, urgency_glob_, N, reverse_mapping, time_arrival_glob)
          %Find cost of the algorithm
          cost_alg = 0;
          for person_val = 1:N
              prob_val = rand;
              if time_arrival_glob(instance, person_val) <= 14
                  if prob_val<p1(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                      cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                  elseif prob_val<p1(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p1(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                      cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                  else
                      cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                  end
              elseif time_arrival_glob(instance, person_val) <= 28
                  if prob_val<p2(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                      cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                  elseif prob_val<p2(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p2(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                      cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                  else
                      cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                  end
              elseif time_arrival_glob(instance, person_val) <= 42
                  if prob_val<p3(reverse_mapping(urgency_glob_(instance, person_val)), 1)
                      cost_alg = cost_alg + S(1)*urgency_glob_(instance, person_val);
                  elseif prob_val<p3(reverse_mapping(urgency_glob_(instance, person_val)), 1) + p3(reverse_mapping(urgency_glob_(instance, person_val)), 2)
                      cost_alg = cost_alg + S(2)*urgency_glob_(instance, person_val);
                  else
                      cost_alg = cost_alg + S(3)*urgency_glob_(instance, person_val);
                  end
              elseif time_arrival_glob(instance, person_val) <= 56
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
          
      end

   end
end



