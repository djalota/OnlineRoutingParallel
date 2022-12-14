total_comp = [];
for k = 1:5
    all_comp = [greedy_comp_dist; comp_ratio_vals_dist; comp_ratio_vals_new_dist; k_rolling_comp_dist(k, :); k_greedy_comp_dist_sc(k, :)];
    total_comp = [total_comp, all_comp];
end

all_comparisons = [];
for k = 1:5
    for i = 1:5
        for j = 1:5
            [p,h,stats] = signrank(total_comp(i,(k-1)*100+1:k*100+1),total_comp(j,(k-1)*100+1:k*100+1),...
                'tail','left','method','exact'); 
            all_comparisons = [all_comparisons; [p,h]];
        end
    end
end