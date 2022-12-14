%% Plot of True Arrival Rates

%X = ['5:00', '6:00', '7:00', '8:00', '9:00', '10:00'];
count = 6;
X = [5, 6, 7, 8, 9, 10];
n = length(count);
year = repmat(1995,1,n);
month = repmat(4,1,n);
day = repmat(18,1,n);
hour = X;
minutes = zeros(1,n);
sdate = datenum(year,month,day,hour,minutes,minutes);

Y1 = [3356, 5160, 6011, 6464, 5950, 5950];
Y2 = [1101, 2334, 3593, 3832, 3511, 3511];
Y3 = [2882, 4257, 4596, 4991, 4487, 4487];

stairs(sdate, Y1)
hold on
stairs(sdate, Y2)
stairs(sdate, Y3)
hold off
ylim([0, 7000])
datetick('x', 'HH:MM')
xlabel('Time of Day')
ylabel('Passenger Flow (veh/h)')
legend('Red', 'Green', 'Blue')

matlab2tikz('real_world_flow_rates.tex');