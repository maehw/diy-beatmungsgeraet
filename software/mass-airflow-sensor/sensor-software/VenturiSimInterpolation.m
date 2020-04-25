% Quadratic interpolation/ curve fitting of the simulation results for the Venturi type tube

% fit the polynomial in a least-squares sense using the polyfit function
% degree n=2 for quadratic fit
flow = [1	2	3	5	10	20	30	40	50	60]; % volume flow rate [l/min]
dp_meas = [0.0368111212	0.1210032302	0.2534600817	0.6540743919	2.4193817115 ...
	9.3366498942	20.7376193487	36.6244692104	57.2241641543	82.1268103469]; % differential pressure [mmH2O]

plot(flow, dp_meas, 'rx', 'linewidth', 3);
hold on;

p = polyfit(flow, dp_meas, 2);

flow_interval = 0:60;
dp_calc = polyval(p, flow_interval);

plot(flow_interval, dp_calc, 'b', 'linewidth', 2);


dp3 = flow_interval .^ 2 * p(1);
plot(flow_interval, dp3, 'm--', 'linewidth', 2);
hold off;

xlabel('Volume flow [l/m] \rightarrow');
ylabel('Differential pressure [mmH2O] \rightarrow');

K = sqrt( 1/p(1) )