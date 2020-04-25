function [flow, K] = VenturiCalcFlow(dp)
% input dp: pressure difference in [Pa] = [kg/(m*s^2)]
% output flow: volume flow in liters per minute
% Calculation based on formulas from https://www.efunda.com/formulae/fluids/venturi_flowmeter.cfm#calc

diameter_A = 1.3e-2; % upstream diameter; [m]
diameter_B = 0.6e-2; % neck diameter; [m]
%dens = 1.29; % fluid density of air; [kg/m^3]
dens = 1.40; % fluid density of air; [kg/m^3]; raised
C_disc = 0.98; % discharge coefficient
area_A = (diameter_A/2)^2 * pi; % [m^2]
area_B = (diameter_B/2)^2 * pi; % [m^2]

% resulting units:
%         1   *          m / s      *   m^2 = m^3 / s
%flow = C_disc * sqrt( 2*dp / dens ) * area_A/sqrt( (area_A/area_B)^2 - 1 );

% convert to liters/s: [m^3] = 1000*[l]
% flow = 1000 * flow;
% convert to liters/min: 1 minute = 60 seconds
% flow = 60 * flow
K = C_disc * sqrt( 2 / dens ) * area_A/sqrt( (area_A/area_B)^2 - 1 ) * 60 * 1000;
% K can be pre-caclulated and then stored in the sensor code

if nargin < 1
  flow = nan();
else
  flow = K * sqrt( dp );
end

end
