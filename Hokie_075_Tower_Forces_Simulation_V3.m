% constants in capitals, variables in lowercase
% coordinate system:    origin at the tower pivot
%                       positive y against gravity
%                       positive x to the right
%                       counter-clockwise moments are positive
%                       tower lifting occurs counter-clockwise
% units are English [;_;] - slug, ft, s, lbf, & degrees

clear
close
clc
clf

% user-supplied values
G   = 32.2;      % accel     gravity
RWO = 0.8;       % length    rocket weight offset away from tower
RW  = 500;       % force     rocket weight
TW  = 100;       % force     tower weight
TCG = 15;        % length    tower cg location along tower
TH  = 30;        % length    tower height
LPL = 5;         % length    lift point location along tower
RBXL = 10;       % location  ram base x location (global)
RBYL = -1.5;     % location  ram base y location (global)
RCG = 8;         % length    rocket cg location along tower
STEP = 0.5;      % resolution of calculations
PLOTSTEP = 0.1;  % time      timestep of plot, seconds
% / end user-supplied values


% variables used in force calculations
% tbx, tby forces      tower base xy (global)
% lpx, lpy forces      lift point xy (global) 
% rbx, rby forces      ram base xy (global)
% theta angle       angle between horizontal and tower
% / end variables used in force calculations

syms tbx tby lpx lpy rbx rby theta

% setting up the system of equations
tower_sum_x = tbx + lpx == 0;
tower_sum_y = tby + lpy == RW + TW;
tower_sum_base = lpy*cosd(theta)*LPL - lpx*sind(theta)*LPL - TW*cosd(theta)*TCG...
    - RW*(RCG*cosd(theta) - RWO*sind(theta)) == 0;
ram_sum_x = rbx - lpx == 0;
ram_sum_y = rby - lpy == 0;
ram_sum_base = lpy*(RBXL - LPL*cosd(theta)) + lpx*(LPL*sind(theta) - RBYL) == 0;


% solving the system - leaving theta as a variable
[solved_tbx, solved_tby, solved_lpx, solved_lpy, solved_rbx, solved_rby] = ...
    solve([tower_sum_x, tower_sum_y, tower_sum_base, ram_sum_x, ram_sum_y, ...
    ram_sum_base], [tbx tby lpx lpy rbx rby]);

% solving for numerical force values by substituting theta values
theta_vals = 0:STEP:90;
tbx_vals = double(subs(solved_tbx, theta, theta_vals));
tby_vals = double(subs(solved_tby, theta, theta_vals));
lpx_vals = double(subs(solved_lpx, theta, theta_vals));
lpy_vals = double(subs(solved_lpy, theta, theta_vals));
rbx_vals = double(subs(solved_rbx, theta, theta_vals));
rby_vals = double(subs(solved_rby, theta, theta_vals));

tower_pivot_force = ((tbx_vals .^ 2) + (tby_vals .^ 2)) .^ (1/2);
ram_internal_force = ((rbx_vals .^ 2) + (rby_vals .^ 2)) .^ (1/2);


% plotting results
grid on
xlim([-5 95]);
ylim([-1*max(max(tower_pivot_force), max(ram_internal_force))*0.05, ...
        max(max(tower_pivot_force), max(ram_internal_force))*1.05]);
title("Forces vs. Tower Angle")
xlabel("Tower angle, degrees")
ylabel("Forces, lbf")

tower_pivot_force_plot_values = [];
ram_internal_force_plot_values = [];
tower_pivot_line = line("Color", "red");
ram_internal_line = line("Color", "blue");

for i = 1:length(theta_vals)
    %pause(PLOTSTEP)
    
    updated_x_vals = theta_vals(1:i);
    
    tower_pivot_force_plot_values(i) = tower_pivot_force(i);
    ram_internal_force_plot_values(i) = ram_internal_force(i);
    
    set(tower_pivot_line, "XData", updated_x_vals , ...
        "YData", tower_pivot_force_plot_values);
    set(ram_internal_line, "XData", updated_x_vals, ...
        "YData", ram_internal_force_plot_values);
end

legend("force in the tower bearing", "force in the ram")