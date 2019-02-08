% Code simulating forces in a launch tower during lift
% Origin is at the pivot point of the tower
% Positive x is to the right, positive y is against gravity
% The tower lies with the pivot point on the left, and rotates to
% vertical in the counter-clockwise direction
% Counter-clockwise is positive for moments/torques
% Additional coordinate system: "along" - collinear to the tower, positive
% away from pivot point ; "away" - perpendicular to tower, in-plane with
% hydraulic ram, positive away from pivot point

% These forces are calculated with the tower in a stable position
% Extra force will be required & generated if the tower is lifted

clear
close all
clc
clf

% [ MOST VALUES TEMPORARY ]

g = 32.2; % gravitational constant, ft/s^2
Th = 30; % tower length, ft
Tcg = Th / 2; % distance of tower CG from pivot, ft - *assumed!* 1/2 tower
    % get a better number if necessary
Tm = 500; % tower mass, lbm [TEMPORARY VALUE]
Rm = 100; % rocket mass, lbm [TEMPORARY VALUE]
Rcg = 10; % rocket cg distance from tower pivot when loaded, ft
% H: having to do with the ([H]ydraulic) ram
Hm = 100; % ram mass, lbm [TEMPORARY VALUE]

Hx = 15; % distance along the I-beam between the pivot and the ram, ft
Hy = -2; % distance above the I-beam that the ram base is mounted, ft

Pd = sqrt((Hx^2) + (Hy^2)); % distance between the pivot points, ft
theta = rand(1,91); % angle the tower is raised from vertical, degrees
theta(1,:) = 0:1:90;
alpha = atand(Hy / Hx); % angle measured from the I-beam to the pivot-pivot line, degrees

Hal = 10; % distance along the tower between the pivot and the ram, ft
Hl = sqrt((Hal^2) + (Pd^2) - (2*Hal*Pd*cosd(theta - alpha))); % length of ram, ft

subplot(2,2,2)
grid on
degrees_vs_ram = plot(theta, Hl);
title("Length of Ram vs. Tower Angle")
xlabel("Tower angle, degrees")
ylabel("Length of ram, ft")

subplot(2,2,[3 4])
grid on
title("Force in Tower Sections vs. Tower Angle")
xlabel("Tower angle, degrees")
ylabel("Force in section, lbf")
Tifx_line = line("Color", "red", "LineStyle", "--");
Tify_line = line("Color", "red", "LineStyle", ":");
Hifx_line = line("Color", "green", "LineStyle", "--");
Hify_line = line("Color", "green", "LineStyle", ":");
Hpfx_line = line("Color", "blue", "LineStyle", "--");
Hpfy_line = line("Color", "blue", "LineStyle", ":");
%legend("Tower internal force", "Ram internal force", "Ram pivot force")

subplot(2,2,1)
grid on
title("Tower Lifting Simulation")
xlabel("X distance from tower pivot, ft")
ylabel("Y distance from tower pivot, ft")
xlim([-5, Th + 5])
ylim([Hy - 5, Th + 5])
ibeam = line([-5 Th*1.1],[0 0], "Color", "black"); % I-beam base
pivotline = line([0 Hx], [0 Hy], "LineStyle", ":");
Hal_line = line("Color", "blue");
extended_tower_line = line("Color", "blue");
ram = line("Color", "blue");
Tifx_data = [];
Tify_data = [];
Hifx_data = [];
Hify_data = [];
Hpfx_data = [];
Hpfy_data = [];

syms Tifx Tify Hifx Hify Hpfx Hpfy

for angle_index = 1:91
    pause(0.05)
    ram_x = [Hx  Hal*cosd(theta(angle_index))];
    ram_y = [Hy  Hal*sind(theta(angle_index))];
    tower_tip = [((cosd(theta(angle_index))*(Th - Hal))+ram_x(2)) ((sind(theta(angle_index))*(Th - Hal))+ram_y(2))];
    gamma = 180 - atan2d((ram_y(2) - ram_y(1)), (ram_x(2) - ram_x(1)));

    % Calculating forces in the tower and in the ram
    % Tifxy: tower internal forces
    % Hifxy: ram internal force
    % Hpfxy: ram base pivot reaction force

    tower_sum_x = Tifx + Hifx == 0;
    tower_sum_y = Tify + Hify - Tm*g - Rm*g == 0;
    tower_sum_m = ...
        -Tm*g*Tcg*cosd(theta(angle_index)) ...
        - Rm*g*Rcg*cosd(theta(angle_index)) ...
        - Hifx*Hal*sind(theta(angle_index)) ...
        + Hify*Hal*cosd(theta(angle_index)) == 0;
    ram_sum_x = Hpfx - Hifx == 0;
    ram_sum_y = Hpfy - Hify - Hm*g == 0;
    ram_sum_m = ...
        Hify*Hl*cosd(gamma) ...
        + Hifx*Hl*sind(gamma) ...
        + Hm*g*(0.5*Hl)*cosd(gamma) == 0;

    forces = solve([tower_sum_x, tower_sum_y, tower_sum_m, ram_sum_x, ram_sum_y, ram_sum_m], [Tifx, Tify, Hifx, Hify, Hpfx, Hpfy], 'ReturnConditions', true)
    Tifx_data = [Tifx_data double(forces.Tifx)];
    Tify_data = [Tify_data double(forces.Tify)];
    Hifx_data = [Hifx_data double(forces.Hifx)];
    Hify_data = [Hify_data double(forces.Hify)];
    Hpfx_data = [Hpfx_data double(forces.Hpfx)];
    Hpfy_data = [Hpfy_data double(forces.Hpfy)];
    
    set(Tifx_line, 'XData', theta(1:length(Tifx_data)), 'YData', Tifx_data)
    set(Tify_line, 'XData', theta(1:length(Tify_data)), 'YData', Tify_data)
    set(Hifx_line, 'XData', theta(1:length(Hifx_data)), 'YData', Hifx_data)
    set(Hify_line, 'XData', theta(1:length(Hify_data)), 'YData', Hify_data)
    set(Hpfx_line, 'XData', theta(1:length(Hpfx_data)), 'YData', Hpfx_data)
    set(Hpfy_line, 'XData', theta(1:length(Hpfy_data)), 'YData', Hpfy_data)
    
    set(ram, 'XData', ram_x, 'YData', ram_y)
    set(Hal_line, 'XData', [0 ram_x(2)], 'YData', [0 ram_y(2)])
    set(extended_tower_line, 'XData', [ram_x(2) tower_tip(1)], 'YData', [ram_y(2) tower_tip(2)])
end