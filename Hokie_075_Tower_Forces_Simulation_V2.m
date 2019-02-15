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
theta = rand(1,91); % angle the tower is raised from horizontal, degrees
theta(1,:) = 0:1:90;
alpha = atand(Hy / Hx); % angle measured from the I-beam to the pivot-pivot line, degrees

Hal = 10; % distance along the tower between the pivot and the ram, ft
Hl = sqrt((Hal^2) + (Pd^2) - (2*Hal*Pd*cosd(theta - alpha))); % length of ram, ft

% grid on
% title("Force in Tower Sections vs. Tower Angle")
% xlabel("Tower angle, degrees")
% ylabel("Force in section, lbf")
% Tifx_line = line("Color", "red", "LineStyle", "--");
% Tify_line = line("Color", "red", "LineStyle", ":");
% Hifx_line = line("Color", "green", "LineStyle", "--");
% Hify_line = line("Color", "green", "LineStyle", ":");
% Hpfx_line = line("Color", "blue", "LineStyle", "--");
% Hpfy_line = line("Color", "blue", "LineStyle", ":");
%legend("Tower internal force", "Ram internal force", "Ram pivot force")

syms Tifx Tify Hifx Hify Hpfx Hpfy

ram_x = Hal .* cosd(theta);
ram_y = Hal .* sind(theta);
tower_tip = [((cosd(theta) .* (Th - Hal)) + ram_x) ((sind(theta) .* (Th - Hal))+ram_y)];
gamma = 180 - atan2d((ram_y - Hy), (ram_x - Hx));

Tifx_data = [];
Tify_data = [];
Hifx_data = [];
Hify_data = [];
Hpfx_data = [];
Hpfy_data = [];

% Calculating forces in the tower and in the ram
% Tifxy: tower internal forces
% Hifxy: ram internal forces
% Hpfxy: ram base pivot reaction forces

for angle_index = 1:91
    
    tower_sum_x = Tifx + Hifx == 0;
    tower_sum_y = Tify + Hify - Tm*g - Rm*g == 0;
    tower_sum_m = ...
        - Tm*g*Tcg .* cosd(theta(angle_index)) ...
        - Rm*g*Rcg .* cosd(theta(angle_index)) ...
        - Hifx*Hal .* sind(theta(angle_index)) ...
        + Hify*Hal .* cosd(theta(angle_index)) == 0;
    ram_sum_x = Hpfx - Hifx == 0;
    ram_sum_y = Hpfy - Hify - Hm*g == 0;
    ram_sum_m = ...
        Hify*Hl .* cosd(gamma(angle_index)) ...
        + Hifx*Hl .* sind(gamma(angle_index)) ...
        + Hm*g*(0.5*Hl) .* cosd(gamma(angle_index)) == 0;
    
    solved_Hifx = solve(tower_sum_x, Hifx);
    solved_Hify = solve(tower_sum_y, Hify);
    solved_tower_sum_m = subs(tower_sum_m, [Hifx Hify], [solved_Hifx solved_Hify]);
    
    solved_Hpfx = solve(ram_sum_x, Hpfx);
    solved_Hpfy = solve(ram_sum_y, Hpfy);
    solved_ram_sum_m = subs(ram_sum_m, [Hpfx Hpfy Hifx Hify], [solved_Hpfx solved_Hpfy solved_Hifx solved_Hify]);
    
    solved_Tifx_tower_sum_m = solve(solved_tower_sum_m, Tifx);
    solved_Tify_ram_sum_m = subs(solved_ram_sum_m, Tifx, solved_Tifx_tower_sum_m)
    if angle_index ~= 1
        Tify_final = solve(solved_Tify_ram_sum_m, Tify)
    end

%     Tifx_data = [Tifx_data double(forces.Tifx)];
%     Tify_data = [Tify_data double(forces.Tify)];
%     Hifx_data = [Hifx_data double(forces.Hifx)];
%     Hify_data = [Hify_data double(forces.Hify)];
%     Hpfx_data = [Hpfx_data double(forces.Hpfx)];
%     Hpfy_data = [Hpfy_data double(forces.Hpfy)];
end

% set(Tifx_line, 'XData', theta(1:length(Tifx_data)), 'YData', Tifx_data)
% set(Tify_line, 'XData', theta(1:length(Tify_data)), 'YData', Tify_data)
% set(Hifx_line, 'XData', theta(1:length(Hifx_data)), 'YData', Hifx_data)
% set(Hify_line, 'XData', theta(1:length(Hify_data)), 'YData', Hify_data)
% set(Hpfx_line, 'XData', theta(1:length(Hpfx_data)), 'YData', Hpfx_data)
% set(Hpfy_line, 'XData', theta(1:length(Hpfy_data)), 'YData', Hpfy_data)

