%% ACS6116 MPC Computer Assignment
% (c) P.Trodden, 2023.

%% Problem setup
clear all
close all
clc

m1 = 1;
m2 = m1;
ks = 1;

ulimit = 1;
ylimit = 2.7;
x3limit = 0.5;
x4limit = 0.5;

N = 20;
Tsim = 50;

%% System matrices
Ac = [0 0 1 0; 0 0 0 1; -ks/m1 0 0 0; ks/m2 -ks/m2 0 0];
Bc = [0 0 1/m1 0]';
Cc = [0 1 0 0];
Ec = [0 0;0 0;1/m1 0;0 1/m2];

%% Dimensions
n = size(Ac,1);  % number of states
m = size(Bc,2);  % number of inputs
p = size(Cc,1);  % number of outputs


%% Initial state
% x0 = [-1; 1; -0.2; 0.2];
Ts = 0.1;

sysc = ss(Ac, [Bc Ec], Cc, zeros(1,size(Bc,2)));
sysd = c2d(sysc,Ts);

A = sysd.A; 
B = sysd.B(:,1); 
C = sysd.C; 
D = sysd.D; 
E = sysd.B(:,2:3);


%% Initial state
% x0 = [0; 0; 0; 0];
x0 = [-1; 1; -0.2; 0.2];
Q = C'*C;
R = 0.01;

[P, ~] = idare(A, B, Q, R, [], []);

%% Check the Reachibility, Observability, and R and Q Definitness
check_ABQR(A,B,Q,R);

%% Automatically tune the best prediction horizon (N)

Pu = [-1 1]';
qu = [1 1]';

Px = [ 0 0  1  0;
       0 0 -1  0;
       0 0  0  1;
       0 0  0 -1 ];

qx = [x3limit; x3limit; x4limit; x4limit];


%% TASK 1 (WITHOUT CONSTRAINT) QUADRATIC MINIMIZATION PROBLEM
% ESTABLISH THE LQR CONTROL USING QUADRATIC MINIMIZATION PROBLEM WITHOUT
% CONSTRAINT

[F,G] = predict_mats(A,B,N);

% a.) Use the function to obtain the matrices 𝑃𝑐, 𝑞𝑐, and 𝑆
[H,L,M] = cost_mats(F,G,Q,R,P);


% Loop for Qudaratic Minimization Problem
x = x0;
for k = 0:Tsim-1
% Step 1: Update prediction, cost, and constraint matrices
    f = L * x;

% Step 2: Solve QP for current state
    zopt = quadprog(H, f);

% Step 3: Extract and apply only the first control input
    u = zopt(1);
    y = C*x;
    xs(:,k+1) = x;
    us(:,k+1) = u;
    ys(:,k+1) = y;
    x = (A*x+B*u);
end

xs(:,Tsim+1) = x;
us(:,Tsim+1) = u;
ys(:,Tsim+1) = y;


%% TASK 1 (WITH INPUT CONSTRAINT) QUADRATIC MINIMIZATION PROBLEM
% ESTABLISH THE LQR CONTROL USING QUADRATIC MINIMIZATION PROBLEM WITH
% INPUT CONSTRAINT

[Pc_1,qc_1,Sc_1] = constraint_mats(F,G,Pu,qu,[],[],[],[]);


% Loop for Qudaratic Minimization Problem
x = x0;
for k = 0:Tsim-1

% Step 1: Update prediction, cost, and constraint matrices
    f = L * x;

% Step 2: Solve QP for current state
    zopt = quadprog(H, f, Pc_1, qc_1);

    % u = zopt(1,k+1);
    u = zopt(1);
    y = C*x;
    xs_1(:,k+1) = x;
    us_1(:,k+1) = u;
    ys_1(:,k+1) = y;
    x = (A*x+B*u);
end

xs_1(:,Tsim+1) = x;
us_1(:,Tsim+1) = u;
ys_1(:,Tsim+1) = y;



%% TASK 1 (WITH INPUT AND STATE CONSTRAINT) QUADRATIC PROGRAMMING
% ESTABLISH THE LQR CONTROL USING QUADRATIC MINIMIZATION PROBLEM WITH
% INPUT AND STATE CONSTRAINT

[Pc_2,qc_2,Sc_2] = constraint_mats(F,G,Pu,qu,Px,qx,Px,qx);

% Loop for Qudaratic Minimization Problem
x = x0;
for k = 0:Tsim-1

% Step 1: Update prediction, cost, and constraint matrices
    f = L * x;

% Step 2: Solve QP for current state
    zopt = quadprog(H, f, Pc_2, qc_2);

    u = zopt(1);
    y = C*x;
    xs_2(:,k+1) = x;
    us_2(:,k+1) = u;
    ys_2(:,k+1) = y;
    x = (A*x+B*u);
end

xs_2(:,Tsim+1) = x;
us_2(:,Tsim+1) = u;
ys_2(:,Tsim+1) = y;

figure

%% Subplot 1: Input u(k)
subplot(2,2,1)
hold on

stairs(0:Tsim, us, 'b-', 'LineWidth',1);
stairs(0:Tsim, us_1, 'r--', 'LineWidth',1);
stairs(0:Tsim, us_2, 'g-.', 'LineWidth',1);

grid on;

plot([0 Tsim], [-ulimit -ulimit], 'k:');
plot([0 Tsim], [ulimit ulimit], 'k:');

xlim([0 50]);
ylim([-3 3]);
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$u(k)$', 'Interpreter', 'latex');
legend(...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Unconstrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ Constrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Task 1 Quadratic minimization problem -- Input $u(k)$', 'Interpreter', 'latex');

%% Subplot 2: Output y(k)
subplot(2,2,2)
hold on

plot(0:Tsim, ys, 'b-', 'LineWidth', 1);
plot(0:Tsim, ys_1, 'r--', 'LineWidth',1);
plot(0:Tsim, ys_2, 'g-.', 'LineWidth',1);

grid on;

plot([0 Tsim], [-ulimit -ulimit], 'k:');
plot([0 Tsim], [ulimit ulimit], 'k:');

xlim([0 50]);
ylim([-5 5]);
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$y(k)$', 'Interpreter', 'latex');
legend(...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Unconstrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ Constrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Task 1 Quadratic minimization problem -- Output $y(k)$', 'Interpreter', 'latex');
hold on

%% Subplot 3: State x3(k)
subplot(2,2,3)
hold on

plot(0:Tsim, xs(3,:), 'b-', 'LineWidth', 1);
plot(0:Tsim, xs_1(3,:), 'r--', 'LineWidth',1);
plot(0:Tsim, xs_2(3,:), 'g-.', 'LineWidth',1);

grid on;

plot([0 Tsim], [-x3limit -x3limit], 'k:');
plot([0 Tsim], [x3limit x3limit], 'k:');

xlim([0 50]);
ylim([-3 3]);
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$x_3(k)$', 'Interpreter', 'latex');
legend(...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Unconstrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ Constrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Task 1 Quadratic minimization problem -- State $x_3(k)$', 'Interpreter', 'latex');
hold on

%% Subplot 4: State x(4)
subplot(2,2,4)
hold on

plot(0:Tsim, xs(4,:), 'b-', 'LineWidth', 1);
plot(0:Tsim, xs_1(4,:), 'r--', 'LineWidth',1);
plot(0:Tsim, xs_2(4,:), 'g-.', 'LineWidth',1);

grid on;

plot([0 Tsim], [-x4limit -x4limit], 'k:');
plot([0 Tsim], [x4limit x4limit], 'k:');

xlim([0 50]);
ylim([-3 3]);
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$x_4(k)$', 'Interpreter', 'latex');
legend(...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Unconstrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ Constrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Task 1 Quadratic minimization problem -- State $x_4(k)$', 'Interpreter', 'latex');
hold off

%% Phase Poytrait Analysis for constrain case
figure;

%% First subplot: x3 vs x4
subplot(1,1,1);
grid on;
hold on;

% Unconstrained: blue line with blue circles
plot(xs(3,:), xs(4,:), 'bo-', 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b');

% Input-constrained: red line with red circles
plot(xs_1(3,:), xs_1(4,:), 'ro-', 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r');

% Input + state-constrained: green line with green circles
plot(xs_2(3,:), xs_2(4,:), 'go-', 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerFaceColor', 'g');

% Initial and final points
% Unconstrained
plot(xs(3,1), xs(4,1), 'bs', 'MarkerSize', 7, 'LineWidth', 1.2, 'DisplayName', '$x_0$ (Unconstrained)', 'MarkerFaceColor','w');
plot(xs(3,end), xs(4,end), 'b*', 'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', '$x_f$ (Unconstrained)');

% Input constrained
plot(xs_1(3,1), xs_1(4,1), 'rs', 'MarkerSize', 7, 'LineWidth', 1.2, 'DisplayName', '$x_0$ (Input Constrained)', 'MarkerFaceColor','w');
plot(xs_1(3,end), xs_1(4,end), 'r*', 'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', '$x_f$ (Input Constrained)');

% Input + state constrained
plot(xs_2(3,1), xs_2(4,1), 'gs', 'MarkerSize', 7, 'LineWidth', 1.2, 'DisplayName', '$x_0$ (Input+State Constrained)', 'MarkerFaceColor','w');
plot(xs_2(3,end), xs_2(4,end), 'g*', 'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', '$x_f$ (Input+State Constrained)');

% plot(xs_2(3,:), xs_2(4,:), 'g-.', 'LineWidth', 1);
% Draw constraint lines
% Draw vertical lines at x1 = ±xlimit1

xline(-x3limit, 'k:','LineWidth', 1);  % Vertical line at x1 = -10
xline(x3limit,  'k:','LineWidth', 1);  % Vertical line at x1 = +10
yline(-x4limit, 'k:','LineWidth', 1);  % Horizontal line at x2 = -2
yline(x4limit,  'k:','LineWidth', 1);  % Horizontal line at x2 = +2
legend(...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Unconstrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ Constrained\ Problem', ...
    '$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
    '$x_0$ (Unconstrained)', '$x_f$ (Unconstrained)', ...
    '$x_0$ (Input Constrained)', '$x_f$ (Input Constrained)', ...
    '$x_0$ (Input+State Constrained)', '$x_f$ (Input+State Constrained)', ...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Phase Portrait between $x_3$ vs $x_4$', 'Interpreter', 'latex');
xlim([-3 3]);
ylim([-3 3]);
xlabel('$x_3$', 'Interpreter', 'latex');
ylabel('$x_4$', 'Interpreter', 'latex');

%% TASK 3: Investigate Limits of Operation for MPC

% Define grid of initial states (x3 and x4)
x3_vals = linspace(-0.6, 0.6, 25);  % Beyond the constraint slightly
x4_vals = linspace(-0.6, 0.6, 25);
[X3, X4] = meshgrid(x3_vals, x4_vals);
feasible_all = zeros(size(X3));
converged = zeros(size(X3));

for i = 1:length(x3_vals)
    for j = 1:length(x4_vals)
        x0_test = [0; 0; X3(j,i); X4(j,i)];
        x = x0_test;
        feasible = true;
        
        for k = 1:Tsim
            f = L * x;
            try
                zopt = quadprog(H, f, Pc_2, qc_2, [], [], [], [], [], optimoptions('quadprog','Display','off'));
                if isempty(zopt)
                    feasible = false;
                    break;
                end
            catch
                feasible = false;
                break;
            end
            u = zopt(1);
            x = A*x + B*u;
            if any(abs(x(3:4)) > [x3limit; x4limit])  % violation of constraints
                feasible = false;
                break;
            end
        end

        % Store feasibility
        feasible_all(j,i) = feasible;
        if feasible && norm(x(1:4)) < 1e-2  % converged close to origin
            converged(j,i) = 1;
        end
    end
end

%% Plotting Feasibility Map
figure;
imagesc(x3_vals, x4_vals, feasible_all);
set(gca, 'YDir', 'normal');
xlabel('$x_3(0)$', 'Interpreter', 'latex');
ylabel('$x_4(0)$', 'Interpreter', 'latex');
title('Feasibility Region for Initial States', 'Interpreter', 'latex');
colorbar;
colormap([1 1 1; 0.2 0.8 0.2]); % white = infeasible, green = feasible
axis tight;

figure;
imagesc(x3_vals, x4_vals, converged);
set(gca, 'YDir', 'normal');
xlabel('$x_3(0)$', 'Interpreter', 'latex');
ylabel('$x_4(0)$', 'Interpreter', 'latex');
title('Convergence to Origin from Initial States', 'Interpreter', 'latex');
colorbar;
colormap([1 1 1; 0.2 0.2 0.8]); % white = no convergence, blue = converged
axis tight;

% === PERFORMANCE METRICS ===

metrics = struct();

% Overshoot calculation helper
compute_overshoot = @(y) max(0, (max(y) - y(end)) / max(abs(y(end)), 1e-6));  % Avoid divide-by-zero

% Labels and data containers
labels = {'Unconstrained', 'InputConstrained', 'InputStateConstrained'};
xs_all = {xs, xs_1, xs_2};
us_all = {us, us_1, us_2};
ys_all = {ys, ys_1, ys_2};

for idx = 1:3
    x_data = xs_all{idx};
    u_data = us_all{idx};
    y_data = ys_all{idx};

    % === COST ===
    J = 0;
    for k = 1:Tsim
        xk = x_data(:,k);
        uk = u_data(:,k);
        J = J + xk'*Q*xk + uk'*R*uk;
    end

    % === CONTROL EFFORT ===
    Ueffort = sum(u_data.^2, 'all');

    % === SETTLING TIME === (states settle within 2% of final value)
    final_x = x_data(:,end);
    tol = 0.02 * abs(final_x + 1e-5);  % avoid divide-by-zero
    settle_idx = find(all(abs(x_data - final_x) < tol, 1), 1);
    settling_time = NaN;
    if ~isempty(settle_idx)
        settling_time = (settle_idx - 1) * Ts;
    end

    % === OVERSHOOT ===
    overshoot = compute_overshoot(y_data(1,:));  % Assuming y is scalar output

    % === Store in struct ===
    scenario = labels{idx};
    metrics.(scenario).Cost = J;
    metrics.(scenario).ControlEffort = Ueffort;
    metrics.(scenario).SettlingTime = settling_time;
    metrics.(scenario).Overshoot = overshoot;
end

% === Display metrics as table ===
summary_table = table(...
    [metrics.Unconstrained.Cost; metrics.InputConstrained.Cost; metrics.InputStateConstrained.Cost], ...
    [metrics.Unconstrained.ControlEffort; metrics.InputConstrained.ControlEffort; metrics.InputStateConstrained.ControlEffort], ...
    [metrics.Unconstrained.SettlingTime; metrics.InputConstrained.SettlingTime; metrics.InputStateConstrained.SettlingTime], ...
    [metrics.Unconstrained.Overshoot; metrics.InputConstrained.Overshoot; metrics.InputStateConstrained.Overshoot], ...
    'VariableNames', {'Cost', 'Control Effort', 'Settling Time', 'Overshoot'}, ...
    'RowNames', {'Unconstrained', 'Input Constrained', 'Input State Constrained'});

disp('Performance Metrics Summary:');
disp(summary_table);
