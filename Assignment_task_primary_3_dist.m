%% Problem setup
clear all
close all
clc

%% Define the each Variables

m1 = 1;
m2 = m1;
ks = 1;
N = 20;
Tsim = 100;

ulimit = 1;
ylimit = 2.7;
x3limit = 0.5;
x4limit = 0.5;
ylimit = 1;

%% System matrices
Ac = [0 0 1 0; 0 0 0 1; -ks/m1 0 0 0; ks/m2 -ks/m2 0 0];
Bc = [0 0 1/m1 0]';
Cc = [0 1 0 0];
Ec = [0 0; 0 0; 1/m1 0; 0 1/m2];

%% Dimensions
n = size(Ac,1);  % number of states
m = size(Bc,2);  % number of inputs
p = size(Cc,1);  % number of outputs

%% User-configurable disturbance flag
% The input disturbance exist
flag = 1;  % Set to 0 for zero disturbance, 1 for nonzero disturbance

if flag == 0
    d1 = 0;
    d2 = 0;
else
    d1 = 0.25;
    d2 = 0.7;
end

d = [d1;d2];

%% Initial state
Ts = 0.1;

sysc = ss(Ac, [Bc Ec], Cc, zeros(1,size(Bc,2)));
sysd = c2d(sysc,Ts);

%% Initiate from origins
flag_1 = 1;  % Set to 0 for zero disturbance, 1 for nonzero disturbance

if flag_1 == 0
    x0 = [-1; 1; -0.2; 0.2];
else
    x0 = [0; 0; 0; 0];
end

x = zeros(n, Tsim+1);   % state trajectory
x(:,1) = x0;         % initial state

u = zeros(1, Tsim);      % input trajectory
y = zeros(1, Tsim);      % output trajectory

Q = Cc'*Cc;
R = 1;
r = 1;

A = sysd.A; 
B = sysd.B(:,1); 
C = sysd.C; 
D = sysd.D; 
E = sysd.B(:,2:3);

[P, ~] = idare(A, B, Q, R, [], []);

%% Check the Reachibility, Observability, and R and Q Definitness
check_ABQR(A,B,Q,R);

%% Constraints
% Input constraints
P_u = [1; -1];   % 2x1
q_u = [1; 1];    % 2x1

% State constraints
P_x = [0 0 1 0;
       0 0 -1 0;
       0 0 0 1;
       0 0 0 -1];
q_x = [0.5; 0.5; 0.5; 0.5];

for k = 1:Tsim

    
    %% Step 1: Solve for x_ss, u_ss
    Tss = [eye(n)-A, -B; C, 0];
    RHS = [E*d; r];
    ans_ss = Tss \ RHS;
    x_ss = ans_ss(1:n);
    u_ss = ans_ss(n+1:end);

    %% Step 2: Compute prediction and cost matrices
    [F,G] = predict_mats(A,B,N);
    cond_F = cond(F);
    cond_G = cond(G);
    fprintf('Condition number of F: %f\n', cond_F);
    fprintf('Condition number of G: %f\n', cond_G);

    [H,L,M] = cost_mats(F,G,Q,R,P);
    
    %% Step 3: Shift constraints
    qx_bar = q_x - P_x * x_ss;
    qu_bar = q_u - P_u * u_ss;
    [Pc, qc, Sc] = constraint_mats(F,G,P_u,qu_bar,P_x,qx_bar,P_x,qx_bar);
    
    %% Step 4: Solve QP for deviation input sequence du
    x_dev = x(:,k) - x_ss;  % deviation state
    f = L * x_dev;
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'none');
    [z, ~, exitflag, output] = quadprog(H, f, Pc, qc + Sc*x_dev, [], [], [], [], [], options);

    if exitflag ~= 1
        disp("quadprog failed at step k = " + k);
        disp(output.message);
        z = zeros(N*m,1);  % fallback zero input sequence
    end
    
    du = z(1);              % first control increment
    u(:,k) = du + u_ss;     % actual control input
    
    %% Step 5: Simulate system
    x(:,k+1) = A * x(:,k) + B * u(:,k) + E* d;
    y(:,k) = C * x(:,k);
end

%% Analyze Steady-State Error and Overshoot

% Reference value (used in tracking)
ref = r;  % Should match the one used during simulation

% Steady-state error = |final y - reference|
steady_state_error = abs(y(end) - ref);

% Overshoot = max(y) - reference (if it exceeds reference)
overshoot = max(y) - ref;
if overshoot < 0
    overshoot = 0;  % no overshoot if response is always below reference
end

% Display results
fprintf('\n--- Performance Metrics ---\n');
fprintf('Reference value: %.4f\n', ref);
fprintf('Final output y(T): %.4f\n', y(end));
fprintf('Steady-state error: %.4f\n', steady_state_error);
fprintf('Overshoot: %.4f\n', overshoot);


%% [--- Plotting code unchanged, continues here ---]

% Plotting
time = 0:Tsim-1;

figure

%% Subplot 1: Output y(k)

subplot(2,2,1)
hold on
stairs(0:Tsim-1, u, 'b-', 'LineWidth', 1);
yline(ulimit, 'k:', 'LineWidth', 1);    % Upper input constraint
yline(-ulimit, 'k:', 'LineWidth', 1);   % Lower input constraint
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$u(k)$', 'Interpreter', 'latex');
legend('$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('Input $u(k)$', 'Interpreter', 'latex');
grid on
xlim([0 Tsim]);
ylim([-1.2*ulimit 1.2*ulimit]);


%% Subplot 2: Input u(k)
subplot(2,2,2)
hold on
plot(0:Tsim-1, y, 'b-', 'LineWidth', 1);
yline(ylimit, 'k:', 'LineWidth', 1);  % Output constraint
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$y(k)$', 'Interpreter', 'latex');
title('Output $y(k)$', 'Interpreter', 'latex');
legend('$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
grid on
xlim([0 Tsim]);
ylim([0 2 * ylimit]);


%% Subplot 3: State x3(k)
subplot(2,2,3)
hold on
plot(0:Tsim, x(3,:), 'b-', 'LineWidth', 1);
yline(x3limit, 'k:', 'LineWidth', 1);   % Upper state constraint
yline(-x3limit, 'k:', 'LineWidth', 1);  % Lower state constraint
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$x_3(k)$', 'Interpreter', 'latex');
legend('$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('State $x_3(k)$', 'Interpreter', 'latex');
grid on
xlim([0 Tsim]);
ylim([-1.2*x3limit 1.2*x3limit]);

%% Subplot 4: State x4(k)
subplot(2,2,4)
hold on
plot(0:Tsim, x(4,:), 'b-', 'LineWidth', 1);
yline(x4limit, 'k:', 'LineWidth', 1);   % Upper state constraint
yline(-x4limit, 'k:', 'LineWidth', 1);  % Lower state constraint
xlabel('$\mathrm{Time\ step,\ } k$', 'Interpreter', 'latex');
ylabel('$x_4(k)$', 'Interpreter', 'latex');
legend('$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Constrained\ Problem',...
    'Location', 'northeast', 'Interpreter', 'latex');
title('State $x_4(k)$', 'Interpreter', 'latex');
grid on
xlim([0 Tsim]);
ylim([-1.2*x4limit 1.2*x4limit]);

%% Phase Poytrait Analysis for constrain case
figure;

%% First subplot: x3 vs x4
subplot(1,1,1);
grid on;
hold on;

% Unconstrained: blue line with blue circles
plot(x(3,:), x(4,:), 'bo-', 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerFaceColor', 'b');

% Initial and final points
% Input and State constrained
plot(x(3,1), x(4,1), 'gs', 'MarkerSize', 7, 'LineWidth', 1.2, 'DisplayName', '$x_0$ (Unconstrained)', 'MarkerFaceColor','w');
plot(x(3,end), x(4,end), 'r*', 'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', '$x_f$ (Unconstrained)');

xline(-x3limit, 'k:','LineWidth', 1);  % Vertical line at x1 = -10
xline(x3limit,  'k:','LineWidth', 1);  % Vertical line at x1 = +10
yline(-x4limit, 'k:','LineWidth', 1);  % Horizontal line at x2 = -2
yline(x4limit,  'k:','LineWidth', 1);  % Horizontal line at x2 = +2
legend('$\mathbf{u}^{0}_{N}(0) = -H^{-1} L x(k)$ \textendash\ Input\ and\ State\ Constrained\ Problem',...
        '$x_0$ (Input+State Constrained)', '$x_f$ (Input+State Constrained)', ...
        'Location', 'northeast', 'Interpreter', 'latex');
title('Phase Portrait between $x_3$ vs $x_4$', 'Interpreter', 'latex');
xlim([-0.6 0.6]);
ylim([-0.6 0.6]);
xlabel('$x_3$', 'Interpreter', 'latex');
ylabel('$x_4$', 'Interpreter', 'latex');

% Define grid for initial x3 and x4 (velocity states)
x3_vals = linspace(-0.6, 0.6, 25);
x4_vals = linspace(-0.6, 0.6, 25);
[X3, X4] = meshgrid(x3_vals, x4_vals);

ROA = zeros(size(X3));   % Region of Attraction
ROF = zeros(size(X3));   % Region of Feasibility

% Loop over grid of initial states (x3, x4)
for i = 1:length(x3_vals)
    for j = 1:length(x4_vals)
        % Initial condition (assume other states zero)
        x0_test = [0; 0; X3(j,i); X4(j,i)];
        x = zeros(n, Tsim+1);
        x(:,1) = x0_test;
        feasible = true;

        for k = 1:Tsim
            % Step 1: solve x_ss, u_ss
            Tss = [eye(n)-A, -B; C, 0];
            RHS = [E*d; r];
            ans_ss = Tss \ RHS;
            x_ss = ans_ss(1:n);
            u_ss = ans_ss(n+1:end);
            
            % Step 2: predict and cost matrices
            [F,G] = predict_mats(A,B,N);
            [H,L,M] = cost_mats(F,G,Q,R,P);
            
            % Step 3: shift constraints
            qx_bar = q_x - P_x * x_ss;
            qu_bar = q_u - P_u * u_ss;
            [Pc, qc, Sc] = constraint_mats(F,G,P_u,qu_bar,P_x,qx_bar,P_x,qx_bar);
            
            % Step 4: solve QP
            x_dev = x(:,k) - x_ss;
            f = L * x_dev;
            options = optimoptions('quadprog','Display','off');
            [z,~,exitflag] = quadprog(H, f, Pc, qc + Sc*x_dev, [], [], [], [], [], options);
            
            if exitflag ~= 1
                feasible = false;
                break;
            end
            
            du = z(1);
            u_k = du + u_ss;
            x(:,k+1) = A*x(:,k) + B*u_k + E*d;
            
            % Check state and input constraints manually
            if any(P_x * x(:,k+1) > q_x + 1e-3) || any(P_u * u_k > q_u + 1e-3)
                feasible = false;
                break;
            end
        end
        
        % Check if system converged close to zero
        if norm(x(:,end),2) < 1e-2 && feasible
            ROA(j,i) = 1;
        end
        if feasible
            ROF(j,i) = 1;
        end
    end
end

% Plot Region of Attraction
figure;
imagesc(x3_vals, x4_vals, ROA);
axis xy;
xlabel('$x_3(0)$', 'Interpreter', 'latex');
ylabel('$x_4(0)$', 'Interpreter', 'latex');
title('Convergence to Origin (Region of Attraction)', 'Interpreter', 'latex');
colorbar;

% Plot Region of Feasibility
figure;
imagesc(x3_vals, x4_vals, ROF);
axis xy;
xlabel('$x_3(0)$', 'Interpreter', 'latex');
ylabel('$x_4(0)$', 'Interpreter', 'latex');
title('Feasibility Region for Initial States', 'Interpreter', 'latex');
colorbar;

% === PERFORMANCE METRICS ===
metrics = struct();

compute_overshoot = @(y) max(0, (max(y) - y(end)) / max(abs(y(end)), 1e-6));  % Avoid divide-by-zero

labels = {'Constrained'};
xs_all = {x};
us_all = {u};
ys_all = {y};

for idx = 1:length(labels)
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

    % === SETTLING TIME ===
    final_x = x_data(:,end);
    tol = 0.02 * abs(final_x + 1e-5);
    settle_idx = find(all(abs(x_data - final_x) < tol, 1), 1);
    settling_time = NaN;
    if ~isempty(settle_idx)
        settling_time = (settle_idx - 1) * Ts;
    end

    % === OVERSHOOT ===
    overshoot = compute_overshoot(y_data(1,:));

    scenario = labels{idx};
    metrics.(scenario).Cost = J;
    metrics.(scenario).ControlEffort = Ueffort;
    metrics.(scenario).SettlingTime = settling_time;
    metrics.(scenario).Overshoot = overshoot;
end

% === Display summary ===
summary_table = table(... 
    metrics.Constrained.Cost, ...
    metrics.Constrained.ControlEffort, ...
    metrics.Constrained.SettlingTime, ...
    100 * metrics.Constrained.Overshoot, ...
    'VariableNames', {'Cost', 'ControlEffort', 'SettlingTime', 'OvershootPercent'}, ...
    'RowNames', {'Constrained'});

disp('Performance Metrics Summary:');
disp(summary_table);

