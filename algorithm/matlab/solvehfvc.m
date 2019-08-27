% Solve for hybrid force-velocity control actions
%
% Input arguments
%   required:
%       N_all: Linear constraints on generalized velocity.
%       G, b_G: Goal description, affine constraints on generalized velocity
%               G*v = b_G
%       F: External force vector. Same size as generalized force
%       Aeq, beq: Guard condition, Aeq*v = beq
%       A, b_A: Guard condition, A*v <= b_A
%       dims:
%           dims.Actualized: number of actualized dimensions
%           dims.UnActualized: number of unactualized dimensions
%           dims.SlidingFriction: number of sliding friction dimensions
%           dims.Lambda: number of reaction forces
%
%   optional:
%       num_seeds: number of random initializations to try when solving for
%               the velocity control
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls


function [n_av, n_af, R_a, w_av, eta_af, scores] = solvehfvc(N_all, G, b_G, F,...
        Aeq, beq, A, b_A, dims, sticking_id, ...
        sliding_id, mu_sticking, mu_sliding, varargin)

persistent para
if isempty(para)
    para = inputParser;
    validMatrix = @(x) isnumeric(x);
    validVector = @(x) isempty(x) || (isnumeric(x) && (size(x, 2) == 1));
    validStruct = @(x) isstruct(x);
    addRequired(para, 'N_all', validMatrix);
    addRequired(para, 'G', validMatrix);
    addRequired(para, 'b_G', validVector);
    addRequired(para, 'F', validVector);
    addRequired(para, 'Aeq', validMatrix);
    addRequired(para, 'beq', validVector);
    addRequired(para, 'A', validMatrix);
    addRequired(para, 'b_A', validVector);
    addRequired(para, 'dims', validStruct);
    addParameter(para, 'num_seeds', 1);
end

parse(para, N_all, G, b_G, F, Aeq, beq, A, b_A, dims, varargin{:});

kNumSeeds = para.Results.num_seeds;

% constants
kDimActualized      = dims.Actualized;
kDimUnActualized    = dims.UnActualized;
kDimSlidingFriction = dims.SlidingFriction;

N = N_all(1 : end - kDimSlidingFriction, :);
kDimLambda       = size(N, 1);
kDimContactForce = kDimLambda + kDimSlidingFriction;
kDimGeneralized  = kDimActualized + kDimUnActualized;

disp('============================================================');
disp('          Begin solving for velocity commands               ');
disp('============================================================');
disp('-------    Determine Possible Dimension of Control   -------');

NG = [N; G];

rank_N = rank(N);
rank_NG = rank(NG);

n_av_min = rank_NG - rank_N;
% n_av_max = kDimGeneralized - rank_N;
n_av = n_av_min;
n_af = kDimActualized - n_av;
% b_NG = [zeros(size(N, 1), 1); b_G];
NG_nullspace_basis = null(NG);
basis_c = null([NG_nullspace_basis';
        eye(kDimUnActualized), zeros(kDimUnActualized,kDimActualized)]);

disp(['r_N + n_a: ', num2str(rank_N + kDimActualized)]);
disp(['n_v: ', num2str(kDimGeneralized)]);
assert(rank_N + kDimActualized > kDimGeneralized);

disp('-------  Solve for Directions  -------')
n_c = rank_NG - kDimUnActualized;

% Projected gradient descent
NIter             = 50;
N_nullspace_basis = null(N);
BB                = basis_c'*basis_c;
NN                = N_nullspace_basis*(N_nullspace_basis');

cost_all = zeros(1, kNumSeeds);
k_all = rand([n_c, n_av, kNumSeeds]);
for seed = 1:kNumSeeds
    k  = k_all(:,:, seed);
    kn = normByCol(basis_c*k);
    k  = bsxfun(@rdivide, k, kn);

    for iter = 1:NIter
        % compute gradient
        g = zeros(n_c, n_av);
        costs = 0;
        for i = 1:n_av
            ki = k(:,i);
            for j = 1:n_av
                if i == j
                    continue;
                end
                kj = k(:,j);
                costs = costs + (ki'*BB*kj)^2;
                g(:, i) = g(:, i) + 2*(ki'*BB*kj)*BB*kj;
            end
            g(:, i) = g(:, i) - 2*(basis_c')*NN*basis_c*ki;
            costs   = costs - ki'*(basis_c')*NN*basis_c*ki;
        end
        % descent
        delta = 10;
        k     = k - delta*g;
        % project
        kn = normByCol(basis_c*k);
        k  = bsxfun(@rdivide, k, kn);
    end
    cost_all(seed) = costs;
    k_all(:,:,seed) = k;
    disp(['cost: ' num2str(costs)]);
end

[~, best_id] = min(cost_all);
k_best = k_all(:,:,best_id);
C_best = (basis_c*k_best)';

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);

b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;
w_av = C_best*v_star;


disp('============================================================');
disp('          Begin solving for force commands');
disp('============================================================');
% unactuated dimensions
H = [eye(kDimUnActualized), zeros(kDimUnActualized, kDimActualized)];
% Newton's laws
T_inv = T^-1;
M_newton = [zeros(kDimUnActualized, kDimContactForce) H*T_inv; ...
            T*N_all' eye(kDimGeneralized); ...
            Aeq];
b_newton = [zeros(size(H,1), 1); -T*F; beq];

M_free = M_newton(:, [1:kDimContactForce+kDimUnActualized, kDimContactForce+kDimUnActualized+n_af+1:end]);
M_eta_af = M_newton(:, [kDimContactForce+kDimUnActualized+1:kDimContactForce+kDimUnActualized+n_af]);

% prepare the QP
%   variables: [free_force, dual_free_force, eta_af]
n_free = kDimContactForce + kDimUnActualized + n_av;
n_dual_free = size(M_newton, 1);
% 0.5 x'Qx + f'x
qp.Q = diag([zeros(1, n_free + n_dual_free), ones(1, n_af)]);
qp.f = zeros(n_free + n_dual_free + n_af, 1);
% Ax<b
A_temp = [A(:, 1:kDimContactForce), A(:, kDimContactForce+1:end)*T_inv];
A_lambda_eta_u = A_temp(:, 1:kDimContactForce+kDimUnActualized);
A_eta_af = A_temp(:, kDimContactForce+kDimUnActualized+1:kDimContactForce+kDimUnActualized+n_af);
A_eta_av = A_temp(:, kDimContactForce+kDimUnActualized+n_af+1:end);

qp.A = [A_lambda_eta_u A_eta_av zeros(size(A, 1), n_dual_free) A_eta_af];
qp.b = b_A;
% Aeq = beq
qp.Aeq = [2*eye(n_free), M_free', zeros(n_free, n_af);
          M_free, zeros(size(M_free, 1)), M_eta_af];
qp.beq = [zeros(n_free, 1); b_newton];

options = optimoptions('quadprog', 'Display', 'final-detailed');
x = quadprog(qp.Q, qp.f, qp.A, qp.b, qp.Aeq, qp.beq, [], [], [],options);

disp('============================================================');
disp('                  Done.                                     ');
disp('============================================================');

lambda = x(1:kDimContactForce);
eta_af = x(n_free + n_dual_free + 1:end);

disp('World frame velocity:');
disp(R_a^-1*[zeros(n_af, 1); w_av]);

disp('World frame force:');
disp(R_a^-1*[eta_af; zeros(n_av, 1)]);

disp('Equality constraints violation:');
disp(sum(abs(qp.beq - qp.Aeq*x)));
disp('Inequality constraints b - Ax > 0 violation:');
b_Ax = qp.b - qp.A*x;
disp(sum(find(b_Ax < 0)));

return;

disp('============================================================');
disp('             Evaluate Robustness Criteria                   ');
disp('============================================================');

% 1. Jamming score
C = C_best;
b_C = w_av;
CRowN = [C; null(N_nullspace_basis')'];
S = svd(CRowN);
scores.non_jamming_score = S(end);

kNumOfStickingContacts = length(sticking_id);
kNumOfSlidingContacts = length(sliding_id);

scores.sticking_contacts = cell(kNumOfStickingContacts, 1);
scores.sliding_contacts = cell(kNumOfSlidingContacts, 1);

for i = 1:kNumOfStickingContacts
    Ni                          = N_all(sticking_id(i, 1), :);
    Nbari                       = N_all;
    Nbari(sticking_id(i, 1), :) = [];
    Lambda                      = [C; Nbari];
    scores.sticking_contacts{i}.non_jamming_score = normByRow(Ni*null(Lambda))';

    fc = lambda(sticking_id(i, :));
    fz = fc(end);
    scores.sticking_contacts{i}.engaging_score = fz;

    mu = mu_sticking(i);
    z = fc*0;
    z(end) = 1; % [0 1] for 2D, [0 0 1] for 3D
    scores.sticking_contacts{i}.non_slipping_score = ...
            (mu*z'*fc - norm(cross(z, fc)))*cos(atan(mu));
end

for i = 1:kNumOfSlidingContacts
    Ni    = N_all(sliding_id(i, :), :);
    Mi    = Ni;
    Nbari = N_all;
    Nbari(sliding_id(i, 1), :) = [];
    Lambda = [C; Nbari];
    Lambda_nullspace_basis = null(Lambda);
    b_Lambda = [b_C; zeros(size(Nbari,1), 1)];
    scores.sliding_contacts{i}.non_jamming_score = ...
            normByRow(Ni*Lambda_nullspace_basis)';

    fc = lambda(sliding_id(i, :));
    fz = fc(end);
    scores.sliding_contacts{i}.engaging_score = fz;

    mu = mu_sliding(i);
    z = fc*0;
    z(end) = 1; % [0 1] for 2D, [0 0 1] for 3D
    % compute vL
    MiNullLambda = Mi*Lambda_nullspace_basis;
    temp = MiNullLambda*pinv(MiNullLambda);
    vL = (eye(size(temp,1)) - temp)*Mi*pinv(Lambda)*b_Lambda;
    scores.sliding_contacts{i}.non_sticking_score = ...
            cos(atan(mu)) - norm(z'*vL)/norm(vL);
end


end