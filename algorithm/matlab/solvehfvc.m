%% Solve the hybrid servoing problem. The algorithm solves for a hybrid force-velocity control command described by
% dimensionality @p n_av, @p n_af, direction @p R_a and magnitudes @p w_av, @p eta_af.
%
% The constraints are natural holonomic constraints: @p N_all * v = 0, goal constraints @p G*v = @p b_G, static force balance, and contact mode guard conditions.
%
% The system has kDOF = dims.Actualized +
% dims.UnActualized dimensions. This means the generalized force f and
% generalized velocity v are kDOF dimensional.
% The first dims.UnActualized dimensions correspond to the free DOFs in the
% system (such as free objects), while the last dims.Actualized DOFs correspond
% to the controllable DOFs (such as robot joints.)
%
%
% N_all: Linear constraints on generalized velocity. N_all v = 0
% N_u: unilateral constraints on generalized velocity.
% G, b_G: Goal description, affine constraints on generalized velocity
%         G*v = b_G
% F: External force vector. Same size as generalized force
% Aeq, beq: Guard condition, Aeq*v = beq
% A, b_A: Guard condition, A*v <= b_A
% dims:
%     dims.Actualized: number of actualized dimensions
%     dims.UnActualized: number of unactualized dimensions
%     dims.SlidingFriction: number of sliding friction dimensions
%
%   optional:
%       num_seeds: number of random initializations to try when solving for
%               the velocity control
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions. f first, v latter.
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls
%
%   TODO: the use of dims.slidingfriction is weird
%         handle n_av_min=0 return in c++ code

function [n_av, n_af, R_a, w_av, eta_af] = solvehfvc(dims, N_all, N_u, G, ...
    b_G, F, Aeq, beq, A, b_A, num_seeds, print)

persistent para
if isempty(para)
    para = inputParser;
    validScalar = @(x) isnumeric(x)&&isscalar(x);
    validMatrix = @(x) isnumeric(x);
    validVector = @(x) isempty(x) || (isnumeric(x) && (size(x, 2) == 1));
    validStruct = @(x) isstruct(x);
    validBool   = @(x) islogical(x);
    addRequired(para, 'dims', validStruct);
    addRequired(para, 'N_all', validMatrix);
    addRequired(para, 'G', validMatrix);
    addRequired(para, 'b_G', validVector);
    addOptional(para, 'F', validVector);
    addOptional(para, 'Aeq', validMatrix);
    addOptional(para, 'beq', validVector);
    addOptional(para, 'A', validMatrix);
    addOptional(para, 'b_A', validVector);
    addOptional(para, 'num_seeds', 1, validScalar);
    addOptional(para, 'print', true, validBool);
end

parse(para, dims, N_all, G, b_G, F, Aeq, beq, A, b_A, num_seeds, print);

kNumSeeds = num_seeds;

% constants
kDimActualized      = dims.Actualized;
kDimUnActualized    = dims.UnActualized;
kDimSlidingFriction = dims.SlidingFriction;

N = N_all(1 : end - kDimSlidingFriction, :);
kDimLambda       = size(N, 1);
kDimContactForce = kDimLambda + kDimSlidingFriction;
kDimGeneralized  = kDimActualized + kDimUnActualized;

if print
    disp('============================================================');
    disp('          Begin solving for velocity commands               ');
    disp('============================================================');
    disp('-------    Determine Possible Dimension of Control   -------');
end

NG = [N; G];

rank_N = rank(N);
rank_NG = rank(NG);

n_av_min = rank_NG - rank_N;

% n_av_max = kDimGeneralized - rank_N;
n_av = n_av_min;
n_af = kDimActualized - n_av;
if n_av_min == 0
    R_a = [];
    w_av = [];
    eta_af = [];
    return;
end

% b_NG = [zeros(size(N, 1), 1); b_G];
NG_nullspace_basis = null(NG);
basis_c = null([NG_nullspace_basis';
        eye(kDimUnActualized), zeros(kDimUnActualized,kDimActualized)]);

if print
    disp(['r_N + n_a: ', num2str(rank_N + kDimActualized)]);
    disp(['n_v: ', num2str(kDimGeneralized)]);
    disp('-------  Solve for Directions  -------')
end
assert(rank_N + kDimActualized > kDimGeneralized);

n_c = rank_NG - kDimUnActualized;
b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;

% Projected gradient descent
NIter             = 50;
N_nullspace_basis = null(N);
BB                = basis_c'*basis_c;
NN                = N_nullspace_basis*(N_nullspace_basis');

% added a term:
%  min kCoefNu*c_i'*v_star*N_u*c_i

VstarN_u = normc(v_star)*sum(normr(N_u));
kCoefNu = 1;

cost_all = zeros(1, kNumSeeds);
k_all = rand([n_c, n_av, kNumSeeds]);
for seed = 1:kNumSeeds
    k  = k_all(:,:, seed);
    kn = normByCol(basis_c*k);
    k  = bsxfun(@rdivide, k, kn);

    for iter = 1:NIter
        % compute gradient0
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
            g(:, i) = g(:, i) + kCoefNu*2*(basis_c')*VstarN_u*basis_c*ki;
            costs   = costs - ki'*(basis_c')*NN*basis_c*ki;
            costs   = costs + kCoefNu*ki'*(basis_c')*VstarN_u*basis_c*ki;
        end
        disp(['cost: ' num2str(costs)]);
        % descent
        delta = 0.1;
        k     = k - delta*g;
        % project
        kn = normByCol(basis_c*k);
        k  = bsxfun(@rdivide, k, kn);
    end
    cost_all(seed) = costs;
    k_all(:,:,seed) = k;
    disp('-------------------');
    if print
        disp(['cost: ' num2str(costs)]);
    end
end

[~, best_id] = min(cost_all);
k_best = k_all(:,:,best_id);
C_best = (basis_c*k_best)';

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);


w_av = C_best*v_star;

if isempty(F)
    return
end

if print
    disp('============================================================');
    disp('          Begin solving for force commands');
    disp('============================================================');
end
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

lambda = x(1:kDimContactForce);
eta_af = x(n_free + n_dual_free + 1:end);

if print
    disp('============================================================');
    disp('                  Done.                                     ');
    disp('============================================================');

    disp('World frame velocity:');
    disp(R_a^-1*[zeros(n_af, 1); w_av]);

    disp('World frame force:');
    disp(R_a^-1*[eta_af; zeros(n_av, 1)]);

    disp('Equality constraints violation:');
    disp(sum(abs(qp.beq - qp.Aeq*x)));
    disp('Inequality constraints b - Ax > 0 violation:');
    b_Ax = qp.b - qp.A*x;
    disp(sum(find(b_Ax < 0)));
end

return;



% disp('============================================================');
% disp('             Evaluate Robustness Criteria                   ');
% disp('============================================================');
% require additional arguments:
%   sticking_id
%   sliding_id
%   mu_sticking
%   mu_sliding

% % 1. Jamming score
% C = C_best;
% b_C = w_av;
% CRowN = [C; null(N_nullspace_basis')'];
% S = svd(CRowN);
% scores.non_jamming_score = S(end);

% kNumOfStickingContacts = length(sticking_id);
% kNumOfSlidingContacts = length(sliding_id);

% scores.sticking_contacts = cell(kNumOfStickingContacts, 1);
% scores.sliding_contacts = cell(kNumOfSlidingContacts, 1);

% for i = 1:kNumOfStickingContacts
%     Ni                          = N_all(sticking_id(i, 1), :);
%     Nbari                       = N_all;
%     Nbari(sticking_id(i, 1), :) = [];
%     Lambda                      = [C; Nbari];
%     scores.sticking_contacts{i}.non_jamming_score = normByRow(Ni*null(Lambda))';

%     fc = lambda(sticking_id(i, :));
%     fz = fc(end);
%     scores.sticking_contacts{i}.engaging_score = fz;

%     mu = mu_sticking(i);
%     z = fc*0;
%     z(end) = 1; % [0 1] for 2D, [0 0 1] for 3D
%     scores.sticking_contacts{i}.non_slipping_score = ...
%             (mu*z'*fc - norm(cross(z, fc)))*cos(atan(mu));
% end

% for i = 1:kNumOfSlidingContacts
%     Ni    = N_all(sliding_id(i, :), :);
%     Mi    = Ni;
%     Nbari = N_all;
%     Nbari(sliding_id(i, 1), :) = [];
%     Lambda = [C; Nbari];
%     Lambda_nullspace_basis = null(Lambda);
%     b_Lambda = [b_C; zeros(size(Nbari,1), 1)];
%     scores.sliding_contacts{i}.non_jamming_score = ...
%             normByRow(Ni*Lambda_nullspace_basis)';

%     fc = lambda(sliding_id(i, :));
%     fz = fc(end);
%     scores.sliding_contacts{i}.engaging_score = fz;

%     mu = mu_sliding(i);
%     z = fc*0;
%     z(end) = 1; % [0 1] for 2D, [0 0 1] for 3D
%     % compute vL
%     MiNullLambda = Mi*Lambda_nullspace_basis;
%     temp = MiNullLambda*pinv(MiNullLambda);
%     vL = (eye(size(temp,1)) - temp)*Mi*pinv(Lambda)*b_Lambda;
%     scores.sliding_contacts{i}.non_sticking_score = ...
%             cos(atan(mu)) - norm(z'*vL)/norm(vL);
% end


end