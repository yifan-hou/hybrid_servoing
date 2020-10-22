% function to solve the block tilting example.
% This file compute the Jacobians, Goal descriptions, external forces,
%   guard conditions, etc, so as to call solvehfvc.m
%

% states
%   [x_o, y_o, z_0, qw_o, qx_o, qy_o, qz_o, x_h, y_h, z_h]'
%
% If run with no input, default state is used.
% If run with state input, compute the controls at the given state.
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls
function [n_av, n_af, R_a, w_av, eta_af] = block_tilting_control(inputs)
addpath ../../algorithm
addpath generated

% weight
kObjectMass = 0.5;
kHandMass = 0.3;
kGravityConstant = 9.8;

% friction
kFrictionCoefficientTable = 0.8;
kFrictionCoefficientHand = 0.8;
kFrictionConeSides = 6;  % polyhedron approximation of friction cone
v_friction_directions = zeros(3, kFrictionConeSides);
kMinNormalForce = 10; % Newton

% inputs
if nargin == 0
    kObjectEdgeLength = 0.075;
    p_WH = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    kGoalVelocity = 0.5; % rad
    kTiltDirection = [1 0 0]';
    m_project = diag([1 0 1]);

    % initial poses
    p_WH0 = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    p_WO0 = [kObjectEdgeLength/2, 0, 0]';
else
    p_WH = inputs.p_WH;

    kObjectEdgeLength = inputs.ObjectEdgeLength;
    kGoalVelocity     = inputs.kGoalRotationVelocity;
    kTiltDirection    = inputs.TiltDirection;
    m_project         = inputs.m_project;
    p_WH0             = inputs.p_WH0;
    p_WO0             = inputs.p_WO0;
end
kRotateAxis = cross([0 0 1]', kTiltDirection); % to the left
p_LineContact = p_WO0;
p_WO = p_WO0;
v_C2C0 = p_WH0 - p_LineContact;
q_WO = quatBTVec(m_project*v_C2C0, m_project*(p_WH - p_WO));

q = [p_WO; q_WO; p_WH];
disp('Input q: ');
disp(q);


for i = 1:kFrictionConeSides
    v_friction_directions(1, i) = sin(2*pi*i/kFrictionConeSides);
    v_friction_directions(2, i) = cos(2*pi*i/kFrictionConeSides);
end

p_OHC = p_WH0 - p_WO0;

% object pose
R_WO = quat2SO3(q_WO);
E_qO = 0.5*[-q_WO(2) -q_WO(3) -q_WO(4);
            q_WO(1) -q_WO(4) q_WO(3);
            q_WO(4) q_WO(1) -q_WO(2);
            -q_WO(3) q_WO(2) q_WO(1)];
Omega = blkdiag(R_WO, E_qO, eye(3));

% contact point with table
p_OTC_all = [kRotateAxis*kObjectEdgeLength/2, -kRotateAxis*kObjectEdgeLength/2];
p_WTC_all = R_WO*p_OTC_all + p_WO*[1, 1];

% goal twist
t_WG = [-cross(kRotateAxis, p_LineContact); kRotateAxis]*kGoalVelocity;
Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];

t_OG = Adj_g_WO_inv*t_WG;

G = [eye(6) zeros(6, 3)    ];
b_G = t_OG;


% external force
F_WGO = [0 0 -kObjectMass*kGravityConstant]';
F_WGH = [0 0 -kHandMass*kGravityConstant]';
F = [R_WO'*F_WGO; zeros(3,1); F_WGH];

% Guard conditions
%   hand contact is sticking; (force is in world frame)
%   table contacts are sticking;
%   hand contact normal force lower bound
A = zeros(kFrictionConeSides*(1+2)+1, ...
        3*(1+2)+9);
z = [0 0 1]';
for i=1:kFrictionConeSides
    A(i, 1:3) = (v_friction_directions(:, i)' - kFrictionCoefficientHand*z')*(R_WO');
    A(kFrictionConeSides+i, 4:6) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
    A(2*kFrictionConeSides+i, 7:9) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
end
A(3*kFrictionConeSides + 1, 1:3) = -z'*(R_WO');
b_A = [zeros(kFrictionConeSides*(1+2), 1); -kMinNormalForce];

% for Robustness computation
sticking_id = [1 2 3;
               4 5 6;
               7 8 9];
sliding_id = [];
sticking_mu = [kFrictionCoefficientHand;
               kFrictionCoefficientTable;
               kFrictionCoefficientTable];
sliding_mu = [];

dims.Actualized      = 3;
dims.UnActualized    = 6;
dims.SlidingFriction = 0;
dims.Lambda          = 3*(1+2);

% Jacobian of Holonomic constraints
Jac_phi_q = jac_phi_q_block_tilting(p_WO, q_WO, p_WH, p_OHC, p_WTC_all, p_OTC_all);
tic
[n_av, n_af, R_a, w_av, eta_af] = solvehfvc(dims, Jac_phi_q*Omega, ...
        G, b_G, F, [], [], A, b_A, 3, false);
toc


% display the scores
% disp(['non_jamming_score: ' num2str(scores.non_jamming_score)]);
% for i = 1:length(scores.sticking_contacts)
%     disp(['sticking contact ' num2str(i) ':']);
%     disp(['     non_jamming_score: ' num2str(scores.sticking_contacts{i}.non_jamming_score)]);
%     disp(['     engaging_score: ' num2str(scores.sticking_contacts{i}.engaging_score)]);
%     disp(['     non_slipping_score: ' num2str(scores.sticking_contacts{i}.non_slipping_score)]);
% end
% for i = 1:length(scores.sliding_contacts)
%     disp(['sliding contact ' num2str(i) ':']);
%     disp(['     non_jamming_score: ' num2str(scores.sliding_contacts{i}.non_jamming_score)]);
%     disp(['     engaging_score: ' num2str(scores.sliding_contacts{i}.engaging_score)]);
%     disp(['     non_sticking_score: ' num2str(scores.sliding_contacts{i}.non_sticking_score)]);
% end


