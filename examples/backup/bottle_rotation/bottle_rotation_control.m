% script to solve the bottle rotation example.
% This file compute the Jacobians, Goal descriptions, external forces,
%   guard conditions, etc, so as to call solvehfvc.m
%
% states
%   [x_o, y_o, z_0, qw_o, qx_o, qy_o, qz_o, x_h, y_h, z_h]'
%
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls
function [n_av, n_af, R_a, w_av, eta_af] = bottle_rotation_control()
addpath ../../algorithm
addpath generated

% weight
kObjectMass = 1.0;
kHandMass = 2.0;
kGravityConstant = 9.8;

% geometry
kObjectRadius = 0.04;
kObjectLength = 0.2;

% friction
kFrictionCoefficientTable = 0.8;
kFrictionCoefficientHand = 1.2;
kPointsPerFaceContact = 4;  % contact points between object and hand
kFrictionConeSides = 6;  % polyhedron approximation of friction cone
v_friction_directions = zeros(3, kFrictionConeSides);
kMinNormalForce = 0.5; % Newton

% inputs
p_WH = [0 0 0.5]';
q_WH = aa2quat(0.2, [1 0 0]');
kGoalVelocity = 0.5; % rad

for i = 1:kFrictionConeSides
    v_friction_directions(1, i) = sin(2*pi*i/kFrictionConeSides);
    v_friction_directions(2, i) = cos(2*pi*i/kFrictionConeSides);
end
% origin of object frame is at the center of its bottom
% origin of gripper frame is at the center of palm flange surface
p_OHC_all = zeros(3, kPointsPerFaceContact);
for i = 1:kPointsPerFaceContact
    p_OHC_all(1, i) = kObjectRadius*sin(2*pi*i/kPointsPerFaceContact);
    p_OHC_all(2, i) = kObjectRadius*cos(2*pi*i/kPointsPerFaceContact);
    p_OHC_all(3, i) = kObjectLength;
end
p_HHC_all = p_OHC_all;
p_HHC_all(3, :) = 0;

% object pose
p_WO = p_WH - quatOnVec([0 0 1]', q_WH)*kObjectLength;
q_WO = q_WH;
q = [p_WO; q_WO; p_WH; q_WH];
disp('Input q: ');
disp(q);

R_WO = quat2m(q_WO);
R_WH = quat2m(q_WH);
E_qO = 0.5*[-q_WO(2) -q_WO(3) -q_WO(4);
			q_WO(1) -q_WO(4) q_WO(3);
			q_WO(4) q_WO(1) -q_WO(2);
			-q_WO(3) q_WO(2) q_WO(1)];
E_qH = 0.5*[-q_WH(2) -q_WH(3) -q_WH(4);
			q_WH(1) -q_WH(4) q_WH(3);
			q_WH(4) q_WH(1) -q_WH(2);
			-q_WH(3) q_WH(2) q_WH(1)];
Omega = blkdiag(R_WO, E_qO, R_WH, E_qH);

% contact point with table
p_Wtemp = quatOnVec([0 0 1]', q_WO);
p_Wtemp(3) = 0;
p_Otemp = R_WO'*p_Wtemp;
p_Otemp(3) = 0;
p_OTC = p_Otemp/norm(p_Otemp)*kObjectRadius; % table contact
p_WTC = R_WO*p_OTC + p_WO;
% p_WTC(3) = kTableZ;

% rotation axis on the table
v_WRotAxis = R_WO*p_Otemp;
v_WRotAxis(3) = 0;
v_WRotAxis = v_WRotAxis/norm(v_WRotAxis);

% goal twist
t_WG = [-cross(v_WRotAxis, p_WTC); v_WRotAxis]*kGoalVelocity;
Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];

t_OG = Adj_g_WO_inv*t_WG;

G = [eye(6) zeros(6)];
b_G = t_OG;

% Holonomic constraints
Jac_phi_q = jac_phi_q_bottle_rotation(p_WO, q_WO, p_WH, q_WH, p_OTC, p_WTC, ...
        p_OHC_all, p_HHC_all);

% external force
F_WGO = [0 0 -kObjectMass*kGravityConstant]';
F_WGH = [0 0 -kHandMass*kGravityConstant]';
F = [R_WO'*F_WGO; zeros(3,1); R_WH'*F_WGH; zeros(3,1)];

% Guard Conditions
A = zeros(kFrictionConeSides*(1+kPointsPerFaceContact)+kPointsPerFaceContact, ...
        3*(1+kPointsPerFaceContact)+12);
z = [0 0 1]';
for i=1:kFrictionConeSides
    A(i, 1:3) = v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
    for j=1:kPointsPerFaceContact
        A(j*kFrictionConeSides+i, j*3+1:j*3+3) = ...
                v_friction_directions(:, i)' - kFrictionCoefficientHand*z';
    end
end
for i = 1:kPointsPerFaceContact
    A(kFrictionConeSides*(1+kPointsPerFaceContact) + i, 3*i+1:3*i+3) = -z';
end
b_A = [zeros(kFrictionConeSides*(1+kPointsPerFaceContact), 1);
       -kMinNormalForce*ones(kPointsPerFaceContact, 1)];

dims.Actualized      = 6;
dims.UnActualized    = 6;
dims.SlidingFriction = 0;
dims.Lambda          = 3*(1+kPointsPerFaceContact);

[n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, Jac_phi_q, ...
        G, b_G, F, [], [], A, b_A, dims, 'num_seeds', 1);


