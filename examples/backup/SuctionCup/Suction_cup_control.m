function [n_av, n_af, R_a, w_av, eta_af] = Suction_cup_control(inputs)
%SUCTION_CUP_CONTROL Solve the suction cup block tilting example.
%   [N_AV, N_AF, R_A, W_AV, ETA_AF] = SUCTION_CUP_CONTROL(INPUTS) computes the
%   dimension, direction and magnitude of force and velocity control for the
%   certain time step, given inputs INPUTS.
%
%   [N_AV, N_AF, R_A, W_AV, ETA_AF] = SUCTION_CUP_CONTROL() do the same
%   computation with default inputs.
%
%   INPUTS is a structure containing the following fields:
%       INPUTS.p_WH: 3x1, hand location in world frame (m)
%       INPUTS.q_WH: 4x1, hand orientation in world frame as a quaternion
%       INPUTS.TiltDirection: 3x1 unit vector, direction of pushing in world
%           frame. Must be one of [1 0 0]', [-1 0 0]', [0 1 0]', [0 -1 0]'
%       INPUTS.p_WH0: 3x1 initial hand location in world frame (m)
%       INPUTS.p_LineCenter: 3x1 world coordinate of the center point of the
%           contacting edge between the object and the table.
%   N_AV: number, dimensionality of velocity controlled actions
%   N_AF: number, dimensionality of force controlled actions
%   R_A: (N_AV+N_AF)x(N_AV+N_AF) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   W_AV: N_AV x 1 vector, magnitudes of velocity controls
%   ETA_AF: N_AF x 1 vector,  magnitudes of force controls
%
%   Example:
%   This shows the default inputs.
%       kObjectEdgeLength = 0.075;
%       ip.p_WH           = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
%       ip.q_WH           = [1 0 0 0]';
%       ip.kTiltDirection = [1 0 0]';
%       ip.p_WH0          = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
%       ip.p_LineCenter   = [kObjectEdgeLength/2, 0, 0]';
%       [n_av, n_af, R_a, w_av, eta_af] = Suction_cup_control(ip);
addpath ../../algorithm

% Physics
kObjectMass = 0.5;
kHandMass = 0.0;
kGravityConstant = 9.8;
kObjectEdgeLength = 0.075;
kGoalVelocity = 0.5; % rad

% contact forces
kFrictionCoefficientTable = 0.8;
kFrictionConeSides = 6;  % polyhedron approximation of friction cone
kMinNormalForce = 5; % Newton

v_friction_directions = zeros(3, kFrictionConeSides);
for i = 1:kFrictionConeSides
    v_friction_directions(1, i) = sin(2*pi*i/kFrictionConeSides);
    v_friction_directions(2, i) = cos(2*pi*i/kFrictionConeSides);
end

% inputs
if nargin == 0
    p_WH           = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    q_WH           = [1 0 0 0]';
    kTiltDirection = [1 0 0]';
    p_WH0          = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    p_LineCenter   = [kObjectEdgeLength/2, 0, 0]';
else
    p_WH           = inputs.p_WH;
    q_WH           = inputs.q_WH;
    kTiltDirection = inputs.TiltDirection;
    p_WH0          = inputs.p_WH0;
    p_LineCenter   = inputs.p_LineCenter;
end

% inputs
%   p_WH, p_WO
%   object frame origin is at the center of the contact edge
p_WO = p_LineCenter;
q_WO = quatBTVec(p_WH0 - p_LineCenter, p_WH - p_LineCenter);
R_WO = quat2SO3(q_WO);

% contact point with table
kRotateAxis = cross([0 0 1]', kTiltDirection); % to the left
p_WTC1 = p_WO + kRotateAxis*kObjectEdgeLength/2;
p_WTC2 = p_WO - kRotateAxis*kObjectEdgeLength/2;

% Jacobian of Holonomic constraints
Adj_WH = SE32Adj(pose2SE3([p_WH; q_WH]));
Adj_WO = SE32Adj(pose2SE3([p_WO; q_WO]));
N = [Adj_WH      -Adj_WO;
     zeros(3, 6) [eye(3), -wedge(p_WTC1)]*Adj_WO;
     zeros(3, 6) [eye(3), -wedge(p_WTC2)]*Adj_WO];

% goal twist
Adj_g_WO_inv = SE32Adj(SE3inv(pose2SE3([p_WO; q_WO])));
t_WG = [-cross(kRotateAxis, p_LineCenter); kRotateAxis]*kGoalVelocity;
t_OG = Adj_g_WO_inv*t_WG;

G = [eye(6) zeros(6, 6)];
b_G = t_OG;

% external force
R_WH = quat2SO3(q_WH);
F_WGO = [0 0 -kObjectMass*kGravityConstant]';
F_WGH = [0 0 -kHandMass*kGravityConstant]';
F     = [R_WO'*F_WGO; zeros(3,1); R_WH'*F_WGH; zeros(3,1)];

% Guard conditions
%   hand contact is sticking; (force within upper/lower bounds)
%   table contacts are sticking;
%   table contact normal force lower bound
A = zeros(12 + kFrictionConeSides*2+2, 12+12);
z = [0 0 1]';
Flimit = 1*[4.5 4.5 10 0.054 0.056 0.028]';

A(1:6, 19:24) = eye(6);
A(7:12, 19:24) = -eye(6);
for i=1:kFrictionConeSides
    A(12+i, 7:9) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
    A(12+kFrictionConeSides+i, 10:12) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
end
A(end-1, 9) = -1;
A(end, 12) = -1;
b_A = [Flimit; Flimit;
       zeros(kFrictionConeSides*2, 1);
       -kMinNormalForce; -kMinNormalForce];

dims.Actualized      = 6;
dims.UnActualized    = 6;
dims.SlidingFriction = 0;

tic
[n_av, n_af, R_a, w_av, eta_af] = solvehfvc(N, ...
        G, b_G, F, [], [], A, b_A, dims, [], ...
        [], [], [], 'num_seeds', 3);
toc
