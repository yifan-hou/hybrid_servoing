% function to solve the control for the levering up problem at one
% timestep..
% This file compute the Jacobians, Goal descriptions, external forces,
%   guard conditions, etc, so as to call solvehfvc.m
%
% states
%   [y_o, z_0, theta_o, y_h, z_h]'
% choice of coordinate frames
%   see figure
% Goal is specified as the velocity of hand contact in z direction
%
% Input arguments
%   Input: specify the state of the system. If empty, use default state.
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls
function [n_av, n_af, R_a, w_av, eta_af] = levering_up_control(inputs)
addpath ../../algorithm/matlab
addpath generated

% how to cope with sliding friciton
% 1. separate jacobian for velocity and force. force has more entries for friction
% 2. Guard condition:
%       equality constraint on friction and normal force
%       normal force upper limit
% 3. for sliding friciton, friciton coefficient should use upper bound

config = yaml.ReadYaml('../../experiments/levering_up/config/task.yaml');

% weight
kObjectMass = config.task.object_mass;
kHandMass = config.task.hand_mass;
kGravityConstant = 9.8;

% length
kObjectLength = config.task.object_length;
kObjectThickness = config.task.object_thickness;

% friction
kFrictionCoefficientTable = config.task.friction_coef.table_object; % upper bound to ensure sliding
kFrictionCoefficientHand = config.task.friction_coef.hand_object; % lower bound to ensure sticking
kFrictionCoefficientBin = config.task.friction_coef.bin_object; % upper bound to ensure sliding
kMinNormalForce = config.task.min_normal_force_sticking; % Newton
kMinNormalForceSliding = config.task.min_normal_force_sliding; % Newton
kMaxNormalForceSliding = config.task.max_normal_force_sliding; % Newton

kGoalRotationVelocity = config.task.goal_rotation_velocity; % rad

% dimensions
kDimGeneralized = 5;
kDimUnActualized = 3;
kDimActualized = 2;
kDimLambda = 4;
kDimSlidingFriction = 2;


% inputs
if nargin == 0
    % feedback
    p_WH = [kObjectLength, kObjectThickness/2]';

    % initial poses
    p_WH0 = [kObjectLength, kObjectThickness/2]';
    p_WO0 = [kObjectLength/2 - 0.0065, kObjectThickness/2-0.003]';
else
    % feedback
    p_WH = inputs.p_WH;

    % initial poses
    p_WH0 = inputs.p_WH0;
    p_WO0 = inputs.p_WO0;
end


% compute object pose
%   Ideally we should use perception; here we hack it by assuming the
%   contact between the hand and the object is sticking, and solve the object
%   pose from hand pose

% 1. Solve for theta
%   This is a a*sin(theta)+bsin(theta)=c problem
a = p_WH0(2) - p_WO0(2) + kObjectThickness/2;
b = kObjectLength;
c = p_WH(2) - p_WO0(2) + kObjectThickness/2;
phi = atan2(a, b);
theta_plus_phi = asin(c/norm([a, b]));
theta = theta_plus_phi - phi;
% 2. solve for z
l_diagonal = sqrt(kObjectThickness^2 + kObjectLength^2);
angle_inner_sharp = asin(kObjectThickness/l_diagonal);
p_temp = [kObjectThickness*sin(theta)+l_diagonal/2*cos(theta+angle_inner_sharp);
        l_diagonal/2*sin(theta+angle_inner_sharp)];
p_temp_w = p_WO0 - [kObjectLength/2; kObjectThickness/2];
p_WO = p_temp + p_temp_w;

R_WO = aa2SO3(theta, [1 0 0]');
R_WO = R_WO(2:3, 2:3);

% contact point with Table, Bin and Hand
p_OHC = p_WH0 - p_WO0;
p_OTC = [-kObjectLength/2; -kObjectThickness/2];
p_OBC = [-kObjectLength/2; kObjectThickness/2];
% goal
G = [0 0 1 0 0];
b_G = kGoalRotationVelocity;

% Holonomic constraints
Jacobian_all = jacobian_levering_up(p_WO, theta, p_WH, p_OHC, p_OTC, p_OBC);
% return;

% external force
F_WGO = [0 -kObjectMass*kGravityConstant]';
F_WGH = [0 -kHandMass*kGravityConstant]';
F = [F_WGO; 0; F_WGH];

% Guard Conditions
%   Inequality A*lambda<b_A
%       hand contact is sticking; (force is in world frame)
%       hand contact normal force lower bound
%       table contact normal force lower bound
%       table contact normal force upper bound
%       binwall contact normal force lower bound
%       binwall contact normal force upper bound

%   Equality Aeq*lambda = beq
%       table contact is sliding;
%       bin wall contact is sliding;
% lambda: f_why, f_whx, f_table_normal, f_binwall_normal,
%         f_table_friction, f_binwall_friction
A = zeros(2 + 1 + 4, kDimLambda + kDimSlidingFriction + kDimGeneralized);
y = [1 0]';
z = [0 1]';
A(1, 1:2) = (z' - kFrictionCoefficientHand*y')*(R_WO');
A(2, 1:2) = (-z' - kFrictionCoefficientHand*y')*(R_WO');
A(3, 1:2) = -y'*(R_WO');
A(4, 3) = -1;
A(5, 3) = 1;
A(6, 4) = -1;
A(7, 4) = 1;
b_A = [0 0 -kMinNormalForce -kMinNormalForceSliding kMaxNormalForceSliding ...
        -kMinNormalForceSliding kMaxNormalForceSliding]';

Aeq = zeros(2, kDimLambda + kDimSlidingFriction + kDimGeneralized);
% % % -y'*ftable = mu*z'*ftable
Aeq(1, [5, 3]) = kFrictionCoefficientTable*z'+y';
% % z'*f = mu*y'*f
Aeq(2, [4, 6]) = kFrictionCoefficientBin*y'-z';
beq = [0; 0];


dims.Actualized      = kDimActualized;
dims.UnActualized    = kDimUnActualized;
dims.SlidingFriction = kDimSlidingFriction;
dims.Lambda          = kDimLambda;

[n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Jacobian_all, ...
        G, b_G, F, Aeq, beq, A, b_A, dims, 'num_seeds', 3);
