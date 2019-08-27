% Derive jacobian function for 'levering up' example.
%
% convention for natural constraints:
%   written in the local frame whose last axis (Z in 3D, Y in 2D) aligns
%       with the contact normal; Positive value for this number must means
%       no contact, not penetration.

clear;clc;
% symbols
p_WO  = sym('p_WO',[2,1],'real');  % position of object in world frame
theta = sym('theta', 'real');  % object tilting angle, start from 0
p_WH  = sym('p_WH',[2,1],'real');  % position of hand in world frame

% location of object contacts in object frame
p_OHC = sym('p_OHC', [2,1], 'real'); % contact with hand
p_OTC = sym('p_OTC', [2,1], 'real'); % contact with table
p_OBC = sym('p_OBC', [2,1], 'real'); % contact with bin wall

% 2D rotation matrix
R_WO = aa2SO3(theta, [1 0 0]');
R_WO = R_WO(2:3, 2:3);

% all the holonomic constraints
holonomic_constraint = sym('Phi', [6, 1], 'real');
% Hand contact
holonomic_constraint(1:2) = R_WO'*p_WH - p_WO - p_OHC;
% table non-penetration
holonomic_constraint(3) = [0 1] * (R_WO*p_OTC+p_WO);
% bin wall non-penetration
holonomic_constraint(4) = [1 0] * (R_WO*p_OBC+p_WO);
% table sliding
holonomic_constraint(5) = [1 0] * (R_WO*p_OTC+p_WO);
% bin wall sliding friction
holonomic_constraint(6) = [0 1] * (R_WO*p_OBC+p_WO);

holonomic_constraint = simplify(holonomic_constraint);

save generated/derivation.mat;
disp('Computing derivatives:');

%% ---------------------------------------------------------------
%           calculate derivatives
% ---------------------------------------------------------------
% load generated/derivation
deriv_q  = @(f) [
        diff(f,'p_WO1'), diff(f,'p_WO2'), ...
        diff(f,'theta'), ...
        diff(f,'p_WH1'), diff(f,'p_WH2')];

Phi_q  = simplify(deriv_q(holonomic_constraint));
disp('Done. Generating file:');

%% ---------------------------------------------------------------
%           write to file
% ---------------------------------------------------------------
matlabFunction(Phi_q, 'File', 'generated/jacobian_levering_up', ...
    'vars', {p_WO, theta, p_WH, p_OHC, p_OTC, p_OBC});
save generated/derivation.mat;
disp('All done');
