% Derive jacobian function for 'block tilting' example.
%
% convention for natural constraints:
%   written in the local frame whose last axis (Z in 3D, Y in 2D) aligns
%       with the contact normal; Positive value for this number must means
%       no contact, not penetration.
clear;clc;
% symbols
%   poses in world frame
p_WO     = sym('p_WO',[3,1],'real');  % object position
q_WO     = sym('q_WO',[4,1],'real');  % object quaternion
p_WH     = sym('p_WH',[3,1],'real');  % hand position

p_OHC = sym('p_OHC',[3,1],'real');  % hand contact in object frame

%   table contacts
p_WTC_all = sym('p_WTC_all',[3, 2],'real'); % in world frame
p_OTC_all = sym('p_OTC_all',[3, 2],'real'); % in object frame

holonomic_constraint = sym('Phi', [3*(1+2), 1], 'real');
% Hand contact
holonomic_constraint(1:3) = quatOnVec(p_WH, quatInv(q_WO)) - p_WO - p_OHC;

% table contacts
holonomic_constraint(4:6) = quatOnVec(p_OTC_all(:,1), q_WO)+p_WO-p_WTC_all(:,1);
holonomic_constraint(7:9) = quatOnVec(p_OTC_all(:,2), q_WO)+p_WO-p_WTC_all(:,2);

holonomic_constraint = simplify(holonomic_constraint);

save generated/derivation.mat;
disp('Computing derivatives:');

%% ---------------------------------------------------------------
%           calculate derivatives
% ---------------------------------------------------------------
% load generated/derivation
deriv_q  = @(f) [
        diff(f,'p_WO1'), diff(f,'p_WO2'), diff(f,'p_WO3'), ...
        diff(f,'q_WO1'), diff(f,'q_WO2'), diff(f,'q_WO3'), diff(f, 'q_WO4'), ...
        diff(f,'p_WH1'), diff(f,'p_WH2'), diff(f,'p_WH3')];

Phi_q  = simplify(deriv_q(holonomic_constraint));
disp('Done. Generating file:');

%% ---------------------------------------------------------------
%           write to file
% ---------------------------------------------------------------
matlabFunction(Phi_q, 'File', 'generated/jac_phi_q_block_tilting', ...
        'vars', {p_WO, q_WO, p_WH, p_OHC, p_WTC_all, p_OTC_all});
save generated/derivation.mat;
disp('All done');
