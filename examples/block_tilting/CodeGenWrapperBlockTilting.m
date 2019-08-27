function [n_av, n_af, R_a, w_av, eta_af] = ...
        CodeGenWrapperBlockTilting(p_WO, q_WO, p_WH, p_OHC, p_WTC_all, p_OTC_all, Omega, G, b_G, F, A, b_A, dims)

% Jacobian of Holonomic constraints
Jac_phi_q = jac_phi_q_block_tilting(p_WO, q_WO, p_WH, p_OHC, p_WTC_all, p_OTC_all);

[n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, Jac_phi_q, ...
        G, b_G, F, [], [], A, b_A, dims, 'num_seeds', 1);
    