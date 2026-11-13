"""Lexicographic least-squares QPs with shared hard linear constraints."""

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import null_space
from scipy.optimize import LinearConstraint, minimize


class QPFailure(RuntimeError):
    pass


def solve_qp(matrix, target, constraints, lower, upper):
    """Minimize ||matrix*x-target||², validating even inaccurate solver results."""
    size = matrix.shape[1]
    hessian = matrix.T @ matrix + 1e-9 * np.eye(size)
    row_scale = np.maximum(np.linalg.norm(constraints, axis=1), 1e-8)
    solver = osqp.OSQP()
    solver.setup(
        P=sparse.csc_matrix(np.triu(hessian)), q=-(matrix.T @ target),
        A=sparse.csc_matrix(constraints / row_scale[:, None]),
        l=lower / row_scale, u=upper / row_scale,
        verbose=False, eps_abs=1e-7, eps_rel=1e-7, max_iter=1000,
        polishing=False, adaptive_rho_interval=25,
    )
    result = solver.solve(raise_error=False)
    if result.x is not None and result.info.status_val in (1, 2):
        values = constraints @ result.x
        if (np.isfinite(result.x).all() and np.all(values >= lower - 1e-8)
                and np.all(values <= upper + 1e-8)):
            return result.x

    # Near a saturated bound, the reduced lower-level QP can be ill-conditioned
    # for ADMM. Solve the same convex quadratic with an active-set method.
    linear = -(matrix.T @ target)
    initial = result.x if result.x is not None and np.max(np.abs(result.x)) < 1e3 else np.zeros(size)
    refined = minimize(
        lambda x: 0.5 * x @ hessian @ x + linear @ x, initial,
        jac=lambda x: hessian @ x + linear, method="SLSQP",
        constraints=[LinearConstraint(constraints / row_scale[:, None],
                                      lower / row_scale, upper / row_scale)],
        options={"ftol": 1e-11, "maxiter": 200},
    )
    values = constraints @ refined.x
    if (not refined.success or not np.isfinite(refined.x).all()
            or np.any(values < lower - 2e-7) or np.any(values > upper + 2e-7)):
        raise QPFailure(f"{result.info.status}; refinement: {refined.message}")
    return refined.x


def solve_hierarchy(tasks, constraints, lower, upper):
    """Preserve every achieved higher-level task before solving the next one.

    Each task is (matrix, target). Preserving the achieved task vector, rather
    than assigning a large weight, gives a numerical lexicographic hierarchy.
    """
    size = constraints.shape[1]
    equality = np.isfinite(lower) & np.isfinite(upper) & (np.abs(upper - lower) < 1e-12)
    fixed_matrix = constraints[equality]
    solution = np.linalg.lstsq(fixed_matrix, lower[equality], rcond=None)[0]
    if np.any(np.abs(fixed_matrix @ solution - lower[equality]) > 1e-7):
        raise QPFailure("Conflicting hard equalities")
    basis = null_space(fixed_matrix) if len(fixed_matrix) else np.eye(size)
    for level, (matrix, target) in enumerate(tasks):
        if basis.shape[1] == 0:
            break
        projected = constraints @ basis
        offset = constraints @ solution
        moving_rows = np.linalg.norm(projected, axis=1) > 1e-10
        if (np.any(offset[~moving_rows] < lower[~moving_rows] - 2e-6)
                or np.any(offset[~moving_rows] > upper[~moving_rows] + 2e-6)):
            raise QPFailure("Higher-priority solution violates constraints")
        reduced_task = matrix @ basis
        try:
            # A tiny feasibility allowance avoids declaring a lower level
            # infeasible from roundoff at a higher level's active bound.
            allowance = level * 2e-7
            increment = solve_qp(reduced_task, target - matrix @ solution,
                                 projected[moving_rows], (lower - offset)[moving_rows] - allowance,
                                 (upper - offset)[moving_rows] + allowance)
        except QPFailure as error:
            raise QPFailure(f"level {level + 1}: {error}") from error
        solution += basis @ increment
        # Lower levels move only in this task's null space: no large weights
        # or nearly duplicated equality constraints are needed.
        basis = basis @ null_space(reduced_task)
    values = constraints @ solution
    if np.any(values < lower - 2e-6) or np.any(values > upper + 2e-6):
        raise QPFailure("No feasible hierarchical solution")
    return solution
