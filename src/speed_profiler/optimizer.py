import rospy
import time

import numpy as np
import osqp
from scipy import sparse

from speed_profiler.ros_interface import Logger as logger

import time


class SpeedProfileOptimizer(object):
    """ 
    Computes an optimal speed profile given a reference path of N waypoints.
    The speed profile is created by solving an optimization problem
    which takes into account the following:
    - An absolute maximum speed.
    - A lateral acceleration that must not be exceeded.
    - The maximum and minimum longitudinal acceleration.
    - A smoothening parameter to penalize fast changes in speed.
    """

    def __init__(self, parameters):
        
        ## Maximum allowed lateral acceleration
        ## (used to determine maximum allowed speed from curvature)
        self._lateral_acceleration_limit = parameters['lateral_acceleration_limit']

        ## Weight for penalizing large accelerations
        self._alpha_smooth = parameters['alpha_smooth']

        ## Minimum desired acceleration along path (negative value)
        self._accel_min = parameters['accel_min']

        ## Maximum desired acceleration along path
        self._accel_max = parameters['accel_max']

        ## Optimization problem dimension (number of waypoints, N)
        self._waypoints_count = 0

        # OSQP solver instance (initialized in `initialize_solver`)
        self._solver = None

    def initialize_solver(self, v_max, v_init, v_final):
        """
        Initialize the OSQP solver with fixed matrix structures.
        """
        # Create problem matrices
        P = self._osqp_get_P(self._D1)
        q = self._osqp_get_q(v_max)
        A, l, u = self._osqp_get_constraints(v_max, v_init, v_final)

        # Initialize the OSQP solver once
        self._solver = osqp.OSQP()
        self._solver.setup(P, q, A, l, u, verbose=False, warm_start=True)

    def update_solver(self, v_max, v_init, v_final):
        """
        Update the OSQP solver's problem matrices with new data without reinitializing.
        """
        # Update problem matrices
        q = self._osqp_get_q(v_max)
        A, l, u = self._osqp_get_constraints(v_max, v_init, v_final)

        # Update the matrices in the OSQP solver
        self._solver.update(q=q, l=l, u=u)

    def compute_speed_profile(self, curvatures, distances, v_init, v_final, speed_limit):
        """
        Computes an optimal speed profile along a path
        as introduced in chapter 7.1 of:

        Pedro F. Lima - "Predictive control for autonomous driving"
        http://www.diva-portal.org/smash/get/diva2:925562/FULLTEXT01.pdf

        @param curvatures: List of path curvatures at N waypoints (in 1/m)
        @param distances: List of N-1 distances between waypoints (in m)
        @param v_init (optional): Initial speed on the path (in m/s)
        @param v_final (optional): Final speed on the path (in m/s)
        @return Speed profile (ndarray, N elements) of optimal speeds (in m/s)
        """

        curvatures = np.array(curvatures)
        distances = np.array(distances)
        self._waypoints_count = curvatures.shape[0]

        # Input validation
        if curvatures.shape != (self._waypoints_count,):
            # raise ValueError(f'Wrong shape of curvatures. Expected: ({self._waypoints_count},)')
            raise ValueError('Wrong shape of curvatures. Expected: ({},)'.format(self._waypoints_count))
        if distances.shape != (self._waypoints_count - 1,):
            # raise ValueError(f'Wrong shape of distances. Expected: ({self._waypoints_count - 1},)')
            raise ValueError('Wrong shape of distances. Expected: ({},)'.format(self._waypoints_count - 1))

        # Compute maximum velocities
        v_max = self._compute_v_max(curvatures, speed_limit)
        
        # Initialize solver only once, then update for subsequent calls
        if self._solver is None:
            self._D1 = self._get_D1(distances)
            self.initialize_solver(v_max, v_init, v_final)
        elif (self._D1.toarray().shape[1] != np.eye(self._waypoints_count).shape[1]):
            self._D1 = self._get_D1(distances)
            self.initialize_solver(v_max, v_init, v_final)
        else:
            self.update_solver(v_max, v_init, v_final)

        # Solve the optimization problem
        result = self._solver.solve()

        if result.info.status != 'solved':
            logger.error('[Speed profiler] OSQP: Problem not solved. Status: {}'.format(result.info.status))
            return None

        return np.sqrt(np.maximum(result.x, 0))

    def _compute_v_max(self, kappa_ref, speed_limit):
        """ Computes the maximum speeds on a track with the given curvatures
        without exceeding a defined lateral acceleration and speed limit.

        @param kappa_ref: ndarray (N elements) of path curvatures (in 1/m)
        @return ndarray (N elements) of max speeds (in m/s)
        """

        # We are only interested in absolute curvature, not the direction
        kappa_ref = np.absolute(kappa_ref)

        # Since speed is calculated from curvature with a monotonous function,
        # a maximum speed limit is equivalent to a minimum curvature limit.
        # Specifying the minimum curvature that equals the speed limit prevents
        # division by small numbers.
        kappa_min = self._lateral_acceleration_limit / (speed_limit ** 2)
        kappa = np.maximum(kappa_ref, kappa_min * np.ones(kappa_ref.size))
        return np.sqrt(self._lateral_acceleration_limit / kappa)

    def _osqp_get_P(self, D1):
        """ Construct P matrix for OSQP problem formulation.

        @param D1: D1 as sparse matrix
        @return P matrix for OSQP problem as sparse matrix
        """
        # Note: there is a difference between numpy transpose/dot and the ones
        # used below. Don't use numpy functions!
        return 2 * (sparse.identity(self._waypoints_count) +
                    self._alpha_smooth * (D1.transpose().dot(D1)))

    def _osqp_get_q(self, v_max):
        """ Construct q vector for OSQP problem formulation.

        @param v_max: ndarray (N elements) of maximum speeds (in m/s)
        @return q vector for OSPQ problem as ndarray.
        """
        return -2 * np.square(v_max)

    def _osqp_get_constraints(self, v_max, v_init, v_final):
        """ Construct constraints for OSQP problem formulation.

        @param D1: D1 as sparse matrix
        @param v_max: ndarray (N elements) of maximum speeds (in m/s)
        @param v_init (optional): Initial speed at first waypoint (in m/s)
        @param v_final (optional): Target speed at last waypoint (in m/s)
        @return (A, l, u) such that l <= Ax <= u is the constraint for the
            optimization problem (A as compressed sparse column matrix)
        """

        A = np.append(self._D1.toarray(), np.eye(self._waypoints_count), axis=0)

        a_min = self._accel_min * np.ones(self._waypoints_count - 1)
        a_max = self._accel_max * np.ones(self._waypoints_count - 1)

        w_min = np.zeros(self._waypoints_count)
        w_max = np.square(v_max)

        if v_init is not None:
            w_min[0] = v_init**2
            w_max[0] = v_init**2

        if v_final is not None:
            w_min[self._waypoints_count - 1] = v_final**2
            w_max[self._waypoints_count - 1] = v_final**2

        l = np.append(a_min, w_min)
        u = np.append(a_max, w_max)

        return sparse.csc_matrix(A), l, u

    def _get_D1(self, distances):
        """ Compute matrix operator to get accelerations from speeds.
        Multiplying this matrix with the squared speeds vector
        returns accelerations by using first order differences.

        @param distances: Numpy array of distances between waypoints (in m)
        @return D1 as sparse matrix
        """
        diag = 0.5 / distances
        return sparse.diags([-diag, diag], [0, 1],
                            shape=(self._waypoints_count - 1, self._waypoints_count))
