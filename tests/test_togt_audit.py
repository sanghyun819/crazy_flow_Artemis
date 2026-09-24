"""Regression checks for audit calculations that decide trajectory validity."""
import unittest
import numpy as np
from togt_trajectory import Trajectory, flatness_dynamics, frame_clearance, gate_events, load_yaml
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class AuditTest(unittest.TestCase):
    def linear_path(self, backwards=False):
        values = np.zeros((1, 33))
        values[0, 0] = 1.
        values[0, 1:3] = [.4, -.8] if backwards else [-.4, .8]
        values[0, 17] = 1.25
        return Trajectory(values)

    def course(self):
        return {'gate': {'inner_size': .5, 'frame_thickness': .08, 'drone_radius': .06},
                'gates': [dict(id=1, x=0., y=0., mount_z=1., yaw_deg=0)]}

    def test_gate_crossing_direction_and_clearance(self):
        for backwards in (False, True):
            events = gate_events(self.linear_path(backwards), self.course())
            self.assertEqual(len(events), 1)
            self.assertAlmostEqual(events[0]['time_s'], .5)
            self.assertAlmostEqual(events[0]['edge_to_vehicle_m'], .19)
            self.assertEqual(events[0]['normal_speed_m_s'] > 0, not backwards)

    def test_time_scaling_preserves_geometry_and_derivative_scaling(self):
        original = self.linear_path()
        slower = original.scaled(2.)
        times = np.linspace(0, 1, 31)
        np.testing.assert_allclose(original.evaluate(times), slower.evaluate(2*times))
        np.testing.assert_allclose(original.evaluate(times, 1)/2, slower.evaluate(2*times, 1))

    def test_bounds_find_interior_extremum(self):
        values = self.linear_path().values.copy()
        # x = 4t - 4t^2, both endpoints 0, interior maximum 1.
        values[0, 1:4] = [0., 4., -4.]
        trajectory = Trajectory(values)
        low, high, maximum_speed = trajectory.exact_bounds()
        self.assertAlmostEqual(low[0], 0)
        self.assertAlmostEqual(high[0], 1)
        self.assertAlmostEqual(maximum_speed, 4)

    def test_frame_contact_is_not_mistaken_for_aperture_clearance(self):
        course = self.course()
        centre = frame_clearance(np.array([[0., 0., 1.25]]), course, .08, .001)[0]
        hit = frame_clearance(np.array([[0., .26, 1.25]]), course, .08, .001)[0]
        self.assertAlmostEqual(centre['conservative_surface_gap_m'], .189)
        self.assertLess(hit['conservative_surface_gap_m'], 0)

    def test_hover_has_weight_divided_by_four_per_rotor(self):
        quad = load_yaml(ROOT/'config/togt/quad.yaml')
        zeros = np.zeros((3, 3))
        thrust, omega, tilt, quaternion = flatness_dynamics(zeros, zeros, zeros, quad, 9.8066)
        np.testing.assert_allclose(thrust, np.full((3, 4), quad['mass']*9.8066/4))
        np.testing.assert_allclose(omega, 0, atol=1e-12)
        np.testing.assert_allclose(tilt, 0, atol=1e-12)
        np.testing.assert_allclose(quaternion, np.tile([1, 0, 0, 0], (3, 1)))

    def test_rotor_torque_matches_angular_momentum_derivative(self):
        quad = load_yaml(ROOT/'config/togt/quad.yaml')
        dt = 1e-5
        t = np.array([-dt, 0, dt])
        a0, j0, s0 = np.array([1.2, -.8, .2]), np.array([.8, .5, -.3]), np.array([-.4, .6, .2])
        acc = a0+t[:, None]*j0+t[:, None]**2*s0/2
        jerk = j0+t[:, None]*s0
        snap = np.tile(s0, (3, 1))
        thrust, omega, _, quaternion = flatness_dynamics(acc, jerk, snap, quad, 9.8066)
        q, dq = quaternion[1], (quaternion[2]-quaternion[0])/(2*dt)
        omega_from_q = 2*(q[0]*dq[1:]-dq[0]*q[1:]-np.cross(q[1:], dq[1:]))
        np.testing.assert_allclose(omega[1], omega_from_q, atol=1e-8)
        inertia = np.array(quad['inertia'])
        expected_torque = inertia*(omega[2]-omega[0])/(2*dt)+np.cross(omega[1], inertia*omega[1])
        actual_torque = (np.array(quad['T_bm']) @ thrust[1])[1:]
        np.testing.assert_allclose(actual_torque, expected_torque, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
