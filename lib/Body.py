import copy

import numpy as np
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols, init_vprinting


class Body:
    id_counter = 0

    def __init__(self, mass, length=0, width=0, height=0, shape="rod", name=None):

        # Denote if the velocity is absolute. By default, as it is a body,
        # is a relative velocity.
        self.__lbs = False

        # Body accounting
        self.body_id = Body.id_counter
        Body.id_counter += 1
        if name is None:
            self.name = str(self.body_id)
        else:
            self.name = str(name)

        self.shape = shape
        self.dims = BodyCoordinate(self.name, length, width, height)

        self.mass = mass
        # Inertial Properties
        self.mass_matrix, self.inertia_matrix = self.__init_inertial_props()

        # Position and velocity variables
        self.q, self.v = self.__init_symbols()

        # Free body state and velocities
        self.__r_body_free, self.__R_body_free = self.__free_body_configuration()
        # self.__v_body_free, self.__w_body_free = self.body_twists(self.__r_body_free, self.__R_body_free)

        self.r_body, self.R_body = None, None

        # Constrained body state and velocities
        self.reset_constraints()

        self.linear_forces = []
        self.linear_torques = []

    def is_absolute(self):
        return self.__lbs

    def __init_inertial_props(self):
        g = sym.Symbol("g")

        mass_symbol = sym.Symbol("m_{}".format(self.name))
        ax = ["x", "y", "z"]
        inertia_matrix = [["I_{}{}_{}".format(a, b, self.name) for a in ax] for b in ax]

        mass_matrix = sym.eye(3) * mass_symbol
        inertia_matrix = sym.Matrix(inertia_matrix)

        return mass_matrix, inertia_matrix

    def __init_symbols(self):
        q = ["x_G", "y_G", "z_G", "theta_x", "theta_y", "theta_z"]
        q = ["{}_{}".format(var, self.name) for var in q]
        q = [dynamicsymbols(var) for var in q]
        v = [sym.diff(var, sym.Symbol("t")) for var in q]

        return q, v

    def __free_body_configuration(self):
        # Define the rotation matrices for each axis
        Rx = sym.rot_axis3(self.q[3]).T
        Ry = sym.rot_axis2(self.q[4]).T
        Rz = sym.rot_axis1(self.q[5]).T
        R = sym.simplify(Rz @ Ry @ Rx)

        r = sym.Matrix(self.q[:3])

        return r, R

    def body_twists(self, r, R):
        # Get the rotational velocity
        omega = sym.simplify(R.inv() @ sym.diff(R, sym.Symbol("t")))
        omega_body = sym.Matrix([omega[2, 1], omega[0, 2], omega[1, 0]])

        # Define the position of COM and get velocity
        v_body = sym.diff(r, sym.Symbol("t"))

        return v_body, omega_body

    def body_velocity(self, point_on_body):
        pob = np.array(point_on_body).reshape(-1,).tolist()
        pob = sym.Matrix(pob)
        return self.v_body + self.w_body.cross(pob)

    def kinetic_energy(self):
        # Define the kinetic energy of the system
        v_body, w_body = self.body_twists(self.r_body, self.R_body)
        KE_tr = sym.simplify((1 / 2) * v_body.T @ self.mass_matrix @ v_body)
        KE_ro = sym.simplify((1 / 2) * w_body.T @ self.inertia_matrix @ w_body)
        return KE_tr[0] + KE_ro[0]

    def potential_energy(self, gravity):
        g = sym.Symbol("g")
        return self.mass_matrix[0, 0] * g * self.r_body.dot(gravity)

    def add_force(self, linear_force):
        assert isinstance(linear_force, BodyForce)
        self.linear_forces.append(linear_force)

    def add_torque(self, linear_torque):
        assert isinstance(linear_torque, BodyTorque)
        loc_torque = sym.Matrix(
            [
                self.q[idx + 3] if t_dir else 0
                for idx, t_dir in enumerate(linear_torque.direction)
            ]
        )
        linear_torque.location = loc_torque
        self.linear_torques.append(linear_torque)

    def apply_constraint(self, idx, const_value=0):
        self.__constrained_state[idx] = True

        self.r_body = sym.simplify(self.r_body.subs(self.q[idx], const_value))
        self.R_body = sym.simplify(self.R_body.subs(self.q[idx], const_value))

    def reset_constraints(self):
        self.r_body = self.__r_body_free.copy()
        self.R_body = self.__R_body_free.copy()
        self.__constrained_state = [False for val in self.q]


class DOF:
    def __init__(self, idx, free=True, const_value=0):
        self.idx = idx
        self.free = free
        self.const_value = const_value


class SpringDamperCoeffs:
    def __init__(self, name):
        dofs = ["x", "y", "z", "theta_x", "theta_y", "theta_z"]
        self.K = ["k_{}_{}".format(name, dof) for dof in dofs]
        self.C = ["c_{}_{}".format(name, dof) for dof in dofs]


class Joint:
    id_counter = 0

    def __init__(
        self,
        body_in_coord,
        body_out_coord,
        dof=[DOF(0), DOF(1), DOF(2), DOF(3), DOF(4), DOF(5)],
        name=None,
    ):
        # Body accounting
        Joint.id_counter += 1
        self.joint_id = Joint.id_counter
        if name is None:
            self.name = str(self.joint_id)
        else:
            self.name = str(name)

        self.dof = dof

        self.body_in_coord = body_in_coord
        self.body_out_coord = body_out_coord

    @property
    def dof(self):
        return self._dof

    @dof.setter
    def dof(self, val):
        def_idx = []

        for v in val:
            assert isinstance(v, DOF)
            def_idx.append(v.idx)

        all_idx = set([i for i in range(6)])
        rem_idx = all_idx - set(def_idx)

        for idx in rem_idx:
            val.append(DOF(idx, False))
        self._dof = val


class Connection:
    def __init__(self, body_in, joint, body_out):
        self.body_in = copy.deepcopy(body_in)
        self.body_out = copy.deepcopy(body_out)
        self.joint = copy.deepcopy(joint)

    @property
    def body_in(self):
        return self._body_in

    @body_in.setter
    def body_in(self, val):
        assert isinstance(val, Body)
        self._body_in = val

    @property
    def body_out(self):
        return self._body_out

    @body_out.setter
    def body_out(self, val):
        assert isinstance(val, Body)
        self._body_out = val


class MultiBody:
    id_counter = 0

    def __init__(self, connections, name=None):
        # Body accounting
        MultiBody.id_counter += 1

        if name is None:
            self.name = str(MultiBody.id_counter)
        else:
            self.name = str(name)

        self.connections = connections
        self.coordinates = []

        self.bodies = {}
        self.joints = {}
        self.forces = []
        self.torques = []

        self.kinetic_energy = 0
        self.potential_energy = 0
        self.G = 0
        self.eom = None
        self.gen_forces = None
        self.__l = None
        self.__q = None
        self.__u = None

        self.__forward_kinematics()
        self.__calculate_energy()

        self.velocities = [sym.diff(var, sym.Symbol("t")) for var in self.coordinates]
        self.__ke_metric()

    @property
    def connections(self):
        return self._connections

    @connections.setter
    def connections(self, val):
        for v in val:
            assert isinstance(v, Connection)

        self._connections = val

    def __forward_kinematics(self):
        for i, cnx in enumerate(self.connections):
            for dof in cnx.joint.dof:
                if dof.free:
                    self.coordinates.append(cnx.body_out.q[dof.idx])
                else:
                    cnx.body_out.apply_constraint(dof.idx, dof.const_value)

            if self.bodies.get(cnx.body_in.name, False):
                body_in = self.bodies[cnx.body_in.name]
                r = body_in.r_body
                R = body_in.R_body
            else:
                r = sym.zeros(3, 1)
                R = sym.eye(3)

            # Propagate rotations
            cnx.body_out.R_body = sym.simplify(R @ cnx.body_out.R_body)

            # Get absolute positions
            p_in = r  # global coordinate of input link
            p_j_in = (
                R @ cnx.joint.body_in_coord.symbols()
            )  # global position of connection on input link
            p_out_j = (
                cnx.body_out.R_body @ cnx.joint.body_out_coord.symbols()
            )  # global position of output link to joint
            add_dof = (
                cnx.body_out.R_body @ cnx.body_out.r_body
            )  # additional dofs from joint

            cnx.body_out.r_body = sym.simplify(p_in + p_j_in + p_out_j + add_dof)

            for f in cnx.body_out.linear_forces:
                loc_force = sym.simplify(
                    cnx.body_out.r_body + cnx.body_out.R_body @ f.location.symbols()
                )
                dir_force = sym.simplify(cnx.body_out.R_body @ f.direction)
                self.forces.append((loc_force, dir_force))

            for t in cnx.body_out.linear_torques:
                self.torques.append((t.location, t.direction))

            self.bodies[cnx.body_out.name] = cnx.body_out
            self.joints[cnx.joint.name] = cnx.joint

    def __calculate_energy(self):
        # Global gravity vector
        g = sym.Matrix([0, 0, 1])

        KE = sum([b.kinetic_energy() for b in self.bodies.values()])
        PE = sum([b.potential_energy(g) for b in self.bodies.values()])

        self.kinetic_energy = KE
        self.potential_energy = PE

    def __ke_metric(self):
        G = sym.Matrix(
            [
                [
                    sym.diff(sym.diff(self.kinetic_energy, v1), v2)
                    for v2 in self.velocities
                ]
                for v1 in self.velocities
            ]
        )

        self.G = sym.simplify(G)

    def el_equations(self):
        dyn_g = sym.zeros(self.G.shape[0], 1)
        L = self.kinetic_energy - self.potential_energy
        for i, (qi, dqi) in enumerate(zip(self.coordinates, self.velocities)):
            # Euler-Lagrange Equation
            dli = sym.simplify(
                sym.diff(sym.diff(L, dqi), sym.Symbol("t")) - sym.diff(L, qi)
            )
            dyn_g[i] = dli

        self.eom = dyn_g
        return self.eom

    def calculate_forces(self):
        gen_forces = {coord: 0 for coord in self.coordinates}
        for coord in gen_forces:
            for loc, force in self.forces:
                d_loc = sym.diff(loc, coord)
                gen_f = d_loc.dot(force)
                gen_forces[coord] = gen_forces[coord] + gen_f

            for loc, torque in self.torques:
                d_loc = sym.diff(loc, coord)
                gen_f = d_loc.dot(torque)
                gen_forces[coord] = gen_forces[coord] + gen_f

        self.gen_forces = sym.Matrix(
            [sym.simplify(gen_forces[k]) for k in self.coordinates]
        )
        return self.gen_forces

    def get_equilibria(self):
        eoms = self.eom - self.gen_forces
        for v in self.velocities:
            eoms = eoms.subs(v, 0)

        # get the force symbols
        f0 = self.force_symbols()
        eoms = sym.simplify(eoms)
        coord_eum = sym.solve(eoms, self.coordinates)
        force_eum = sym.solve(eoms, f0)

        return coord_eum, force_eum

    def force_symbols(self):
        return list(set(self.gen_forces.free_symbols) - set(self.coordinates))

    def system_matrices(self, linearized=False):
        a = [sym.diff(v, sym.Symbol("t")) for v in self.velocities]
        f = self.force_symbols()
        G = self.eom.jacobian(a)

        # Need to remove the accelerations
        LHS = G @ sym.Matrix(a)
        eoms = sym.simplify(self.eom - LHS)

        self.__l = sym.Matrix(self.velocities + a)
        self.__q = sym.Matrix(self.coordinates + self.velocities)
        self.__u = sym.Matrix(f)

        # Input matrix.
        B = self.gen_forces.jacobian(f)
        U = sym.simplify(G.inv() @ B)

        ns, nu = B.shape
        B = sym.zeros(2 * ns, nu)
        B[ns:, :] = U

        if linearized:
            A = sym.zeros(2 * ns, 2 * ns)

            # Get the position coefficient matrix
            K = eoms.jacobian(self.coordinates)
            K = sym.simplify(G.inv() @ K)

            # Get the velocity coefficient matrix
            C = eoms.jacobian(self.velocities)
            C = sym.simplify(G.inv() @ C)

            A[:ns, ns:] = sym.eye(ns)
            A[ns:, :ns] = -K
            A[ns:, ns:] = -C
        else:
            A = sym.Matrix(self.velocities + list(-G.inv() @ eoms))

        return A, B

    def as_latex(self, linearized=False):

        # Forward Kinematic Maps
        maps = [
            "\\PI_{} = \\left({}, {}\\right)".format(
                body.name, body.r_body, body.R_body
            )
            for body in self.bodies.values()
        ]

        # Lagrangian
        T = sym.latex(self.kinetic_energy)
        V = sym.latex(self.potential_energy)
        energy_eq = "L = " + T + V

        # Equations of motion
        A, B = self.system_matrices(linearized)
        if linearized:
            eoms = (
                sym.latex(self.__l)
                + " = "
                + sym.latex(A)
                + sym.latex(self.__q)
                + " + "
                + sym.latex(B)
                + sym.latex(self.__u)
            )
        else:
            eoms = (
                sym.latex(self.__l) + " = " + sym.latex(sym.simplify(A + B @ self.__u))
            )

    def symbols(self):
        self.el_equations()
        return self.eom.free_symbols


class BodyCoordinate:
    def __init__(self, name, x=0, y=0, z=0):
        self.name = name
        self.properties = {
            "x": x,
            "y": y,
            "z": z,
        }
        self.__symbols = sym.Matrix(
            [
                sym.Symbol("l_{}_{}".format(self.name, k)) if v else 0
                for k, v in self.properties.items()
            ]
        )

    def symbols(self):
        return self.__symbols

    def values(self):
        return np.array(list(self.properties.values()))


class BodyForce:
    def __init__(
        self, name, location, x_dir=False, y_dir=False, z_dir=False, prefix="F"
    ):
        self.name = str(name)
        self.location = location
        ax = ["x", "y", "z"]
        direction = [x_dir, y_dir, z_dir]
        self.direction = sym.Matrix(
            [
                sym.Symbol("{}_{}_{}".format(prefix, self.name, a)) if f else 0
                for a, f in zip(ax, direction)
            ]
        )

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, val):
        self._location = val
        # if val is None:
        #     self._location = val
        # elif isinstance(val, BodyCoordinate):
        #     self._location = val
        # else:
        #     raise TypeError("Force location must be a BodyCoordinate (force) or None (torque).")


class BodyTorque(BodyForce):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False, prefix="T"):
        super().__init__(name, None, x_dir, y_dir, z_dir, prefix)


# l = 2
# b0 = Body(0, 0)
# b1 = Body(1, l)
# b2 = Body(1, 2*l)

# # Cart on the ground
# p0 = BodyCoordinate("O")
# p1 = BodyCoordinate("G1/O", 0, 0, 0)
# j1 = Joint(p0, p1, [DOF(0,)])

# # Link on cart
# p2 = BodyCoordinate("A/G1", 0, 0, 0)
# p3 = BodyCoordinate("G2/A", l, 0, 0)
# j2 = Joint(p2, p3, [DOF(4,)])

# # Body force
# p_F1 = BodyCoordinate("F1", 0, 0, 0)
# F_1 = BodyForce("1", p_F1, x_dir=True)
# T_1 = BodyTorque("2", y_dir=True)

# # Add force to trolley
# b1.add_force(F_1)
# # Add torque to the arm
# b2.add_torque(T_1)

# # Create the system
# # A rigid body is just a collection of connected Bodies
# body = MultiBody([
#     Connection(b0, j1, b1),
#     Connection(b1, j2, b2),
# ])

# body.el_equations()
# body.calculate_forces()
