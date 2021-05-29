#!/usr/bin/python3
import sympy as sym

from .connectors import Connection


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
        for cnx in self.connections:
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
        _maps = [
            "\\PI_{} = \\left({}, {}\\right)".format(
                body.name, body.r_body, body.R_body
            )
            for body in self.bodies.values()
        ]

        # Lagrangian
        T = sym.latex(self.kinetic_energy)
        V = sym.latex(self.potential_energy)
        _energy_eq = "L = " + T + V

        # Equations of motion
        A, B = self.system_matrices(linearized)
        if linearized:
            _eoms = (
                sym.latex(self.__l)
                + " = "
                + sym.latex(A)
                + sym.latex(self.__q)
                + " + "
                + sym.latex(B)
                + sym.latex(self.__u)
            )
        else:
            _eoms = (
                sym.latex(self.__l) + " = " + sym.latex(sym.simplify(A + B @ self.__u))
            )

    def symbols(self):
        self.el_equations()
        return self.eom.free_symbols
