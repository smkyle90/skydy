#!/usr/bin/python3
import sympy as sym
from sympy.physics.vector.printing import vlatex

from ..connectors import Connection


class MultiBody:
    id_counter = 0
    id_names = []

    def __init__(self, connections, name=None):
        # Body accounting
        MultiBody.id_counter += 1

        if name is None:
            self.name = str(MultiBody.id_counter)
        else:
            self.name = str(name)

        if self.name in MultiBody.id_names:
            raise ValueError("Body name already exists.")
        else:
            MultiBody.id_names.append(self.name)

        self.connections = connections
        self.coordinates = []
        self.velocities = []
        self.accelerations = []

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

        self.__ke_metric()
        self.__el_equations()
        self.__calculate_forces()

    @property
    def connections(self):
        return self._connections

    @connections.setter
    def connections(self, val):
        for v in val:
            assert isinstance(v, Connection)

        self._connections = val

    def add_connection(self, connection):
        self.connections.append(connection)

    def __forward_kinematics(self):
        for cnx in self.connections:
            q = cnx.body_out.positions()
            v = cnx.body_out.velocities()
            a = cnx.body_out.accelerations()

            for dof in cnx.joint.dof:
                if dof.free:
                    self.coordinates.append(q[dof.idx])
                    self.velocities.append(v[dof.idx])
                    self.accelerations.append(a[dof.idx])
                else:
                    cnx.body_out.apply_constraint(dof.idx, dof.const_value)

            if self.bodies.get(cnx.body_in.name, False):
                body_in = self.bodies[cnx.body_in.name]
                r = body_in.pos_body
                R = body_in.rot_body
            else:
                r = sym.zeros(3, 1)
                R = sym.eye(3)

            # Propagate rotations
            cnx.body_out.rot_body = sym.simplify(R @ cnx.body_out.rot_body)

            # Get absolute positions
            p_in = r  # global coordinate of input link
            p_j_in = (
                R @ cnx.joint.body_in_coord.symbols()
            )  # global position of connection on input link
            p_out_j = (
                cnx.body_out.rot_body @ cnx.joint.body_out_coord.symbols()
            )  # global position of output link to joint
            add_dof = (
                cnx.body_out.rot_body @ cnx.body_out.pos_body
            )  # additional dofs from joint

            cnx.body_out.pos_body = sym.simplify(p_in + p_j_in + p_out_j + add_dof)

            for F, P in cnx.body_out.linear_forces:
                loc_force = sym.simplify(
                    cnx.body_out.pos_body + cnx.body_out.rot_body @ P.symbols()
                )
                dir_force = sym.simplify(cnx.body_out.rot_body @ F.symbols())
                self.forces.append((loc_force, dir_force, F.name))

            for T, P in cnx.body_out.linear_torques:
                loc_torque = sym.Matrix(
                    [
                        q if t else 0
                        for t, q in zip(T.symbols(), cnx.body_out.symbols()[3:])
                    ]
                )
                self.torques.append((loc_torque, T.symbols(), T.name))

            self.bodies[cnx.body_out.name] = cnx.body_out
            self.joints[cnx.joint.name] = cnx.joint

        self.coordinates = sym.Matrix(self.coordinates)
        self.velocities = sym.Matrix(self.velocities)
        self.accelerations = sym.Matrix(self.accelerations)

    def __calculate_energy(self):
        # Global gravity vector
        g = sym.Matrix([0, 0, 1])

        KE = sum([b.kinetic_energy() for b in self.bodies.values()])
        PE = sum([b.potential_energy(g) for b in self.bodies.values()])

        self.kinetic_energy = sym.simplify(KE)
        self.potential_energy = sym.simplify(PE)

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

    def __el_equations(self):
        dyn_g = sym.zeros(self.G.shape[0], 1)
        L = self.kinetic_energy - self.potential_energy
        for i, (qi, dqi) in enumerate(zip(self.coordinates, self.velocities)):
            # Euler-Lagrange Equation
            dli = sym.diff(sym.diff(L, dqi), sym.Symbol("t")) - sym.diff(L, qi)

            dyn_g[i] = dli

        self.eom = sym.simplify(dyn_g)
        return self.eom

    def __calculate_forces(self):
        gen_forces = {coord: 0 for coord in self.coordinates}
        for coord in gen_forces:
            for loc, force, _ in self.forces:
                d_loc = sym.diff(loc, coord)
                gen_f = d_loc.dot(force)
                gen_forces[coord] = gen_forces[coord] + gen_f

            for loc, torque, _ in self.torques:
                d_loc = sym.diff(loc, coord)
                gen_f = d_loc.dot(torque)
                gen_forces[coord] = gen_forces[coord] + gen_f

        self.gen_forces = sym.Matrix([gen_forces[k] for k in self.coordinates])
        return sym.simplify(self.gen_forces)

    def get_equilibria(self, unforced=True):
        LHS, RHS = self.eoms()
        for v in self.velocities:
            RHS = RHS.subs(v, 0)

        f0 = self.force_symbols()
        if unforced:
            for f in f0:
                RHS = RHS.subs(f, 0)

        # get the force symbols
        coord_eum = sym.solve(RHS, self.coordinates)
        force_eum = sym.solve(RHS, f0)

        return coord_eum, force_eum

    def force_symbols(self):
        return list(set(self.gen_forces.free_symbols) - set(self.coordinates))

    def eoms(self):
        # Algebra on the calculated equations of motion
        # The LHS is the KE metric times accelerations
        LHS = self.G @ sym.Matrix(self.accelerations)

        # The RHS is the eoms less the LHS, plus the generalised forces
        RHS = -(self.eom - LHS) + self.gen_forces

        return sym.simplify(LHS), sym.simplify(RHS)

    def system_matrices(self, linearized=False):
        f = self.force_symbols()

        # get left and right hand sides of equtions of motion
        LHS, RHS = self.eoms()

        self.__l = sym.Matrix.vstack(self.velocities, self.accelerations)
        self.__q = sym.Matrix.vstack(self.coordinates, self.velocities)
        self.__u = sym.Matrix(f)

        if linearized:
            # Input matrix.
            B = RHS.jacobian(f)
            U = self.G.inv() @ B

            ns, nu = B.shape
            B = sym.zeros(2 * ns, nu)
            B[ns:, :] = U

            A = sym.zeros(2 * ns, 2 * ns)

            # Get the position coefficient matrix
            K = RHS.jacobian(self.coordinates)
            K = self.G.inv() @ K

            # Get the velocity coefficient matrix
            C = RHS.jacobian(self.velocities)
            C = self.G.inv() @ C

            A[:ns, ns:] = sym.eye(ns)
            A[ns:, :ns] = -K
            A[ns:, ns:] = -C
        else:
            A = sym.Matrix.vstack(self.velocities, self.G.inv() @ RHS)
            B = sym.zeros(*RHS.shape)

        return sym.simplify(A), sym.simplify(B)

    def get_configuration(self):
        return {name: body.as_dict() for name, body in self.bodies.items()}

    def as_latex(self, linearized=False, output_dir=None):

        # Coordintes
        _coordinates = "q_{" + self.name + "} = " + sym.latex(self.coordinates)
        _coordinates = _coordinates.replace("[", "(").replace("]", ")")

        # Forward Kinematic Maps
        _maps = [
            "\\Pi_{"
            + body.name
            + "} = \\left("
            + sym.latex(body.pos_body)
            + ", "
            + sym.latex(body.rot_body)
            + "\\right)"
            for body in self.bodies.values()
        ]

        # Lagrangian
        t_plus_v = sym.latex(self.kinetic_energy + self.potential_energy)
        # t_plus_v = vlatex(self.kinetic_energy + self.potential_energy)

        _energy_eq = "L = " + t_plus_v

        # Forces
        _forces = []
        _torques = []

        for loc, force, name in self.forces:
            _forces.append(
                "F_{"
                + str(name)
                + "} = \\left("
                + sym.latex(force)
                + ", "
                + sym.latex(loc)
                + "\\right)"
            )

        for loc, torque, name in self.torques:
            _torques.append(
                "\\tau_{"
                + str(name)
                + "} = \\left("
                + sym.latex(torque)
                + ", "
                + sym.latex(loc)
                + "\\right)"
            )

        _forces_and_torques = _forces + _torques

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
            _eoms = sym.latex(self.__l) + " = " + sym.latex(sym.simplify(A))

        latex_framework = (
            "\n\\documentclass[12pt]{article}\n"
            + "\n\\usepackage{amsmath}\n"
            # + "\n\\usepackage{breqn}\n"
            + "\n\\begin{document}\n"
            + "\n\\subsection*{Coordinates}\n"
            + latexify(_coordinates)
            + "\n\\subsection*{Configuration}\n"
            + "\n".join([latexify(m) for m in _maps])
            + "\n\\subsection*{Energy}\n"
            + latexify(_energy_eq)
            + "\n\\subsection*{Forces and Torques}\n"
            + "\n".join([latexify(ft) for ft in _forces_and_torques])
            + "\n\\subsection*{Equations of Motion}\n"
            + latexify(_eoms)
            + "\n\\end{document} \
            ".replace(
                "            ", ""
            )
        )

        if output_dir is None:
            output_dir = "./tex"

        file_name = "out_{}".format(self.name)
        with open(f"{output_dir}/{file_name}.tex", "w") as text_file:
            text_file.write(latex_framework)

        import os

        os.system(f"pdflatex {output_dir}/{file_name}.tex")
        os.system(f"cp {file_name}.pdf {output_dir}")
        os.system(f"rm {file_name}.aux {file_name}.log {file_name}.pdf")

    def symbols(self):
        return self.eom.free_symbols


def latexify(string):
    return "\\begin{equation}" + str(string) + "\\end{equation} \\\\"
