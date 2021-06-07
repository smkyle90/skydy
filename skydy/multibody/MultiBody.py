#!/usr/bin/python3
import sympy as sym

from ..connectors import Connection
from ..rigidbody import Ground

# from sympy.physics.vector.printing import vlatex


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

        g = Ground()
        self.bodies = {g.name: g}
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
        self.__lhs = None
        self.__rhs = None

        self.__forward_kinematics()
        self.__calculate_energy()
        self.__ke_metric()

        self.__forces_and_torques()
        self.__el_equations()

    @property
    def connections(self):
        return self._connections

    @connections.setter
    def connections(self, val):
        # Ensure first object is ground
        assert isinstance(val[0].body_in, Ground)

        for v in val:
            assert isinstance(v, Connection)

        self._connections = val

    def add_connection(self, connection):
        self.connections.append(connection)

    def __forward_kinematics(self):
        for cnx in self.connections:
            if self.bodies.get(cnx.body_in.name, False):
                # need to set the global position of the input body
                cnx.body_in = self.bodies[cnx.body_in.name]

                # Propagate the global configuration
                cnx.global_configuration()
            else:
                raise ValueError("Body is not connected to a grounded object.")

            self.bodies[cnx.body_out.name] = cnx.body_out
            self.joints[cnx.joint.name] = cnx.joint

            self.coordinates.extend(cnx.body_out.free_coordinates())
            self.velocities.extend(cnx.body_out.free_velocities())
            self.accelerations.extend(cnx.body_out.free_accelerations())

        self.coordinates = sym.Matrix(self.coordinates)
        self.velocities = sym.Matrix(self.velocities)
        self.accelerations = sym.Matrix(self.accelerations)

    def __forces_and_torques(self):
        for body in self.bodies.values():
            # Calculate the globl magnitude, direction and location of forces and torques
            for F, P in body.linear_forces:
                loc_force = sym.simplify(body.pos_body + body.rot_body @ P.symbols())
                dir_force = sym.simplify(body.rot_body @ F.symbols())
                self.forces.append((loc_force, dir_force, F.name))

            # Calculate the global torques
            for T, P in body.linear_torques:
                loc_torque = sym.Matrix(
                    [q if t else 0 for t, q in zip(T.symbols(), body.symbols()[3:])]
                )
                self.torques.append((loc_torque, T.symbols(), T.name))

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
        # Get unforced dynamics
        dyn_g = sym.zeros(self.G.shape[0], 1)
        L = self.kinetic_energy - self.potential_energy
        for i, (qi, dqi) in enumerate(zip(self.coordinates, self.velocities)):
            # Euler-Lagrange Equation
            dli = sym.diff(sym.diff(L, dqi), sym.Symbol("t")) - sym.diff(L, qi)

            dyn_g[i] = dli

        self.eom = sym.simplify(dyn_g)

        # Add generalised forces
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

        # Get the left and right hand side of the equations of motion
        # The LHS is the KE metric times accelerations
        self.__lhs = sym.simplify(self.G @ sym.Matrix(self.accelerations))

        # The RHS is the eoms less the LHS, plus the generalised forces
        self.__rhs = sym.simplify(-(self.eom - self.__lhs) + self.gen_forces)

    def get_equilibria(self, unforced=True):
        _LHS, RHS = self.eoms()
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
        return self.__lhs, self.__rhs

    def system_matrices(self, linearized=False):
        f = self.force_symbols()

        self.__l = sym.Matrix.vstack(self.velocities, self.accelerations)
        self.__q = sym.Matrix.vstack(self.coordinates, self.velocities)
        self.__u = sym.Matrix(f)

        if linearized:
            # Input matrix.
            B = self.__rhs.jacobian(f)
            U = B

            ns, nu = B.shape
            B = sym.zeros(2 * ns, nu)
            B[ns:, :] = U

            A = sym.zeros(2 * ns, 2 * ns)

            # Get the position coefficient matrix
            K = self.__rhs.jacobian(self.coordinates)
            K = K

            # Get the velocity coefficient matrix
            C = self.__rhs.jacobian(self.velocities)
            C = C

            A[:ns, ns:] = sym.eye(ns)
            A[ns:, :ns] = -K
            A[ns:, ns:] = -C
        else:
            A = sym.Matrix.vstack(self.velocities, self.__rhs)
            B = sym.zeros(*self.__rhs.shape)

        return sym.simplify(A), sym.simplify(B)

    def get_configuration(self):
        config_dict = {
            name: {"coords": body.as_dict(), "dims": body.dims.as_dict()}
            for name, body in self.bodies.items()
        }
        return config_dict

        # import yaml
        # with open('./configs/result.yml', 'w') as yaml_file:
        #     yaml.dump(config_dict, yaml_file, default_flow_style=False)

    def as_latex(self, linearized=False, output_dir=None):

        # Coordintes
        _coordinates = "q_{" + self.name + "} = " + sym.latex(self.coordinates)
        _coordinates = _coordinates.replace("[", "(").replace("]", ")")

        # Forward Kinematic Maps
        _maps = [
            "\\Pi_{"
            + body.name
            + "} = \\left("
            + sym.latex(sym.simplify(body.pos_body))
            + ", "
            + sym.latex(sym.simplify(body.rot_body))
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
                sym.latex(sym.Matrix([[sym.Symbol("I"), 0], [0, sym.Symbol("G")]]))
                + sym.latex(self.__l)
                + " = "
                + sym.latex(A)
                + sym.latex(self.__q)
                + " + "
                + sym.latex(B)
                + sym.latex(self.__u)
            )
        else:
            _eoms = (
                sym.latex(sym.Matrix([[sym.Symbol("I"), 0], [0, sym.Symbol("G")]]))
                + sym.latex(self.__l)
                + " = "
                + sym.latex(sym.simplify(A))
            )

        _ke_metric = "G = " + sym.latex(self.G)

        latex_framework = (
            "\n\\documentclass[8pt]{article}\n"
            + "\n\\usepackage{amsmath}\n"
            # + "\n\\usepackage{flexisym}\n"
            # + "\n\\usepackage{breqn}\n"
            + "\n\\usepackage{geometry}\n"
            + "\n\\geometry{margin=1cm}"
            + "\n\\begin{document}\n"
            + "\n\\subsection*{Coordinates}\n"
            + latexify(_coordinates)
            + "\n\\subsection*{Configuration}\n"
            + latexify(_maps)
            + "\n\\subsection*{Energy}\n"
            + latexify(_energy_eq)
            + "\n\\subsection*{Kinetic Energy Metric}\n"
            + latexify(_ke_metric)
            + "\n\\subsection*{Forces and Torques}\n"
            + latexify(_forces_and_torques)
            + "\n\\subsection*{Equations of Motion}\n"
            + latexify(_eoms)
            + "\n\\end{document} \
            "
        )

        remove_strs = [
            "            ",
            "{\\left(t \\right)}",
            "1.0",
        ]

        for r_str in remove_strs:
            latex_framework = latex_framework.replace(r_str, "")

        replace_strs = [("0.5", "\\frac{1}{2}")]

        for old_str, new_str in replace_strs:
            latex_framework = latex_framework.replace(old_str, new_str)

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


def latexify(string_item):
    if isinstance(string_item, list):
        return "\n".join([latexify(item) for item in string_item])
    else:
        return "\\begin{equation}" + str(string_item) + "\\end{equation} \\\\"
