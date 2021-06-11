#!/usr/bin/python3

import matplotlib.pyplot as plt
import sympy as sym

from ..connectors import Connection
from ..output import Arrow3D, LatexDocument, get_output_dir
from ..rigidbody import Ground

# from sympy.physics.vector.printing import vlatex


class MultiBody:
    id_counter = 0
    id_names = []

    def __init__(self, connections=None, name=None):
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

        if self.connections:
            g = Ground()
            self.bodies = {g.name: g}
        else:
            self.bodies = {}

        self.joints = {}
        self.forces = []
        self.torques = []

        self.kinetic_energy = 0
        self.potential_energy = 0
        self.G = None

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

        if (val is None) or not val:
            self._connections = []
        else:
            # Ensure first object is ground
            if not isinstance(val[0].body_in, Ground):
                raise TypeError(
                    "The first connection must be have a Ground object as its first body."
                )

            body_names = []

            for v in val:
                if not isinstance(v, Connection):
                    raise TypeError("Connections must be a list of Connection objects.")
                if v.body_out.name in body_names:
                    raise ValueError(
                        "Duplicate body names exist. Ensure bodies are uniquely named."
                    )
                body_names.append(v.body_out.name)

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
                raise ValueError(
                    f"Bodies connected through joint {cnx.joint.name} are not connected to a grounded object."
                )

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
                loc_force = body.pos_body + body.rot_body @ P.symbols()
                dir_force = body.rot_body @ F.symbols()
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

        self.G = G

    def __el_equations(self):
        # Get unforced dynamics
        dyn_g = sym.zeros(self.G.shape[0], 1)
        L = self.kinetic_energy - self.potential_energy
        for i, (qi, dqi) in enumerate(zip(self.coordinates, self.velocities)):
            # Euler-Lagrange Equation
            dli = sym.diff(sym.diff(L, dqi), sym.Symbol("t")) - sym.diff(L, qi)

            dyn_g[i] = dli

        self.eom = dyn_g

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

        if self.connections:
            # Get the left and right hand side of the equations of motion
            # The LHS is the KE metric times accelerations
            self.__lhs = self.G @ sym.Matrix(self.accelerations)

            # The RHS is the eoms less the LHS, plus the generalised forces
            self.__rhs = -(self.eom - self.__lhs) + self.gen_forces

    def get_equilibria(self, unforced=True):
        if not self.connections:
            return None, None

        _LHS, RHS = self.eoms()
        for v in self.velocities:
            RHS = RHS.subs(v, 0)

        f0 = self.force_symbols()
        if unforced:
            for f in f0:
                RHS = RHS.subs(f, 0)

        # Ensure 0s are evaluated
        RHS = sym.simplify(RHS)

        # get the force symbols
        coord_eum = sym.solve(RHS, self.coordinates)
        force_eum = sym.solve(RHS, f0)

        return coord_eum, force_eum

    def force_symbols(self):
        return list(
            set(self.gen_forces.free_symbols)
            - set(self.coordinates)
            - set([sym.Symbol("t")])
        )

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

            # Get the velocity coefficient matrix
            C = self.__rhs.jacobian(self.velocities)

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

    def __latex_fk_maps(self):
        fk_maps = [
            "\\Pi_{"
            + body.name
            + "} = \\left("
            + sym.latex(sym.simplify(body.pos_body))
            + ", "
            + sym.latex(sym.simplify(body.rot_body))
            + "\\right)"
            for body in self.bodies.values()
        ]
        return latexify(fk_maps)

    def __latex_lagrangian(self):
        # Lagrangian
        t_plus_v = sym.latex(sym.simplify(self.kinetic_energy + self.potential_energy))

        str_lagrangian = "L = " + t_plus_v
        return latexify(str_lagrangian)

    def __latex_forces_and_torques(self):
        # Forces
        forces = []
        torques = []

        for loc, force, name in self.forces:
            forces.append(
                "F_{"
                + str(name)
                + "} = \\left("
                + sym.latex(sym.simplify(force))
                + ", "
                + sym.latex(sym.simplify(loc))
                + "\\right)"
            )

        for loc, torque, name in self.torques:
            torques.append(
                "\\tau_{"
                + str(name)
                + "} = \\left("
                + sym.latex(sym.simplify(torque))
                + ", "
                + sym.latex(sym.simplify(loc))
                + "\\right)"
            )

        return latexify(forces + torques)

    def __latex_coordinates(self):
        # Coordintes
        coordinates = "q_{" + self.name + "} = " + sym.latex(self.coordinates)
        coordinates = coordinates.replace("[", "(").replace("]", ")")
        return latexify(coordinates)

    def __latex_ke_metric(self):
        ke_metric = "G = " + sym.latex(self.G)
        return latexify(ke_metric)

    def __latex_eoms(self, linearized):
        # Equations of motion
        A, B = self.system_matrices(linearized)

        if linearized:
            eoms = (
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
            eoms = (
                sym.latex(sym.Matrix([[sym.Symbol("I"), 0], [0, sym.Symbol("G")]]))
                + sym.latex(self.__l)
                + " = "
                + sym.latex(sym.simplify(A))
            )

        return latexify(eoms)

    def as_latex(
        self, linearized=False, output_dir=None, file_name=None, include_diag=True
    ):

        output_dir = get_output_dir(output_dir)

        if file_name is None:
            file_name = "out"

        latex_doc = LatexDocument()
        # Coordinates
        _coordinates = self.__latex_coordinates()
        latex_doc.add_section("Coordinates", _coordinates)

        # Forward Kinematic Maps
        _maps = self.__latex_fk_maps()
        latex_doc.add_section("Configuration", _maps)
        # Lagrangian
        _energy_eq = self.__latex_lagrangian()
        latex_doc.add_section("Energy", _energy_eq)
        # KE Metric
        _ke_metric = self.__latex_ke_metric()
        latex_doc.add_section("Kinetic Energy Metric", _ke_metric)
        # Forces
        _forces_and_torques = self.__latex_forces_and_torques()
        latex_doc.add_section("Forces and Torques", _forces_and_torques)
        # Equations of Motion
        _eoms = self.__latex_eoms(linearized)
        latex_doc.add_section("Equations of Motion", _eoms)

        if include_diag:
            _, diag_dir = self.draw(output_dir)
            latex_doc.add_figure(f"Diagram of {self.name}", diag_dir)

        latex_doc.write_pdf(f"multibody_{self.name}", output_dir)

    def __draw_spatial_axes(self, ax):
        basis_vectors = [
            (1, 0, 0),
            (0, 1, 0),
            (0, 0, 1),
        ]

        basis_labels = ["$X$", "$Y$", "$Z$"]

        for vec, label in zip(basis_vectors, basis_labels):
            ax.text(*vec, label, c="r", fontsize="x-small")

            basis = [(0, i) for i in vec]
            arrow = Arrow3D(
                *basis, mutation_scale=5, lw=1, arrowstyle="-|>", color="r",
            )
            ax.add_artist(arrow)

        return ax

    def __equalize_axes(self, ax, mult=0.5):

        x_lim = ax.get_xlim3d()
        y_lim = ax.get_ylim3d()
        z_lim = ax.get_ylim3d()

        x_min, x_max = min(-2, min(x_lim)), max(2, max(x_lim))
        y_min, y_max = min(-2, min(y_lim)), max(2, max(y_lim))
        z_min, z_max = min(-2, min(z_lim)), max(2, max(z_lim))

        ax.set_xlim3d(x_min, x_max)
        ax.set_ylim3d(y_min, y_max)
        ax.set_zlim3d(z_min, z_max)

        # y0 = np.array(y_lim).mean()
        # z0 = np.array(z_lim).mean()

        # max_ax = 0
        # for lim in [x_lim, y_lim, z_lim]:
        #     dlim = np.abs(lim[1] - lim[0])
        #     max_ax = max(dlim, max_ax)

        # ax.set_xlim3d(x0 - mult * max_ax, x0 + mult * max_ax)
        # ax.set_ylim3d(y0 - mult * max_ax, y0 + mult * max_ax)
        # ax.set_zlim3d(z0 - mult * max_ax, z0 + mult * max_ax)

        return ax

    def draw(self, output_dir=None, save_fig=True):
        output_dir = get_output_dir(output_dir)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_axis_off()

        sub_vals = {}
        for cnx in self.connections:
            sub_vals = {
                **sub_vals,
                **cnx.as_dict(),
            }
            ax = cnx.draw(ax=ax, sub_vals=sub_vals)

        ax = self.__draw_spatial_axes(ax)
        ax = self.__equalize_axes(ax)

        if save_fig:
            output_dir = f"{output_dir}/diagram_{self.name}.pdf"
            plt.tight_layout()
            plt.savefig(output_dir)

        return ax, output_dir

    def symbols(self):
        return self.eom.free_symbols


def latexify(string_item):
    if isinstance(string_item, list):
        return "\n".join([latexify(item) for item in string_item])
    else:
        return "\\begin{equation}" + str(string_item) + "\\end{equation} \\\\"
