#!/usr/bin/python3
from ..rigidbody import BodyCoordinate
from .DOF import DOF


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
    def body_in_coord(self):
        return self._body_in_coord

    @body_in_coord.setter
    def body_in_coord(self, val):
        assert isinstance(val, BodyCoordinate)
        self._body_in_coord = val

    @property
    def body_out_coord(self):
        return self._body_out_coord

    @body_out_coord.setter
    def body_out_coord(self, val):
        assert isinstance(val, BodyCoordinate)
        self._body_out_coord = val

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
