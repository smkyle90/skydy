#!/usr/bin/python3


class DOF:
    def __init__(self, idx, free=True, const_value=0):
        self.idx = idx
        self.free = free
        self.const_value = const_value
