"""Simple data models for the piano keyboard."""

import numpy as np
from pydrake.all import RotationMatrix


class Keyboard_Key():
    def __init__(self, name="key", pos = np.array([0,0,0]),rotation=RotationMatrix() ,frame = None, pitch = 0,keytype="flat"):
        self.name = name
        self.pos = pos
        self.rotation = rotation
        self.pitch = pitch
        self.frame = frame
        self.keytype = keytype
