#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# (x,y,ω)に対する回転行列
def R(theta):
    return [[np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]]

