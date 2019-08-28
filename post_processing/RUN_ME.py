#!/usr/bin/env python

import move_to_see
import numpy

w = numpy.array([-0.00423205, -0.00351088, -0.00280758, -0.00071615,  0.0, 0.00064091, 0.00312503,  0.00375221,  0.00438546])

Q = numpy.array([[  7.25125790e-01,  -1.89881306e-02,   6.88118696e-01,   1.80189610e-02], [  7.25374401e-01,  -4.43833379e-08,   6.88354552e-01,  -2.98023188e-08], [  7.25125790e-01,   1.89880598e-02,   6.88118696e-01,  -1.80191100e-02], [  7.06864476e-01,  -1.85099337e-02,   7.06864536e-01,   1.85099244e-02], [  7.07106829e-01,  -1.98633643e-08,   7.07106709e-01,  -5.96046448e-08], [  7.06864476e-01,   1.85098872e-02,   7.06864536e-01,  -1.85099542e-02], [  6.87941611e-01,  -1.80237200e-02,   7.25293875e-01,   1.89836323e-02], [  6.88229322e-01,  -1.29075488e-08,   7.25493193e-01,  -2.98023224e-08], [  6.88118756e-01,   1.80189777e-02,   7.25125790e-01,  -1.89881325e-02]])

move_to_see.weighted_avg_quaternions(Q,w)
