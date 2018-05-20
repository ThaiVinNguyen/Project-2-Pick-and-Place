import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

# Create symbols for DH param
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')              # joint angles theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')              # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')              # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # joint twist angles

# DH Table
DH = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
      alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
      alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
      alpha3: -pi/2., a3:  -0.054,d4:  1.50, q4:         q4,
      alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
      alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
      alpha6:      0, a6:      0, d7: 0.303, q7:         0}

def TF_Mat(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
    return TF

 ## Substitute DH_Table
T0_1 = TF_Mat(alpha0, a0, d1, q1).subs(DH)
T1_2 = TF_Mat(alpha1, a1, d2, q2).subs(DH)
T2_3 = TF_Mat(alpha2, a2, d3, q3).subs(DH)
T3_4 = TF_Mat(alpha3, a3, d4, q4).subs(DH)
T4_5 = TF_Mat(alpha4, a4, d5, q5).subs(DH)
T5_6 = TF_Mat(alpha5, a5, d6, q6).subs(DH)
T6_7 = TF_Mat(alpha6, a6, d7, q7).subs(DH)

# Composition of Homogeneous Transforms
# Transform from Base link to end effector (Gripper)
T0_2 = (T0_1 * T1_2) 
T0_3 = (T0_2 * T2_3) 
T0_4 = (T0_3 * T3_4) 
T0_5 = (T0_4 * T4_5) 
T0_6 = (T0_5 * T5_6) 
T0_7 = (T0_6 * T6_7) ## (Base) Link_0 to Link_7 (End Effector)

### Numerically evaluate transforms (compare this to output of tf_echo/rviz)

print("\nT0_7 = \n")
pprint(T0_7.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
