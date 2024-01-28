from sympy import *

init_printing(use_unicode=True)

w1 = Symbol('w1')
w2 = Symbol('w2')
w3 = Symbol('w3')
c = Symbol('cos(t)')
s = Symbol('sin(t)')
px = Symbol('px')
py = Symbol('py')
pz = Symbol('pz')

exp = w1 + w2 + w3

print(exp)

omega = Matrix([
    [0, -w3, w2],
    [w3, 0, - w1],
    [-w2, w1, 0]])

omega2 = omega @ omega
I = Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])

print(omega)
print(omega2)

R = I + s * omega + (1 - c) * omega2

P = Matrix([[px], [py], [pz]])
tmp = R @ P
# tmp = tmp.subs(w2, 0)
# tmp = tmp.subs(w1, 0)


print()
print(tmp[0, 0].simplify())
print(tmp[1, 0].simplify())
print(tmp[2, 0].simplify())

R11 = Symbol('R11')
R12 = Symbol('R12')
R13 = Symbol('R13')
R21 = Symbol('R21')
R22 = Symbol('R22')
R23 = Symbol('R23')
R31 = Symbol('R31')
R32 = Symbol('R32')
R33 = Symbol('R33')

c = Symbol('cos(t)')
s = Symbol('sin(t)')

R = Matrix([
    [R11, R12, R13],
    [R21, R22, R23],
    [R31, R32, R33]])

Rj = Matrix([
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1]])

# tmp = P.transpose() @ (R @ Rj).transpose()
tmp = R @ Rj
tmp.simplify()

print()
# print(tmp[0, 0].simplify())
# print(tmp[1, 0].simplify())
# print(tmp[2, 0].simplify())
print(tmp)



