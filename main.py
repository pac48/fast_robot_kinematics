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

print()
print(tmp[0,0].simplify())
print(tmp[1,0].simplify())
print(tmp[2,0].simplify())
