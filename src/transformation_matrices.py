#!/usr/bin/env python3

import numpy as np
import sympy as sym 

def transMatrix():
  theta, d, a, alpha = sym.symbols('theta, d, a, alpha')
  r1 = sym.Array([sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), a*sym.cos(theta)])
  r2 = sym.Array([sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)])
  r3 = sym.Array([0, sym.sin(alpha), sym.cos(alpha), d])
  r4 = sym.Array([0, 0, 0, 1])
  return sym.Matrix([r1, r2, r3, r4])
  
def transMatrix0To1():
  matrix = transMatrix()
  theta1 = sym.symbols('theta1')
  theta, d, a, alpha = sym.symbols('theta, d, a, alpha')
  matrix = matrix.subs(theta, theta1 + sym.pi/2)
  matrix = matrix.subs(d, 2.5)
  matrix = matrix.subs(a, 0)
  matrix = matrix.subs(alpha, sym.pi/2)
  return matrix
  
def transMatrix1To2():
  matrix = transMatrix()
  theta2 = sym.symbols('theta2')
  theta, d, a, alpha = sym.symbols('theta, d, a, alpha')
  matrix = matrix.subs(theta, theta2 + sym.pi/2)
  matrix = matrix.subs(d, 0)
  matrix = matrix.subs(a, 0)
  matrix = matrix.subs(alpha, sym.pi/2)
  return matrix
  
def transMatrix2To3():
  matrix = transMatrix()
  theta3 = sym.symbols('theta3')
  theta, d, a, alpha = sym.symbols('theta, d, a, alpha')
  matrix = matrix.subs(theta, theta3)
  matrix = matrix.subs(d, 0)
  matrix = matrix.subs(a, 3.5)
  matrix = matrix.subs(alpha, -sym.pi/2)
  return matrix
  
def transMatrix3To4():
  matrix = transMatrix()
  theta4 = sym.symbols('theta4')
  theta, d, a, alpha = sym.symbols('theta, d, a, alpha')
  matrix = matrix.subs(theta, theta4)
  matrix = matrix.subs(d, 0)
  matrix = matrix.subs(a, 3)
  matrix = matrix.subs(alpha, sym.pi/2)
  return matrix
  
def transMatrix0To2():
  matrix0To1 = transMatrix0To1()
  matrix1To2 = transMatrix1To2()
  return matrix0To1*matrix1To2
  
def transMatrix0To3():
  matrix0To2 = transMatrix0To2()
  matrix2To3 = transMatrix2To3()
  return matrix0To2*matrix2To3
  
def transMatrix0To4():
  matrix0To3 = transMatrix0To3()
  matrix3To4 = transMatrix3To4()
  return matrix0To3*matrix3To4
  
def forward_kinematics():
  last_column = transMatrix0To4().col(-1)
  x = last_column.row(0)
  y = last_column.row(1)
  z = last_column.row(2)
  return sym.Array([x, y, z])
  
def verify_FK(t1, t2, t3, t4):
  fk = forward_kinematics()
  theta1, theta2, theta3, theta4 = sym.symbols('theta1, theta2, theta3, theta4')
  fk= fk.subs(theta1, t1)
  fk= fk.subs(theta2, t2)
  fk= fk.subs(theta3, t3)
  fk= fk.subs(theta4, t4)
  return fk

if __name__=='__main__':
  #print(transMatrix())
  #print()
  #print(transMatrix0To1())
  #print()
  #print(transMatrix0To2())
  #print()
  #print(transMatrix0To3())
  #print()
  #print(transMatrix0To4())
  #print()
  #print(forward_kinematics())
  #print()
  print(verify_FK(1.5,-0.3,1.0,0.8))
  
  
