from sympy import *
from sympy.abc import s, epsilon, k

from mathprint import *

'''
This function is to simplify one line/row by dividing all its elements 
with the greatest common divisor. However, before that, we must exclude
any element that is zero.
'''
def simplify_line(line):
    l = []
    for k in range(len(line)):
        if line[k] != 0.0:
            l.append(line[k])

    return gcd(l)
    
'''
This is the core function that generates the RH-table.
'''
def make_routh_table(sys):
    table = []

    # convert into fraction, 
    # take the coefficients of the denominator
    f = fraction(sys)
    den = Poly(f[1], s)
    a = den.all_coeffs()
    
    norder = len(a) - 1 # system order number
    nrow = norder + 1
    ncol = round(norder / 2)
    if ncol % 2 == 0:
        ncol = ncol + 1

    table = zeros(nrow, ncol)

    # Fill the first two rows
    table[0, 0:len(a[0::2])] = [a[0::2]]
    table[1, 0:len(a[1::2])] = [a[1::2]]
    
    for j in range(norder-1):
        r = simplify_line(table[j,:])
        table[j,:] = table[j,:] / r

        if table[j+1,:].is_zero_matrix == False:  # NOT a row of all zeros
            if (table[j+1,0]== 0):                # case 1: first column is zero, avoid division by zero
                table[j+1,0] = epsilon
        elif table[j+1,:].is_zero_matrix == True: # case2: a row of all zeros  
            expr = 0.0
            for k in range(0,ncol):
                expr =  expr + table[j,k]*s**((norder-2)-2*k)

            a = Poly(diff(expr,s), s).all_coeffs()[0::2]
            table[j+1,0:len(a)] = [a]

        for k in range(ncol-1):
            A = Matrix([[table[j,0],    table[j,k+1]],
                       [table[j+1,0],  table[j+1,k+1]]])
            
            table[j+2,k] = simplify(-A.det()/A[1,0])
        
    return table


'''
This function does not conclude the stability based on the generated Routh table. 
It simply generates and print the first-column signs which we can use to draw the conclusion.
'''
def evaluate_table(table):
    signs = []
    for k in range(len(table[:,0])):
        table[k,0] = table[k,0].subs(epsilon, 0.001) 
        if table[k,0].evalf() < 0:
            signs.append('-')
        elif table[k,0].evalf() > 0:
            signs.append('+')

    sp = ''
    cnt = 0
    for k, s in enumerate(signs):
        if (k > 0) and (s != sp):
            cnt = cnt + 1
        sp = s

    print(cnt, 'sign changes.')
    
    return signs


'''
This prints the Routh table
'''
def print_table(table):
    mprint(latex(Matrix(table)))