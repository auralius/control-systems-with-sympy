'''
Auralius Manurung
Universitas Telkom
2025
'''

from IPython.display import display, Math, Latex

def mprint(*s, boxed=False):
    r = r""
    
    for s_ in s:
        r = r + s_
    
    r = r

    display(Math(r))


def mprintb(*s):
    r = r""
    r = r + "\\boxed{"
        
    for s_ in s:
        r = r + s_
    
    r = r + "}"
    r = r + "$"

    display(Math(r))