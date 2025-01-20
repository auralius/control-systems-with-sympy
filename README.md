# control-systems-with-sympy
Learning control systems with symbolic mathematics, specifically with SymPy

Topics covered so far:

- [Root locus plotting](https://github.com/auralius/control-systems-with-sympy/blob/main/root-locus.ipynb)

The root-locus plot is animated. The animation shows the movements of the closed-loop poles with respect to $K$.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/4.gif" alt="Alt Text" style="width:70%; height:auto;">

Addiitionally, $j\omega$-crossings and break-away / entry points are also calculated analytically. The provided function is also capable of defining whether the calculated points act as break-away points or entry points.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/demo-rlocus.png" alt="Alt Text" style="width:50%; height:auto;">

- [Block diagram simplifications](https://github.com/auralius/control-systems-with-sympy/blob/main/block-diagram.ipynb)

Exisiting block-diagram simplification functions only take transfer functions as the inputs. Here, we allow functions of any variable to be used as the inputs to our block-diagram simplification functions. 

- [Routh-Hurwitz criterion](https://github.com/auralius/control-systems-with-sympy/blob/main/routh-hurwitz.ipynb)

Here, we can generate the RH-table, including for the two special cases: only the first element in any one row is zero and all elements in any one row are zero. The numbers in each row of the generated table are already simplified by dividing them with their greatest common divisor (GCD). Finally, the signs of each row of the first column are also presented.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/demo-rh-table.png" alt="Alt Text" style="width:60%; height:auto;">

- [Rectilinear system](https://github.com/auralius/control-systems-with-sympy/blob/main/rectilinear.ipynb)

This is to perform modeling for a general mass-spring-damper system in one dimensional space (rectilinear system). To build the model, we must first define two adjacency matrices, one for the springs and another one for the dampers. Principally, we use a parallel unidirectional graph to model the mass-spring-damper system. 
