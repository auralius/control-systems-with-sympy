# control-systems-with-sympy
Learning control systems with symbolic mathematics, specifically with SymPy

Topics covered so far:

- [Root locus plotting](https://github.com/auralius/control-systems-with-sympy/blob/main/root-locus.ipynb)

The root-locus plot is animated. Here, we can see the movements of the closed loop poles.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/4.gif" alt="Alt Text" style="width:70%; height:auto;">

Addiitionally, $j\omega$-crossings and break-away / entry points are also calculated analytically. The provided function is also capable of defining whether the points act as break-away point or entry point.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/demo-rlocus.png" alt="Alt Text" style="width:50%; height:auto;">

- [Block diagram simplifications](https://github.com/auralius/control-systems-with-sympy/blob/main/block-diagram.ipynb)

Exisiting simplification functions only take a transfer function as their input. Here, we allow general variables / functions to be used as inputs to our simplification functions. 

- [Routh-Hurwitz criterion](https://github.com/auralius/control-systems-with-sympy/blob/main/routh-hurwitz.ipynb)

Here, we can generate the RH-table, including for the two special cases: only the first element in any one row is zero and all elements in any one row are zero. The numbers in each row of the generated table are already simmplified by dividing them with their greatest common divisor (GCD). Finally, the signs of each row of the first columns are also presented.

<img src="https://github.com/auralius/control-systems-with-sympy/blob/main/images/demo-rh-table.png" alt="Alt Text" style="width:50%; height:auto;">

