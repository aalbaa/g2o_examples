The C++ files are from g2o [repository](https://github.com/RainerKuemmerle/g2o/tree/master/g2o/examples/tutorial_slam2d). I simply added the `CMakeLists.txt` file.

In this repo, I implement a batch optimization using a linear system of the form
$$
\mathbf{x}_{k} = \mathbf{A}\mathbf{x}_{k-1} + \mathbf{B} \mathbf{u}_{k-1}
$$

# To do
1. Update the $\mathbf{B}$ matrix to be of size $2\times 1$.