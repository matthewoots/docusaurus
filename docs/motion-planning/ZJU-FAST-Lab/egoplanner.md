---
id: egoplanner
title: Ego Planner
sidebar_label: Ego Planner 
---

## Overview
ZJU-FAST-Lab has constructed an Euclidean Signed Distance Field free (ESDF-free) Gradient-based Local Planner for Quadrotors. 

`ESDF` is used construct objects (etc point clouds) from noisy sensor data, what is needed to construct the ESDF is a dynamic updating of the  field is often needed during quadrotor flight.

Proposed by the ZJU team is that the generated ESDF contains **redundant information** that may not be used in the planning procedure at all. Such as the example given of **Searching** through a small portion of the ESDF and not the entire map.

- The GitHub repository can be found here (https://github.com/ZJU-FAST-Lab/ego-planner)
- The paper published can be found here (https://arxiv.org/abs/2008.08835)

## Method

Variable and notations are displayed in the table below :

| Notations | Descriptions |
|:----:|:----:| 
| $Q$ | Control points of a said trajectory (etc polynomial, B-spline curve) |
| $\Phi$ | Represents the trajectory (while satisfying the terminal constaints) |
| $\Tau$ | Represents the collision free path |

| Notations | Descriptions |
|:----:|:----:| 
| $Q_i$ | Control points at $i_{th}$ moment |
| $p_{ij}$ | Assigned anchor point at the **surface** of the obstacle |
| $v_{ij}$ | Assigned repulsion vector from **$Q_i$ to $p_{ij}$** |
| {$p,v$} | A pair that comes together with the $i$ (**index repsenting which control point**) and $j$ (**index of the pair**) notation repulsion vector from **$Q_i$ to $p_{ij}$** |
| $d_{ij}$ | Obstacle distance from $Q_i$ to the $j^{th}$ obstacle. $d_{ij} = (Q_i - p_{ij}) \cdot v_{ij}$ |

:::note
Seems like the $v_{ij}$ is a normal to the tangent of the trajectory curve at $Q_i$
:::

Hence a criteria must be that the obstacle distance is negative (which means that the control point is not "inside" the obstacle, meaning a collision-free control point $Q_i$) &nbsp &nbsp $d_{ij} \le 0$.   

## Optimization Problem

### Cost Generation

Optimization problem (formulation) where 
$$
min_Q J = \lambda_s J_s + \lambda_c J_c + \lambda_d J_d
$$
- $J_s$ = Smoothness penalty
- $J_c$ = Collision penalty
- $J_d$ = Feasibility penalty
- $\lambda_s,\lambda_c,\lambda_d$ = Respective weights (gains)

The optimizer can be found in the ego-planner repository under `bspline_optimizer.cpp`. which consist of the various params as stated below 
```cpp
nh.param("optimization/lambda_smooth", lambda1_, -1.0);
nh.param("optimization/lambda_collision", lambda2_, -1.0);
nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
nh.param("optimization/lambda_fitness", lambda4_, -1.0);

nh.param("optimization/dist0", dist0_, -1.0);
nh.param("optimization/max_vel", max_vel_, -1.0);
nh.param("optimization/max_acc", max_acc_, -1.0);

nh.param("optimization/order", order_, 3);
```

The combined cost function can be found in `void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)` which contains 
```cpp
calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility); 
```
- `cps_` represents `ControlPoints`, which contains the specific points in a 3 x N matrix `Eigen::MatrixXd points`
- `g_smoothness`, `g_distance`, `g_feasibility` represents the respective costs
-`Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;` the cost is 

- Main function is `BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)`, the idea is to pass a set of `cps_.points` which are the **optimal control** points, if a solution is not found, it will be handled by `rebound_optimize()` which has a while loop to handle the **flag** status of the results (check the enum status in `lbfgs.hpp` which mentions the return types)

- Function route called in 

| Function | Tagged Functions | Description | 
| --- | --- | --- |
|`BsplineOptimizeTrajRebound`|  `rebound_optimize` | `rebound_optimize` is a function that returns a `bool` this will indicate success and a **optimal control** path is found |
|`rebound_optimize`| `lbfgs::lbfgs_optimize()` | `int result` is what `lbfgs_optimize()` will return which a flag and `rebound_optimize` will help to filter the flag results. what lbfgs needs is |

### lbfgs_optimize() 

| Variable | Input | Description | 
| --- | --- | --- |
| `variable_num_` | `n` | **The number of variables**. |
| `q` | `x` | **The array of variables**. A client program can set default values for the optimization and receive the optimization result through this array. |
| `&final_cost` | `ptr_fx` | **The pointer to the variable that receives the final value of the objective function for the variables**. This argument can be set to NULL if the final value of the objective function is unnecessary. |
| `BsplineOptimizer::costFunctionRebound` | `proc_evaluate` | **The callback function to provide function and gradient evaluations** given a current values of variables. A client program must implement a callback function compatible with lbfgs_evaluate_t and pass the pointer to the callback function. |
| `NULL` | `proc_stepbound` | **The callback function to provide values of the upperbound of the stepsize to search in**, provided with the beginning values of variables before the linear search, and the current step vector (can be negative gradient). A client program can implement this function for more efficient linesearch. If it is not used, just set it NULL or nullptr. |
|`BsplineOptimizer::earlyExit` | `proc_progress` | The **callback function to receive the progress** (the number of iterations, the current value of the objective function) of the minimization process. This argument can be set to NULL if a progress report is unnecessary. |
| `this` | `instance` | A user data for the client program. **The callback functions will receive the value of this argument**. |
| `&lbfgs_params` | `param` |The pointer to a **structure representing parameters for L-BFGS optimization**. A client program can set this parameter to NULL to use the default parameters. Call lbfgs_load_default_parameters() function to fill a structure with the default values.|

Inside `lbfgs.hpp` while calling the `lbfgs_optimize()` function, the evaluation function contains the gradiant, and function data
```cpp
fx = cd.proc_evaluate(cd.instance, x, g, cd.n);
```
```cpp
double BsplineOptimizer::costFunctionRebound(void *func_data, 
const double *x, 
double *grad, 
const int n)
{
BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

double cost;
opt->combineCostRebound(x, grad, cost, n);

opt->iter_num_ += 1;
return cost;
}
```

### Smoothness Penalty

```cpp
calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness)
```
ZJU-team has combined the method of merging `Real-time  trajectory  replanning  for  mavs  using  uniform  b-splines  and  a  3dcircular  buffer` (time integral over square derivatives of the trajectory such as acceleration and jerk) and `Robust  and  effi-cient quadrotor trajectory generation for fast autonomous flight` (geometric information of the trajectory is taken regardless of time allocation)

Since **B-spline** has the convex hull property (https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-prop.html), and so by minimizing the second and third order derivatives of the B-spline, the derivatives can be minimized.

|Description|Formula|
|--|--|
| Cost | $J_s = \sum_{i = 1}^{N_c - 1} norm(A_i)_2^2 + \sum_{i = 1}^{N_c - 2} norm(J_i)_2^2$ |
| Velocity |$V_i = \frac{Q_{i+1} - Q_i}{\Delta{t}}$|
| Acceleration |$A_i = \frac{V_{i+1} - V_i}{\Delta{t}}$|
| Jerk |$J_i = \frac{A_{i+1} - A_i}{\Delta{t}}$|

### Collision Penalty

```cpp
calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness)
```
The collision penalty is used to push controls away (obstacle free control points), ZJU adopted safety clearance $s_f$ and punishing control points with $d_{ij}$
$$
j_c(i,j) = \left \{ \begin{array}{lll} 0 \quad \quad \quad (c_{ij} \le 0) \\ c_{ij}^3 \quad \quad \quad(0 < c_{ij} \le s_{f}) \\ 3s_fc_{ij}^2 - 3s_f^2c_{ij} +s_f^3 \quad (c_{ij} > s_{f}) \end{array} \right. 
$$
$$
c_{i,j} = s_f - d_{ij}
$$

Combining costs on all $Q_i$ yields the total cost $J_c$:
$$
J_c = \sum_{i = 1}^{N_c}j_c(Q_i)
$$

### Feasibility Penalty



## Helper Functions and Libraries

### L-BFGS Optimization

This library should be similar to `LBFGS++` which is a header only C++ library that implements the Limited-memory BFGS algorithm (L-BFGS) for unconstrained minimization (optimization) on twice continuously differentiable functions, the header repository can be found here (https://github.com/ZJU-FAST-Lab/LBFGS-Lite).

Flag status for the result of the optimization `Return values of lbfgs_optimize()`, usually a negative value indicates an error. 
```cpp
enum
{
    /** L-BFGS reaches convergence. */
    LBFGS_CONVERGENCE = 0,
    /** L-BFGS satisfies stopping criteria. */
    LBFGS_STOP,
    /** The initial variables already minimize the objective function. */
    LBFGS_ALREADY_MINIMIZED,

    /** Unknown error. */
    LBFGSERR_UNKNOWNERROR = -1024,
    /** Logic error. */
    LBFGSERR_LOGICERROR,
    /** The minimization process has been canceled. */
    LBFGSERR_CANCELED,
    /** Invalid number of variables specified. */
    LBFGSERR_INVALID_N,
    /** Invalid parameter lbfgs_parameter_t::mem_size specified. */
    LBFGSERR_INVALID_MEMSIZE,
    /** Invalid parameter lbfgs_parameter_t::g_epsilon specified. */
    LBFGSERR_INVALID_GEPSILON,
    /** Invalid parameter lbfgs_parameter_t::past specified. */
    LBFGSERR_INVALID_TESTPERIOD,
    /** Invalid parameter lbfgs_parameter_t::delta specified. */
    LBFGSERR_INVALID_DELTA,
    /** Invalid parameter lbfgs_parameter_t::min_step specified. */
    LBFGSERR_INVALID_MINSTEP,
    /** Invalid parameter lbfgs_parameter_t::max_step specified. */
    LBFGSERR_INVALID_MAXSTEP,
    /** Invalid parameter lbfgs_parameter_t::f_dec_coeff specified. */
    LBFGSERR_INVALID_FDECCOEFF,
    /** Invalid parameter lbfgs_parameter_t::wolfe specified. */
    LBFGSERR_INVALID_SCURVCOEFF,
    /** Invalid parameter lbfgs_parameter_t::xtol specified. */
    LBFGSERR_INVALID_XTOL,
    /** Invalid parameter lbfgs_parameter_t::max_linesearch specified. */
    LBFGSERR_INVALID_MAXLINESEARCH,
    /** The line-search step went out of the interval of uncertainty. */
    LBFGSERR_OUTOFINTERVAL,
    /** A logic error occurred; alternatively, the interval of uncertainty
    became too small. */
    LBFGSERR_INCORRECT_TMINMAX,
    /** A rounding error occurred; alternatively, no line-search step
    satisfies the sufficient decrease and curvature conditions. */
    LBFGSERR_ROUNDING_ERROR,
    /** The line-search step became smaller than lbfgs_parameter_t::min_step. */
    LBFGSERR_MINIMUMSTEP,
    /** The line-search step became larger than lbfgs_parameter_t::max_step. */
    LBFGSERR_MAXIMUMSTEP,
    /** The line-search routine reaches the maximum number of evaluations. */
    LBFGSERR_MAXIMUMLINESEARCH,
    /** The algorithm routine reaches the maximum number of iterations. */
    LBFGSERR_MAXIMUMITERATION,
    /** Relative width of the interval of uncertainty is at most
    lbfgs_parameter_t::xtol. */
    LBFGSERR_WIDTHTOOSMALL,
    /** A logic error (negative line-search step) occurred. */
    LBFGSERR_INVALIDPARAMETERS,
    /** The current search direction increases the objective function value. */
    LBFGSERR_INCREASEGRADIENT,
};
```

This is the formula of `lbfgs_optimize()`
```cpp
/*
*  @param  n           The number of variables.
*  @param  x           The array of variables. A client program can set
*                      default values for the optimization and receive the
*                      optimization result through this array.
*  @param  ptr_fx      The pointer to the variable that receives the final
*                      value of the objective function for the variables.
*                      This argument can be set to NULL if the final
*                      value of the objective function is unnecessary.
*  @param  proc_evaluate   The callback function to provide function and
*                          gradient evaluations given a current values of
*                          variables. A client program must implement a
*                          callback function compatible with 
*                          lbfgs_evaluate_t and pass the pointer to the
*                          callback function.
*  @param  proc_stepbound  The callback function to provide values of the
*                          upperbound of the stepsize to search in, provided
*                          with the beginning values of variables before the 
*                          linear search, and the current step vector (can 
*                          be negative gradient). A client program can implement
*                          this function for more efficient linesearch. If it is
*                          not used, just set it NULL or nullptr.
*  @param  proc_progress   The callback function to receive the progress
*                          (the number of iterations, the current value of
*                          the objective function) of the minimization
*                          process. This argument can be set to NULL if
*                          a progress report is unnecessary.
*  @param  instance    A user data for the client program. The callback
*                      functions will receive the value of this argument.
*  @param  param       The pointer to a structure representing parameters for
*                      L-BFGS optimization. A client program can set this
*                      parameter to NULL to use the default parameters.
*                      Call lbfgs_load_default_parameters() function to 
*                      fill a structure with the default values.
*  @retval int         The status code. This function returns zero if the
*                      minimization process terminates without an error. A
*                      non-zero value indicates an error.
*/
inline int lbfgs_optimize(int n,
                            double *x,
                            double *ptr_fx,
                            lbfgs_evaluate_t proc_evaluate,
                            lbfgs_stepbound_t proc_stepbound,
                            lbfgs_progress_t proc_progress,
                            void *instance,
                            lbfgs_parameter_t *_param)
```

### Math Examples
In side a text chunk, you can use mathematical notation if you surround it by dollar signs `$` for “inline mathematics” and `$$` for “displayed equations”. Do not leave a space between the `$` and your mathematical notation.

Example: `$\sum_{n=1}^{10} n^2$` is rendered as $\sum_{n=1}^{10} n^2$


Example: `$$\sum_{n=1}^{10} n^2$$` is rendered as
$$\sum_{n=1}^{10} n^2$$

The mathematical typesetting is based on LaTeX, so if you need to search for the way to make a particular symbol, include latex in your search. But note: Not all LaTeX macros are available without using additional packages, and those packages likely will only work if you are creating a PDF. On the plus side, if you are working in PDF, you can use additional packages that give much better control and/or easier syntax.

$$  
f(x) = \left\{ \begin{array}{ll} x^2 \quad x > 0 \\ 0 \quad \text{else} \end{array} \right. 
$$
$$
\begin{array}{|l|cr}
left1 & center1 & right1\\
\hline
d & e & f
\end{array}
$$
$$
\begin{array}{lcl}
z & = & a \\
& = & a \\
f(x,y,z) & = & x + y + z
\end{array} 
$$  