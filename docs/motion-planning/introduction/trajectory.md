---
id: trajectory
title: Trajectory
sidebar_label: Trajectory Generation
---

### Overview

The whole trajectory is divided into a series of segments indicated by waypoints. Each of these segments has a polynomial-based trajectory from its starting waypoint to an ending waypoint. This relationship can be described by the **$N^{th}$-order polynomial** (**P(t)**) as shown below. 

However, this function only covers the trajectory of one axis. Thus, in order to fully represent a 3D trajectory, the formula must be used on **all axes** (**k**).
$$
P(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + ... + a_Nt^N
$$

### Optimization 
:::info
From `University of Pennsylvania`, `Vijay Kumar` online course notes found [here](https://prod-edxapp.edx-cdn.org/assets/courseware/v1/834346628b86ff76fe98a975b10f51cf/asset-v1:PennX+ROBO3x+2T2017+type@asset+block/Robo3x-Week11-final.pdf)
:::
To find the optimal position at a given axis. 
$$
x^{*}(t) = argmin_{x(t)} \int_{0}^T L(\dot{x},x,t)dt
$$

**Euler Lagrange Equation** : Necessary condition satisfied by the “optimal” function $x(t)$.

$$
\frac{d}{dt} \frac{dL}{d\dot{x}} - \frac{dL}{dx} = 0
$$

For $n = 1$, $\dot{x}$ represented n = 1
$$
L(\dot{x},x,t) = (\dot{x})^2 \to \ddot{x} = 0, x = c_1t + c_0
$$
$$
x^{*}(t) = argmin_{x(t)} \int_{0}^T \dot{x}^2dt
$$

For general $n$
$$
x^{*}(t) = argmin_{x(t)} \int_{0}^T (x^{(n)})^2dt
$$
$$
x^{*}(t) = argmin_{x(t)} \int_{0}^T L(x^{n}, x^{n-1}, ... ,\dot{x}, x, t)dt
$$
$$
\frac{dL}{dx} - \frac{d}{dt} \frac{dL}{d\dot{x}} + \frac{d^2}{dt^2} \frac{dL}{d\ddot{x}} + ... + (-1)^n \frac{d^n}{dt^n} \frac{dL}{dx^{n}} = 0
$$
- $n = 1$, Shortest Distance
- $n = 2$, Minimum Acceleration
- $n = 3$, Minimum Jerk
- $n = 4$, Minimum Snap

### Minimum Jerk Trajectory 
The Minimum Jerk trajectory is a 5th order polynomial, since there are 6 coefficients, with characteristics of :

- **Polynomial** &nbsp &nbsp $x = c_5t^5 + c_4t^4 + c_3t^3 + c_2t^2 + c_1t + c_0$
- **Euler-Lagrange** &nbsp &nbsp $x^{(6)} = 0$


### Minimum Snap Trajectory
The Minimum Snap trajectory is a 7th order polynomial, since there are 8 coefficients, with characteristics of :
- **Polynomial** &nbsp &nbsp $x = c_7t^7 + c_6t^6 + c_5t^5 + c_4t^4 + c_3t^3 + c_2t^2 + c_1t + c_0$
- **Euler-Lagrange** &nbsp &nbsp $x^{(8)} = 0$

#### Optimizing Trajectory (Minimum Snap Trajectory)
:::info
Following `Time-Continuous Real-Time Trajectory Generation for Safe
Autonomous Flight of a Quadrotor in Unknown Environment` found [here](https://www.mdpi.com/2076-3417/11/7/3238)
:::
For an $N^{th}$-order polynomial, the trajectory is represented below

1. **Polynomial** &nbsp $x = c_Nt^N + c_{N-1}t^{N-1} + ... + c_2t^2 + c_1t + c_0$
2. **Polynomial Coefficients** &nbsp $p = [ c_0 , c_1 , c_2, ··· , c_N]^T$, &nbsp where $p_{size} = (N * 1)$ are polynomial coefficients
3. **Minimizing Trajectory** &nbsp By minimizing trajectory, an optimal trajectory can be obtained. Using 
$$
Ap = d
$$
$$
d_m = [p_m , v_m , a_m , j_m , s_m , p_{m + 1} , v_{m + 1} , a_{m + 1} , j_{m + 1} , s_{m + 1}]^T
$$ 
- Where $A_{size} = (N * N)$.
- Where $p_m , v_m , a_m , j_m , s_m$ is the $m_{th}$ segment of position, velocity, acceleration, jerk, and snap.
- Vector d represents trajectory constraints of position (p), velocity (v), acceleration (a), jerk (j), and snap (s)


### ZJU-FAST-Lab
:::note
The content below is from **PolynomialTraj::minSnapTraj()** function.
```cpp
PolynomialTraj PolynomialTraj::minSnapTraj(
    const Eigen::MatrixXd &Pos, // Waypoint list
    const Eigen::Vector3d &start_vel, // Start velocity
    const Eigen::Vector3d &end_vel, // End velocity
    const Eigen::Vector3d &start_acc, // Start acceleration
    const Eigen::Vector3d &end_acc, // End acceleration
    const Eigen::VectorXd &Time // The time array for the waypoint list
    )
```
:::

For the output it is in PolynomialTraj class format
```cpp
vector<double> times;       // time of each segment
vector<vector<double>> cxs; // coefficient of x of each segment, from n-1 -> 0
vector<vector<double>> cys; // coefficient of y of each segment
vector<vector<double>> czs; // coefficient of z of each segment
```

In addition for every `poly_coeff.rows()` a add segment function is called `poly_traj.addSegment(cx, cy, cz, ts);` this would add the vector cx, cy, cz and t to the list.
```cpp
void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
{
  cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
}
```

#### Individual Segments of the code
```cpp
int seg_num = Time.size(); // The time array for the waypoint list
Eigen::MatrixXd poly_coeff(seg_num, 3 * 6); // (size of waypoint list, number of axes * coefficients)
Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

int num_f, num_p; // number of fixed and free variables
int num_d;        // number of all segments' derivatives
```
```cpp
/* ---------- end point derivative ---------- */
Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

// To initialize the waypoints and the states
for (int k = 0; k < seg_num; k++)
{
  /* position to derivative */
  Dx(k * 6) = Pos(0, k);
  Dx(k * 6 + 1) = Pos(0, k + 1);
  Dy(k * 6) = Pos(1, k);
  Dy(k * 6 + 1) = Pos(1, k + 1);
  Dz(k * 6) = Pos(2, k);
  Dz(k * 6 + 1) = Pos(2, k + 1);

  if (k == 0) // Starting Velocity and Acceleration
  {
    Dx(k * 6 + 2) = start_vel(0);
    Dy(k * 6 + 2) = start_vel(1);
    Dz(k * 6 + 2) = start_vel(2);

    Dx(k * 6 + 4) = start_acc(0);
    Dy(k * 6 + 4) = start_acc(1);
    Dz(k * 6 + 4) = start_acc(2);
  }
  else if (k == seg_num - 1) // Ending Velocity and Acceleration
  {
    Dx(k * 6 + 3) = end_vel(0);
    Dy(k * 6 + 3) = end_vel(1);
    Dz(k * 6 + 3) = end_vel(2);

    Dx(k * 6 + 5) = end_acc(0);
    Dy(k * 6 + 5) = end_acc(1);
    Dz(k * 6 + 5) = end_acc(2);
  }
}
```

Using the formula Factorial
```cpp
const static auto Factorial = [](int x) {
  int fac = 1;
  for (int i = x; i > 0; i--)
    fac = fac * i;
  return fac;
};
```

We can now construct the A matrix that is suppose to be $(6*6)$ however because of the multiple segments (waypoints) the matrix is multiplied with the `seg_num`
```cpp
/* ---------- Mapping Matrix A ---------- */
Eigen::MatrixXd Ab;
Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

for (int k = 0; k < seg_num; k++)
{
  Ab = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 3; i++)
  {
    Ab(2 * i, i) = Factorial(i);
    for (int j = i; j < 6; j++)
      Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
  }
  // BlockXpr block(Index startRow, Index startCol, Index blockRows, Index blockCols), block method is part of Eigen
  // Meaning place values in a section of the matrixs
  A.block(k * 6, k * 6, 6, 6) = Ab;
}
```

    
```cpp
/* ---------- Produce Selection Matrix C' ---------- */
Eigen::MatrixXd Ct, C;

num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
num_d = 6 * seg_num;
Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
Ct(0, 0) = 1;
Ct(2, 1) = 1;
Ct(4, 2) = 1; // stack the start point
Ct(1, 3) = 1;
Ct(3, 2 * seg_num + 4) = 1;
Ct(5, 2 * seg_num + 5) = 1;

Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

for (int j = 2; j < seg_num; j++)
{
  Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
  Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
  Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
  Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
  Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
  Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
}

C = Ct.transpose();

Eigen::VectorXd Dx1 = C * Dx;
Eigen::VectorXd Dy1 = C * Dy;
Eigen::VectorXd Dz1 = C * Dz;

/* ---------- minimum snap matrix ---------- */
Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

for (int k = 0; k < seg_num; k++)
{
  for (int i = 3; i < 6; i++)
  {
    for (int j = 3; j < 6; j++)
    {
      Q(k * 6 + i, k * 6 + j) =
          i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
    }
  }
}

/* ---------- R matrix ---------- */
Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

Dxf = Dx1.segment(0, 2 * seg_num + 4);
Dyf = Dy1.segment(0, 2 * seg_num + 4);
Dzf = Dz1.segment(0, 2 * seg_num + 4);

Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

/* ---------- close form solution ---------- */

Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

Px = (A.inverse() * Ct) * Dx1;
Py = (A.inverse() * Ct) * Dy1;
Pz = (A.inverse() * Ct) * Dz1;

for (int i = 0; i < seg_num; i++)
{
  poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
  poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
  poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
}

/* ---------- use polynomials ---------- */
PolynomialTraj poly_traj;
for (int i = 0; i < poly_coeff.rows(); ++i)
{
  vector<double> cx(6), cy(6), cz(6);
  for (int j = 0; j < 6; ++j)
  {
    cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
  }
  reverse(cx.begin(), cx.end());
  reverse(cy.begin(), cy.end());
  reverse(cz.begin(), cz.end());
  double ts = Time(i);
  poly_traj.addSegment(cx, cy, cz, ts);
}

return poly_traj;
}
```

