# Object Fusion

This package contains a library that implements a multi-instance Kalman filter for object-level sensor data fusion and tracking. 
An overview over the functionality is given in [../README.md](../README.md).

## Mathematical symbols

The Kalman filter symbols in the code are:

| **Symbol in code**        | **Meaning**                                                             | **Symbol in ACDC slides, remarks** |
| ----------------- |:------------------------------------------------------                          | :---------------  |
| `x_hat_G`             | Global object state vector                                                  |   $\hat{x}_g$     |
| `globalObject.P()`      | Global object state vector error covariance                               |   $\mathbf{P_G}$  |
| `F`             | State transition matrix (motion model matrix)                                     |   $\mathbf{F}$    |
| `Q`             | Process noise matrix (adds noise during prediction)                               |   $\mathbf{Q}$    |
| `C`             | Measurement matrix (reduces global state space to measured space)                 |   $\mathbf{C}$    |
| `x_hat_S`       | Vector of measured and non-measured variables of the sensor-level object          |   $\hat{x}_S$, to get it in code: `IkaUtilities::getEigenStateVec(&measuredObject)` |
| `z`             | Vector of actually measured variables (in the measured space)                     |   $\mathbf{z}$    |
| `P_S_diag`      | Variance vector of measured and non-measured variables of the sensor-level object |   $\mathbf{P_S}$, but only its diagonal in the code: `IkaUtilities::getEigenVarianceVec(&measuredObject)`             |
| `R`             | Measured variables error covariance matrix                                        |   $\mathbf{R}$    |
| `S`             | Innovation error covariance (or residual error covariance)                        |   $\mathbf{S}$    |
| `K`             | Kalman gain                                                                       |   $\mathbf{K}$    |
