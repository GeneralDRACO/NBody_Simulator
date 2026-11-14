# N-Body Gravitational Simulator

A 2D N-body gravitational simulator built using Python and Pygame. The project models the gravitational interactions between the Sun and five planets — Venus, Earth, Mars, Jupiter, and Saturn — using Newton's law of universal gravitation.

The simulation uses a fourth-order Runge–Kutta (RK4) numerical integration method to solve the equations of motion and includes real-time energy monitoring to observe numerical energy drift.

## Features

- Simulates the mutual gravitational interactions of six celestial bodies.
- Implements Newton's law of universal gravitation.
- Uses the fourth-order Runge–Kutta (RK4) integration method.
- Displays planetary trajectories in real time.
- Supports interactive camera panning and zooming.
- Uses a fixed physics timestep for numerical integration.
- Calculates total kinetic and gravitational potential energy.
- Displays relative energy drift during the simulation.

## Physics and Mathematical Model

### 1. Newtonian Gravitation

The gravitational acceleration of body *i* due to all other bodies is calculated as:

$$
\mathbf{a}_i =
G\sum_{\substack{j=1\\j\ne i}}^N
m_j
\frac{\mathbf{r}_j-\mathbf{r}_i}
{|\mathbf{r}_j-\mathbf{r}_i|^3}
$$

where:

- $G$ is the gravitational constant.
- $m_j$ is the mass of the interacting body.
- $\mathbf{r}_i$ and $\mathbf{r}_j$ are position vectors.
- $N$ is the total number of bodies.

### 2. Equations of Motion

The gravitational acceleration is used to solve the coupled equations of motion:

$$
\frac{d\mathbf{r}_i}{dt}=\mathbf{v}_i
$$

$$
\frac{d\mathbf{v}_i}{dt}=\mathbf{a}_i
$$

These equations describe the evolution of the position and velocity of every body in the system.

### 3. Numerical Integration — RK4

The system of differential equations is solved using the fourth-order Runge–Kutta method.

The update equation is:

$$
y_{n+1}=y_n+\frac{\Delta t}{6}
(k_1+2k_2+2k_3+k_4)
$$

where $k_1$, $k_2$, $k_3$, and $k_4$ represent the four intermediate evaluations of the derivative.

All bodies are updated using the intermediate states of the complete system.

### 4. Energy Monitoring

The total mechanical energy is calculated as the sum of kinetic and gravitational potential energy:

$$
E=K+U
$$

**Kinetic energy:**

$$
K=\sum_{i=1}^{N}\frac{1}{2}m_i|\mathbf{v}_i|^2
$$

**Gravitational potential energy:**

$$
U=-\sum_{i=1}^{N-1}\sum_{j=i+1}^{N}
\frac{Gm_im_j}
{|\mathbf{r}_i-\mathbf{r}_j|}
$$

The relative energy drift is monitored using:

$$
\Delta E(\%)=
\frac{E(t)-E(0)}{|E(0)|}\times100
$$

This provides a way to observe how the numerical integration affects the system's total energy over time.

## Simulation Units and Parameters

The simulation uses astronomical units:

| Quantity | Unit |
|---|---|
| Distance | Astronomical Unit (AU) |
| Mass | Solar Mass |
| Time | Year |
| Velocity | AU/year |
| Gravitational Constant | $4\pi^2$ |

The gravitational constant in these units is:

$$
G=4\pi^2
$$

The physics timestep is fixed at one simulated day:

$$
\Delta t=\frac{1}{365.25}\text{ years}
$$

The initial planetary positions and velocities are approximate. The planets are initialized using approximate orbital radii and circular heliocentric velocities, followed by a transformation into the barycentric reference frame.

## Technologies Used

- Python
- Pygame
- Math module

## Installation and Execution

### 1. Clone the repository

```bash
git clone https://github.com/GeneralDRACO/NBody_Simulator.git
```

### 2. Navigate to the project directory

```bash
cd NBody_Simulator
```

### 3. Install the required dependency

```bash
pip install pygame
```

### 4. Run the simulation

```bash
python main.py
```

## Controls

| Input | Action |
|---|---|
| Middle Mouse Button + Drag | Pan the camera |
| Mouse Wheel Up | Zoom in |
| Mouse Wheel Down | Zoom out |

## Current Limitations

- The simulation is restricted to two dimensions.
- Initial planetary orbits are approximated as circular.
- Orbital eccentricity and inclination are not included.
- Initial conditions are not epoch-specific astronomical ephemerides.
- The simulation uses a conventional RK4 integrator; long-term accuracy and convergence have not yet been systematically validated.

## Future Improvements

- Extend the simulation to three dimensions.
- Incorporate more realistic orbital initial conditions using astronomical ephemerides.
- Compare RK4 with symplectic integration methods.
- Study long-term energy and angular momentum conservation.
- Improve numerical accuracy and computational efficiency.

## Author

**GeneralDRACO**

GitHub Repository: [NBody_Simulator](https://github.com/GeneralDRACO/NBody_Simulator)