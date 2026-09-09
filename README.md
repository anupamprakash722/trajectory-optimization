# trajectory-optimization

> Asking a drone for motion it can actually produce — minimum-snap trajectories as a
> convex program, time allocation, corridors and potential fields, handed to the
> controller from Project 5.
>
> **Project 6** of my [drone robotics learning roadmap](https://github.com/anupamprakash722/drone-robotics-learning-roadmap).

---

## What this is

Ten Jupyter notebooks in the same style as
[`math-foundations-sandbox`](https://github.com/anupamprakash722/math-foundations-sandbox):
one idea per section, small readable code cells, two exercises with worked solutions, a
mini-project with an animation, and a closing robotics connection.

Project 5 ended with a working quadcopter and one uncomfortable detail: every step
command saturated the motors. The fix is not a better controller — it is a better
question, and that is what this project builds.

| # | Notebook | What you build |
|---|---|---|
| 01 | `01_Why_Not_a_Straight_Line.ipynb` | step vs ramp vs smooth; reading peaks before flying |
| 02 | `02_Polynomials_as_Trajectories.ipynb` | derivatives, boundary conditions, a requirement as a row |
| 03 | `03_Minimum_Jerk.ipynb` | the cost integral, the cost matrix, the quintic |
| 04 | `04_From_Jerk_to_Snap.ipynb` | snap → torque → motor imbalance, and the septic |
| 05 | `05_One_Segment_as_a_QP.ipynb` | quadratic cost, linear constraints, the KKT system |
| 06 | `06_Multi_Segment_Trajectories.ipynb` | continuity, and flying *through* waypoints |
| 07 | `07_Solving_with_CVXPY.ipynb` | inequality constraints, speed caps, infeasibility |
| 08 | `08_Time_Allocation.ipynb` | why times sit outside the QP; scaling laws; margin |
| 09 | `09_Obstacles_and_Potential_Fields.ipynb` | corridors (convex) and potential fields (not) |
| 10 | `10_Tracking_the_Trajectory.ipynb` | handed to Project 5's cascade, unchanged |

## The idea in one line

$$\min_c \; c^\top Q c \quad \text{s.t.} \quad Ac = b, \; Gc \le h$$

$Q$ integrates squared snap; $Ac = b$ holds the waypoints and continuity; $Gc \le h$ holds
the speed caps and corridors. Snap is the fourth derivative of position, and Notebook 04
traces the chain that makes it the one worth minimising: acceleration is tilt, jerk is
angular rate, snap is torque — which is the difference between the four motor thrusts.

## Results the notebooks produce

* **The solver is exact.** A single rest-to-rest segment reproduces the closed-form
  $35\tau^4 - 84\tau^5 + 70\tau^6 - 20\tau^7$ to about 1e-13, and CVXPY reproduces the KKT
  answer to the same precision (Notebooks 04, 07).
* **Continuity to machine precision.** At every interior join the two polynomials agree in
  position, velocity, acceleration and jerk to ~1e-15 — and the velocity there is *not*
  zero, so the drone flies through (Notebook 06).
* **Stopping at waypoints is far more expensive** than flying through them, on the same
  route and timing (Notebook 06, E1).
* **Minimum snap tracks better than minimum jerk** on an identical route and duration,
  with less tilt — and the controller never changed (Notebook 10).
* **Feedforward is worth an order of magnitude** in tracking error on the same path
  (Notebook 10).

## Three findings that contradict the tidy story

**"Minimum snap" is not the lowest-snap trajectory.** With position, velocity and
acceleration fixed at the endpoints and jerk left free, the snap-optimal polynomial beats
both the minimum-jerk quintic and the standard minimum-snap septic. It is minimal
*subject to* zero jerk at the endpoints, and those two extra constraints raise the
achievable minimum. Notebook 04, Section 4.

**Slack time makes the path wander, not slow down.** Give a segment four times its share
and the trajectory swings metres off a route whose two endpoints are in the same place —
because a wide smooth excursion has less snap than decelerating to a stop and starting
again. Notebook 06, E2.

**More time cannot fix a tight corridor.** Scaling all segment times changes the clock,
not the path, so a corridor that is infeasible stays infeasible at four times the
duration. What fixes it is more waypoints. Notebook 09, Section 2.

## And one about planning margin

A trajectory sized for *exactly* the controller's 35° tilt limit is flown at rather more
than 35°, because the planner's acceleration is the feedforward demand and the feedback
loops add corrections on top. Planning to around 70% of the limit leaves room for that.
The right fraction is a property of your controller and your vehicle — Notebook 08
measures it rather than quoting it.

## Running it

```bash
pip install -r requirements.txt
jupyter lab
```

CVXPY is the only new dependency; everything else is NumPy, SciPy and Matplotlib. Each
notebook is self-contained, and all ten contain animations rendered as an in-browser
JavaScript player, so no ffmpeg is needed.

## Tests

```bash
pytest -q
```

`tests/test_notebooks_run.py` executes every notebook in a clean kernel and fails on the
first error. The numerical checks — closed-form comparisons, KKT-vs-CVXPY agreement,
continuity to machine precision — are inside the notebooks and print their own results.

## What this planner does not do

The corridor assumes something else has already found a safe route; Notebook 09 places the
detour waypoint by hand. Segment times come from a heuristic plus a bisection, with no
optimality guarantee, because optimising over times is non-convex. A keep-out region is
not a convex constraint, so obstacles are handled either by routing around them or by a
potential field — and Notebook 09 shows the potential field getting stuck in a local
minimum, which is the honest cost of leaving convexity.

## License

MIT — see [LICENSE](LICENSE).
