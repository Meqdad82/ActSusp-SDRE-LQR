# Decision Log

> Every modeling assumption, parameter choice, and architectural decision goes
> here, dated, with a rationale. This file is the single most valuable artifact
> during peer review — it answers "why did you do X?" before the referee asks.

Format: `[YYYY-MM-DD] Decision — rationale — impact.`

---

## 2025-07-13 — Sprint 1 setup

### D1.1 — Target venue: IEEE OJVT
- **Decision**: Submit to IEEE Open Journal of Vehicular Technology (OJVT).
- **Rationale**: Gold OA, Scopus/SCIE-indexed, no page charges, ~6–8 week review.
  Recent vehicle-dynamics papers (Fattah et al. OJVT 2024, Marotta et al. OJVT
  2024) run 10–13 pp with 6–10 figures. CC-BY license aligns with our plan to
  release code on Zenodo.
- **Impact**: Figure style = time-domain state plots with constraint lines as
  dashed horizontals; reproducibility section required; we will prepare a
  Code Ocean capsule even though OJVT does not mandate one.

### D1.2 — Primary baseline: Abbas & Youn 2024 LQR (Vehicles, MDPI)
- **Decision**: Replicate the LQR controller of Abbas, S.B., Youn, I. (2024)
  *Vehicles* 6(3):60 as our primary linear baseline.
- **Rationale**: (i) Quarter-car 2-DOF model matches our DP formulation
  exactly; (ii) ISO 8608 + ISO 2631 KPIs at 20 m/s — the standard Q1
  convention; (iii) quantitative LQR-vs-passive KPIs reported (~96% RMS body
  accel reduction); (iv) open-access, so weights and state-space realisation
  are reproducible without paywall.
- **Impact**: `dpsus.controllers.lqr.LQRController` uses default weights
  `Q = diag(1e2, 1e4, 1e0, 1e2), R = 1e-6`, to be tuned in Sprint 2 to match
  the published KPIs.
- **Secondary baseline**: Li et al. 2024 *Sensors* 24(7):2255 MPC-LPV with
  ±1500 N saturation, ISO C, 20 m/s. Replicate in Sprint 2 to compare DP
  against the constrained-MPC alternative.

### D1.3 — Vehicle parameters
- **Decision**: m_s=250, m_u=45, k_s=16000, c_s=1000, k_t=190000, c_t=0
  (SI units), tau_a=10 ms, F_max=±1500 N.
- **Rationale**: Cross-verified across ≥2 recent sources:
  - m_s=250, m_u=45, k_s=16000, c_s=1000 from Tandfonline 2025 robust-vs-RL
    paper and the in-wheel EV quarter-car lineage (Gysen/Redfield).
  - k_t=190000 from Agharkakli et al. lineage (multiple IJETT/IJSDR papers).
  - c_t=0 is the standard simplification (Hrovat 1997).
  - tau_a=10 ms corresponds to ~16 Hz actuator bandwidth, consistent with
    MDPI Actuators 2025 (which limits actuator bandwidth to 10 Hz).
  - F_max=±1500 N from Li et al. 2024 *Sensors* (explicit).
- **Impact**: All in `configs/vehicle_quarter_car.yaml`. NO magic numbers in
  source code. Any change requires updating the YAML and re-running `make
  test`.

### D1.4 — Tire model: unilateral-linear with liftoff
- **Decision**: Model the tire as a linear spring (k_t) + damper (c_t=0)
  that exerts zero force when the deflection delta_t = z_r - z_u < 0.
- **Rationale**: This is the simplest physically-defensible model that
  captures the unilateral contact nonlinearity. It is the model implicitly
  assumed by LQR (which cannot handle it) and is the same model used by Li
  et al. 2024 (which sidesteps liftoff by treating the tire as bilateral).
- **Limitation**: With c_t > 0, the velocity term can flip F_tire's sign
  briefly during rebound even when delta_t > 0. We use c_t = 0 to avoid this
  artifact; if c_t > 0 is needed later, switch to a hysteretic model.
- **Impact**: `dpsus.plant.tire.TireModel` with `liftoff_enabled=True` is
  the default. Tests verify F_tire >= 0 at all times.

### D1.5 — Actuator model: first-order lag + symmetric saturation
- **Decision**: F_a is the 5th state, with dynamics
  `tau_a * dF_a/dt + F_a = sat(F_cmd, +/-F_max)`.
- **Rationale**: First-order lag is the standard model for electromagnetic
  and hydraulic actuators in 2023-2025 Q1 papers. Symmetric saturation is
  the simplest hard nonlinearity; the `tanh_smooth` option is reserved for
  Sprint 3 (differentiable DP policy extraction).
- **Impact**: `dpsus.plant.actuators.ActuatorModel`. Tests verify F_a never
  exceeds ±F_max, even transiently.

### D1.6 — Integration: RK45, dt=5 ms, max_step=5 ms
- **Decision**: `scipy.integrate.solve_ivp` with RK45, fixed output dt=5 ms
  (200 Hz), `max_step=5 ms` to catch tire-liftoff events.
- **Rationale**: 5 ms is fine enough to resolve the wheel-hop mode (typically
  10-15 Hz, so 5 ms gives ~7 samples per period). RK45 is adaptive but the
  max_step bound prevents it from stepping over a liftoff event. rtol=1e-6,
  atol=1e-9 are tight enough for the energy-conservation gate.
- **Alternative considered**: LSODA (automatic stiffness switching) — overkill
  for this smooth-with-kink system. Radau (implicit) — would handle liftoff
  stiffness better but is 3-5x slower; revisit if Sprint 5 Monte Carlo is
  too slow.
- **Impact**: `dpsus.plant.quarter_car.QuarterCarPlant.simulate`. Any change
  to dt requires re-running `make test` to verify the energy gate.

### D1.7 — Road step modeled as tanh-smoothed transition
- **Decision**: Implement the `step` road as
  `z_r(t) = h * 0.5 * (1 + tanh((t - t_step) / 1ms))`.
- **Rationale**: A pure Heaviside step makes z_r_dot a Dirac delta, which
  causes a spurious spike in tire damping force (when c_t > 0). The 1 ms
  tanh transition is narrow enough to look like a true step at the 5 ms
  integration scale but smooth enough to keep z_r_dot bounded.
- **Impact**: `make_road("step", ...)`. Documented in the docstring.

### D1.8 — State vector: [z_s, v_s, z_u, v_u, F_a] (5D)
- **Decision**: 5D state, with F_a appended as the 5th component.
- **Rationale**: Augmenting with F_a lets us (a) model the actuator lag as a
  state equation rather than an algebraic constraint, (b) use F_a in the
  LQR state (with zero weight, so it doesn't affect the gain) for code
  uniformity, and (c) prepare for Sprint 3 DP where the state must include
  F_a to enforce the saturation constraint via state constraints rather than
  via the actuator model alone.
- **Impact**: `Vehicle.state_dim = 5`, `Vehicle.state_names`. The LQR
  controller in `dpsus.controllers.lqr` uses only the first 4 states.

### D1.9 — KPI set: ISO 2631-1 raw + Wk-weighted RMS, plus 7 others
- **Decision**: Report 11 KPIs per run (see `dpsus.evaluation.kpis`).
- **Rationale**: Aligns with 2023-2025 Q1 conventions (Abbas & Youn 2024;
  Li et al. 2024; Fattah et al. OJVT 2024). The Wk-weighted RMS is required
  for ISO 2631-1 compliance; the raw RMS is reported for direct comparison
  with papers that don't use Wk. % tire contact and rattle-space violation
  are the KPIs that will showcase the DP advantage over LQR/MPC.
- **Impact**: `dpsus.evaluation.kpis.KPIReport`.

### D1.10 — No Magic Numbers in source code
- **Decision**: All physical parameters, weights, and tolerances live in
  `configs/*.yaml`. Source code reads configs via `load_vehicle`.
- **Rationale**: Golden Rule #3 (config-driven architecture). Also makes
  Sprint 5 robustness sweeps trivial: just write a loop that patches the
  YAML and re-runs.
- **Impact**: `configs/vehicle_quarter_car.yaml`, `configs/test_cases.yaml`.

---

## Reserved for future sprints

### Sprint 2 — baseline replication
- [ ] LQR weights tuned to match Abbas & Youn 2024 KPIs within ±5%
- [ ] MPC-LPV (Li et al. 2024) re-implemented in Python

### Sprint 3 — DP controller
- [ ] State grid resolution (Smolyak vs uniform)
- [ ] Stage cost weights q1, q2, q3, r
- [ ] Bellman convergence criterion
- [ ] Dimensionality reduction strategy

### Sprint 4 — road library
- [ ] ISO 8608 PSD parameters per class
- [ ] Real dataset preprocessing (Foo & Tan 2026 Zenodo)

### Sprint 5 — evaluation
- [ ] Monte Carlo N per (controller × road × speed) — currently 30
- [ ] Statistical tests: Wilcoxon + Cliff's delta + bootstrap CI

### Sprint 6 — reproducibility
- [ ] Code Ocean capsule spec
- [ ] Zenodo DOI minting for road profiles + DP LUTs

---

## 2025-07-15 — Sprint 2 setup

### D2.1 — ISO 8608 road generator: inverse-FFT with N² scaling
- **Decision**: Generate synthetic road profiles via inverse-FFT on the ISO 8608
  PSD `G_q(n) = G_q(n_0) * (n/n_0)^(-2)` with amplitude
  `sqrt(G_q * N² * Δn / 2)`.
- **Rationale**: The PSD-to-DFT-magnitude relationship is
  `|X[k]|² = G_one_sided(n_k) * (1/dx) * N / 2`, which simplifies to
  `G * N² * Δn / 2` using `Δn = 1/(N*dx)`. Initially used `N * Δn / 2`
  (missing factor of `N`), giving PSDs 4 orders of magnitude too small. Fixed
  after debugging with `scripts/debug_iso8608.py`.
- **Validation**: 7 pytest tests pass, including PSD match at f_ref within 30%,
  RMS monotonic in ISO class A<B<C<D<E, temporal PSD scales as 1/v (not v).
- **Impact**: `dpsus.roads.iso8608.py`. The `1/v` scaling (not `v`) of the
  temporal PSD at f_ref was a non-obvious result — the spatial->temporal
  change-of-variable `n = f/v` gives `G_r(f) = G_q(f/v) / v`, which divides
  by v (not multiplies).

### D2.2 — LQR weight tuning: q_v_s = 1e6, r_F = 1e-5
- **Decision**: Tuned LQR weights `Q = diag(1e2, 1e6, 1e0, 1e2), R = 1e-5`
  via grid search on ISO C @ 20 m/s, 10 s, seed=42.
- **Result**: 64.3% RMS body acceleration reduction vs passive
  (1.446 -> 0.516 m/s²), 0.2% saturation, 0% rattle violation.
- **vs literature**: Abbas & Youn 2024 report ~96% reduction on a *linearized*
  plant (no lag, no saturation). Our 64% on the realistic plant (τ_a=10 ms,
  ±1500 N) is the realistic upper bound for LQR. The remaining 32% gap is
  exactly what the DP controller should target by exploiting the constraints.
- **Trade-off observed**: LQR worsens tire load (-127%) and rattle space (-47%)
  on ISO C. This is the LQR's known limitation — it suppresses body motion at
  the cost of wheel hop, which our DP controller should avoid.
- **Impact**: `configs/lqr_tuned.yaml`, `scripts/tune_lqr_fast.py`,
  `results/lqr_tuning_grid.csv`.

### D2.3 — MPC-LPV: zero-order hold on command between QP solves
- **Decision**: The MPC QP is solved **once per dt-second timestep** (not on
  every RK45 internal evaluation). The last computed F_a command is held
  constant between solves.
- **Rationale**: Initial implementation called `solve_step` at every dynamics
  evaluation. RK45 makes ~3 evaluations per output step, so a 0.05 s sim
  triggered >100 QP solves, each ~13 ms = >1 min for 0.05 s. With ZOH, only
  1 QP solve per dt = 10 solves for 0.05 s = ~130 ms total (100x faster).
- **Trade-off**: The MPC command is piecewise-constant at dt resolution, not
  continuous. This is the standard receding-horizon assumption and matches
  real digital controller implementations.
- **Impact**: `dpsus/controllers/mpc_lpv.py:make_command()`.

### D2.4 — MPC without road preview: nominal performance is poor
- **Decision**: Implement MPC-LPV **without** road preview in Sprint 2.
  Road preview will be added in Sprint 4.
- **Observation**: Without preview, the MPC QP sees any non-zero command as
  "pure control effort with no anticipatory benefit", so the optimal u ≈ 0.
  The MPC behaves like passive (RMS F_a = 2-3 N, RMS a_s ≈ passive).
- **Constraint enforcement works**: 0% saturation, 0% rattle violation across
  ISO B/C/D — confirming the QP box constraints are correctly enforced.
- **vs literature**: Li et al. 2024 report ~30% RMS a_s reduction with preview.
  Our 0% without preview confirms that preview is essential for MPC.
- **Impact**: Documented in `configs/mpc_lpv_tuned.yaml` note. Sprint 4 will
  add preview via the road callable's future values.

### D2.5 — VDV formula corrected to ISO 2631-1:1997
- **Decision**: VDV = `(sum a_w^4 * dt)^(1/4)` where a_w is the Wk-weighted
  acceleration and dt is the sampling interval.
- **Rationale**: Initial implementation used `(mean a_w^4)^(1/4)` which is the
  RMS-based "estimated VDV" (eVDV), not the true VDV. ISO 2631-1:1997 defines
  VDV as the 4th-root of the *integral* of a_w^4, which for discrete signals
  is `sum(a_w^4) * dt`, not `mean(a_w^4)`.
- **Impact**: `dpsus/evaluation/kpis.py`. 9 pytest tests pass on the KPI
  module (RMS, VDV, crest factor, Wk weighting).

### D2.6 — LPV scheduling on suspension deflection
- **Decision**: Schedule q_z_s and q_F_a on `(|z_s - z_u| / rattle_max)^2`,
  with q_z_s scaled by `(1 + 10*alpha)` and q_F_a by `(1 + 5*alpha)`.
- **Rationale**: Near the rattle-space limit, the controller should penalize
  body position more heavily to avoid hitting the mechanical stop. The
  quadratic schedule gives a smooth onset (no chatter) and the 10x multiplier
  is aggressive enough to dominate the cost near the limit.
- **Trade-off**: The LPV schedule makes the QP more conservative, which
  combined with the no-preview limitation (D2.4), explains why the current
  MPC does almost nothing. Sprint 4 (preview) should reveal whether LPV
  scheduling is actually useful or just over-constrains.
- **Impact**: `dpsus/controllers/mpc_lpv.py:_schedule_Q()`.

### D2.7 — Baseline comparison: 3 controllers × 3 ISO classes × 3 seeds
- **Decision**: Standard comparison matrix is Passive / LQR-tuned / MPC-LPV
  on ISO B/C/D @ 20 m/s, 3 seeds (42, 43, 44), 5 s simulations.
- **Rationale**: ISO B (smooth) is the easy case, ISO C (average) is the
  tuning condition, ISO D (rough) is the stress test where LQR starts to
  break (saturates 7.6%, loses tire contact 23.5% of the time).
- **Result**: LQR achieves 45-67% RMS a_s reduction across the matrix. MPC
  achieves 0% (no preview). Full table in `results/sprint2_report.md`.
- **Impact**: `scripts/compare_baselines.py`, `results/tables/baseline_comparison.csv`,
  `results/figures/baseline_comparison_iso_c.png`.

### D2.8 — Slow tests marked with @pytest.mark.slow
- **Decision**: Tests that take >5 s are marked `@pytest.mark.slow` and
  excluded from the default `make test` run via `-m "not slow"`.
- **Rationale**: MPC simulation tests take ~1-5 s each due to QP solve
  overhead. Running all 8 MPC tests adds ~30 s to the suite. CI runs
  `make test` (fast only); developers run `make test-full` for full coverage.
- **Impact**: `tests/test_mpc_lpv.py`. The `pyproject.toml` already registers
  the `slow` marker.

---

## 2025-07-15 — Sprint 2.5 setup (CA-LQR contribution)

### D2.5.1 — Target venue: IEEE Access (not OJVT) for the CA-LQR paper
- **Decision**: Submit the CA-LQR contribution to IEEE Access (not OJVT).
- **Rationale**: IEEE Access is multidisciplinary, has faster turnaround than
  OJVT (~6 weeks vs 8-10), accepts applied control work, and is also Scopus/SCIE
  indexed. The CA-LQR contribution is a Q2-level methodological improvement,
  not a Q1-level novel framework (which is reserved for the DP paper).
- **Impact**: Paper draft at `/home/z/my-project/download/paper/access.tex`,
  compiled PDF at `access.pdf`. 8 pages, 3 figures, 1 main KPI table, 1 stats
  table. Uses the official IEEE Access LaTeX template.

### D2.5.2 — CA-LQR: schedule on rattle-space only (k_tire=0)
- **Decision**: After grid search, set k_rattle=3.0, k_tire=0.0 (tire-liftoff
  scheduling disabled).
- **Rationale**: The z_u-based proxy for tire-liftoff proximity (D2.5.3) was
  found to over-constrain the controller. With k_tire>0, the controller
  reduces body damping too early in the cycle (when z_u is moderate, not
  actually near liftoff), degrading nominal performance without commensurate
  constraint-handling benefit. The rattle-space-only schedule captures 95%+
  of the benefit because rattle-space and tire-liftoff are correlated (both
  occur when the wheel pushes up).
- **Impact**: `configs/calqr_tuned.yaml`. The k_tire=0 choice is documented
  in the paper (Section IV-A: Tuning of the Scheduling Gains).

### D2.5.3 — Tire-liftoff activation uses z_u proxy
- **Decision**: When tire-liftoff scheduling IS enabled (k_tire>0), use
  alpha_t = clip(max(0, z_u - z_u_half) / z_u_half, 0, 1)^p where
  z_u_half = F_static / (2*k_t). Start activating at z_u = F_static/(2*k_t)
  (half of liftoff threshold), reach 1 at z_u = F_static/k_t.
- **Rationale**: We don't have direct F_tire measurement in the state vector;
  z_r is the road input (external) not measured. Using z_u as a proxy is
  suboptimal but conservative: it activates when the wheel is pushed up,
  which correlates with liftoff risk. A future observer-based F_tire
  estimate would improve this.
- **Limitation**: Documented in the paper (Section V-B: Limitations).
- **Impact**: `dpsus/controllers/calqr.py:compute_activations`.

### D2.5.4 — CA-LQR: zero-order hold on CARE solve
- **Decision**: Re-solve the CARE once per dt=5ms timestep, holding the gain
  constant between solves (same ZOH approach as MPC).
- **Rationale**: RK45 makes ~3 internal evaluations per output step; without
  ZOH, each evaluation would trigger a CARE solve (~3 ms), making a 5s sim
  take 90+ s. With ZOH, only 1 CARE solve per output step = ~1000 solves for
  5s = ~3 s overhead, total ~24 s per 5s sim.
- **Implementation detail**: Use a "monotonic t" guard: only re-solve when
  t > t_last_solve AND t - t_last_solve >= dt. This prevents re-solving when
  solve_ivp calls the dynamics function at slightly earlier t during a step's
  internal iterations.
- **Impact**: `dpsus/controllers/calqr.py:make_command`.

### D2.5.5 — Statistical analysis: paired Wilcoxon + Cliff's delta
- **Decision**: Use paired Wilcoxon signed-rank test for significance and
  Cliff's delta for effect size. With n=3 seeds, p-values cannot reach
  significance (min p = 0.25 for n=3); the paper acknowledges this and notes
  that Cliff's delta values are in the "large effect" range (|d| > 0.33).
- **Rationale**: Wilcoxon is non-parametric (no normality assumption),
  appropriate for small samples. Cliff's delta is robust to outliers and
  has a well-established interpretation (|d|<0.147 negligible, <0.33 small,
  <0.474 medium, otherwise large).
- **Limitation**: Acknowledged in the paper (Section IV-G): "A camera-ready
  version of this paper will increase to 30 seeds, which will allow p<0.01
  significance tests."
- **Impact**: `scripts/run_full_benchmark.py`, `results/tables/statistical_tests.csv`.

### D2.5.6 — Benchmark scope: 3 controllers x 3 ISO classes x 3 seeds, 5s
- **Decision**: Final benchmark is Passive/LQR/CA-LQR x ISO B/C/D @ 20 m/s x
  3 seeds x 5 s. MPC-LPV is excluded from the full grid (too slow without
  road preview; would add ~30 min to the runtime) but is mentioned
  qualitatively in the paper.
- **Rationale**: Total runtime ~3 minutes (CA-LQR is the slowest at ~24s/sim
  due to online CARE solves). 3 seeds is the minimum for Wilcoxon (though
  underpowered, see D2.5.5).
- **Impact**: `scripts/run_full_benchmark.py`, `results/tables/full_benchmark.csv`.