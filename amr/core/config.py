"""Fleet-wide constants. Every robot runs with the same Config; nothing here is shared state."""
from __future__ import annotations

from dataclasses import dataclass, asdict, replace


@dataclass(frozen=True)
class Config:
    # --- robot body / kinematics (differential drive) ---
    radius: float = 0.3
    v_max: float = 1.0          # m/s
    w_max: float = 1.5          # rad/s
    a_max: float = 1.5          # m/s^2 linear accel/brake
    aw_max: float = 6.0         # rad/s^2 angular accel

    # --- loop rates ---
    control_hz: float = 20.0    # ORCA / motion loop
    beacon_hz: float = 10.0     # StateBeacon rate
    lidar_hz: float = 10.0

    # --- lidar ---
    lidar_rays: int = 120       # 3 deg resolution
    lidar_range: float = 8.0
    lidar_noise: float = 0.01

    # --- membership / staleness ---
    stale_s: float = 0.30       # peer silent this long -> unknown obstacle, not live. 300 ms (< the
                                # 500 ms rule) so a dead peer is detected <= 500 ms after it dies with
                                # 5 processes on one laptop: last beacon <= 100 ms before death +
                                # 300 ms + one 50 ms control tick + scheduling jitter (measured spread 120 ms)
    orphan_bundle_s: float = 4.0     # re-auction a lost peer's uncommitted tasks
    orphan_commit_s: float = 12.0    # re-auction a lost peer's committed (in-progress) task
    stale_inflate_rate: float = 1.0  # m of extra radius per second of silence
    stale_inflate_max: float = 1.0

    # --- ORCA / safety ---
    orca_margin: float = 0.1    # added to radius for peers
    orca_tau: float = 2.0
    orca_tau_obst: float = 0.6
    orca_neighbor_dist: float = 3.5
    wall_margin: float = 0.05
    safety_t_react: float = 0.1
    safety_margin: float = 0.08
    safety_half_width_extra: float = 0.05

    # --- contention (L2) ---
    lock_request_dist: float = 4.0   # request a section this far ahead (m, along path)
    hold_dist: float = 1.3           # wait this far before a section boundary without a grant
    entry_stop_dist: float = 0.42    # stop line before boundary when granted but not sensor-clear
    vantage_tol: float = 0.45        # must be this close to the vantage cell centre to sensor-check
    release_clearance: float = 0.42  # release a section once centre is this far outside it
    lease_margin_s: float = 4.0
    clock_error_s: float = 0.05      # padding on every lease/reservation window
    sensor_block_s: float = 3.0      # granted but not sensor-clear this long -> blockage
    grant_timeout_s: float = 6.0     # a grant not used (entered) this long is handed back
    alpha_age: float = 0.2           # priority ageing per second waited
    backoff_jitter_s: tuple = (0.5, 2.0)
    deadlock_check_hz: float = 5.0

    # --- route (L3) ---
    right_hand_penalty: float = 0.3
    wrong_way_penalty: float = 20.0  # soft one-way rule for picking aisles (only used if unavoidable)
    narrow_penalty: float = 0.1
    replan_wait_s: float = 3.0
    use_reservations: bool = True    # L3 look-ahead on peers' planned paths (off only to force demos)
    replan_gain_s: float = 2.0
    blockage_ttl_s: float = 30.0
    stuck_s: float = 8.0

    # --- task (L4) ---
    task_reward: float = 100.0
    discount: float = 0.99      # per second (time-discounted reward, keeps DMG)
    bundle_max: int = 2
    admit_max_queue: int = 2         # don't start a task whose aisle already has this many robots on it
    commit_stable_s: float = 0.4
    dwell_pick_s: float = 2.0
    dwell_drop_s: float = 2.0

    # --- battery ---
    battery_per_m: float = 0.08      # % per metre travelled
    battery_idle_per_s: float = 0.005
    battery_charge_per_s: float = 0.6
    battery_reserve: float = 12.0    # % that must remain after reaching a dock
    battery_full: float = 90.0

    def to_dict(self) -> dict:
        return asdict(self)

    def with_(self, **kw) -> "Config":
        return replace(self, **kw)


DEFAULT = Config()
