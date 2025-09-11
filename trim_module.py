import time
import math
from simple_bias_kalman import SimpleBiasKalman 


class TrimModule:
    """
    TrimModule: 自适应 trim 学习模块，集成:
      - long-term EMA (trim_est)
      - zero-trigger quick update (when rate approx 0 for a stable window)
      - adaptive learning rate based on short-term variance (confidence)
      - stability window gating (require N frames stable)
      - Kalman bias estimator for additional confidence
      - bumpless application: produce trim_applied (smoothed) and function to compute PID integrator compensation
      - diagnostics dict returned per update

    使用方式:
      - 每帧调用 update(measured_rate, control_preview, manual_active, dt)
      - 从模块读取 trim_applied 用于叠加到最终输出
      - 在 trim_applied 有显著变化时，可调用 module.compute_integrator_compensation(delta_trim, Ki)
        来获得应当对 PID integrator 做的调整量（位置式 PID 假设 u = Kp*e + Ki*I + ...）。
    """

    def __init__(self,
                 # learning / EMA params
                 base_lr=0.003,              # 基本学习率 (per second scale, applied with dt)
                 trim_long_lp_alpha=0.005,   # long-term EMA alpha (for long_lp of rate) (very slow)
                 trim_alpha_slow=1e-4,       # EMA for conservative trim_est slow update
                 zero_rate_threshold=0.5,    # deg/s below which considered "near zero"
                 zero_stable_seconds=1.0,    # need stable window (s) to trigger zero-fast-learning
                 zero_fast_gain=0.05,        # zero-trigger quick mixing gain (0~1)
                 max_fast_updates_per_sec=1.0, # limit zero-trigger rate
                 # confidence/adaptive lr
                 var_ref=1e-4,               # normalization for short-term variance -> confidence
                 min_confidence=0.2,
                 max_confidence=3.0,
                 # short-term statistics
                 short_lp_alpha=0.2,         # short-term LP for variance estimation
                 # kalman bias estimator params
                 use_kalman=True,
                 kf_q=1e-6,
                 kf_r=1e-3,
                 kf_init_p=1e-3,
                 # trim applied smoothing / limits
                 apply_lp_alpha=0.2,         # 低通 alpha(for trim_applied)
                 apply_slew_rate=0.3,        # max change per second for trim_applied (units of output)
                 trim_min=-0.5, trim_max=0.5,
                 # cooldown
                 trim_update_cooldown=1.0,   # s between substantial trim updates
                 # diagnostics / logging
                 enable_diag=False):
        # params
        self.base_lr = base_lr
        self.trim_long_lp_alpha = trim_long_lp_alpha
        self.trim_alpha_slow = trim_alpha_slow
        self.zero_rate_threshold = zero_rate_threshold
        self.zero_stable_seconds = zero_stable_seconds
        self.zero_fast_gain = zero_fast_gain
        self.max_fast_updates_per_sec = max_fast_updates_per_sec

        self.var_ref = var_ref
        self.min_confidence = min_confidence
        self.max_confidence = max_confidence

        self.short_lp_alpha = short_lp_alpha

        self.use_kalman = use_kalman
        if use_kalman:
            self.kf = SimpleBiasKalman(q_bias=kf_q, r_meas=kf_r, init_bias=0.0, init_p=kf_init_p)
        else:
            self.kf = None

        self.apply_lp_alpha = apply_lp_alpha
        self.apply_slew_rate = apply_slew_rate
        self.trim_min = trim_min
        self.trim_max = trim_max

        self.trim_update_cooldown = trim_update_cooldown

        self.enable_diag = enable_diag

        # state
        self.trim_est = 0.0            # main estimate (can be slow EMA updated)
        self.trim_applied = 0.0        # what is actually applied (smoothed + slew limited)
        self.last_trim_time = -999.0

        # short-term stats
        self.short_lp = 0.0
        self.short_lp_sq = 0.0
        self.stable_counter = 0

        # cache
        self.prev_measured_rate = 0.0
        self.prev_control_preview = 0.0
        self.prev_time = None
        self.last_fast_update_time = -999.0

        # diagnostics
        self.last_diag = {}

    # --------------------------
    # helpers
    # --------------------------
    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _lpf_step(self, prev, target, alpha):
        # simple EMA step: prev + alpha*(target-prev)
        return prev + alpha * (target - prev)

    # --------------------------
    # main update (call every frame)
    # measured_rate: measured angular rate (deg/s or chosen units)
    # control_preview: current total control output preview (u_pid + trim_applied) or PID output if you want
    # manual_active: bool whether pilot has manual takeover
    # dt: seconds since last call
    # optional: predicted_rate_from_control if you want to feed into Kalman residual (can be None)
    # --------------------------
    def update(self, measured_rate, control_preview, manual_active, dt, predicted_rate_from_control=None):
        now = time.time()
        if self.prev_time is None:
            self.prev_time = now

        # --- short-term stats for variance ---
        # short_lp, short_lp_sq to estimate variance of measured_rate
        a = self.short_lp_alpha
        self.short_lp = (1 - a) * self.short_lp + a * measured_rate
        self.short_lp_sq = (1 - a) * self.short_lp_sq + a * (measured_rate * measured_rate)
        short_var = max(0.0, self.short_lp_sq - self.short_lp * self.short_lp)

        # --- long-term lowpass of rate (for bias detection) ---
        long_lp = self._lpf_step(getattr(self, "long_lp", 0.0), measured_rate, self.trim_long_lp_alpha)

        # --- Kalman update: use residual z = measured_rate - predicted_rate (if provided) else measured_rate ---
        kf_bias = None
        if self.use_kalman and self.kf is not None:
            self.kf.predict()
            # If you can supply predicted_rate_from_control, use residual; else use measured_rate as z (coarse)
            if predicted_rate_from_control is not None:
                z = measured_rate - predicted_rate_from_control
            else:
                # treat measured_rate as observation of bias+noise when rate should be near 0
                z = measured_rate
            kf_bias = self.kf.update(z)
        else:
            kf_bias = None

        # --- stable window counting for zero-trigger fast learning ---
        near_zero = abs(measured_rate) <= self.zero_rate_threshold
        # also require low short-term variance
        stable_noise = short_var < max(self.var_ref * 10.0, 1e-6)  # tolerate var scale
        if near_zero and stable_noise and (not manual_active):
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        # compute seconds stable
        stable_seconds = self.stable_counter * dt

        # --- adaptive confidence and learning rate ---
        # confidence inversely related to short_var; protect division
        confidence = 1.0 / (1.0 + short_var / max(self.var_ref, 1e-12))
        confidence = self._clamp(confidence, self.min_confidence, self.max_confidence)
        adaptive_lr = self.base_lr * confidence

        # allow faster learning when zero-triggered and stable for enough time
        fast_update_happened = False
        if stable_seconds >= self.zero_stable_seconds:
            # ensure we don't do fast updates too often (rate limit)
            if now - self.last_fast_update_time >= (1.0 / max(1e-6, self.max_fast_updates_per_sec)):
                # perform fast zero-cross update using control_preview as proxy of required bias
                # idea: when rate ~ 0, control_preview should be the trim
                # mix trim_est towards control_preview with zero_fast_gain (conservative)
                z_target = control_preview
                # Option A: mix into trim_est quickly
                self.trim_est = (1 - self.zero_fast_gain) * self.trim_est + self.zero_fast_gain * z_target
                self.last_fast_update_time = now
                fast_update_happened = True

        # --- regular slow EMA update toward current preview (very slow) ---
        # preview = control_preview (or combine with kf_bias if available)
        # We DO NOT overwrite trim_est when fast update occurred; we allow both to blend
        if not fast_update_happened:
            # use slow EMA to asymptotically track long-term bias
            preview = control_preview
            # optionally use kalman bias to bias preview: if kf_bias small-uncertainty, incorporate
            if kf_bias is not None:
                # get kalman uncertainty
                p = self.kf.get_uncertainty()
                # if uncertainty small, trust kalman more
                if p < 1e-2:
                    # mix preview and kalman estimate: measured residual -> bias estimate
                    preview = 0.5 * preview + 0.5 * (control_preview - kf_bias)
            self.trim_est = (1 - self.trim_alpha_slow) * self.trim_est + self.trim_alpha_slow * preview

        # --- long_lp book-keeping ---
        self.long_lp = long_lp

        # --- apply cooldown gating for larger updates ---
        now = time.time()
        if (now - self.last_trim_time) < self.trim_update_cooldown:
            # optionally we could suppress strong trim changes; here we just respect cooldown for analytics
            pass

        # --- compute trim_applied: lowpass + slew limit for bumplessness ---
        # first lowpass target
        lp_target = (1 - self.apply_lp_alpha) * self.trim_applied + self.apply_lp_alpha * self.trim_est
        # slew limit
        max_step = self.apply_slew_rate * dt
        delta = lp_target - self.trim_applied
        if delta > max_step:
            new_applied = self.trim_applied + max_step
        elif delta < -max_step:
            new_applied = self.trim_applied - max_step
        else:
            new_applied = lp_target

        # clamp within trim bounds
        new_applied = self._clamp(new_applied, self.trim_min, self.trim_max)

        # compute delta for potential integrator compensation
        delta_trim = new_applied - self.trim_applied

        # update applied trim
        self.trim_applied = new_applied

        # record lasts
        self.prev_measured_rate = measured_rate
        self.prev_control_preview = control_preview
        self.prev_time = now

        # diagnostics
        diag = {
            "trim_est": self.trim_est,
            "trim_applied": self.trim_applied,
            "short_var": short_var,
            "long_lp": long_lp,
            "confidence": confidence,
            "fast_update": fast_update_happened,
            "stable_seconds": stable_seconds,
            "kf_bias": (self.kf.get() if self.kf is not None else None),
            "kf_P": (self.kf.get_uncertainty() if self.kf is not None else None),
            "delta_trim": delta_trim,
            "manual_active": manual_active
        }
        self.last_diag = diag
        return diag

    # --------------------------
    # integrator compensation for bumpless transfer
    # 当 trim_applied 改变 delta_trim 时，若你用位置式 PID u = Kp*e + Ki*I + Kd*d,
    # 则希望保持 u_pid + trim_applied 连续。近似调整方法：
    #   desired_delta_I = - delta_trim / ( -Ki )   (注意你 Ki 的符号约定)
    # 返回要加到 PID integrator 的 delta_I（caller 负责 clamp）
    # --------------------------
    def compute_integrator_compensation(self, delta_trim, Ki):
        """
        delta_trim: new_trim_applied - old_trim_applied (signed)
        Ki: inner loop Ki (位置式) -- must be non-zero to perform compensation.
        Returns: delta_integrator (to add to PID integrator)
        """
        # derive: u_total = u_pid + trim; u_pid contains -Ki * I (if you used negative sign)
        # Here assume PID uses u = Kp*e + Ki*I + ...  (Ki positive multiplies integrator)
        # To keep u_total continuous we need delta_u_pid = - delta_trim
        # and delta_u_pid = Ki * delta_I  => delta_I = -delta_trim / Ki
        if abs(Ki) < 1e-12:
            return 0.0
        delta_I = - delta_trim / Ki
        return delta_I

    # --------------------------
    # simple setters / getters
    # --------------------------
    def set_trim(self, value):
        self.trim_est = self._clamp(value, self.trim_min, self.trim_max)

    def get_trim_applied(self):
        return self.trim_applied

    def get_trim_est(self):
        return self.trim_est

    def reset(self):
        self.trim_est = 0.0
        self.trim_applied = 0.0
        self.short_lp = 0.0
        self.short_lp_sq = 0.0
        self.stable_counter = 0
        if self.kf is not None:
            self.kf = SimpleBiasKalman(q_bias=self.kf.q, r_meas=self.kf.r, init_bias=0.0, init_p=self.kf.P)
