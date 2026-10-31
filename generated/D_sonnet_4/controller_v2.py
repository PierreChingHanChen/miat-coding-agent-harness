"""AGV pallet-pocket docking controller.

Architecture (MIAT style: discrete-event state machine on top of a pallet pose
estimate kept in the odometry frame):

  WAIT0 -> SEARCH -> APPROACH -> VERIFY            (A3: pallet detector, candidates)
  any of the above --tag seen--> SETTLE            (A4: tag fused into pallet pose)
  SETTLE -> STAGE -> ALIGN -> HOLD -> FINAL -> DOCKING   (A5: guide + stop)

The pallet pose (tag position + face orientation) is fused from every tag
detection into the odometry frame, so tag drop-outs are bridged by odometry.
"""
import math
from sim.api import (Command, FX, IMG_W, CAM_X, DOCK_DIST, V_MAX, W_MAX,
                     TAG_ID, PALLET_WIDTH, DT)

BASE_DOCK = DOCK_DIST + CAM_X          # base_link -> face distance when docked
STAGE_DIST = BASE_DOCK + 0.9           # staging point on the centre line (base)
STAND = 2.1                            # stand-off (base -> candidate surface)
R_SWEEP = 0.34                         # swept radius used for obstacle checks
D2R = math.pi / 180.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def robust_depth(samples):
    vals = []
    for d in samples:
        if d is None:
            continue
        try:
            if math.isfinite(d) and 0.3 < d < 9.0:
                vals.append(d)
        except Exception:
            pass
    if len(vals) < 3:
        return None
    vals.sort()
    return vals[int(0.3 * (len(vals) - 1))]


class DockingController:
    def __init__(self):
        self.reset('approach')

    # ------------------------------------------------------------------
    def reset(self, start_stage):
        self.stage0 = 'guidance' if start_stage == 'guidance' else 'approach'
        self.deadline = 111.0 if self.stage0 == 'guidance' else 141.0
        self.t0 = None
        self.state = 'WAIT0' if self.stage0 == 'guidance' else 'SEARCH'
        self.state_t = 0.0
        self.est = None
        self.out_cnt = 0
        self.tag_now = False
        self.last_tag_t = -1e9
        self.n_since = 0
        self.cands = []
        self.black = []
        self.target = None
        self.target_seen = 0.0
        self.search_acc = 0.0
        self.prev_yaw = 0.0
        self.blocked_t = 0.0
        self.turning = False
        self.prev_sgn = 0
        self.retries = 0
        self.exc = 0
        self.stop_cnt = 0
        self.hold_t = 0.0
        self.wander_n = 0
        self.wander_goal = None
        self.wander_t = 0.0
        self.final_t0 = 0.0
        self.pts = []
        self.polar = []
        self.last_close_t = -1e9
        self.inhib_t0 = None

    # ------------------------------------------------------------------
    def step(self, obs):
        try:
            cmd = self._step(obs)
            self.exc = 0
        except Exception:
            self.exc += 1
            if self.exc > 5:
                return Command(0.0, 0.0, 'ABORTED')
            cmd = Command(0.0, 0.0, 'RUNNING')
        if cmd.status != 'RUNNING':
            return Command(0.0, 0.0, cmd.status)
        v = clamp(float(cmd.v), -V_MAX, V_MAX)
        w = clamp(float(cmd.w), -W_MAX, W_MAX)
        # R-10 safety supervisor: forward-motion inhibit
        try:
            t = obs.t
            sc = obs.scan
            if sc is not None:
                for i, r in enumerate(sc.ranges):
                    if r is not None and math.isfinite(r) and r < 0.55:
                        a = sc.angle_min + i * sc.angle_increment
                        if abs(wrap(a)) <= 30.0 * D2R + 1e-9:
                            self.last_close_t = t
                            break
            inhibited = (t - self.last_close_t) < 1.0
            if inhibited:
                if self.inhib_t0 is None:
                    self.inhib_t0 = t
                if t - self.inhib_t0 >= 20.0:
                    return Command(0.0, 0.0, 'ABORTED')
                if v > 0.0:
                    v = 0.0
            else:
                self.inhib_t0 = None
        except Exception:
            v = min(v, 0.0)
        return Command(v, w, 'RUNNING')

    # ------------------------------------------------------------------
    def _step(self, obs):
        t = obs.t
        od = obs.odom
        if self.t0 is None:
            self.t0 = t
            self.state_t = t
            self.prev_yaw = od.yaw
        self._prep_scan(obs.scan)
        self._update_tags(obs)
        if t - self.t0 > self.deadline:
            return self._abort()
        s = self.state
        if self.tag_now and self.est is not None and s in (
                'WAIT0', 'SEARCH', 'APPROACH', 'VERIFY', 'WANDER'):
            self._enter('SETTLE', t)
            self.n_since = 1
            self.target = None
            s = 'SETTLE'
        cmd = getattr(self, '_s_' + s)(obs, t)
        self.prev_yaw = od.yaw
        return cmd

    def _enter(self, state, t):
        self.state = state
        self.state_t = t
        self.n_since = 0
        self.turning = False
        self.stop_cnt = 0
        self.hold_t = 0.0

    @staticmethod
    def _stop():
        return Command(0.0, 0.0)

    @staticmethod
    def _abort():
        return Command(0.0, 0.0, 'ABORTED')

    # ------------------------------------------------------------------ sensing
    def _prep_scan(self, scan):
        self.pts = []
        self.polar = []
        if scan is None:
            return
        for i, r in enumerate(scan.ranges):
            if r is None:
                continue
            try:
                if not math.isfinite(r) or r < 0.05 or r > scan.range_max:
                    continue
            except Exception:
                continue
            a = scan.angle_min + i * scan.angle_increment
            self.polar.append((a, r))
            self.pts.append((r * math.cos(a), r * math.sin(a)))

    def _free(self, rel):
        c, s = math.cos(rel), math.sin(rel)
        best = 10.0
        R = R_SWEEP
        for px, py in self.pts:
            a = px * c + py * s
            if a <= 0.0:
                continue
            l = -px * s + py * c
            if l >= R or l <= -R:
                continue
            f = a - math.sqrt(R * R - l * l)
            if f < best:
                best = f
        return best

    def _range_dir(self, rel, half):
        best = None
        for a, r in self.polar:
            if abs(wrap(a - rel)) <= half:
                if best is None or r < best:
                    best = r
        return best

    def _update_tags(self, obs):
        self.tag_now = False
        best = None
        for tg in obs.tags:
            if tg.id != TAG_ID:
                continue
            if not (0.4 < tg.z < 3.6):
                continue
            if abs(tg.yaw) > 75.0 * D2R or abs(tg.x) > 1.0 * tg.z:
                continue
            if best is None or tg.z < best.z:
                best = tg
        if best is None:
            return
        od = obs.odom
        cyw, syw = math.cos(od.yaw), math.sin(od.yaw)
        camx = od.x + CAM_X * cyw
        camy = od.y + CAM_X * syw
        tx = camx + best.z * cyw + best.x * syw
        ty = camy + best.z * syw - best.x * cyw
        alpha = wrap(od.yaw + math.pi / 2 - best.yaw)
        self.tag_now = True
        self.last_tag_t = obs.t
        e = self.est
        if e is None:
            self.est = {'tx': tx, 'ty': ty, 'a': alpha, 'n': 1}
            self.out_cnt = 0
            self.n_since += 1
            return
        dp = math.hypot(tx - e['tx'], ty - e['ty'])
        da = wrap(alpha - e['a'])
        if e['n'] >= 3 and (dp > 0.7 or abs(da) > 25.0 * D2R):
            self.out_cnt += 1
            if self.out_cnt >= 6:
                self.est = {'tx': tx, 'ty': ty, 'a': alpha, 'n': 1}
                self.out_cnt = 0
            return
        self.out_cnt = 0
        e['n'] += 1
        floor = 0.3 if best.z < 1.2 else (0.2 if best.z < 2.0 else 0.12)
        g = max(1.0 / e['n'], floor)
        e['tx'] += g * (tx - e['tx'])
        e['ty'] += g * (ty - e['ty'])
        e['a'] = wrap(e['a'] + g * da)
        self.n_since += 1

    def _pf(self, od):
        """base pose in pallet frame -> (bx, by, bth, cam_x, cam_y, heading err)."""
        e = self.est
        dx = od.x - e['tx']
        dy = od.y - e['ty']
        ca, sa = math.cos(e['a']), math.sin(e['a'])
        bx = ca * dx + sa * dy
        by = -sa * dx + ca * dy
        bth = wrap(od.yaw - e['a'])
        cxp = bx + CAM_X * math.cos(bth)
        cyp = by + CAM_X * math.sin(bth)
        eth = wrap(bth + math.pi / 2)
        return bx, by, bth, cxp, cyp, eth

    def _stage_point(self):
        e = self.est
        return (e['tx'] - STAGE_DIST * math.sin(e['a']),
                e['ty'] + STAGE_DIST * math.cos(e['a']))

    # ------------------------------------------------------------------ pallet candidates
    def _detect(self, obs):
        out = []
        od = obs.odom
        cyw, syw = math.cos(od.yaw), math.sin(od.yaw)
        camx = od.x + CAM_X * cyw
        camy = od.y + CAM_X * syw
        for pd in obs.pallets:
            if pd.score < 0.40:
                continue
            wpx = max(float(pd.width_px), 1.0)
            Z = robust_depth(pd.depth_samples)
            if Z is None:
                Z = 0.88 * FX * PALLET_WIDTH / wpx
            wapp = wpx * Z / FX
            plaus = 0.45 <= wapp <= 1.4
            xr = (pd.u - IMG_W / 2.0) / FX * Z
            px = camx + Z * cyw + xr * syw
            py = camy + Z * syw - xr * cyw
            bad = False
            for bx, by in self.black:
                if math.hypot(px - bx, py - by) < 1.0:
                    bad = True
                    break
            if bad:
                continue
            out.append({'x': px, 'y': py, 'score': pd.score, 'plaus': plaus})
        return out

    def _add_cand(self, dd):
        for c in self.cands:
            if math.hypot(c['x'] - dd['x'], c['y'] - dd['y']) < 0.8:
                c['n'] += 1
                g = max(1.0 / c['n'], 0.25)
                c['x'] += g * (dd['x'] - c['x'])
                c['y'] += g * (dd['y'] - c['y'])
                c['ssum'] += dd['score']
                c['hi'] += 1 if dd['score'] >= 0.9 else 0
                c['pl'] += 1 if dd['plaus'] else 0
                return
        self.cands.append({'x': dd['x'], 'y': dd['y'], 'n': 1,
                           'ssum': dd['score'],
                           'hi': 1 if dd['score'] >= 0.9 else 0,
                           'pl': 1 if dd['plaus'] else 0})

    def _rank(self, c):
        mean = c['ssum'] / c['n']
        r = mean + 0.12 * min(c['n'], 10) / 10.0
        plaus = c['pl'] >= 0.5 * c['n']
        if not plaus:
            r -= 0.3
        d0 = math.hypot(c['x'], c['y'])
        ang0 = abs(math.atan2(c['y'], c['x']))
        prior = 0.0
        if self.stage0 == 'guidance':
            if 1.9 <= d0 <= 3.1:
                prior += 0.25
            if ang0 < 0.3:
                prior += 0.3
        else:
            if 2.9 <= d0 <= 4.4:
                prior += 0.25
        if c['hi'] > 0:
            r += 0.3
        c['prior'] = prior
        c['plausible'] = plaus
        return r + prior

    def _best_cand(self):
        best, br = None, -1e9
        for c in self.cands:
            r = self._rank(c)
            if r > br:
                br, best = r, c
        return best

    def _reset_search(self):
        self.search_acc = 0.0
        self.cands = []
        self.target = None

    # ------------------------------------------------------------------ states: A3
    def _s_WAIT0(self, obs, t):
        if t - self.state_t >= 1.0:
            self._reset_search()
            self._enter('SEARCH', t)
        return self._stop()

    def _s_SEARCH(self, obs, t):
        od = obs.odom
        for dd in self._detect(obs):
            self._add_cand(dd)
        self.search_acc += abs(wrap(od.yaw - self.prev_yaw))
        best = self._best_cand()
        if best is not None:
            commit = False
            if best['hi'] >= 2 and best['plausible']:
                commit = True
            elif (self.stage0 == 'guidance' and best['prior'] >= 0.5
                  and best['plausible'] and best['n'] >= 2):
                commit = True
            elif self.search_acc >= 2 * math.pi + 0.1:
                commit = True
            if commit:
                self.target = [best['x'], best['y']]
                self.target_seen = t
                self._enter('APPROACH', t)
                return self._stop()
        elif self.search_acc >= 2 * math.pi + 0.1:
            self.wander_n += 1
            if self.wander_n > 3:
                return self._abort()
            self.wander_goal = None
            self._enter('WANDER', t)
            return self._stop()
        return Command(0.0, 0.5)

    def _s_WANDER(self, obs, t):
        od = obs.odom
        if self.wander_goal is None:
            bestf, bk = -1.0, 0
            for k in range(-11, 13):
                rel = k * math.pi / 12.0
                f = min(self._free(rel), 2.5) - 0.01 * abs(k)
                if f > bestf:
                    bestf, bk = f, k
            if bestf < 0.5:
                self.wander_goal = (od.x, od.y)
            else:
                dist = min(0.8, bestf - 0.2)
                hd = od.yaw + bk * math.pi / 12.0
                self.wander_goal = (od.x + dist * math.cos(hd),
                                    od.y + dist * math.sin(hd))
            self.wander_t = t
        gx, gy = self.wander_goal
        d = math.hypot(gx - od.x, gy - od.y)
        if d < 0.1 or t - self.wander_t > 8.0:
            self.wander_goal = None
            self._reset_search()
            self._enter('SEARCH', t)
            return self._stop()
        return self._drive_to(od, gx, gy, d, 0.1, 0.25, t)

    def _s_APPROACH(self, obs, t):
        od = obs.odom
        tx, ty = self.target
        bd, bestd = 1.0, None
        for dd in self._detect(obs):
            dist = math.hypot(dd['x'] - tx, dd['y'] - ty)
            if dist < bd:
                bd, bestd = dist, dd
        if bestd is not None:
            tx += 0.35 * (bestd['x'] - tx)
            ty += 0.35 * (bestd['y'] - ty)
            self.target = [tx, ty]
            self.target_seen = t
        if t - self.target_seen > 2.5:
            self._reset_search()
            self._enter('SEARCH', t)
            return self._stop()
        dx, dy = tx - od.x, ty - od.y
        dist = math.hypot(dx, dy)
        rl = self._range_dir(wrap(math.atan2(dy, dx) - od.yaw), 8.0 * D2R)
        d_goal = dist - STAND
        if d_goal <= 0.05 or (rl is not None and rl <= STAND + 0.05):
            self._enter('VERIFY', t)
            return self._stop()
        gx = od.x + dx / dist * d_goal
        gy = od.y + dy / dist * d_goal
        return self._drive_to(od, gx, gy, d_goal, 0.05, V_MAX, t)

    def _s_VERIFY(self, obs, t):
        od = obs.odom
        tx, ty = self.target
        dx, dy = tx - od.x, ty - od.y
        hdg = math.atan2(dy, dx)
        err = wrap(hdg - od.yaw)
        rl = self._range_dir(err, 8.0 * D2R)
        if rl is not None and rl > STAND + 0.6 and math.hypot(dx, dy) < rl:
            self.target = [od.x + rl * math.cos(hdg), od.y + rl * math.sin(hdg)]
            self.target_seen = t
            self._enter('APPROACH', t)
            return self._stop()
        if t - self.state_t > 3.0:
            self.black.append((tx, ty))
            self._reset_search()
            self._enter('SEARCH', t)
            return self._stop()
        w = clamp(1.5 * err, -0.3, 0.3) if abs(err) > 0.05 else 0.0
        return Command(0.0, w)

    # ------------------------------------------------------------------ states: A4/A5
    def _s_SETTLE(self, obs, t):
        dt = t - self.state_t
        e = self.est
        if (e['n'] >= 4 and dt >= 0.3) or dt >= 2.5:
            if e['n'] >= 2:
                self._enter('STAGE', t)
            else:
                self.est = None
                self._reset_search()
                self._enter('SEARCH', t)
        return self._stop()

    def _s_STAGE(self, obs, t):
        od = obs.odom
        bx, by, bth, cxp, cyp, eth = self._pf(od)
        if (self.retries == 0 and 1.2 <= cyp <= 3.0 and abs(cxp) <= 0.2
                and abs(eth) <= 15.0 * D2R):
            self._enter('HOLD', t)
            return self._stop()
        gx, gy = self._stage_point()
        d = math.hypot(gx - od.x, gy - od.y)
        if d <= 0.08:
            self._enter('ALIGN', t)
            return self._stop()
        return self._drive_to(od, gx, gy, d, 0.08, V_MAX, t)

    def _s_ALIGN(self, obs, t):
        od = obs.odom
        e = self.est
        gx, gy = self._stage_point()
        if math.hypot(gx - od.x, gy - od.y) > 0.4:
            self._enter('STAGE', t)
            return self._stop()
        err = wrap(wrap(e['a'] - math.pi / 2) - od.yaw)
        if abs(err) < 2.0 * D2R:
            self._enter('HOLD', t)
            return self._stop()
        w = clamp(1.8 * err, -W_MAX, W_MAX)
        if abs(w) < 0.12:
            w = 0.12 if err > 0 else -0.12
        return Command(0.0, w)

    def _s_HOLD(self, obs, t):
        dt = t - self.state_t
        if (dt >= 0.8 and self.n_since >= 3) or dt >= 2.5:
            self.final_t0 = t
            self._enter('FINAL', t)
        return self._stop()

    def _recover(self, t):
        self.retries += 1
        if self.retries > 2:
            return self._abort()
        self._enter('STAGE', t)
        return self._stop()

    def _s_FINAL(self, obs, t):
        od = obs.odom
        bx, by, bth, cxp, cyp, eth = self._pf(od)
        if t - self.final_t0 > 50.0:
            return self._recover(t)
        if (cyp < 1.3 and (abs(cxp) > 0.3 or abs(eth) > 35.0 * D2R)) or cyp > 2.8:
            return self._recover(t)
        lat_ok = abs(cxp) <= 0.03 and abs(eth) <= 3.0 * D2R
        delta = clamp(math.atan(1.6 * cxp), -0.38, 0.38)
        h = wrap(eth + delta)
        w = clamp(-1.8 * h, -W_MAX, W_MAX)
        band_hi = DOCK_DIST + 0.06
        band_lo = DOCK_DIST - 0.06
        v = 0.0
        if cyp > band_hi:
            v = min(0.25, 0.6 * (cyp - band_hi) + 0.05)
            v *= max(0.0, 1.0 - abs(h) / (40.0 * D2R))
            self.hold_t = 0.0
        elif cyp >= band_lo:
            if lat_ok and self.est['n'] >= 6:
                if t - self.last_tag_t <= 1.0:
                    self._enter('DOCKING', t)
                    return self._stop()
                v, w = 0.0, 0.0
                self.hold_t += DT
                if self.hold_t > 4.0:
                    return self._recover(t)
            else:
                v = 0.04 * max(0.0, 1.0 - abs(h) / (40.0 * D2R))
                self.hold_t = 0.0
        else:
            return self._recover(t)
        if v > 0.0:
            v = min(v, max(0.0, 0.8 * (self._free(0.0) - 0.2)))
            if v < 0.02:
                self.blocked_t += DT
                if self.blocked_t > 15.0:
                    return self._abort()
            else:
                self.blocked_t = 0.0
        return Command(v, w)

    def _s_DOCKING(self, obs, t):
        od = obs.odom
        bx, by, bth, cxp, cyp, eth = self._pf(od)
        ok = (abs(cxp) <= 0.04 and abs(eth) <= 3.8 * D2R
              and abs(cyp - DOCK_DIST) <= 0.07
              and (t - self.last_tag_t) <= 2.0)
        if not ok:
            self._enter('FINAL', t)
            return self._stop()
        self.stop_cnt += 1
        if ((self.stop_cnt >= 5 and abs(od.v) <= 0.03 and abs(od.w) <= 0.03)
                or (self.stop_cnt >= 12 and abs(od.v) <= 0.08 and abs(od.w) <= 0.08)):
            return Command(0.0, 0.0, 'DOCKED')
        return self._stop()

    # ------------------------------------------------------------------ motion helpers
    def _pick_dir(self, od, hg, need):
        rel_goal = wrap(hg - od.yaw)
        if self._free(rel_goal) >= need:
            self.prev_sgn = 0
            return hg
        best, bc, bs = None, 1e9, 0
        for k in range(1, 10):
            for sgn in (1, -1):
                off = sgn * k * 10.0 * D2R
                if self._free(wrap(rel_goal + off)) >= need:
                    cost = k - (0.6 if sgn == self.prev_sgn else 0.0)
                    if cost < bc:
                        best, bc, bs = hg + off, cost, sgn
        if best is not None:
            self.prev_sgn = bs
        return best

    def _blocked(self, od):
        self.blocked_t += DT
        if self.blocked_t > 15.0:
            return self._abort()
        bf, brel = -1.0, 0.0
        for k in range(-12, 12):
            rel = k * math.pi / 12.0
            f = self._free(rel)
            if f > bf + 1e-9:
                bf, brel = f, rel
        w = 0.0
        if abs(brel) > 0.1:
            w = 0.4 if brel > 0 else -0.4
        return Command(0.0, w)

    def _drive_to(self, od, gx, gy, d, stop, vmax, t):
        hg = math.atan2(gy - od.y, gx - od.x)
        need = min(d, 0.8) + 0.1
        h = self._pick_dir(od, hg, need)
        if h is None:
            self.turning = False
            return self._blocked(od)
        err = wrap(h - od.yaw)
        if self.turning:
            if abs(err) < 0.12:
                self.turning = False
        elif abs(err) > 0.6:
            self.turning = True
        if self.turning:
            self.blocked_t = 0.0
            w = clamp(1.6 * err, -W_MAX, W_MAX)
            if abs(w) < 0.1:
                w = 0.1 if err > 0 else -0.1
            return Command(0.0, w)
        v = min(vmax, 0.6 * d + 0.05) * max(0.25, math.cos(err))
        v = min(v, max(0.0, 0.8 * (self._free(0.0) - 0.2)))
        w = clamp(2.0 * err, -W_MAX, W_MAX)
        if v < 0.02:
            self.blocked_t += DT
            if self.blocked_t > 15.0:
                return self._abort()
        else:
            self.blocked_t = 0.0
        return Command(v, w)
