#! /usr/bin/python
# -*- coding: utf-8 -*-
import math
from math import sin, cos, radians, degrees, hypot, atan2

from robot import Robot


class Naochan_Robo(Robot):
    """
    機動重視のストレーフ＋薄ビーム索敵＋線形予測射撃。
    ・短時間の全周スキャン後に薄ビーム狭角スイープへ移行
    ・常時ストレーフ、被弾・衝突で強ジンク＆強制ダッシュ
    ・壁スムージングでマージン維持
    ・線形予測で誤差が小さいときだけ撃つ
    """

    def init(self):
        # 外観
        self.setColor(0, 10, 0)
        self.setGunColor(0, 10, 0)
        self.setRadarColor(0, 0, 0)
        self.setBulletsColor(255, 255, 255)

        size = self.getMapSize()
        self.mapW, self.mapH = float(size.width()), float(size.height())

        # 時間・状態（sensorsで毎フレーム進める）
        self.tick = 0
        self.hit_evasive_until = -1
        self.last_fire_tick = -999
        self.last_hit_tick = -999
        self.last_hit_by = None
        self.consec_hits = 0

        # 強制ダッシュ回避
        self.force_dash_until = -1
        self.force_heading = None

        # 壁衝突回避
        self.wall_hit_heading = None  # 壁にあたった方向
        self.wall_avoid_until = -1    # この時間まで壁方向を避ける

        # レーダー
        self.setRadarField("round")
        self.radarVisible(True)
        self.lockRadar(False)  # レーダーと砲塔は独立
        self.radar_spin_dir = 1
        self.use_gun_lock = False  # T800方式の銃連動モード

        # 敵モデル {id: {x,y,vx,vy,last_seen}}
        self.enemies = {}
        self.target_id = None

        # ---- TUNE パラメータ ----
        self.INIT_SCAN_TICKS = 20       # TUNE: 初動フルスキャン継続tick（延長して確実に検知）
        self.NARROW_SWEEP_HALF = 70     # TUNE: 狭角スイープ半角度(度)（広げて見失い防止）
        self.WIDE_SWEEP_HALF = 160      # TUNE: 再探索時の広めスイープ半角度(度)
        self.RADAR_STEP_DEG = 70        # TUNE: レーダー1回の指示量(度)（大きく振る）
        self.MOVE_STEP = 25             # TUNE: 平常時の移動キュー距離(px)（機動性向上）
        self.JINK_PERIOD = 30           # TUNE: 周期ジンク間隔(tick)
        self.HIT_JINK_TICKS = 50        # TUNE: 被弾後の強ジンク継続tick（延長）
        self.WALL_MARGIN = 60           # TUNE: 絶対に近づかない壁マージン(px) T800方式
        self.WALL_AVOID_TICKS = 40      # TUNE: 壁衝突後の回避継続tick
        self.BAND_INNER = 0.70          # TUNE: 中心からの最小距離比率（70%）
        self.BAND_OUTER = 0.90          # TUNE: 中心からの最大距離比率（90%）
        self.AIM_ERR_STATIONARY = 5.0   # TUNE: 静止目標への高精度エイム(度) T800方式
        self.AIM_ERR_MOVING = 20.0      # TUNE: 移動目標へのエイム許容(度)
        self.STATIONARY_THRESHOLD = 2.0 # TUNE: 静止とみなす速度しきい値(px/tick)
        self.STATIONARY_TICKS = 3       # TUNE: 静止判定に必要なtick数
        self.FIRE_CD_NEAR = 3           # TUNE: 近距離時の最小クールダウン(tick)（連射重視）
        self.POWER_NEAR_DIST = 300.0    # TUNE: 高パワー適用の距離境界(px)
        self.MIN_FIRE_DIST = 15.0       # TUNE: これより近いと撃たない/回避優先
        self.GUN_STEP_MAX = 45          # TUNE: 銃口回転1回の上限(度)（速く向ける）
        self.ORBIT_RATE_DEG = 2.5       # TUNE: ターゲット不在時の周回速度(度/tick)
        self.REACQUIRE_STALE_TICKS = 40 # TUNE: 未観測がこのtickを超えたら広角スイープ（短縮）
        self.HIT_CHAIN_WINDOW = 20      # TUNE: 連続被弾とみなす間隔(tick)（20tickに変更）
        self.FORCE_DASH_TICKS = 35      # TUNE: 強制ダッシュ回避の継続tick（延長）

    # === 毎ループ ===
    def run(self):
        # ターゲット選択
        self._prune_stale_enemies()
        self.target_id = self._choose_target()
        
        # 静止目標検知時はレーダーを銃に連動（T800方式）
        if self.target_id and self._is_target_stationary(self.target_id):
            if not self.use_gun_lock:
                self.lockRadar("gun")
                self.use_gun_lock = True
                self.rPrint("Stationary target detected! Switching to gun-lock mode.")
        else:
            if self.use_gun_lock:
                self.lockRadar(False)
                self.use_gun_lock = False

        # 移動戦略を決定（優先順位: 強制回避 > 壁回避 > 通常移動）
        desired_heading, move_step = self._decide_movement()

        # 移動実行
        self._execute_movement(desired_heading, move_step)

        # レーダー制御
        self._do_radar(desired_heading)

        # 照準・射撃
        if self.target_id is not None:
            self._aim_and_fire(self.target_id)

        self.stop()

    def _decide_movement(self):
        """移動戦略を決定（優先順位ベースでシンプルに）"""
        move_step = self.MOVE_STEP
        
        # 優先度1: 強制ダッシュ回避（連続被弾時）
        if self.tick < self.force_dash_until and self.force_heading is not None:
            return self.force_heading, self.MOVE_STEP * 2.0
        
        # 優先度2: 壁衝突回避
        if self.tick < self.wall_avoid_until and self.wall_hit_heading is not None:
            return (self.wall_hit_heading + 180) % 360, self.MOVE_STEP * 1.5
        
        # 優先度3: 被弾時の回避移動
        if self.tick < self.hit_evasive_until:
            base_heading = self._heading_strafe(self.target_id) if self.target_id else self._heading_no_target()
            return (base_heading + 25) % 360, self.MOVE_STEP * 1.5
        
        # 優先度4: 通常移動（ターゲット追跡 or バンド周回）
        if self.target_id is not None:
            return self._heading_strafe(self.target_id), self.MOVE_STEP
        else:
            return self._heading_no_target(), self.MOVE_STEP
    
    def _execute_movement(self, desired_heading, move_step):
        """移動を実行（壁チェックと移動処理を統合）"""
        # 旋回
        turn_err = self._angle_diff(self.getHeading(), desired_heading)
        self.turn(max(-20, min(20, turn_err)))
        
        # 壁チェック付き移動
        if self._can_move_safely(move_step):
            self.move(move_step)
        else:
            # 壁に近い場合は中心方向へ少し移動
            me = self.getPosition()
            center_ang = degrees(atan2(self.mapH/2 - me.y(), self.mapW/2 - me.x()))
            if abs(self._angle_diff(self.getHeading(), center_ang)) < 90:
                self.move(move_step * 0.5)

    # === レーダー ===
    def _do_radar(self, desired_heading):
        """レーダー制御（銃連動モードと独立モード）"""
        # 銃連動モード時は銃を小刻みに回転（T800方式）
        if self.use_gun_lock:
            # 銃が回るとレーダーも連動する
            return  # gunTurnは_aim_and_fireで処理
        
        # 独立モード：通常のレーダー制御
        if not self.enemies:
            self.radarTurn(self.RADAR_STEP_DEG)
            return
        
        target_angle = self._target_angle() if self.target_id in self.enemies else desired_heading
        err = self._angle_diff(self.getRadarHeading(), target_angle)
        
        is_stale = (self.target_id not in self.enemies or 
                    self.tick - self.enemies[self.target_id]["last_seen"] > self.REACQUIRE_STALE_TICKS)
        sweep = (self.WIDE_SWEEP_HALF if is_stale else self.NARROW_SWEEP_HALF) * 0.3
        
        self.radarTurn(max(-self.RADAR_STEP_DEG, min(self.RADAR_STEP_DEG, err)) + sweep * self.radar_spin_dir)
        
        if abs(err) < 15:
            self.radar_spin_dir *= -1

    # === 射撃 ===
    def _aim_and_fire(self, bot_id):
        """照準と射撃（2段階精度：静止目標は高精度、移動目標は緩和）"""
        info = self.enemies.get(bot_id)
        if not info:
            return
        
        # 静止目標判定
        is_stationary = self._is_target_stationary(bot_id)
        
        # エイム位置決定
        if is_stationary:
            # 静止目標：現在位置を直接狙う（T800方式）
            target_x, target_y = info["x"], info["y"]
        else:
            # 移動目標：線形予測
            pred, dist_pred = self._predict(bot_id)
            if pred is None:
                return
            target_x, target_y = pred
        
        me = self.getPosition()
        dist = hypot(info["x"] - me.x(), info["y"] - me.y())
        
        # エイム角度計算
        aim_angle = degrees(atan2(target_y - me.y(), target_x - me.x())) - 90
        err = self._angle_diff(self.getGunHeading(), aim_angle)
        
        # 銃を回転（銃連動モード時はレーダーも連動）
        gun_turn = max(-self.GUN_STEP_MAX, min(self.GUN_STEP_MAX, err))
        self.gunTurn(gun_turn)

        # 射撃判定：静止目標は高精度、移動目標は緩和
        aim_threshold = self.AIM_ERR_STATIONARY if is_stationary else self.AIM_ERR_MOVING
        
        can_fire = (
            dist > self.MIN_FIRE_DIST and
            abs(err) < aim_threshold and
            self.tick - self.last_fire_tick >= self.FIRE_CD_NEAR
        )
        
        if not can_fire:
            return
        
        # パワー計算
        power = max(1, min(10, int(1000 / dist) + 1))
        if dist < self.POWER_NEAR_DIST:
            power = min(10, power + 2)
        
        self.fire(power)
        self.last_fire_tick = self.tick
        mode = "STATIONARY" if is_stationary else "MOVING"
        self.rPrint(f"Fire[{mode}]! dist={int(dist)}, power={power}, err={int(err)}°")

    # === ターゲット関連 ===
    def _choose_target(self):
        if not self.enemies:
            return None
        my = self.getPosition()
        return min(self.enemies.keys(),
                   key=lambda bid: hypot(self.enemies[bid]["x"] - my.x(),
                                         self.enemies[bid]["y"] - my.y()))

    def _predict(self, bot_id):
        info = self.enemies.get(bot_id)
        if not info:
            return None, None
        me = self.getPosition()
        dx, dy = info["x"] - me.x(), info["y"] - me.y()
        dist = math.hypot(dx, dy)
        t_flight = dist / 10.0  # 弾速は約10px/tick
        pred_x = info["x"] + info["vx"] * t_flight
        pred_y = info["y"] + info["vy"] * t_flight
        return (pred_x, pred_y), dist

    def _target_angle(self):
        if self.target_id not in self.enemies:
            return self.getRadarHeading()
        me = self.getPosition()
        info = self.enemies[self.target_id]
        return degrees(atan2(info["y"] - me.y(), info["x"] - me.x())) - 90
    
    def _is_target_stationary(self, bot_id):
        """ターゲットが静止しているか判定（T800方式）"""
        info = self.enemies.get(bot_id)
        if not info:
            return False
        
        # 速度ベクトルの大きさを計算
        speed = hypot(info["vx"], info["vy"])
        
        # 一定期間観測しており、速度がしきい値以下
        is_observed_long_enough = (self.tick - info["last_seen"] <= self.STATIONARY_TICKS)
        is_slow = speed < self.STATIONARY_THRESHOLD
        
        return is_observed_long_enough and is_slow

    # === 移動関連 ===
    def _heading_no_target(self):
        # 中心から70-90%のバンド内で反時計回りに周回
        cx, cy = self.mapW / 2, self.mapH / 2
        me = self.getPosition()
        
        # 中心からの距離と最大距離を計算
        dist_from_center = hypot(me.x() - cx, me.y() - cy)
        max_radius = min(cx, cy)  # フィールド中心から壁までの最短距離
        
        # 目標バンド（70-90%）
        target_min = max_radius * self.BAND_INNER
        target_max = max_radius * self.BAND_OUTER
        
        # 基本的な周回角度（反時計回り）
        to_center = degrees(atan2(cy - me.y(), cx - me.x()))
        orbit_heading = (to_center + 90 + self.tick * self.ORBIT_RATE_DEG) % 360
        
        # バンド調整：内側すぎたら外へ、外側すぎたら内へ
        if dist_from_center < target_min:
            # 内側すぎる：中心から離れる方向へ
            orbit_heading = (to_center + 180) % 360
        elif dist_from_center > target_max:
            # 外側すぎる：中心へ向かう方向へ
            orbit_heading = to_center
        
        return orbit_heading

    def _heading_strafe(self, bot_id):
        me = self.getPosition()
        info = self.enemies[bot_id]
        ang_to = degrees(atan2(info["y"] - me.y(), info["x"] - me.x()))
        # 距離計算
        dist = hypot(info["x"] - me.x(), info["y"] - me.y())
        # 反時計回りストレーフ（周期的に変化）
        strafe_angle = 90 if (self.tick // 50) % 2 == 0 else -90
        # 距離で調整（近距離ではランダムに動く）
        if dist < 200:
            strafe_angle += (self.tick % 40 - 20)  # ±20°のノイズ
        return (ang_to + strafe_angle) % 360

    def _can_move_safely(self, step):
        """T800方式: 移動前に壁との衝突を事前チェック"""
        me = self.getPosition()
        angle = self.getHeading()
        
        # 移動後の予測位置を計算
        deltaY = step * cos(radians(angle))
        deltaX = -step * sin(radians(angle))
        
        next_x = me.x() + deltaX
        next_y = me.y() + deltaY
        
        # 壁マージン内に入るかチェック（4方向）
        if next_x < self.WALL_MARGIN:
            return False
        if next_x > self.mapW - self.WALL_MARGIN:
            return False
        if next_y < self.WALL_MARGIN:
            return False
        if next_y > self.mapH - self.WALL_MARGIN:
            return False
        
        return True

    # === ユーティリティ ===
    @staticmethod
    def _angle_diff(current, target):
        d = (target - current + 180) % 360 - 180
        return d

    def _prune_stale_enemies(self):
        alive_ids = {r["id"] for r in self.getEnemiesLeft()}
        for bid in list(self.enemies.keys()):
            if bid not in alive_ids:
                del self.enemies[bid]

    # === イベント ===
    def sensors(self):
        # 毎フレーム呼ばれるので時間をここで進める
        self.tick += 1
        # レーダーモード切替（敵を見つけるまでround、その後thin）
        if self.tick > self.INIT_SCAN_TICKS and self.enemies:
            self.setRadarField("thin")
        else:
            # 敵未検知または初動時は全周スキャン
            self.setRadarField("round")
        self._prune_stale_enemies()

    def onTargetSpotted(self, botId, botName, botPos):
        # 速度推定（改善版：複数フレームの平均）
        if botId in self.enemies:
            prev = self.enemies[botId]
            dt = max(1, self.tick - prev["last_seen"])
            
            # 新しい速度を計算
            new_vx = (botPos.x() - prev["x"]) / dt
            new_vy = (botPos.y() - prev["y"]) / dt
            
            # 指数移動平均で滑らかに（ノイズ削減）
            alpha = 0.7  # 新しい値の重み
            vx = alpha * new_vx + (1 - alpha) * prev["vx"]
            vy = alpha * new_vy + (1 - alpha) * prev["vy"]
        else:
            vx = vy = 0.0
            # 初検知時にログ出力
            self.rPrint(f"Target spotted: {botName} (id={botId}) at ({int(botPos.x())}, {int(botPos.y())})")
        
        self.enemies[botId] = {
            "x": botPos.x(),
            "y": botPos.y(),
            "vx": vx,
            "vy": vy,
            "last_seen": self.tick,
        }

    def onHitByBullet(self, bulletBotId, bulletBotName, bulletPower):
        # 連続被弾判定（20tick以内）
        if bulletBotId == self.last_hit_by and self.tick - self.last_hit_tick <= self.HIT_CHAIN_WINDOW:
            self.consec_hits += 1
        else:
            self.consec_hits = 1
        self.last_hit_by = bulletBotId
        self.last_hit_tick = self.tick

        # 強ジンク延長
        self.hit_evasive_until = self.tick + self.HIT_JINK_TICKS

        # 同一敵から2発以上連続（20tick以内）で食らったら強制ダッシュ回避
        if self.consec_hits >= 2:
            self.force_dash_until = self.tick + self.FORCE_DASH_TICKS
            # より激しく方向転換（90-180度のランダム変化）
            turn_angle = 90 + (self.tick % 90)
            self.force_heading = (self.getHeading() + turn_angle) % 360
            self.rPrint(f"Consecutive hits! Evading! hits={self.consec_hits}")

    def onHitWall(self):
        # T800方式で事前回避しているため、通常は発生しない
        # 万が一の場合のみ発動
        self.wall_hit_heading = self.getHeading()
        self.wall_avoid_until = self.tick + self.WALL_AVOID_TICKS
        
        me = self.getPosition()
        center_ang = degrees(atan2(self.mapH/2 - me.y(), self.mapW/2 - me.x()))
        self.turn(self._angle_diff(self.getHeading(), center_ang))
        self.move(-10)  # 少し後退
        self.rPrint(f"CRITICAL: Wall hit despite prediction! Heading={int(self.wall_hit_heading)}")

    def onHitByRobot(self, robotId, robotName):
        self.hit_evasive_until = self.tick + self.HIT_JINK_TICKS
        self.last_hit_tick = self.tick
        self.rPrint(f"ram hit by {robotName}")

    def onRobotHit(self, robotId, robotName):
        self.rPrint(f"rammed {robotName}")

    def onBulletHit(self, botId, bulletId):
        self.rPrint(f"bullet {bulletId} hit bot {botId}")

    def onBulletMiss(self, bulletId):
        pass

    def onRobotDeath(self):
        self._prune_stale_enemies()
