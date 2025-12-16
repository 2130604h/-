import math
from math import sin, cos, radians, degrees, hypot, atan2
from robot import Robot

FIRE_DISTANCE = 500
BULLET_POWER = 2

class Naochan_Robo_I(Robot):
    """
    中心から壁側70～90％を基準に時計回りで移動。
    2連続被弾（20tick以内）で方向転換。
    初動25tickは全周索敵。
    """

    def init(self):
        self.setColor(0, 10, 0)
        self.setGunColor(0, 10, 0)
        self.setRadarColor(0, 0, 0)
        self.setBulletsColor(255, 255, 255)

        self.radarVisible(True)

        # マップ情報
        size = self.getMapSize()
        self.mapW = float(size.width())
        self.mapH = float(size.height())

        # 時間管理
        self.tick = 0
        
        # 被弾追跡
        self.last_hit_tick = -999
        self.last_hit_by = None
        self.consec_hits = 0
        self.evasion_until = -1
        self.evasion_heading = None
        
        # レーダー
        self.lockRadar("gun")  # 初動は独立
        self.setRadarField("thin")  
        
        # パラメータ
        self.INIT_SCAN_TICKS = 5      # 初動全周索敵期間
        self.BAND_INNER = 0.70         # 中心から70%
        self.BAND_OUTER = 0.90         # 中心から90%
        self.HIT_CHAIN_WINDOW = 20     # 連続被弾判定間隔
        self.EVASION_TICKS = 40        # 回避継続時間
        self.MOVE_STEP = 25            # 移動距離
        self.ORBIT_RATE_DEG = -2.5     # 時計回り（負の値）
        self.WALL_MARGIN = 60          # 壁マージン
        
        # 敵情報
        self.enemies = {}
        self.target_id = None

    def run(self):
        """メインループ：時計回り移動＋索敵＋射撃"""
        # 敵情報更新
        alive_ids = {r["id"] for r in self.getEnemiesLeft()}
        for bid in list(self.enemies.keys()):
            if bid not in alive_ids:
                del self.enemies[bid]
        
        if self.enemies:
            me = self.getPosition()
            self.target_id = min(self.enemies.keys(),
                                key=lambda bid: hypot(self.enemies[bid]["x"] - me.x(),
                                                     self.enemies[bid]["y"] - me.y()))
        else:
            self.target_id = None
        
        # 移動方向決定
        cx, cy = self.mapW / 2, self.mapH / 2
        me = self.getPosition()
        
        dist_from_center = hypot(me.x() - cx, me.y() - cy)
        max_radius = min(cx, cy)
        
        target_min = max_radius * self.BAND_INNER
        target_max = max_radius * self.BAND_OUTER
        
        to_center = degrees(atan2(cy - me.y(), cx - me.x()))
        
        # 時計回り（-90度）
        heading = (to_center - 90 + self.tick * self.ORBIT_RATE_DEG) % 360
        
        # バンド調整
        if dist_from_center < target_min:
            heading = (to_center + 180) % 360
        elif dist_from_center > target_max:
            heading = to_center
        
        # 回避中は回避方向優先
        if self.tick < self.evasion_until and self.evasion_heading is not None:
            heading = self.evasion_heading
        
        # 移動実行
        turn_err = (heading - self.getHeading() + 180) % 360 - 180
        self.turn(max(-20, min(20, turn_err)))
        
        # 壁チェック
        angle = self.getHeading()
        deltaY = self.MOVE_STEP * cos(radians(angle))
        deltaX = -self.MOVE_STEP * sin(radians(angle))
        next_x = me.x() + deltaX
        next_y = me.y() + deltaY
        
        if (self.WALL_MARGIN < next_x < self.mapW - self.WALL_MARGIN and
            self.WALL_MARGIN < next_y < self.mapH - self.WALL_MARGIN):
            self.move(self.MOVE_STEP)
        else:
            center_ang = degrees(atan2(cy - me.y(), cx - me.x()))
            if abs((center_ang - self.getHeading() + 180) % 360 - 180) < 90:
                self.move(self.MOVE_STEP * 0.5)
        
        # レーダー
        if self.tick <= self.INIT_SCAN_TICKS:
            self.radarTurn(70)
        elif not self.enemies:
            self.radarTurn(50)
        
        # 射撃
        if self.target_id and self.target_id in self.enemies:
            info = self.enemies[self.target_id]
            pos = self.getPosition()
            
            dx = info["x"] - pos.x()
            dy = info["y"] - pos.y()
            dist = math.sqrt(dx**2 + dy**2)
            
            my_gun_angle = self.getGunHeading() % 360
            enemy_angle = degrees(atan2(dy, dx)) - 90
            a = enemy_angle - my_gun_angle
            if a < -180:
                a += 360
            elif 180 < a:
                a -= 360
            self.gunTurn(a)
            
            if dist < FIRE_DISTANCE and abs(a) < 15:
                self.fire(BULLET_POWER)
        
        self.stop()
            
    def onHitWall(self):
        pass

    def sensors(self): 
        pass
        
    def onRobotHit(self, robotId, robotName):
        pass
        
    def onHitByRobot(self, robotId, robotName):
        pass

    def onHitByBullet(self, bulletBotId, bulletBotName, bulletPower):
        # 2連続被弾検知
        if self.tick - self.last_hit_tick <= self.HIT_CHAIN_WINDOW and self.last_hit_by == bulletBotId:
            self.consec_hits += 1
            if self.consec_hits >= 2:
                self.evasion_until = self.tick + self.EVASION_TICKS
                self.evasion_heading = (self.getHeading() + 180) % 360
        else:
            self.consec_hits = 1
        
        self.last_hit_tick = self.tick
        self.last_hit_by = bulletBotId
        
    def onBulletHit(self, botId, bulletId):
        pass
        
    def onBulletMiss(self, bulletId):
        pass
        
    def onRobotDeath(self):
        pass
        
    def onTargetSpotted(self, botId, botName, botPos):
        self.enemies[botId] = {"id": botId, "x": botPos.x(), "y": botPos.y()}
