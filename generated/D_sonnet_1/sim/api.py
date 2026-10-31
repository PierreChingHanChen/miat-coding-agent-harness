"""ROS 2-like message types shared by the simulator and every controller.

These mirror the topics of the original system:
  /tag_detection  -> TagDetection   (apriltag_ros, camera frame)
  /pallet_dets    -> PalletDetection (YOLACT bbox + depth samples)
  /odom           -> Odom
  /scan           -> Scan            (2D LiDAR)
  /cmd_vel        <- Command
"""
from dataclasses import dataclass, field
from typing import List, Optional
import math

# ---- fixed platform constants (published to every designer) -------------
IMG_W = 640                      # RGB width [px]
HFOV = math.radians(69.0)        # RealSense D435i RGB horizontal FOV
VFOV = math.radians(42.0)
FX = (IMG_W / 2) / math.tan(HFOV / 2)   # focal length [px] ~ 465.9
CTRL_HZ = 10.0                   # controller tick rate
DT = 1.0 / CTRL_HZ
V_MAX = 0.30                     # |v| limit [m/s]
W_MAX = 0.60                     # |w| limit [rad/s]
PALLET_WIDTH = 1.20              # pocket-face width [m]
TAG_ID = 7                       # id of the pocket-centre tag (family 36h11, 10 cm)
CAM_X = 0.28                     # camera optical centre is CAM_X m ahead of base_link on the centre line
DOCK_DIST = 0.85                 # required camera-to-pocket-face distance at docking [m]
                                 # (= base_link 1.13 m from the face when aligned)


@dataclass
class TagDetection:
    id: int
    x: float      # lateral position of tag centre in camera frame [m], + = right
    z: float      # forward distance along optical axis [m]
    yaw: float    # angle from optical axis to the tag's inward normal [rad], + = right;
                  # 0 when the camera faces the tag squarely
    stamp: float  # time of image [s]


@dataclass
class PalletDetection:
    u: float                 # bbox centre column [px] (0..639)
    width_px: float          # bbox width [px]
    score: float             # detector confidence 0..1
    depth_samples: List[float]   # 25 depth readings [m] around bbox centre; 0.0 = invalid pixel
    stamp: float


@dataclass
class Odom:
    x: float      # odometry frame, starts at (0,0,0) each trial [m]
    y: float
    yaw: float    # [rad]
    v: float      # measured linear speed [m/s]
    w: float      # measured angular speed [rad/s]


@dataclass
class Scan:
    angle_min: float          # -pi
    angle_increment: float    # radians between beams (2 deg)
    ranges: List[float]       # metres, inf = no return; index 0 at angle_min, 0 rad = straight ahead
    range_max: float = 10.0


@dataclass
class Observation:
    t: float
    odom: Odom
    tags: List[TagDetection] = field(default_factory=list)
    pallets: List[PalletDetection] = field(default_factory=list)
    scan: Optional[Scan] = None


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    status: str = "RUNNING"   # RUNNING | DOCKED | ABORTED


def clamp_cmd(c: Command) -> Command:
    return Command(max(-V_MAX, min(V_MAX, float(c.v))),
                   max(-W_MAX, min(W_MAX, float(c.w))), c.status)
