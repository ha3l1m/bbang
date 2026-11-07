"""철판(석쇠) 뚜껑 틀 — 기하 계산 (ROS 의존성 없음).

구조 (2026-11-07 사진 기준)
  - 22.5 × 17 cm 석쇠 철판 2장. 아래 판에 붕어빵 틀 2개의 아래 반쪽, 뚜껑 판에 위 반쪽이 붙어 있다.
  - 두 판은 틀 자체의 경첩으로 이어져 책처럼 열고 닫는다. 닫으면 높이 closed_mm(≈35mm).
  - 철판은 판보다 작은 받침 박스 위에 있어서, 가장자리 아래로 그리퍼 손가락이 들어갈 틈이 있다.

좌표 약속 (poses.yaml)
  frame: {type: plate, plate_mm: [170, 225], grip_r_mm: 150, closed_mm: 35, steps: 8, roll_sign: -1, hook_lift_mm: 0}
    plate_mm  = [경첩에 수직(x) 길이, 경첩 방향(y) 길이]
    grip_r_mm = 그리퍼가 잡는 곳 ~ 경첩까지의 x 거리 (자유단 쪽, 170 보다 조금 작게)
  mold_handle = 열린 상태에서 뚜껑 판을 잡는 TCP 좌표. 툴은 수평 -y 방향, 손가락은 위아래(c=90) 로 판 가장자리를 문다.
    경첩(회전축)은 mold_handle 에서 +x 로 grip_r_mm, 높이는 판 면 + closed_mm/2 (틀 경첩 = 닫힌 높이의 가운데).
  뚜껑 판은 경첩 둘레로 180° 넘어가며, 툴(J6, 툴 z축 = 경첩 방향)도 같은 각도만큼 같이 돈다.
"""
import math


def is_plate(frame) -> bool:
    return bool(frame) and frame.get("type") == "plate"


def pivot(grip, frame):
    """경첩(회전축) 위치 (x, z)."""
    return grip[0] + float(frame["grip_r_mm"]), grip[2] + float(frame["closed_mm"]) / 2


def rotate_about_pivot(pt_xz, piv_xz, deg):
    """x–z 평면에서 경첩 둘레로 deg 만큼 넘긴 점. 0° = 열림(로봇 쪽 -x 에 펼침), 180° = 닫힘(+x 쪽에 얹힘)."""
    vx, vz = pt_xz[0] - piv_xz[0], pt_xz[1] - piv_xz[1]
    r = math.radians(deg)
    return (piv_xz[0] + vx * math.cos(r) + vz * math.sin(r),
            piv_xz[1] - vx * math.sin(r) + vz * math.cos(r))


def grip_pose(grip, frame, deg):
    """뚜껑 판을 deg 만큼 넘긴 상태에서 그리퍼(TCP) 좌표. 손목 롤(c)도 같이 돈다."""
    piv = pivot(grip, frame)
    x, z = rotate_about_pivot((grip[0], grip[2]), piv, deg)
    q = list(grip)
    q[0], q[2] = round(x, 4), round(z, 4)
    q[5] = grip[5] + float(frame.get("roll_sign", -1)) * deg
    return q


def stack_pose(grip, frame, deg):
    """닫힌 철판 두 장(묶음)을 뒤집을 때 그리퍼(TCP) 좌표. deg 0 = 닫힌 자리(+x 쪽), 180 = 로봇 쪽(-x)으로 통째로 넘어감.
    잡는 곳 = 닫힌 묶음 가장자리, 두 판 사이 높이(경첩 높이). 아래로 돌면 테이블을 뚫으므로 '위로' 넘기는 방향(-deg)."""
    px, pz = pivot(grip, frame)
    r = float(frame["grip_r_mm"])
    x, z = rotate_about_pivot((px + r, pz), (px, pz), -deg)
    q = list(grip)
    q[0], q[2] = round(x, 4), round(z, 4)
    q[5] = grip[5] + float(frame.get("roll_sign", -1)) * (180.0 - deg)
    return q
