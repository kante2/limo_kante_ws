from PIL import Image, ImageDraw
import os

# === 캔버스 ===
W, H = 1024, 1024
BG = (0, 0, 0, 255)           # ⚫️ 검정 바탕
FG = (255, 255, 255, 255)     # ⚪️ 흰색 라인

im = Image.new('RGBA', (W, H), BG)
d = ImageDraw.Draw(im)

# === 파라미터 ===
marg = int(0.10 * min(W, H))  # 바깥 여백
lane_w   = 14                 # 라인 두께(px)
lane_gap = 12                 # 라인 간격(px)
n_lanes  = 2                  # 레인(라인) 개수

# 직선 구간 비율(가로/세로). 1.6~2.0이 일반적인 육상트랙 느낌
straight_ratio = 1.8

# === 외곽 트랙 크기 산정 (가로 > 세로) ===
avail_w = W - 2 * marg
avail_h = H - 2 * marg
outer_h = min(avail_h, int(avail_w / straight_ratio))
outer_w = int(outer_h * straight_ratio)

# 중앙 정렬된 외곽 bbox
left  = (W - outer_w) // 2
top   = (H - outer_h) // 2
right = left + outer_w
bottom= top + outer_h

# 코너 반경: 세로의 절반(반원)
radius_outer = outer_h // 2

def draw_stadium_filled(draw: ImageDraw.ImageDraw, bbox, radius, fill):
    """가로 캡슐(반원+직선)을 '채워서' 한 덩어리로 그림."""
    l, t, r, b = map(int, bbox)
    w = r - l
    h = b - t
    if w <= 0 or h <= 0:
        return
    rad = max(0, min(radius, h // 2, w // 2))
    # 가운데 직사각형
    if r - l > 2 * rad:
        draw.rectangle([l + rad, t, r - rad, b], fill=fill)
    # 좌/우 반원
    if rad > 0:
        draw.pieslice([l, t, l + 2 * rad, b], start=90, end=270, fill=fill)            # left
        draw.pieslice([r - 2 * rad, t, r, b], start=270, end=90, fill=fill)            # right
    else:
        # rad==0이면 그냥 사각형 전체
        draw.rectangle([l, t, r, b], fill=fill)

def draw_stadium_ring(draw, bbox, radius, width, color, bg):
    """바깥 채움(색) + 안쪽 채움(배경)으로 '링(외곽선)' 효과."""
    if width <= 0:
        return
    # 바깥(흰색)
    draw_stadium_filled(draw, bbox, radius, color)
    # 안쪽(검정) : bbox와 반경을 width만큼 줄여서 속을 파낸다
    l, t, r, b = bbox
    inner_bbox = (l + width, t + width, r - width, b - width)
    inner_radius = max(0, radius - width)
    draw_stadium_filled(draw, inner_bbox, inner_radius, bg)

# === 레인 라인 그리기 ===
for i in range(n_lanes):
    inset = i * (lane_w + lane_gap)
    l = left   + inset
    t = top    + inset
    r = right  - inset
    b = bottom - inset

    # 현재 라인의 반경(안쪽으로 갈수록 작아짐)
    radius_i = max(2, radius_outer - inset)

    # 흰색 링(라인) 그리기
    draw_stadium_ring(d, (l, t, r, b), radius_i, lane_w, FG, BG)

# === 저장 ===
outdir = "/root/ws/src/limo_ros/limo_gazebo_sim/worlds/blacktrack/materials/textures"
os.makedirs(outdir, exist_ok=True)
png_path = os.path.join(outdir, "blacktrack.png")
im.save(png_path)
print("saved:", png_path)
