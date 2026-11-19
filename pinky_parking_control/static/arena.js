/*
  경기장 도면 계산 (layout.py 와 같은 규칙). 대시보드 그리기와 데모 모의 로봇이 함께 쓴다.
  트랙 = 직선 + 반원 두 개, 반시계 방향. s = 0 은 오른쪽 직선 아래 끝.
*/
window.ArenaGeo = (() => {
  'use strict';
  const TAU = Math.PI * 2;
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const PER_LAP = 12;

  class Stadium {
    constructor(radius, straight) {
      this.r = radius; this.h = straight / 2;
      this.length = 4 * this.h + TAU * this.r;
    }

    point(s) {
      const { r, h } = this;
      s = ((s % this.length) + this.length) % this.length;
      if (s < 2 * h) return [r, -h + s, Math.PI / 2];
      s -= 2 * h;
      if (s < Math.PI * r) { const a = s / r; return [r * Math.cos(a), h + r * Math.sin(a), wrap(a + Math.PI / 2)]; }
      s -= Math.PI * r;
      if (s < 2 * h) return [-r, h - s, -Math.PI / 2];
      s -= 2 * h;
      const a = Math.PI + s / r;
      return [r * Math.cos(a), -h + r * Math.sin(a), wrap(a + Math.PI / 2)];
    }

    pose(s) { const [x, y, yaw] = this.point(s); return { x, y, yaw }; }

    nearest(x, y, samples = 360) {
      let best = [Infinity, 0];
      for (let i = 0; i < samples; i += 1) {
        const s = (this.length * i) / samples;
        const [px, py] = this.point(s);
        const d = Math.hypot(px - x, py - y);
        if (d < best[0]) best = [d, s];
      }
      return { s: best[1], d: best[0] };
    }
  }

  function tracks(layout) {
    const out = {};
    for (const t of layout.tracks || []) out[t.id] = new Stadium(t.radius, layout.straight);
    return out;
  }

  function nearestTrack(layout, pose) {
    const ts = tracks(layout);
    let best = null;
    for (const id of Object.keys(ts)) {
      const d = ts[id].nearest(pose.x, pose.y).d;
      if (!best || d < best.d) best = { id: Number(id), d };
    }
    return best;
  }

  function drive(layout, trackId, pose, laps = 1) {
    const t = tracks(layout)[trackId];
    if (!t) throw new Error(`없는 트랙입니다: ${trackId}`);
    const s0 = pose ? t.nearest(pose.x, pose.y).s : 0;
    const out = [];
    for (let i = 1; i <= PER_LAP * laps; i += 1) out.push(t.pose(s0 + (t.length * i) / PER_LAP));
    return out;
  }

  function laneChange(layout, to, pose) {
    const ts = tracks(layout);
    const target = ts[to];
    if (!target) throw new Error(`없는 트랙입니다: ${to}`);
    const cur = ts[nearestTrack(layout, pose).id];
    const f = cur.nearest(pose.x, pose.y).s / cur.length;
    return [0.06, 0.12].map((a) => target.pose((f + a) * target.length));
  }

  return { Stadium, tracks, nearestTrack, drive, laneChange, wrap };
})();
