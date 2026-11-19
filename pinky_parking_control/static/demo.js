/*
  모의 로봇: 대시보드를 서버 없이 확인할 때만 쓴다 (주소 끝에 ?demo).
  실제 API 와 같은 모양으로 답하고, 로봇은 장애물을 무시하고 목표까지 직선으로 0.18 m/s 로 간다.
  지도, 주차 칸, 경로는 저장소의 설정(factory 지도, slots.yaml, route.yaml, control.yaml)을 옮긴 값이다.
*/
window.PinkyDemo = (() => {
  'use strict';
  if (!new URLSearchParams(location.search).has('demo')) return null;

  const MAP = {"width":82,"height":63,"resolution":0.05,"origin":{"x":-2.033,"y":-1.576},"cells":"uooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooouuuuouooofffffffffooofooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouooffffffofoffffofffffooffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoooofoooofoooooooofooooooooooofffffffffffffffffffoooffooooooooooooffoooooofoooooouuuoffuuoooofoooooffoooofouoooofffffffffffffffffffoooffooooooooooooffooooooooooooouuoofuuufoououfouofooofoooooooofffffffffffffffffffoooffooofooooooooffouuooufufouoouuuouuuufouuoufoooooooouoouofoofffffffffffffffffffoooooooooooffooooooouooouffoouoouuuuuuuuuuuufuouuouuooouuuoouoofffffffffffffffffffooooooofoooffoooooooofoooofoououuuooouufuuuufuuuuouooooouuuuuoofffffffffffffffffffoooooooooooffooooooooooooffooooouoffffffuuuufuuuuuuuuuffffuouoofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffffffuuuuuuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffuuuffffuuuuuuuuuuuoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououuuffffffffffffffffuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouofuuffuuuuffuffffffffffffoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououffffufffffuufuuuuufffufffoooffffffffffffffffffffffffffffffffffffffffffooooffuouoooffffffffffffffffffffffffffooffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuofffffffffffffffffffffffffofooffffffffffffffffffffffffffffffffffffffffffoouuuuuoouuufuuuuuuufuuufuuuuuuuuuuuouooffffffffffffffffffffffffffffffffffffffffffoooffffoouuufuuuuuuufuuufufuuuuuuuuuoooofffffffffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuuuufuuufuuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuuuffuuufffuuffuuuuuoffffffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuuffuuufuuuuuuffufoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuufuuuufuuuffffffoooooofffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuufffuuffffffufffooouoofffffffffffffffffffffffffffffffffffffffffffffffffoouoofufffufffffffuuuuuuuuuuofooooffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffuuuuufuuuuuuuuuoooofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffuufuuuuffuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouooffuuffffffffuuuuuuuuuuuuoooofffffffffffffffffffoooffooooooooooooffooooooooooooouoofffffffffffuufuuuuuuuuuuoooofffffffffffffffffffooofoooofoooooooofooooooooooooooooofffffffffffffffffuuuuufffuoofffffffffffffffffffoofffooofofooooffffoouuuouuuouoouofffffffffffffffffffffffffffoofffffffffffffffffffoooooooooouffoouuuooouuouuufouuuuoofffffffffffffffffffffuuuuuoofffffffffffffffffffoooooooooooofoooooooooooooffoooouoofffffffffffffffffffffffooooofffffffffffffffffffooooooooooooffooofffofoofffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouoffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouuoffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoffouoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooofoouoofuououoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooouououuuouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"};
  const LAYOUT = {
    slots: { slot_10: { x: 0.3, y: -1.1, yaw: 0 }, slot_11: { x: -0.8, y: 1.15, yaw: 3.14159 } },
    routes: { lap: [{ x: -0.8, y: 1.15, yaw: 3.14159 }, { x: 0.3, y: -1.0, yaw: -1.5708 }, { x: 0.5, y: 1.0, yaw: 3.14159 }] },
    exits: { slot_10: { x: 0, y: 0, yaw: 0 }, slot_11: { x: 0.5, y: 1.0, yaw: 3.14159 } },
  };
  const TIMEOUT = { goto: 180, park: 180, unpark: 180, lap: 300 };
  const SPEED = 0.18;
  const now = () => Date.now() / 1000;
  let nextId = 7;

  const robots = {
    pinky1: { name: 'pinky1', domain_id: 11, pose: { x: 0.06, y: -0.31, yaw: -1.32 }, speed: 0, task: null },
    pinky2: { name: 'pinky2', domain_id: 12, pose: { x: 0.5, y: 1.0, yaw: 3.14159 }, speed: 0, task: null },
  };

  function task(robot, action, target, poses, extra = {}) {
    const t = { id: nextId++, robot, action, target, poses, state: 'running', detail: '', started: now(), finished: null,
      waypoint_index: 0, distance_remaining: null, timeout_s: TIMEOUT[action], ...extra };
    return t;
  }

  // 지난 작업 기록 (오늘 앞서 한 작업)
  const t0 = now();
  const log = [
    { id: 6, robot: 'pinky2', action: 'lap', target: 'lap', state: 'succeeded', detail: '', started: t0 - 214.6, finished: t0 - 141.2 },
    { id: 5, robot: 'pinky1', action: 'unpark', target: 'slot_10', state: 'succeeded', detail: '', started: t0 - 262.0, finished: t0 - 244.3 },
    { id: 4, robot: 'pinky2', action: 'goto', target: '(-0.80, 1.15)', state: 'canceled', detail: '정지 명령', started: t0 - 398.1, finished: t0 - 391.7 },
    { id: 3, robot: 'pinky1', action: 'park', target: 'slot_10', state: 'succeeded', detail: '', started: t0 - 520.4, finished: t0 - 501.9 },
    { id: 2, robot: 'pinky2', action: 'park', target: 'slot_11', state: 'failed', detail: 'Nav2 가 작업을 중단했습니다', started: t0 - 655.0, finished: t0 - 602.8 },
    { id: 1, robot: 'pinky1', action: 'goto', target: '(0.30, -1.10)', state: 'succeeded', detail: '', started: t0 - 760.2, finished: t0 - 741.0 },
  ].map((t) => ({ ...t, elapsed_s: +(t.finished - t.started).toFixed(1), poses: [], waypoint_index: 0, waypoint_total: 1, distance_remaining: null, goal: null }));

  // 지금 진행 중: pinky1 이 slot_10 으로 주차하러 가는 중
  robots.pinky1.task = task('pinky1', 'park', 'slot_10', [LAYOUT.slots.slot_10], { started: now() - 6.8 });

  function remaining(r) {
    const t = r.task;
    let d = Math.hypot(t.poses[t.waypoint_index].x - r.pose.x, t.poses[t.waypoint_index].y - r.pose.y);
    for (let i = t.waypoint_index + 1; i < t.poses.length; i += 1) d += Math.hypot(t.poses[i].x - t.poses[i - 1].x, t.poses[i].y - t.poses[i - 1].y);
    return d;
  }

  function finish(r, state, detail = '') {
    const t = r.task;
    if (!t || t.state !== 'running') return false;
    Object.assign(t, { state, detail, finished: now() });
    r.speed = 0;
    log.unshift(view(t));
    return true;
  }

  function step(dt) {
    for (const r of Object.values(robots)) {
      const t = r.task;
      if (!t || t.state !== 'running') continue;
      if (now() - t.started > t.timeout_s) { finish(r, 'canceled', `시간 초과 (${t.timeout_s}s)`); continue; }
      const g = t.poses[t.waypoint_index];
      const dx = g.x - r.pose.x; const dy = g.y - r.pose.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 0.02) {
        if (t.waypoint_index < t.poses.length - 1) { t.waypoint_index += 1; continue; }
        r.pose.yaw = g.yaw;
        finish(r, 'succeeded');
        continue;
      }
      const heading = Math.atan2(dy, dx);
      const turn = Math.atan2(Math.sin(heading - r.pose.yaw), Math.cos(heading - r.pose.yaw));
      r.pose.yaw += Math.max(-2.5 * dt, Math.min(2.5 * dt, turn));
      const move = Math.abs(turn) < 0.6 ? Math.min(dist, SPEED * dt) : 0;
      r.pose.x += Math.cos(r.pose.yaw) * move;
      r.pose.y += Math.sin(r.pose.yaw) * move;
      r.speed = move / dt;
      t.distance_remaining = remaining(r);
    }
  }

  function view(t) {
    const end = t.finished || now();
    return {
      id: t.id, robot: t.robot, action: t.action, target: t.target, state: t.state, detail: t.detail,
      started: t.started, finished: t.finished, elapsed_s: +(end - t.started).toFixed(1),
      goal: t.poses[Math.min(t.waypoint_index, t.poses.length - 1)], poses: t.poses,
      waypoint_index: t.waypoint_index, waypoint_total: t.poses.length,
      distance_remaining: t.distance_remaining == null ? null : +t.distance_remaining.toFixed(3),
    };
  }

  function status(r) {
    return {
      name: r.name, domain_id: r.domain_id, nav2_ready: true, localized: true,
      pose: { ...r.pose }, pose_age_s: 0.1, speed: { linear: +r.speed.toFixed(3), angular: 0 },
      task: r.task ? view(r.task) : null,
    };
  }

  function snapshot() {
    return { time: now(), robots: Object.values(robots).map(status), log: log.slice(0, 20) };
  }

  function fail(code, detail) { const e = new Error(detail); e.status = code; return Promise.reject(e); }

  function start(r, action, target, poses) {
    if (r.task && r.task.state === 'running') return fail(409, `${r.name} 은(는) ${r.task.action} 작업 중입니다`);
    r.task = task(r.name, action, target, poses);
    return Promise.resolve({ accepted: true, task: view(r.task) });
  }

  // ?demo=lock 이면 팀 비밀번호 'demo' 가 걸린 서버처럼 동작한다
  const PASSWORD = new URLSearchParams(location.search).get('demo') === 'lock' ? 'demo' : '';
  const MOVES = ['goto', 'park', 'unpark', 'lap'];

  function api(method, path, body = {}, password = '') {
    if (method === 'GET' && path === '/api/auth') return Promise.resolve({ required: Boolean(PASSWORD) });
    const moving = path === '/api/auth/check' || MOVES.some((m) => path.endsWith(`/${m}`));
    if (method === 'POST' && moving && PASSWORD && password !== PASSWORD) return fail(401, '팀 비밀번호가 틀렸거나 없습니다');
    if (method === 'POST' && path === '/api/auth/check') return Promise.resolve({ ok: true });
    if (method === 'GET' && path === '/api/map') return Promise.resolve(MAP);
    if (method === 'GET' && path === '/api/layout') return Promise.resolve(LAYOUT);
    if (method === 'POST' && path === '/api/stop') {
      const out = {};
      for (const r of Object.values(robots)) out[r.name] = finish(r, 'canceled', '정지 명령');
      return Promise.resolve({ canceled: out });
    }
    const m = path.match(/^\/api\/robots\/([^/]+)\/(\w+)$/);
    if (!m || method !== 'POST') return fail(404, '없는 주소입니다');
    const r = robots[decodeURIComponent(m[1])];
    if (!r) return fail(404, `없는 로봇입니다: ${m[1]}`);
    const act = m[2];
    if (act === 'goto') {
      const pose = { x: +body.x, y: +body.y, yaw: +body.yaw || 0 };
      return start(r, 'goto', `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})`, [pose]);
    }
    if (act === 'park' || act === 'unpark') {
      const table = act === 'park' ? LAYOUT.slots : LAYOUT.exits;
      if (!table[body.slot]) return fail(404, `없는 칸입니다: ${body.slot}`);
      return start(r, act, body.slot, [table[body.slot]]);
    }
    if (act === 'lap') {
      const pts = LAYOUT.routes[body.route];
      if (!pts) return fail(404, `없는 경로입니다: ${body.route}`);
      return start(r, 'lap', body.route, pts);
    }
    if (act === 'cancel') return Promise.resolve({ canceled: finish(r, 'canceled', '취소했습니다') });
    if (act === 'stop') return Promise.resolve({ canceled: finish(r, 'canceled', '정지 명령') });
    return fail(404, '없는 행동입니다');
  }

  let last = performance.now();
  function subscribe(fn) {
    fn(snapshot());
    setInterval(() => {
      const t = performance.now();
      step(Math.min(0.2, (t - last) / 1000));
      last = t;
      fn(snapshot());
    }, 100);
  }

  return { api, subscribe };
})();
