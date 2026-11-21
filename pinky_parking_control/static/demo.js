/*
  모의 로봇: 관제센터를 서버 없이 확인할 때만 쓴다 (주소 끝에 ?demo).
    ?demo          경기장 도면 (config/arena.yaml, scenario.yaml 을 옮긴 값)
    ?demo=factory  지금 시뮬레이션용 공장 월드
    ?demo=lock     경기장 + 팀 비밀번호 'demo'
  실제 API 와 같은 모양으로 답한다. 로봇은 장애물을 무시하고 경유점을 따라 0.18 m/s 로 간다.
*/
window.PinkyDemo = (() => {
  'use strict';
  const params = new URLSearchParams(location.search);
  if (!params.has('demo')) return null;
  const mode = params.get('demo');
  const G = window.ArenaGeo;
  const PASSWORD = mode === 'lock' ? 'demo' : '';
  const PI2 = Math.PI / 2;

  const VEHICLES = {
    T00: { length: 0.14, width: 0.13, trailer: 0.0, name: '트레일러 없음' },
    T10: { length: 0.24, width: 0.13, trailer: 0.10, name: '트레일러 10 cm' },
    T15: { length: 0.29, width: 0.13, trailer: 0.15, name: '트레일러 15 cm' },
  };
  const PENDING = {
    gap: '앞뒤 간격 조절은 준비 중입니다 (#12)',
    park_precise: '주차 칸 정밀 정렬은 준비 중입니다 (#7 마커 검출, #8 정밀 정렬)',
  };
  const CALLSIGNS = { A: 'pinky1', B: 'pinky2' };
  const ARENA = {
    kind: 'arena', name: '경기장', board: { x: -1.2, y: -1.0, width: 2.0, height: 2.0 }, straight: 0.6, lane_width: 0.19, tape_width: 0.02,
    tracks: [{ id: 1, name: '1번 트랙 (안쪽)', radius: 0.405, vehicles: ['T00'] }, { id: 2, name: '2번 트랙 (바깥쪽)', radius: 0.595, vehicles: ['T10', 'T15'] }],
    slots: {
      slot_10: { x: 0, y: -0.15, yaw: PI2, width: 0.19, length: 0.3, marker: 10 },
      slot_11: { x: 0, y: 0.15, yaw: PI2, width: 0.19, length: 0.3, marker: 11 },
    },
    exits: { slot_10: { x: 0.405, y: -0.15, yaw: PI2 }, slot_11: { x: 0.405, y: 0.15, yaw: PI2 } },
    fuel: {
      zone: { id: 20, x: -0.995, y: -0.195, width: 0.19, length: 0.35, marker: 20 },
      wait_line: { id: 30, x: -0.995, y: 0.045, width: 0.19, marker: 30 },
      wait: { x: -0.995, y: 0.17, yaw: -PI2 }, enter: { x: -0.995, y: -0.195, yaw: -PI2 }, leave: { x: -0.595, y: -0.28, yaw: -PI2 },
    },
    vehicles: VEHICLES, pending: PENDING, callsigns: CALLSIGNS, teleop: { linear: 0.15, angular: 1.2 },
  };
  const FACTORY = {
    kind: 'factory', name: '공장 월드 (시뮬레이션 임시)',
    slots: { slot_10: { x: 0.3, y: -1.1, yaw: 0 }, slot_11: { x: -0.8, y: 1.15, yaw: 3.14159 } },
    routes: { lap: [{ x: -0.8, y: 1.15, yaw: 3.14159 }, { x: 0.3, y: -1.0, yaw: -1.5708 }, { x: 0.5, y: 1.0, yaw: 3.14159 }] },
    exits: { slot_10: { x: 0, y: 0, yaw: 0 }, slot_11: { x: 0.5, y: 1.0, yaw: 3.14159 } },
    vehicles: VEHICLES, pending: PENDING, callsigns: CALLSIGNS, teleop: { linear: 0.15, angular: 1.2 },
  };
  const MAP = {"width":82,"height":63,"resolution":0.05,"origin":{"x":-2.033,"y":-1.576},"cells":"uooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooouuuuouooofffffffffooofooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffooooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouooffffffofoffffofffffooffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouoooofoooofoooooooofooooooooooofffffffffffffffffffoooffooooooooooooffoooooofoooooouuuoffuuoooofoooooffoooofouoooofffffffffffffffffffoooffooooooooooooffooooooooooooouuoofuuufoououfouofooofoooooooofffffffffffffffffffoooffooofooooooooffouuooufufouoouuuouuuufouuoufoooooooouoouofoofffffffffffffffffffoooooooooooffooooooouooouffoouoouuuuuuuuuuuufuouuouuooouuuoouoofffffffffffffffffffooooooofoooffoooooooofoooofoououuuooouufuuuufuuuuouooooouuuuuoofffffffffffffffffffoooooooooooffooooooooooooffooooouoffffffuuuufuuuuuuuuuffffuouoofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffffffuuuuuuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffuuuffffuuuuuuuuuuuoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououuuffffffffffffffffuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouofuuffuuuuffuffffffffffffoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououffffufffffuufuuuuufffufffoooffffffffffffffffffffffffffffffffffffffffffooooffuouoooffffffffffffffffffffffffffooffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuofffffffffffffffffffffffffofooffffffffffffffffffffffffffffffffffffffffffoouuuuuoouuufuuuuuuufuuufuuuuuuuuuuuouooffffffffffffffffffffffffffffffffffffffffffoooffffoouuufuuuuuuufuuufufuuuuuuuuuoooofffffffffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuuuufuuufuuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouuuuuuuuuuffuuufffuuffuuuuuoffffffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuuffuuufuuuuuuffufoofoofffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuufuuuufuuuffffffoooooofffffffffffffffffffffffffffffffffffffffffffffffffoououuuuuuuufffuuffffffufffooouoofffffffffffffffffffffffffffffffffffffffffffffffffoouoofufffufffffffuuuuuuuuuuofooooffffffffffffffffffffffffffffffffffffffffffffffffoouoofffffffffuuuuufuuuuuuuuuoooofffffffffffffffffffffffffffffffffffffffffffffffffoouoffffffuufuuuuffuuuuuuuuuooooofffffffffffffffffffffffffffffffffffffffffffffffffoouooffuuffffffffuuuuuuuuuuuuoooofffffffffffffffffffoooffooooooooooooffooooooooooooouoofffffffffffuufuuuuuuuuuuoooofffffffffffffffffffooofoooofoooooooofooooooooooooooooofffffffffffffffffuuuuufffuoofffffffffffffffffffoofffooofofooooffffoouuuouuuouoouofffffffffffffffffffffffffffoofffffffffffffffffffoooooooooouffoouuuooouuouuufouuuuoofffffffffffffffffffffuuuuuoofffffffffffffffffffoooooooooooofoooooooooooooffoooouoofffffffffffffffffffffffooooofffffffffffffffffffooooooooooooffooofffofoofffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouoffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoouuoffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouooffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffouoofffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffoffouoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooofoouoofuououoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooouououuuouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"};
  const LAYOUT = mode === 'factory' ? FACTORY : ARENA;
  const TIMEOUT = { goto: 180, park: 180, unpark: 180, lap: 300, drive: 300, lane_change: 120, fuel: 180 };
  const FUEL_WORDS = { wait: '대기선 30', enter: '주유구역 20', leave: '2번 트랙 복귀' };
  const SPEED = 0.18;
  const now = () => Date.now() / 1000;
  let nextId = 7;

  const start = LAYOUT.kind === 'arena'
    ? { pinky1: { x: 0, y: -0.15, yaw: PI2 }, pinky2: { x: 0, y: 0.15, yaw: PI2 } }
    : { pinky1: { x: 0, y: 0, yaw: 0 }, pinky2: { x: 0.5, y: 1.0, yaw: 3.14159 } };
  const robots = {
    pinky1: { name: 'pinky1', domain_id: 11, callsign: 'A', vehicle: 'T00', pose: { ...start.pinky1 }, v: 0, w: 0, task: null, teleop: 0 },
    pinky2: { name: 'pinky2', domain_id: 12, callsign: 'B', vehicle: 'T10', pose: { ...start.pinky2 }, v: 0, w: 0, task: null, teleop: 0 },
  };

  const t0 = now();
  const log = [
    { id: 6, robot: 'pinky2', action: 'park', target: 'slot_11', state: 'succeeded', detail: '', started: t0 - 214.6, finished: t0 - 196.2 },
    { id: 5, robot: 'pinky1', action: 'park', target: 'slot_10', state: 'succeeded', detail: '', started: t0 - 262.0, finished: t0 - 244.3 },
    { id: 4, robot: 'pinky2', action: 'goto', target: '(-0.60, 0.30)', state: 'canceled', detail: '정지 명령', started: t0 - 398.1, finished: t0 - 391.7 },
  ].map((t) => ({ ...t, elapsed_s: +(t.finished - t.started).toFixed(1), poses: [], waypoint_index: 0, waypoint_total: 1, distance_remaining: null, goal: null }));

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

  function finish(r, state, detail = '') {
    const t = r.task;
    if (!t || t.state !== 'running') return false;
    Object.assign(t, { state, detail, finished: now() });
    r.v = 0; r.w = 0;
    log.unshift(view(t));
    return true;
  }

  function remaining(r) {
    const t = r.task;
    let d = Math.hypot(t.poses[t.waypoint_index].x - r.pose.x, t.poses[t.waypoint_index].y - r.pose.y);
    for (let i = t.waypoint_index + 1; i < t.poses.length; i += 1) d += Math.hypot(t.poses[i].x - t.poses[i - 1].x, t.poses[i].y - t.poses[i - 1].y);
    return d;
  }

  function step(dt) {
    const tnow = now();
    for (const r of Object.values(robots)) {
      if (r.teleop) {
        if (tnow - r.teleop > 0.5) { r.teleop = 0; r.v = 0; r.w = 0; continue; }
        r.pose.yaw = G.wrap(r.pose.yaw + r.w * dt);
        r.pose.x += Math.cos(r.pose.yaw) * r.v * dt;
        r.pose.y += Math.sin(r.pose.yaw) * r.v * dt;
        continue;
      }
      const t = r.task;
      if (!t || t.state !== 'running') continue;
      if (tnow - t.started > TIMEOUT[t.action]) { finish(r, 'canceled', `시간 초과 (${TIMEOUT[t.action]}s)`); continue; }
      const g = t.poses[t.waypoint_index];
      const dx = g.x - r.pose.x; const dy = g.y - r.pose.y;
      const dist = Math.hypot(dx, dy);
      const last = t.waypoint_index === t.poses.length - 1;
      if (dist < (last ? 0.015 : 0.05)) {
        if (!last) { t.waypoint_index += 1; continue; }
        r.pose.yaw = g.yaw;
        finish(r, 'succeeded');
        continue;
      }
      const turn = G.wrap(Math.atan2(dy, dx) - r.pose.yaw);
      r.w = Math.max(-2.5, Math.min(2.5, turn * 4));
      r.pose.yaw = G.wrap(r.pose.yaw + r.w * dt);
      r.v = Math.abs(turn) < 0.7 ? Math.min(SPEED, dist / dt) : 0;
      r.pose.x += Math.cos(r.pose.yaw) * r.v * dt;
      r.pose.y += Math.sin(r.pose.yaw) * r.v * dt;
      t.distance_remaining = remaining(r);
    }
  }

  function status(r) {
    return {
      name: r.name, domain_id: r.domain_id, callsign: r.callsign, nav2_ready: true, localized: true,
      pose: { ...r.pose }, pose_age_s: 0.1, speed: { linear: +r.v.toFixed(3), angular: +r.w.toFixed(3) },
      teleop: Boolean(r.teleop), vehicle: { type: r.vehicle, ...VEHICLES[r.vehicle] },
      task: r.task ? view(r.task) : null,
    };
  }

  const err = (code, detail) => { const e = new Error(detail); e.status = code; return e; };

  function startTask(r, action, target, poses) {
    if (r.task && r.task.state === 'running') throw err(409, `${r.name} 은(는) ${r.task.action} 작업 중입니다`);
    r.teleop = 0;
    r.task = { id: nextId++, robot: r.name, action, target, poses, state: 'running', detail: '', started: now(), finished: null, waypoint_index: 0, distance_remaining: null };
    return r.task;
  }

  function arenaOnly() { if (LAYOUT.kind !== 'arena') throw err(400, '경기장 배치에서만 됩니다 (지금은 공장 월드)'); }

  function command(r, act, body) {
    switch (act) {
      case 'goto': { const p = { x: +body.x, y: +body.y, yaw: +body.yaw || 0 }; return startTask(r, 'goto', `(${p.x.toFixed(2)}, ${p.y.toFixed(2)})`, [p]); }
      case 'park': if (!LAYOUT.slots[body.slot]) throw err(404, `없는 주차 칸입니다: ${body.slot}`); return startTask(r, 'park', body.slot, [LAYOUT.slots[body.slot]]);
      case 'unpark': if (!LAYOUT.exits[body.slot]) throw err(404, `없는 출차 위치입니다: ${body.slot}`); return startTask(r, 'unpark', body.slot, [LAYOUT.exits[body.slot]]);
      case 'lap': if (!(LAYOUT.routes || {})[body.route]) throw err(404, `없는 경로입니다: ${body.route}`); return startTask(r, 'lap', body.route, LAYOUT.routes[body.route]);
      case 'drive': arenaOnly(); return startTask(r, 'drive', `${body.track}번 트랙`, G.drive(LAYOUT, Number(body.track), r.pose, body.laps || 1));
      case 'lane_change': arenaOnly(); return startTask(r, 'lane_change', `${body.to}번 트랙으로`, G.laneChange(LAYOUT, Number(body.to), r.pose));
      case 'fuel': arenaOnly(); if (!FUEL_WORDS[body.step]) throw err(422, `주유 단계는 wait, enter, leave 중 하나입니다: ${body.step}`); return startTask(r, 'fuel', FUEL_WORDS[body.step], [LAYOUT.fuel[body.step]]);
      default: throw err(404, '없는 행동입니다');
    }
  }

  // ---- 시연 시나리오 (scenario.yaml 과 같은 단계) --------------------------------
  const STEPS = [
    { title: '주차 구역에서 시작', detail: 'A(트레일러 없음)는 10번, B(10 cm)는 11번 칸에 선다', actions: [
      { robot: 'A', do: 'vehicle', type: 'T00' }, { robot: 'B', do: 'vehicle', type: 'T10' },
      { robot: 'A', do: 'park', slot: 'slot_10' }, { robot: 'B', do: 'park', slot: 'slot_11' }] },
    { title: '1바퀴 직진', detail: 'A는 1번 트랙, B는 2번 트랙을 한 바퀴', actions: [
      { robot: 'A', do: 'unpark', slot: 'slot_10' }, { robot: 'B', do: 'unpark', slot: 'slot_11' },
      { robot: 'A', do: 'drive', track: 1 }, { robot: 'B', do: 'lane_change', to: 2 }, { robot: 'B', do: 'drive', track: 2 }] },
    { title: '2번 트랙 간격 조절 1바퀴', detail: 'A와 B가 같은 2번 트랙에서 앞뒤 간격을 지키며 한 바퀴', pending: '앞뒤 간격 조절 (#12)' },
    { title: 'A 차선 변경 후 주유구역으로', detail: 'A가 2번 트랙으로 옮겨 대기선(30)에 섰다가 주유구역(20)에 들어간다', actions: [
      { robot: 'A', do: 'lane_change', to: 2 }, { robot: 'A', do: 'fuel', step: 'wait' }, { robot: 'A', do: 'fuel', step: 'enter' }] },
    { title: 'B 대기선에서 차례 기다리기', detail: 'A가 주유구역 안에 있는 동안 B는 대기선(30) 앞에 선다', actions: [{ robot: 'B', do: 'fuel', step: 'wait' }] },
    { title: 'A 트레일러 바꾸기', detail: 'A 주유가 끝나면 사람이 A에 15 cm 트레일러를 단다', actions: [
      { robot: 'A', do: 'confirm', text: 'A에 15 cm 트레일러를 달았나요?' }, { robot: 'A', do: 'vehicle', type: 'T15' }, { robot: 'A', do: 'fuel', step: 'leave' }] },
    { title: 'B 주유 후 제자리 주차', detail: 'B가 주유구역에 들어갔다 나와 11번 칸으로', actions: [
      { robot: 'B', do: 'fuel', step: 'enter' }, { robot: 'B', do: 'fuel', step: 'leave' }, { robot: 'B', do: 'park', slot: 'slot_11' }] },
    { title: 'A(15 cm) 2번 트랙 1바퀴', actions: [{ robot: 'A', do: 'drive', track: 2 }] },
    { title: 'A 주차하고 종료', detail: 'A(15 cm)는 10번, B(10 cm)는 11번 칸에서 끝', actions: [{ robot: 'A', do: 'park', slot: 'slot_10' }] },
  ];
  const WORDS = { park: '주차', unpark: '출차', drive: '트랙 주행', lane_change: '차선 변경', fuel: '주유', vehicle: '차종', confirm: '사람 확인' };
  const describe = (a) => {
    const what = a.do === 'drive' ? `${a.track}번 트랙 1바퀴` : a.do === 'lane_change' ? `${a.to}번 트랙으로` : a.do === 'fuel' ? FUEL_WORDS[a.step] : a.slot || a.type || a.text || '';
    return `${a.robot} ${WORDS[a.do] || a.do} ${what}`;
  };
  const sc = { states: STEPS.map(() => 'ready'), notes: STEPS.map(() => ''), running: false, current: null, waiting: null, stop: false, confirm: null, mode: null };
  const sleep = (ms) => new Promise((res) => setTimeout(res, ms));

  async function doAction(a) {
    const r = robots[CALLSIGNS[a.robot]];
    if (a.do === 'vehicle') { r.vehicle = a.type; return ''; }
    if (a.do === 'confirm') {
      sc.waiting = a.text;
      await new Promise((res) => { sc.confirm = res; });
      sc.waiting = null; sc.confirm = null;
      return sc.stop ? '멈춤' : '';
    }
    const t = command(r, a.do, a);
    while (t.state === 'running' && !sc.stop) await sleep(100);
    return t.state === 'succeeded' ? '' : `${describe(a)}: ${t.detail || t.state}`;
  }

  async function runStep(i) {
    const s = STEPS[i];
    if (s.pending) { sc.states[i] = 'skipped'; sc.notes[i] = `건너뜀: ${s.pending} 준비 중`; return true; }
    sc.states[i] = 'running'; sc.notes[i] = '';
    const queues = {};
    for (const a of s.actions) (queues[a.robot] = queues[a.robot] || []).push(a);
    const results = await Promise.all(Object.values(queues).map(async (q) => {
      for (const a of q) {
        if (sc.stop) return '';
        let msg;
        try { msg = await doAction(a); } catch (e) { msg = `${describe(a)}: ${e.message}`; }
        if (msg) return msg;
      }
      return '';
    }));
    if (sc.stop) { sc.states[i] = 'stopped'; sc.notes[i] = '멈춤'; return false; }
    const bad = results.find(Boolean);
    if (bad) { sc.states[i] = 'failed'; sc.notes[i] = bad; return false; }
    sc.states[i] = 'done';
    return true;
  }

  function runScenario(all) {
    const from = sc.states.findIndex((s) => ['ready', 'stopped', 'failed'].includes(s));
    if (from < 0) throw err(422, '남은 단계가 없습니다. 처음부터 하려면 초기화하세요');
    sc.running = true; sc.stop = false; sc.mode = all ? 'all' : 'step';
    (async () => {
      for (let i = from; i < STEPS.length; i += 1) {
        if (sc.stop) break;
        sc.current = i;
        const ok = await runStep(i);
        if (!ok || !all) break;
      }
      sc.running = false; sc.current = null; sc.waiting = null;
    })();
  }

  function scenarioStatus() {
    return {
      running: sc.running, mode: sc.mode, current: sc.current, waiting: sc.waiting, callsigns: CALLSIGNS,
      steps: STEPS.map((s, i) => ({ index: i, title: s.title, detail: s.detail || '', pending: s.pending || null, state: sc.states[i], note: sc.notes[i], actions: (s.actions || []).map(describe) })),
    };
  }

  function snapshot() {
    const snap = { time: now(), layout: LAYOUT.kind, robots: Object.values(robots).map(status), log: log.slice(0, 20) };
    if (LAYOUT.kind === 'arena') snap.scenario = scenarioStatus();
    return snap;
  }

  const MOVES = ['goto', 'park', 'unpark', 'lap', 'drive', 'lane_change', 'fuel', 'teleop', 'vehicle', 'gap', 'precise'];

  async function api(method, path, body = {}, password = '') {
    body = body || {};
    if (method === 'GET' && path === '/api/auth') return { required: Boolean(PASSWORD) };
    if (method === 'GET' && path === '/api/layout') return LAYOUT;
    if (method === 'GET' && path === '/api/map') { if (LAYOUT.kind === 'arena') throw err(404, '경기장은 도면으로 그립니다'); return MAP; }
    const scenarioMove = path.startsWith('/api/scenario/') && !path.endsWith('/stop');
    const moving = path === '/api/auth/check' || scenarioMove || MOVES.some((m) => path.endsWith(`/${m}`));
    if ((method === 'POST' || method === 'PUT') && moving && PASSWORD && password !== PASSWORD) throw err(401, '팀 비밀번호가 틀렸거나 없습니다');
    if (path === '/api/auth/check') return { ok: true };
    if (path === '/api/stop') {
      sc.stop = true; if (sc.confirm) sc.confirm();
      const out = {};
      for (const r of Object.values(robots)) { r.teleop = 0; r.v = 0; r.w = 0; out[r.name] = finish(r, 'canceled', '정지 명령'); }
      return { canceled: out };
    }
    if (path.startsWith('/api/scenario')) {
      if (LAYOUT.kind !== 'arena') throw err(404, '시나리오는 경기장 배치에서만 있습니다');
      const act = path.split('/').pop();
      if (act === 'next' || act === 'run') { if (sc.running) throw err(409, '시나리오가 이미 실행 중입니다'); runScenario(act === 'run'); }
      else if (act === 'stop') { sc.stop = true; if (sc.confirm) sc.confirm(); for (const r of Object.values(robots)) finish(r, 'canceled', '시나리오 멈춤'); }
      else if (act === 'confirm') { if (!sc.confirm) throw err(422, '확인을 기다리는 단계가 없습니다'); sc.confirm(); }
      else if (act === 'reset') { if (sc.running) throw err(409, '실행 중에는 초기화할 수 없습니다'); sc.states = STEPS.map(() => 'ready'); sc.notes = STEPS.map(() => ''); }
      return scenarioStatus();
    }
    const m = path.match(/^\/api\/robots\/([^/]+)\/([\w/]+)$/);
    if (!m) throw err(404, '없는 주소입니다');
    const r = robots[decodeURIComponent(m[1])];
    if (!r) throw err(404, `없는 로봇입니다: ${m[1]}`);
    const act = m[2];
    if (act === 'cancel') { r.teleop = 0; return { canceled: finish(r, 'canceled', '취소했습니다') }; }
    if (act === 'stop') { r.teleop = 0; r.v = 0; r.w = 0; return { canceled: finish(r, 'canceled', '정지 명령') }; }
    if (act === 'gap') throw err(501, PENDING.gap);
    if (act === 'park/precise') throw err(501, PENDING.park_precise);
    if (act === 'vehicle') { if (!VEHICLES[body.type]) throw err(404, `없는 차종입니다: ${body.type}`); r.vehicle = body.type; return { type: body.type, ...VEHICLES[body.type] }; }
    if (act === 'teleop') {
      if (r.task && r.task.state === 'running') throw err(409, `${r.name} 은(는) ${r.task.action} 작업 중입니다. 먼저 취소하세요`);
      r.v = Math.max(-0.15, Math.min(0.15, +body.linear || 0)); r.w = Math.max(-1.2, Math.min(1.2, +body.angular || 0)); r.teleop = now();
      return { linear: r.v, angular: r.w };
    }
    const t = command(r, act, body);
    return { accepted: true, task: view(t) };
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
