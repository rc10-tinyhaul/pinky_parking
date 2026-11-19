/*
  Pinky 관제센터 (게임 조종석 방식).
  - 유닛(로봇)을 숫자키·카드·지도에서 고르고, 명령 막대의 버튼이나 단축키로 행동을 지시한다.
  - 경기장을 우클릭하면 그 자리로 이동, 주차 칸이면 주차, 주유구역이면 주유 진입.
  - 직접 운전(M): WASD·방향키·조이스틱, 손을 떼면 선다.
  상태는 WebSocket(/api/ws)으로 초당 10번 받는다. ?demo 를 붙이면 demo.js 의 모의 로봇으로 돈다.
*/
(() => {
  'use strict';

  const $ = (sel, el = document) => el.querySelector(sel);
  const $$ = (sel, el = document) => [...el.querySelectorAll(sel)];
  const G = window.ArenaGeo;
  const ACTION = { goto: '이동', park: '주차', unpark: '출차', lap: '경로 주행', drive: '트랙 주행', lane_change: '차선 변경', fuel: '주유' };
  const RUNNING = { goto: '이동 중', park: '주차 중', unpark: '출차 중', lap: '주행 중', drive: '주행 중', lane_change: '차선 변경 중', fuel: '주유 이동 중' };
  const RESULT = { running: '진행 중', succeeded: '완료', failed: '실패', canceled: '취소됨' };
  const STEP_MARK = { running: '▸', done: '✓', skipped: '–', failed: '✕', stopped: '■' };
  const TRAIL_POINTS = 150;
  const TELEOP = { linear: 0.15, angular: 1.2 };

  const demo = new URLSearchParams(location.search).has('demo') ? window.PinkyDemo : null;
  const state = {
    layout: null, map: null, robots: [], log: [], scenario: null, selected: null,
    connected: false, lastRecv: 0, trails: {}, units: {}, startDist: {}, hover: null,
    auth: { required: false, password: '' },
    drive: { on: false, keys: new Set(), joy: null, lin: 0, ang: 0, sending: false, busy: false },
  };
  const PASSWORD_KEY = 'pinky-control-password';
  try { state.auth.password = sessionStorage.getItem(PASSWORD_KEY) || ''; } catch (e) { /* 매번 입력 */ }
  const locked = () => state.auth.required && !state.auth.password;
  const isArena = () => state.layout && state.layout.kind === 'arena';

  // ---- 서식 -----------------------------------------------------------------
  const minus = (s) => s.replace('-', '−');
  const fixed = (v, d = 2) => (v == null || !isFinite(v) ? '—' : minus(Number(v).toFixed(d)));
  const deg = (rad) => (rad == null ? '—' : minus(String(Math.round((rad * 180) / Math.PI))));
  const pad2 = (n) => String(n).padStart(2, '0');
  const hms = (d) => `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
  const clock = (t) => (t ? hms(new Date(t * 1000)) : '—');
  const seconds = (s) => (s == null ? '—' : s < 60 ? `${s.toFixed(1)} s` : `${Math.floor(s / 60)}분 ${Math.round(s % 60)}초`);
  const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const busy = (r) => r && r.task && r.task.state === 'running';
  const robotByName = (name) => state.robots.find((r) => r.name === name);
  const sel = () => robotByName(state.selected);
  const sign = (r) => (r && r.callsign) || (r && r.name) || '?';
  const unitColor = (r) => {
    const i = state.robots.indexOf(r);
    const key = sign(r) === 'B' || (sign(r) !== 'A' && i === 1) ? '--unit-b' : '--unit-a';
    return css(key);
  };

  // ---- 통신 -----------------------------------------------------------------
  async function api(method, path, body, password = state.auth.password) {
    let data;
    let status = 200;
    if (demo) {
      try { data = await demo.api(method, path, body, password); } catch (error) { status = error.status || 500; data = { detail: error.message }; }
    } else {
      const headers = body ? { 'Content-Type': 'application/json' } : {};
      if (password) headers['X-Control-Password'] = password;
      const res = await fetch(path, { method, headers, body: body ? JSON.stringify(body) : undefined });
      status = res.status;
      data = await res.json().catch(() => ({}));
    }
    if (status === 401) setPassword('');
    if (status >= 400) throw new Error(data.detail || `요청 실패 (${status})`);
    return data;
  }

  function connect() {
    if (demo) { state.connected = true; demo.subscribe(onSnapshot); return; }
    const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/ws`);
    ws.onopen = () => { state.connected = true; renderFresh(); };
    ws.onmessage = (event) => onSnapshot(JSON.parse(event.data));
    ws.onclose = () => { state.connected = false; renderAll(); setTimeout(connect, 2000); };
  }

  function onSnapshot(snap) {
    state.robots = snap.robots || [];
    state.log = snap.log || [];
    state.scenario = snap.scenario || null;
    state.lastRecv = Date.now();
    for (const r of state.robots) {
      if (!r.pose) continue;
      const trail = state.trails[r.name] || (state.trails[r.name] = []);
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(last.x - r.pose.x, last.y - r.pose.y) > 0.005) trail.push({ x: r.pose.x, y: r.pose.y });
      if (trail.length > TRAIL_POINTS) trail.shift();
      if (busy(r) && r.task.distance_remaining != null && state.startDist[r.task.id] == null) {
        state.startDist[r.task.id] = Math.max(r.task.distance_remaining, 0.01);
      }
    }
    if (!state.selected && state.robots.length) state.selected = state.robots[0].name;
    renderAll();
  }

  function setPassword(value) {
    state.auth.password = value;
    try { if (value) sessionStorage.setItem(PASSWORD_KEY, value); else sessionStorage.removeItem(PASSWORD_KEY); } catch (e) { /* 무시 */ }
    renderAll();
  }

  // ---- 명령 -----------------------------------------------------------------
  let toastTimer = 0;
  function toast(text, tone = '') {
    const el = $('#cmd-toast');
    el.textContent = text;
    el.dataset.tone = tone;
    el.setAttribute('data-show', '');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.removeAttribute('data-show'), 2600);
  }

  async function order(r, path, body, label, method = 'POST') {
    if (!r) return;
    try {
      await api(method, `/api/robots/${encodeURIComponent(r.name)}/${path}`, body);
      toast(`${sign(r)} · ${label}`);
    } catch (error) {
      toast(`${sign(r)} · ${error.message}`, 'negative');
    }
  }

  function nearestSlot(r) {
    const slots = Object.entries(state.layout ? state.layout.slots : {});
    if (!slots.length) return '';
    if (!r || !r.pose) return slots[0][0];
    slots.sort((a, b) => Math.hypot(a[1].x - r.pose.x, a[1].y - r.pose.y) - Math.hypot(b[1].x - r.pose.x, b[1].y - r.pose.y));
    return slots[0][0];
  }

  const COMMANDS = {
    drive1: (r) => order(r, 'drive', { track: 1 }, '1번 트랙 1바퀴'),
    drive2: (r) => order(r, 'drive', { track: 2 }, '2번 트랙 1바퀴'),
    lane: (r) => {
      const cur = r && r.pose ? G.nearestTrack(state.layout, r.pose).id : 1;
      const to = cur === 1 ? 2 : 1;
      return order(r, 'lane_change', { to }, `${to}번 트랙으로 차선 변경`);
    },
    park: (r) => { const slot = $('#slot-pick').value; return order(r, 'park', { slot }, `${slot} 주차`); },
    unpark: (r) => { const slot = nearestSlot(r); return order(r, 'unpark', { slot }, `${slot}에서 출차`); },
    fuel_wait: (r) => order(r, 'fuel', { step: 'wait' }, '대기선 30으로'),
    fuel_enter: (r) => order(r, 'fuel', { step: 'enter' }, '주유구역 20 진입'),
    fuel_leave: (r) => order(r, 'fuel', { step: 'leave' }, '2번 트랙으로 복귀'),
    cancel: (r) => order(r, 'cancel', null, '작업 취소'),
    stop: (r) => order(r, 'stop', null, '정지'),
  };
  const ARENA_ONLY = new Set(['drive1', 'drive2', 'lane', 'fuel_wait', 'fuel_enter', 'fuel_leave']);
  const KEYS = { q: 'drive1', e: 'drive2', c: 'lane', p: 'park', u: 'unpark', f: 'fuel_wait', g: 'fuel_enter', h: 'fuel_leave', x: 'cancel' };

  function why(r, cmd) {
    if (cmd === 'stop') return state.connected ? '' : '서버와 연결이 끊겼습니다';
    if (cmd === 'cancel') return busy(r) ? '' : '진행 중인 작업이 없습니다';
    if (locked()) return '팀 비밀번호를 먼저 넣으세요';
    if (!state.connected) return '서버와 연결이 끊겼습니다';
    if (!r) return '유닛을 고르세요';
    if (ARENA_ONLY.has(cmd) && !isArena()) return '경기장 배치에서만 됩니다';
    if (state.drive.on) return '직접 운전 중입니다 (M으로 끄기)';
    if (!r.nav2_ready) return 'Nav2가 아직 켜지지 않았습니다';
    if (busy(r)) return '작업 중입니다. 먼저 취소하세요 (X)';
    if ((cmd === 'lane' || cmd === 'drive1' || cmd === 'drive2') && !r.pose) return '로봇 위치를 아직 모릅니다';
    return '';
  }

  function runCommand(cmd) {
    const r = sel();
    const reason = why(r, cmd);
    const button = $(`.cmd[data-cmd="${cmd}"]`);
    if (reason) { toast(reason, 'negative'); return; }
    if (button) { button.setAttribute('data-flash', ''); setTimeout(() => button.removeAttribute('data-flash'), 220); }
    COMMANDS[cmd](r);
  }

  // ---- 위 HUD -----------------------------------------------------------------
  function taskRest(t) {
    if (t.action === 'drive' || t.action === 'lap' || t.waypoint_total > 1) {
      return `경유점 ${Math.min(t.waypoint_index + 1, t.waypoint_total)}/${t.waypoint_total}`;
    }
    return t.distance_remaining == null ? '출발 중' : `${fixed(t.distance_remaining)} m 남음`;
  }

  function renderFinding() {
    const el = $('#finding');
    if (!state.connected && !demo) { el.textContent = '관제 서버와 연결이 끊겼습니다. 2초마다 다시 연결합니다.'; return; }
    if (state.scenario && state.scenario.waiting) { el.innerHTML = `<b>시나리오 대기</b> · ${esc(state.scenario.waiting)}`; return; }
    if (!state.robots.length) { el.textContent = '로봇 상태를 받는 중입니다.'; return; }
    const running = state.robots.filter(busy);
    if (running.length) {
      el.innerHTML = running.map((r) => `<b>${esc(sign(r))}</b> ${esc(r.task.target)} ${RUNNING[r.task.action] || ''}, ${taskRest(r.task)}`).join(' · ');
      return;
    }
    const driving = state.robots.filter((r) => r.teleop);
    if (driving.length) { el.innerHTML = `<b>${esc(sign(driving[0]))}</b> 직접 운전 중`; return; }
    const notReady = state.robots.filter((r) => !r.nav2_ready).map(sign);
    if (notReady.length) { el.textContent = `${notReady.join(', ')}의 Nav2가 아직 켜지지 않았습니다.`; return; }
    const failed = state.log.find((t) => t.state === 'failed' && Date.now() / 1000 - t.finished < 600);
    if (failed) { el.innerHTML = `<b>${esc(failed.robot)}</b> ${esc(failed.target)} ${ACTION[failed.action] || ''} 실패: ${esc(failed.detail)} (${clock(failed.finished)})`; return; }
    el.textContent = `유닛 ${state.robots.length}대 모두 대기 중입니다. 유닛을 고르고 명령하세요.`;
  }

  function renderFresh() {
    const el = $('#fresh');
    const text = $('#fresh-text');
    if (demo) { el.dataset.state = 'live'; text.textContent = '데모 · 모의 로봇'; return; }
    if (state.connected && state.lastRecv) { el.dataset.state = 'live'; text.textContent = `실시간 · ${hms(new Date(state.lastRecv))}`; } else if (state.lastRecv) { el.dataset.state = 'lost'; text.textContent = `끊김 · ${hms(new Date(state.lastRecv))} 이후`; } else { el.dataset.state = 'wait'; text.textContent = '연결 중'; }
  }

  function renderLock() {
    $('#unlock').hidden = !locked();
    $('#lock').hidden = !(state.auth.required && state.auth.password);
  }

  // ---- 유닛 카드 --------------------------------------------------------------
  function buildUnits() {
    const host = $('#units');
    const names = state.robots.map((r) => r.name).join();
    if (host.dataset.names === names) return;
    host.dataset.names = names;
    host.textContent = '';
    state.units = {};
    state.robots.forEach((r, i) => {
      const node = $('#unit-card').content.firstElementChild.cloneNode(true);
      const q = (k) => node.querySelector(`[data-r="${k}"]`);
      q('callsign').textContent = sign(r);
      q('callsign').style.background = unitColor(r);
      q('name').textContent = r.name;
      q('domain').textContent = `도메인 ${r.domain_id}`;
      q('hotkey').textContent = String(i + 1);
      q('select').addEventListener('click', () => select(r.name));
      const types = Object.keys((state.layout && state.layout.vehicles) || {});
      for (const t of types) {
        const b = document.createElement('button');
        b.type = 'button'; b.textContent = t; b.dataset.type = t;
        b.title = state.layout.vehicles[t].name || t;
        b.addEventListener('click', () => { const rr = robotByName(r.name); order(rr, 'vehicle', { type: t }, `차종 ${t}`, 'PUT'); });
        q('vehicles').appendChild(b);
      }
      host.appendChild(node);
      state.units[r.name] = { node, q };
    });
  }

  function select(name) {
    if (state.drive.on && state.selected !== name) stopDriving();
    state.selected = name;
    renderAll();
  }

  function drawSprite(canvas, r) {
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 240;
    const h = canvas.clientHeight || 48;
    if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const v = r.vehicle || { length: 0.14, width: 0.13, trailer: 0, type: 'T00' };
    const scale = 128 / 0.29;            // 가장 긴 T15 가 128 px
    const right = 140;
    const color = unitColor(r);
    // 오른쪽이 앞. 몸체 0.14 m, 뒤에 트레일러
    const bodyL = 0.14 * scale; const bodyW = 0.13 * scale * 0.55;
    const cy = h / 2;
    ctx.fillStyle = color;
    ctx.fillRect(right - bodyL, cy - bodyW / 2, bodyL, bodyW);
    ctx.fillStyle = css('--sunk');
    ctx.fillRect(right - 6, cy - bodyW / 2 + 3, 3, bodyW - 6);
    if (v.trailer > 0) {
      const tl = v.trailer * scale;
      const x0 = right - bodyL - 6 - tl;
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(right - bodyL, cy); ctx.lineTo(right - bodyL - 6, cy); ctx.stroke();
      ctx.globalAlpha = 0.5; ctx.fillRect(x0, cy - bodyW * 0.42, tl, bodyW * 0.84); ctx.globalAlpha = 1;
      ctx.strokeRect(x0 + 0.5, cy - bodyW * 0.42 + 0.5, tl - 1, bodyW * 0.84 - 1);
    }
    ctx.fillStyle = css('--soft');
    ctx.font = `500 12px ${css('--mono')}`;
    ctx.textAlign = 'left';
    ctx.fillText(`${v.type}`, right + 12, cy - 4);
    ctx.fillStyle = css('--muted');
    ctx.font = `400 11px ${css('--font')}`;
    ctx.fillText(`전체 ${Math.round(v.length * 1000)} mm`, right + 12, cy + 12);
  }

  function robotState(r) {
    if (!state.connected) return { text: '연결 끊김', tone: 'negative' };
    if (!r.nav2_ready) return { text: 'Nav2 대기', tone: 'warning' };
    if (busy(r)) return { text: RUNNING[r.task.action] || '작업 중', tone: 'active' };
    if (r.teleop) return { text: '직접 운전', tone: 'active' };
    if (!r.localized) return { text: '위치 모름', tone: 'warning' };
    if (r.task && r.task.state === 'failed') return { text: '실패', tone: 'negative' };
    return { text: '대기', tone: 'neutral' };
  }

  function renderUnits() {
    buildUnits();
    for (const r of state.robots) {
      const u = state.units[r.name];
      if (!u) continue;
      const { q, node } = u;
      node.toggleAttribute('data-selected', state.selected === r.name);
      q('select').setAttribute('aria-pressed', String(state.selected === r.name));
      const st = robotState(r);
      q('state').textContent = st.text; q('state').dataset.tone = st.tone;
      const t = r.task;
      q('task').textContent = busy(r) ? `${t.target} · ${taskRest(t)}` : t ? `마지막: ${t.target} ${ACTION[t.action] || ''} ${RESULT[t.state] || ''}` : '받은 작업 없음';
      let progress = 0;
      if (busy(r)) {
        if (t.waypoint_total > 1) progress = t.waypoint_index / t.waypoint_total;
        else if (t.distance_remaining != null && state.startDist[t.id]) progress = 1 - t.distance_remaining / state.startDist[t.id];
      }
      q('progress').style.width = `${Math.max(0, Math.min(1, progress)) * 100}%`;
      q('x').textContent = r.pose ? fixed(r.pose.x) : '—';
      q('y').textContent = r.pose ? fixed(r.pose.y) : '—';
      q('yaw').textContent = r.pose ? `${deg(r.pose.yaw)}°` : '—';
      q('speed').textContent = r.speed ? fixed(r.speed.linear) : '—';
      for (const b of $$('button', q('vehicles'))) {
        b.setAttribute('aria-pressed', String(r.vehicle && r.vehicle.type === b.dataset.type));
        b.disabled = locked() || !state.connected;
      }
      q('link').innerHTML = `<span ${r.nav2_ready ? '' : 'data-bad'}>Nav2 <b>${r.nav2_ready ? '켜짐' : '꺼짐'}</b></span>`
        + `<span ${r.localized ? '' : 'data-bad'}>위치 <b>${r.pose_age_s == null ? '없음' : `${fixed(r.pose_age_s, 1)} s 전`}</b></span>`
        + `<span>${r.teleop ? '직접 운전' : `도메인 <b>${r.domain_id}</b>`}</span>`;
      drawSprite(q('sprite'), r);
    }
  }

  // ---- 명령 막대 --------------------------------------------------------------
  function renderCommand() {
    const r = sel();
    $('#cmd-badge').textContent = sign(r);
    $('#cmd-badge').style.background = r ? unitColor(r) : css('--raised');
    $('#cmd-name').textContent = r ? `${r.name} · ${r.vehicle ? r.vehicle.type : ''}` : '유닛을 고르세요';
    $('#cmd-state').textContent = r ? robotState(r).text + (busy(r) ? ` · ${r.task.target}` : '') : '숫자키 1, 2';
    for (const b of $$('.cmd')) {
      if (b.dataset.pending) {
        b.disabled = true;
        b.title = (state.layout && state.layout.pending && state.layout.pending[b.dataset.pending]) || '준비 중';
        continue;
      }
      const reason = why(r, b.dataset.cmd);
      b.disabled = Boolean(reason);
      b.title = reason;
    }
    const slotPick = $('#slot-pick');
    const slots = Object.keys((state.layout && state.layout.slots) || {});
    if (slotPick.dataset.names !== slots.join()) {
      slotPick.dataset.names = slots.join();
      slotPick.textContent = '';
      for (const s of slots) slotPick.add(new Option(s, s));
    }
    const canDrive = r && state.connected && !locked() && !busy(r);
    const toggle = $('#drive-toggle');
    toggle.setAttribute('aria-pressed', String(state.drive.on));
    toggle.disabled = !state.drive.on && !canDrive;
    toggle.title = toggle.disabled ? why(r, 'drive1') || '지금은 직접 운전할 수 없습니다' : '';
    $('#joystick').toggleAttribute('data-off', !state.drive.on);
    $('#g-lin').textContent = fixed(state.drive.lin);
    $('#g-ang').textContent = fixed(state.drive.ang);
    const bar = (el, v, max) => { const f = Math.max(-1, Math.min(1, v / max)); el.style.left = f >= 0 ? '50%' : `${50 + f * 50}%`; el.style.width = `${Math.abs(f) * 50}%`; };
    bar($('#g-lin-bar'), state.drive.lin, TELEOP.linear);
    bar($('#g-ang-bar'), state.drive.ang, TELEOP.angular);
  }

  // ---- 직접 운전 --------------------------------------------------------------
  function toggleDriving() {
    if (state.drive.on) { stopDriving(); return; }
    const r = sel();
    if (!r || !state.connected || locked() || busy(r)) { toast(why(r, 'drive1') || '지금은 직접 운전할 수 없습니다', 'negative'); return; }
    state.drive.on = true;
    toast(`${sign(r)} · 직접 운전 시작 (WASD, 손 떼면 정지)`);
    renderAll();
  }

  function stopDriving() {
    const r = sel();
    state.drive.on = false; state.drive.keys.clear(); state.drive.joy = null;
    state.drive.lin = 0; state.drive.ang = 0;
    resetStick();
    if (r && state.drive.sending) api('POST', `/api/robots/${encodeURIComponent(r.name)}/teleop`, { linear: 0, angular: 0 }).catch(() => {});
    state.drive.sending = false;
    renderAll();
  }

  function driveTick() {
    const d = state.drive;
    if (!d.on) return;
    const r = sel();
    let lin = 0; let ang = 0;
    const k = d.keys;
    if (k.has('w') || k.has('arrowup')) lin += TELEOP.linear;
    if (k.has('s') || k.has('arrowdown')) lin -= TELEOP.linear;
    if (k.has('a') || k.has('arrowleft')) ang += TELEOP.angular;
    if (k.has('d') || k.has('arrowright')) ang -= TELEOP.angular;
    if (d.joy) { lin = -d.joy.y * TELEOP.linear; ang = -d.joy.x * TELEOP.angular; }
    d.lin = lin; d.ang = ang;
    renderCommand();
    if (!r || d.busy) return;
    const moving = lin !== 0 || ang !== 0;
    if (!moving && !d.sending) return;
    d.busy = true;
    api('POST', `/api/robots/${encodeURIComponent(r.name)}/teleop`, { linear: lin, angular: ang })
      .then(() => { d.sending = moving; })
      .catch((error) => { toast(error.message, 'negative'); stopDriving(); })
      .finally(() => { d.busy = false; });
  }

  const pad = $('#joystick');
  const stick = $('#stick');
  function resetStick() { stick.style.transform = ''; }
  pad.addEventListener('pointerdown', (event) => {
    if (!state.drive.on) { toast('먼저 직접 운전을 켜세요 (M)', 'negative'); return; }
    pad.setPointerCapture(event.pointerId);
    moveStick(event);
  });
  function moveStick(event) {
    if (!state.drive.on || !pad.hasPointerCapture(event.pointerId)) return;
    const rect = pad.getBoundingClientRect();
    const rad = rect.width / 2;
    let x = (event.clientX - rect.left - rad) / rad;
    let y = (event.clientY - rect.top - rad) / rad;
    const len = Math.hypot(x, y);
    if (len > 1) { x /= len; y /= len; }
    state.drive.joy = { x: Math.abs(x) < 0.12 ? 0 : x, y: Math.abs(y) < 0.12 ? 0 : y };
    stick.style.transform = `translate(${x * (rad - 13)}px, ${y * (rad - 13)}px)`;
  }
  pad.addEventListener('pointermove', moveStick);
  const releaseStick = () => { state.drive.joy = null; resetStick(); };
  pad.addEventListener('pointerup', releaseStick);
  pad.addEventListener('pointercancel', releaseStick);

  // ---- 미션 (시연 시나리오) ----------------------------------------------------
  function renderMission() {
    const sc = state.scenario;
    const list = $('#steps');
    const buttons = ['#sc-next', '#sc-run', '#sc-stop', '#sc-reset'].map((s) => $(s));
    if (!sc) {
      list.innerHTML = `<li class="step"><span class="step-mark">–</span><span class="step-title">${isArena() ? '시나리오를 불러오는 중입니다' : '시연 시나리오는 경기장 배치에서만 실행합니다'}</span><span class="step-sub">지금 배치: ${esc(state.layout ? state.layout.name : '—')}</span></li>`;
      buttons.forEach((b) => { b.disabled = true; });
      $('#mission-progress').textContent = '—';
      $('#confirm').hidden = true;
      return;
    }
    const finished = sc.steps.filter((s) => s.state === 'done' || s.state === 'skipped').length;
    $('#mission-progress').textContent = `${finished} / ${sc.steps.length}`;
    const key = JSON.stringify([sc.steps.map((s) => [s.state, s.note]), sc.current]);
    if (list.dataset.key !== key) {
      list.dataset.key = key;
      list.innerHTML = sc.steps.map((s, i) => `<li class="step" data-state="${s.state}">
        <span class="step-mark">${STEP_MARK[s.state] || i + 1}</span>
        <span class="step-title">${esc(s.title)}</span>
        <span class="step-sub">${s.pending ? `<span class="tag">준비 중 · ${esc(s.pending)}</span>` : esc(s.note || s.detail)}</span>
        ${s.state === 'running' && s.actions.length ? `<ul class="step-actions">${s.actions.map((a) => `<li>${esc(a)}</li>`).join('')}</ul>` : ''}
      </li>`).join('');
    }
    $('#confirm').hidden = !sc.waiting;
    $('#confirm-text').textContent = sc.waiting || '';
    const blocked = locked() || !state.connected;
    $('#sc-next').disabled = sc.running || blocked || finished === sc.steps.length;
    $('#sc-run').disabled = sc.running || blocked || finished === sc.steps.length;
    $('#sc-stop').disabled = !sc.running;
    $('#sc-reset').disabled = sc.running || blocked;
    $('#confirm-btn').disabled = blocked;
  }

  async function scenario(path, body) {
    $('#mission-error').textContent = '';
    try { await api('POST', `/api/scenario/${path}`, body || {}); } catch (error) { $('#mission-error').textContent = error.message; }
  }

  // ---- 작업 기록 화면 ---------------------------------------------------------
  function renderLogView() {
    if ($('#view-log').hidden) return;
    const rows = state.log;
    $('#log-full').innerHTML = `<thead><tr><th class="num">끝난 시각</th><th>유닛</th><th>행동</th><th>대상</th><th>결과</th><th>사유</th><th class="num">걸린 시간</th></tr></thead><tbody>${
      rows.length ? rows.map((t) => {
        const r = robotByName(t.robot);
        return `<tr><td class="num dim">${clock(t.finished)}</td>
          <td><span class="who"><i style="background:${r ? unitColor(r) : css('--soft')}">${esc(sign(r) || '?')}</i>${esc(t.robot)}</span></td>
          <td>${ACTION[t.action] || t.action}</td><td>${esc(t.target)}</td>
          <td><span class="result" data-state="${t.state}">${RESULT[t.state] || t.state}</span></td>
          <td class="dim">${esc(t.detail) || '—'}</td><td class="num">${seconds(t.elapsed_s)}</td></tr>`;
      }).join('') : '<tr class="empty"><td colspan="7">아직 끝난 작업이 없습니다.</td></tr>'
    }</tbody>`;
    const ok = rows.filter((t) => t.state === 'succeeded').length;
    const bad = rows.filter((t) => t.state === 'failed').length;
    $('#log-summary').textContent = rows.length ? `최근 ${rows.length}건 중 완료 ${ok}, 실패 ${bad}, 취소 ${rows.length - ok - bad}` : '아직 끝난 작업이 없습니다.';
  }

  // ---- 경기장 그리기 -----------------------------------------------------------
  const canvas = $('#field');
  const ctx = canvas.getContext('2d');
  let view = null;

  function bounds() {
    if (isArena()) { const b = state.layout.board; return { x0: -b.width / 2, y0: -b.height / 2, w: b.width, h: b.height }; }
    const m = state.map;
    if (m) return { x0: m.origin.x, y0: m.origin.y, w: m.width * m.resolution, h: m.height * m.resolution };
    return { x0: -2, y0: -2, w: 4, h: 4 };
  }

  function layoutField() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    if (!state.layout || !rect.width) { view = null; return; }
    const b = bounds();
    const p = { l: 34, r: 6, t: 4, b: 38 };
    const s = Math.min((rect.width - p.l - p.r) / b.w, (rect.height - p.t - p.b) / b.h);
    view = { dpr, s, ...b, ox: p.l + (rect.width - p.l - p.r - b.w * s) / 2, oy: p.t + (rect.height - p.t - p.b - b.h * s) / 2 };
  }
  const toPx = (x, y) => [view.ox + (x - view.x0) * view.s, view.oy + (view.h - (y - view.y0)) * view.s];
  const toWorld = (px, py) => ({ x: view.x0 + (px - view.ox) / view.s, y: view.y0 + view.h - (py - view.oy) / view.s });

  function stadiumPath(radius) {
    const st = new G.Stadium(radius, state.layout.straight);
    ctx.beginPath();
    for (let i = 0; i <= 160; i += 1) {
      const [x, y] = st.point((st.length * i) / 160);
      const [px, py] = toPx(x, y);
      if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py);
    }
    ctx.closePath();
  }

  function zoneRect(z, length, width, fill, stroke) {
    const [px, py] = toPx(z.x, z.y);
    ctx.save();
    ctx.translate(px, py); ctx.rotate(-(z.yaw ?? Math.PI / 2));
    const L = length * view.s; const W = width * view.s;
    ctx.fillStyle = fill; ctx.globalAlpha = 0.16; ctx.fillRect(-L / 2, -W / 2, L, W);
    ctx.globalAlpha = 1; ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.strokeRect(-L / 2, -W / 2, L, W);
    ctx.restore();
  }

  function markerTag(x, y, text) {
    const [px, py] = toPx(x, y);
    ctx.font = `600 11px ${css('--mono')}`;
    const w = ctx.measureText(text).width + 8;
    ctx.fillStyle = css('--text'); ctx.fillRect(px - w / 2, py - 8, w, 16);
    ctx.fillStyle = '#0c0c0e'; ctx.textAlign = 'center'; ctx.fillText(text, px, py + 4);
  }

  function drawArena() {
    const L = state.layout;
    const b = bounds();
    const [bx, by] = toPx(b.x0, b.y0 + b.h);
    ctx.fillStyle = css('--board'); ctx.fillRect(bx, by, b.w * view.s, b.h * view.s);
    // 0.5 m 격자와 눈금
    ctx.strokeStyle = css('--rule'); ctx.lineWidth = 1;
    ctx.fillStyle = css('--muted'); ctx.font = `11px ${css('--mono')}`;
    for (let v = -1; v <= 1.001; v += 0.5) {
      const [px] = toPx(v, 0); const [, py] = toPx(0, v);
      ctx.beginPath(); ctx.moveTo(Math.round(px) + 0.5, by); ctx.lineTo(Math.round(px) + 0.5, by + b.h * view.s); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(bx, Math.round(py) + 0.5); ctx.lineTo(bx + b.w * view.s, Math.round(py) + 0.5); ctx.stroke();
      ctx.textAlign = 'center'; ctx.fillText(minus(v.toFixed(1)), px, by + b.h * view.s + 15);
      ctx.textAlign = 'right'; ctx.fillText(minus(v.toFixed(1)), bx - 6, py + 4);
    }
    // 트랙: 바깥 R + 차선 폭 반 ~ 안쪽 R - 차선 폭 반 을 아스팔트로
    const radii = L.tracks.map((t) => t.radius);
    const half = L.lane_width / 2;
    const outer = Math.max(...radii) + half; const inner = Math.min(...radii) - half;
    stadiumPath(outer); ctx.fillStyle = css('--asphalt'); ctx.fill();
    stadiumPath(inner); ctx.fillStyle = css('--board'); ctx.fill();
    // 테이프 선
    ctx.strokeStyle = css('--tape'); ctx.lineWidth = Math.max(1.5, L.tape_width * view.s);
    const lines = new Set([inner, outer, ...radii.slice(0, -1).map((r) => r + half)]);
    for (const r of lines) { stadiumPath(r); ctx.stroke(); }
    // 차선 중심선과 방향 화살표 (반시계)
    for (const t of L.tracks) {
      ctx.setLineDash([4, 6]); ctx.strokeStyle = css('--control'); ctx.lineWidth = 1;
      stadiumPath(t.radius); ctx.stroke(); ctx.setLineDash([]);
      const st = new G.Stadium(t.radius, L.straight);
      ctx.fillStyle = css('--muted');
      for (const f of [0.08, 0.58]) {
        const [x, y, yaw] = st.point(st.length * f);
        const [px, py] = toPx(x, y);
        ctx.save(); ctx.translate(px, py); ctx.rotate(-yaw);
        ctx.beginPath(); ctx.moveTo(7, 0); ctx.lineTo(-4, -5); ctx.lineTo(-4, 5); ctx.closePath(); ctx.fill();
        ctx.restore();
      }
      const [lx, ly] = toPx(-t.radius, 0);
      ctx.save(); ctx.translate(lx, ly); ctx.rotate(Math.PI / 2);
      ctx.fillStyle = css('--soft'); ctx.font = `500 11px ${css('--font')}`; ctx.textAlign = 'center';
      ctx.fillText(`${t.id}번 · ${(t.vehicles || []).join('/')}`, 0, 4);
      ctx.restore();
    }
    // 주차 칸
    for (const [name, s] of Object.entries(L.slots)) {
      zoneRect(s, s.length, s.width, css('--zone-park'), css('--zone-park'));
      const fx = s.x + Math.cos(s.yaw) * (s.length / 2); const fy = s.y + Math.sin(s.yaw) * (s.length / 2);
      markerTag(fx, fy, String(s.marker ?? name));
    }
    // 주유구역과 대기선
    const f = L.fuel;
    if (f) {
      zoneRect({ ...f.zone, yaw: Math.PI / 2 }, f.zone.length, f.zone.width, css('--zone-fuel'), css('--zone-fuel'));
      const [zx, zy] = toPx(f.zone.x, f.zone.y);
      ctx.fillStyle = css('--zone-fuel'); ctx.font = `600 12px ${css('--font')}`; ctx.textAlign = 'center';
      ctx.fillText('주유', zx, zy + 4);
      markerTag(f.zone.x, f.zone.y - f.zone.length / 2, String(f.zone.marker));
      const w = f.wait_line;
      const [ax, ay] = toPx(w.x - w.width / 2, w.y); const [cx] = toPx(w.x + w.width / 2, w.y);
      ctx.fillStyle = css('--zone-wait'); ctx.fillRect(ax, ay - 3, cx - ax, 6);
      markerTag(w.x, w.y + 0.07, String(w.marker));
    }
  }

  function drawFactory() {
    const m = state.map;
    if (m) {
      const cs = m.resolution * view.s;
      const colors = { f: css('--map-free'), o: css('--map-wall') };
      for (let row = 0; row < m.height; row += 1) {
        for (let col = 0; col < m.width; col += 1) {
          const k = m.cells[row * m.width + col];
          if (k === 'u') continue;
          ctx.fillStyle = colors[k];
          ctx.fillRect(view.ox + col * cs, view.oy + row * cs, Math.ceil(cs), Math.ceil(cs));
        }
      }
    }
    ctx.setLineDash([4, 5]); ctx.strokeStyle = css('--control');
    for (const pts of Object.values(state.layout.routes || {})) {
      ctx.beginPath();
      pts.forEach((p, i) => { const [px, py] = toPx(p.x, p.y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const [name, s] of Object.entries(state.layout.slots)) {
      zoneRect({ ...s, yaw: s.yaw }, 0.24, 0.2, css('--zone-park'), css('--zone-park'));
      const [px, py] = toPx(s.x, s.y);
      ctx.fillStyle = css('--zone-park'); ctx.font = `500 11px ${css('--mono')}`; ctx.textAlign = 'center';
      ctx.fillText(name, px, py - 0.1 * view.s - 6);
    }
  }

  function drawVehicle(r) {
    const v = r.vehicle || { length: 0.14, width: 0.13, trailer: 0 };
    const [px, py] = toPx(r.pose.x, r.pose.y);
    const s = view.s; const color = unitColor(r);
    ctx.save(); ctx.translate(px, py); ctx.rotate(-r.pose.yaw);
    const bl = 0.14 * s; const bw = (v.width || 0.13) * s;
    if (v.trailer > 0) {
      const tl = v.trailer * s; const gap = 0.012 * s;
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(-bl / 2, 0); ctx.lineTo(-bl / 2 - gap, 0); ctx.stroke();
      ctx.fillStyle = color; ctx.globalAlpha = 0.45; ctx.fillRect(-bl / 2 - gap - tl, -bw * 0.42, tl, bw * 0.84); ctx.globalAlpha = 1;
      ctx.lineWidth = 1.5; ctx.strokeRect(-bl / 2 - gap - tl, -bw * 0.42, tl, bw * 0.84);
    }
    ctx.fillStyle = color; ctx.fillRect(-bl / 2, -bw / 2, bl, bw);
    ctx.fillStyle = css('--board'); ctx.fillRect(bl / 2 - 0.018 * s, -bw / 2 + 0.02 * s, 0.01 * s, bw - 0.04 * s);
    if (r.name === state.selected) {
      const ext = bl / 2 + 6; const back = -bl / 2 - (v.trailer > 0 ? v.trailer * s + 0.012 * s : 0) - 6; const half = bw / 2 + 6;
      const c = 8;
      ctx.strokeStyle = css('--select'); ctx.lineWidth = 1.5;
      ctx.beginPath();
      [[ext, half, -1, -1], [ext, -half, -1, 1], [back, half, 1, -1], [back, -half, 1, 1]].forEach(([x, y, dx, dy]) => {
        ctx.moveTo(x + dx * c, y); ctx.lineTo(x, y); ctx.lineTo(x, y + dy * c);
      });
      ctx.stroke();
    }
    ctx.restore();
    // 호출 부호
    ctx.font = `700 12px ${css('--mono')}`; ctx.textAlign = 'center';
    const tag = sign(r);
    const reach = (0.07 + (v.trailer > 0 ? v.trailer + 0.012 : 0)) * s;   // 로봇이 차지하는 반경
    const tx = px + Math.max(bw / 2, reach * Math.abs(Math.sin(r.pose.yaw)), bl / 2) + 14;
    ctx.fillStyle = color; ctx.fillRect(tx - 9, py - 9, 18, 17);
    ctx.fillStyle = '#0c0c0e'; ctx.fillText(tag, tx, py + 4);
  }

  function drawField() {
    if (!view || $('#view-board').hidden) return;
    ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (isArena()) drawArena(); else drawFactory();
    // 자취와 남은 경로
    for (const r of state.robots) {
      const color = unitColor(r);
      const trail = state.trails[r.name] || [];
      if (trail.length > 1) {
        ctx.globalAlpha = 0.3; ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.beginPath(); trail.forEach((p, i) => { const [x, y] = toPx(p.x, p.y); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); }); ctx.stroke();
        ctx.globalAlpha = 1;
      }
      if (busy(r) && r.pose) {
        const rest = r.task.poses.slice(r.task.waypoint_index);
        ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.setLineDash([3, 4]);
        ctx.beginPath();
        let [x0, y0] = toPx(r.pose.x, r.pose.y); ctx.moveTo(x0, y0);
        for (const p of rest) { [x0, y0] = toPx(p.x, p.y); ctx.lineTo(x0, y0); }
        ctx.stroke(); ctx.setLineDash([]);
        const g = rest[rest.length - 1] || r.task.goal;
        const [gx, gy] = toPx(g.x, g.y);
        ctx.beginPath(); ctx.arc(gx, gy, 7, 0, Math.PI * 2); ctx.stroke();
      }
    }
    for (const r of state.robots) if (r.pose) drawVehicle(r);
    // 우클릭 미리보기
    if (state.hover && state.hover.label) {
      const [hx, hy] = toPx(state.hover.x, state.hover.y);
      ctx.strokeStyle = css('--select'); ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.arc(hx, hy, 6, 0, Math.PI * 2); ctx.stroke();
      ctx.font = `500 12px ${css('--font')}`;
      const text = `우클릭: ${state.hover.label}`;
      const w = ctx.measureText(text).width + 14;
      const left = Math.min(hx + 12, view.ox + view.w * view.s - w);
      ctx.fillStyle = css('--raised'); ctx.fillRect(left, hy - 26, w, 20);
      ctx.fillStyle = css('--text'); ctx.textAlign = 'left'; ctx.fillText(text, left + 7, hy - 12);
    }
  }

  // 우클릭으로 할 일: 칸이면 주차, 주유구역이면 진입, 대기선이면 대기, 아니면 이동
  function contextAt(p) {
    const L = state.layout;
    const inRect = (z, length, width) => {
      const yaw = z.yaw ?? Math.PI / 2;
      const dx = p.x - z.x; const dy = p.y - z.y;
      const along = dx * Math.cos(yaw) + dy * Math.sin(yaw);
      const across = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
      return Math.abs(along) <= length / 2 && Math.abs(across) <= width / 2;
    };
    for (const [name, s] of Object.entries(L.slots || {})) {
      if (inRect(s, s.length || 0.24, s.width || 0.2)) return { cmd: 'park', slot: name, label: `${name} 주차`, x: s.x, y: s.y };
    }
    if (isArena() && L.fuel) {
      const z = L.fuel.zone; const w = L.fuel.wait_line;
      if (inRect({ ...z, yaw: Math.PI / 2 }, z.length, z.width)) return { cmd: 'fuel_enter', label: '주유구역 20 진입', x: z.x, y: z.y };
      if (Math.abs(p.x - w.x) <= w.width / 2 && Math.abs(p.y - w.y) < 0.08) return { cmd: 'fuel_wait', label: '대기선 30에서 대기', x: w.x, y: w.y };
    }
    return { cmd: 'goto', label: `이동 (${fixed(p.x)}, ${fixed(p.y)})`, x: p.x, y: p.y };
  }

  function pointerWorld(event) {
    const rect = canvas.getBoundingClientRect();
    const p = toWorld(event.clientX - rect.left, event.clientY - rect.top);
    if (p.x < view.x0 || p.y < view.y0 || p.x > view.x0 + view.w || p.y > view.y0 + view.h) return null;
    return p;
  }

  function act(p) {
    const r = sel();
    const c = contextAt(p);
    if (c.cmd === 'park') { $('#slot-pick').value = c.slot; }
    if (c.cmd === 'goto') {
      const reason = why(r, 'park');
      if (reason) { toast(reason, 'negative'); return; }
      const yaw = r.pose ? Math.atan2(p.y - r.pose.y, p.x - r.pose.x) : 0;
      order(r, 'goto', { x: +p.x.toFixed(3), y: +p.y.toFixed(3), yaw: +yaw.toFixed(3) }, c.label);
      return;
    }
    runCommand(c.cmd);
  }

  canvas.addEventListener('contextmenu', (event) => {
    event.preventDefault();
    if (!view) return;
    const p = pointerWorld(event);
    if (p) act(p);
  });
  canvas.addEventListener('pointermove', (event) => {
    if (!view || event.pointerType === 'touch') return;
    const p = pointerWorld(event);
    state.hover = p && sel() ? contextAt(p) : null;
    drawField();
  });
  canvas.addEventListener('pointerleave', () => { state.hover = null; drawField(); });
  let pressTimer = 0;
  canvas.addEventListener('pointerdown', (event) => {
    if (!view) return;
    const p = pointerWorld(event);
    if (!p) return;
    if (event.button === 0) {
      const hit = state.robots.find((r) => r.pose && Math.hypot(r.pose.x - p.x, r.pose.y - p.y) < 0.12);
      if (hit) { select(hit.name); return; }
    }
    if (event.pointerType === 'touch') {   // 길게 누르면 우클릭과 같다
      clearTimeout(pressTimer);
      pressTimer = setTimeout(() => act(p), 550);
    }
  });
  ['pointerup', 'pointercancel'].forEach((t) => canvas.addEventListener(t, () => clearTimeout(pressTimer)));

  function renderLegend() {
    const items = state.robots.map((r) => `<li><i style="background:${unitColor(r)}"></i>${esc(sign(r))} ${esc(r.name)}</li>`);
    if (isArena()) {
      items.push(`<li><i style="background:${css('--zone-park')}"></i>주차</li>`, `<li><i style="background:${css('--zone-fuel')}"></i>주유</li>`, `<li><i style="background:${css('--zone-wait')}"></i>대기선</li>`);
    } else {
      items.push(`<li><i style="background:${css('--map-free')}"></i>바닥</li>`, `<li><i style="background:${css('--map-wall')}"></i>벽</li>`);
    }
    const html = items.join('');
    if ($('#legend').innerHTML !== html) $('#legend').innerHTML = html;
  }

  // ---- 키보드 -----------------------------------------------------------------
  document.addEventListener('keydown', (event) => {
    const tag = (event.target.tagName || '').toLowerCase();
    if (tag === 'input' || tag === 'select' || tag === 'textarea') return;
    const k = event.key.toLowerCase();
    if (k === 'escape') { event.preventDefault(); stopAll(); return; }
    if (k === ' ') { event.preventDefault(); if (state.drive.on) stopDriving(); runCommand('stop'); return; }
    if (k === 'm') { toggleDriving(); return; }
    if (/^[1-9]$/.test(k)) { const r = state.robots[Number(k) - 1]; if (r) select(r.name); return; }
    if (state.drive.on && ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) {
      event.preventDefault(); state.drive.keys.add(k); return;
    }
    if (!state.drive.on && KEYS[k] && !event.ctrlKey && !event.metaKey && !event.altKey) runCommand(KEYS[k]);
  });
  document.addEventListener('keyup', (event) => { state.drive.keys.delete(event.key.toLowerCase()); });
  window.addEventListener('blur', () => { state.drive.keys.clear(); });

  async function stopAll() {
    if (state.drive.on) stopDriving();
    try { await api('POST', '/api/stop'); toast('전체 정지'); } catch (error) { toast(`전체 정지 실패: ${error.message}`, 'negative'); }
  }

  // ---- 화면 전환과 시작 ---------------------------------------------------------
  function route() {
    const v = location.hash === '#log' ? 'log' : 'board';
    $('#view-board').hidden = v !== 'board';
    $('#view-log').hidden = v !== 'log';
    $$('.rail-item[data-view]').forEach((a) => { if (a.dataset.view === v) a.setAttribute('aria-current', 'page'); else a.removeAttribute('aria-current'); });
    if (v === 'board') requestAnimationFrame(() => { layoutField(); drawField(); });
    renderAll();
  }

  function renderAll() {
    renderLock(); renderFresh(); renderFinding(); renderUnits(); renderCommand();
    renderMission(); renderLegend(); renderLogView(); drawField();
  }

  $$('.cmd').forEach((b) => b.addEventListener('click', () => { if (!b.dataset.pending) runCommand(b.dataset.cmd); }));
  $('#stop-all').addEventListener('click', stopAll);
  $('#drive-toggle').addEventListener('click', toggleDriving);
  $('#sc-next').addEventListener('click', () => scenario('next'));
  $('#sc-run').addEventListener('click', () => scenario('run'));
  $('#sc-stop').addEventListener('click', () => scenario('stop'));
  $('#sc-reset').addEventListener('click', () => scenario('reset'));
  $('#confirm-btn').addEventListener('click', () => scenario('confirm'));
  $('#lock').addEventListener('click', () => setPassword(''));
  $('#unlock').addEventListener('submit', async (event) => {
    event.preventDefault();
    const value = $('#password').value;
    $('#unlock-error').textContent = '';
    if (!value) { $('#unlock-error').textContent = '비밀번호를 넣으세요.'; return; }
    try { await api('POST', '/api/auth/check', null, value); $('#password').value = ''; setPassword(value); } catch (error) { $('#unlock-error').textContent = error.message; }
  });
  window.addEventListener('hashchange', route);
  new ResizeObserver(() => { layoutField(); drawField(); }).observe(canvas);
  setInterval(renderFresh, 1000);
  setInterval(driveTick, 100);

  Promise.all([api('GET', '/api/layout').catch(() => null), api('GET', '/api/map').catch(() => null), api('GET', '/api/auth').catch(() => ({ required: false }))])
    .then(([layout, map, auth]) => {
      state.layout = layout || { kind: 'factory', name: '배치를 받지 못함', slots: {}, routes: {}, vehicles: {} };
      state.map = map;
      state.auth.required = Boolean(auth && auth.required);
      if (layout && layout.teleop) Object.assign(TELEOP, layout.teleop);
      $('#layout-chip').textContent = isArena() ? '경기장' : '공장 월드';
      $('#field-title').textContent = isArena() ? '경기장' : '공장 월드';
      $('#field-meta').textContent = isArena() ? '2000 × 2000 mm · 반시계 주행 · 원점 = 트랙 중심' : '시뮬레이션 임시 배치 · 경기장 월드(#1) 전까지';
      if (document.fonts) document.fonts.ready.then(() => { layoutField(); drawField(); });
      layoutField();
      route();
      connect();
    });
})();
