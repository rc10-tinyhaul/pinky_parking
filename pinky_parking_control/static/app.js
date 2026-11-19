/*
  Pinky 관제실 대시보드.
  상태는 WebSocket(/api/ws)으로 초당 10번 받고, 명령은 행동 단위 HTTP API 로 보낸다.
  ?demo 를 붙이면 서버 없이 demo.js 의 모의 로봇으로 돈다 (화면 확인용).
*/
(() => {
  'use strict';

  const $ = (sel, el = document) => el.querySelector(sel);
  const ACTION = { goto: '이동', park: '주차', unpark: '출차', lap: '주행' };
  const RUNNING = { goto: '이동 중', park: '주차 중', unpark: '출차 중', lap: '주행 중' };
  const RESULT = { running: '진행 중', succeeded: '완료', failed: '실패', canceled: '취소됨' };
  const TRAIL_POINTS = 150;          // 위치 자취: 10 Hz 로 15초
  const LOG_ROWS = 6;

  const demo = new URLSearchParams(location.search).has('demo') ? window.PinkyDemo : null;

  const state = {
    robots: [], log: [], map: null, layout: { slots: {}, routes: {}, exits: {} },
    selected: null, pending: null, connected: false, lastRecv: 0, trails: {}, cards: {},
    auth: { required: false, password: '' },
  };
  const PASSWORD_KEY = 'pinky-control-password';
  try { state.auth.password = sessionStorage.getItem(PASSWORD_KEY) || ''; } catch (e) { /* 저장소를 못 쓰면 매번 입력 */ }
  const locked = () => state.auth.required && !state.auth.password;

  // ---- 통신 ----------------------------------------------------------------
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
    if (status === 401) { setPassword(''); }   // 비밀번호가 바뀌었으면 다시 잠근다
    if (status >= 400) throw new Error(data.detail || `요청 실패 (${status})`);
    return data;
  }

  function setPassword(value) {
    state.auth.password = value;
    try { if (value) sessionStorage.setItem(PASSWORD_KEY, value); else sessionStorage.removeItem(PASSWORD_KEY); } catch (e) { /* 무시 */ }
    renderAll();
  }

  function renderLock() {
    $('#unlock').hidden = !locked();
    $('#lock').hidden = !(state.auth.required && state.auth.password);
  }

  function connect() {
    if (demo) { state.connected = true; demo.subscribe(onSnapshot); return; }
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${scheme}://${location.host}/api/ws`);
    ws.onopen = () => { state.connected = true; renderFresh(); };
    ws.onmessage = (event) => onSnapshot(JSON.parse(event.data));
    ws.onclose = () => {
      state.connected = false; renderAll(); setTimeout(connect, 2000);
    };
  }

  function onSnapshot(snap) {
    state.robots = snap.robots || [];
    state.log = snap.log || [];
    state.lastRecv = Date.now();
    for (const r of state.robots) {
      if (!r.pose) continue;
      const trail = state.trails[r.name] || (state.trails[r.name] = []);
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(last.x - r.pose.x, last.y - r.pose.y) > 0.005) trail.push({ x: r.pose.x, y: r.pose.y });
      if (trail.length > TRAIL_POINTS) trail.shift();
    }
    if (!state.selected && state.robots.length) state.selected = state.robots[0].name;
    renderAll();
  }

  // ---- 서식 ----------------------------------------------------------------
  const minus = (s) => s.replace('-', '−');
  const fixed = (v, d = 2) => (v == null || !isFinite(v) ? '—' : minus(Number(v).toFixed(d)));
  const deg = (rad) => (rad == null ? '—' : minus(String(Math.round(rad * 180 / Math.PI))));
  const pad2 = (n) => String(n).padStart(2, '0');
  const hms = (d) => `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
  const clock = (t) => (t ? hms(new Date(t * 1000)) : '—');
  const seconds = (s) => (s == null ? '—' : s < 60 ? `${s.toFixed(1)} s` : `${Math.floor(s / 60)}분 ${Math.round(s % 60)}초`);
  const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const robotColor = (name) => {
    const i = state.robots.findIndex((r) => r.name === name);
    return getComputedStyle(document.documentElement).getPropertyValue(`--robot-${(i % 2) + 1}`).trim() || '#fff';
  };
  const busy = (r) => r && r.task && r.task.state === 'running';

  function robotState(r) {
    if (!state.connected) return { text: '연결 끊김', tone: 'negative' };
    if (!r.nav2_ready) return { text: 'Nav2 대기', tone: 'warning' };
    if (busy(r)) return { text: RUNNING[r.task.action], tone: 'active' };
    if (!r.localized) return { text: '위치 모름', tone: 'warning' };
    if (r.task && r.task.state === 'failed') return { text: '실패', tone: 'negative' };
    return { text: '대기', tone: 'neutral' };
  }

  function taskSentence(r) {
    const t = r.task;
    if (!t) return '받은 작업이 없습니다';
    if (t.state === 'running') {
      const rest = t.action === 'lap'
        ? `경유점 ${Math.min(t.waypoint_index + 1, t.waypoint_total)}/${t.waypoint_total}`
        : (t.distance_remaining == null ? '출발 중' : `${fixed(t.distance_remaining)} m 남음`);
      return `<b>${esc(t.target)} ${ACTION[t.action]}</b> · ${rest} · ${seconds(t.elapsed_s)}`;
    }
    const why = t.detail ? ` · ${esc(t.detail)}` : '';
    return `마지막 작업: ${esc(t.target)} ${ACTION[t.action]} ${RESULT[t.state]}${why}`;
  }

  // ---- 제목 줄: 지금 가장 먼저 알아야 할 것 -------------------------------
  function renderFinding() {
    const el = $('#finding');
    if (!state.connected && !demo) { el.textContent = '관제 서버와 연결이 끊겼습니다. 2초마다 다시 연결합니다.'; return; }
    if (!state.robots.length) { el.textContent = '로봇 상태를 받는 중입니다.'; return; }
    const running = state.robots.filter(busy);
    if (running.length) {
      el.innerHTML = running.map((r) => {
        const t = r.task;
        const rest = t.action === 'lap'
          ? `경유점 ${Math.min(t.waypoint_index + 1, t.waypoint_total)}/${t.waypoint_total}`
          : (t.distance_remaining == null ? '출발 중' : `${fixed(t.distance_remaining)} m 남음`);
        return `<b>${esc(r.name)}</b> ${esc(t.target)} ${RUNNING[t.action]}, ${rest}`;
      }).join(' · ');
      return;
    }
    const notReady = state.robots.filter((r) => !r.nav2_ready).map((r) => r.name);
    if (notReady.length) { el.textContent = `${notReady.join(', ')}의 Nav2가 아직 켜지지 않았습니다.`; return; }
    const failed = state.log.find((t) => t.state === 'failed' && Date.now() / 1000 - t.finished < 600);
    if (failed) {
      el.innerHTML = `<b>${esc(failed.robot)}</b> ${esc(failed.target)} ${ACTION[failed.action]} 실패: ${esc(failed.detail)} (${clock(failed.finished)})`;
      return;
    }
    el.textContent = `로봇 ${state.robots.length}대 모두 대기 중입니다.`;
  }

  function renderFresh() {
    const el = $('#fresh');
    const text = $('#fresh-text');
    if (demo) { el.dataset.state = 'live'; text.textContent = '데모 데이터 · 서버 없이 실행 중'; return; }
    if (state.connected && state.lastRecv) {
      el.dataset.state = 'live';
      text.textContent = `실시간 · ${hms(new Date(state.lastRecv))}`;
    } else if (state.lastRecv) {
      el.dataset.state = 'lost';
      text.textContent = `연결 끊김 · ${hms(new Date(state.lastRecv))} 이후 수신 없음`;
    } else {
      el.dataset.state = 'wait';
      text.textContent = '연결 중';
    }
  }

  // ---- 로봇 카드 -----------------------------------------------------------
  function buildCards() {
    const host = $('#robots');
    const names = state.robots.map((r) => r.name).join();
    if (host.dataset.names === names) return;
    host.dataset.names = names;
    host.textContent = '';
    state.cards = {};
    for (const r of state.robots) {
      const node = $('#robot-card').content.firstElementChild.cloneNode(true);
      const q = (k) => node.querySelector(`[data-r="${k}"]`);
      const id = `r-${r.name}`;
      q('slot').id = `${id}-slot`; q('slot-label').htmlFor = `${id}-slot`;
      q('route').id = `${id}-route`; q('route-label').htmlFor = `${id}-route`;
      q('name').textContent = r.name;
      q('domain').textContent = `도메인 ${r.domain_id}`;
      q('swatch').style.background = robotColor(r.name);
      fillOptions(q('slot'), Object.keys(state.layout.slots));
      fillOptions(q('route'), Object.keys(state.layout.routes));
      q('select').addEventListener('click', () => { state.selected = r.name; state.pending = null; renderAll(); });
      const run = (path, body) => send(r.name, path, body);
      q('park').addEventListener('click', () => run('park', { slot: q('slot').value }));
      q('unpark').addEventListener('click', () => run('unpark', { slot: q('slot').value }));
      q('lap').addEventListener('click', () => run('lap', { route: q('route').value }));
      q('cancel').addEventListener('click', () => run('cancel'));
      q('stop').addEventListener('click', () => run('stop'));
      host.appendChild(node);
      state.cards[r.name] = q;
    }
  }

  function fillOptions(select, names) {
    select.textContent = '';
    for (const n of names) select.add(new Option(n, n));
    if (!names.length) select.add(new Option('—', ''));
  }

  async function send(name, path, body) {
    const q = state.cards[name];
    if (q) q('error').textContent = '';
    try {
      await api('POST', `/api/robots/${encodeURIComponent(name)}/${path}`, body);
    } catch (error) {
      if (!q) return;
      q('error').textContent = error.message;
      clearTimeout(q.errorTimer);
      q.errorTimer = setTimeout(() => { q('error').textContent = ''; }, 6000);
    }
  }

  function renderCards() {
    buildCards();
    for (const r of state.robots) {
      const q = state.cards[r.name];
      if (!q) continue;
      const card = q('name').closest('.robot');
      card.toggleAttribute('data-loading', !r.pose);
      q('select').setAttribute('aria-pressed', String(state.selected === r.name));
      const st = robotState(r);
      q('state').textContent = st.text;
      q('state').dataset.tone = st.tone;
      q('task').innerHTML = taskSentence(r);
      q('x').innerHTML = r.pose ? `${fixed(r.pose.x)}<small>m</small>` : '—';
      q('y').innerHTML = r.pose ? `${fixed(r.pose.y)}<small>m</small>` : '—';
      q('yaw').innerHTML = r.pose ? `${deg(r.pose.yaw)}<small>°</small>` : '—';
      q('speed').innerHTML = `${fixed(r.speed ? r.speed.linear : null)}<small>m/s</small>`;
      const ready = state.connected && r.nav2_ready;
      const why = locked() ? '팀 비밀번호를 먼저 넣으세요' : !state.connected ? '서버와 연결이 끊겼습니다'
        : !r.nav2_ready ? 'Nav2가 아직 켜지지 않았습니다' : busy(r) ? '작업 중입니다. 먼저 취소하세요' : '';
      for (const k of ['park', 'unpark', 'lap']) {
        q(k).disabled = !ready || busy(r) || locked();
        q(k).title = q(k).disabled ? why : '';
      }
      q('slot').disabled = q('route').disabled = !ready || busy(r);
      q('cancel').disabled = !busy(r);
      q('stop').disabled = !state.connected;
      const age = r.pose_age_s;
      q('link').innerHTML = `<span ${r.nav2_ready ? '' : 'data-bad'}>Nav2 <b>${r.nav2_ready ? '켜짐' : '꺼짐'}</b></span>`
        + `<span ${r.localized ? '' : 'data-bad'}>위치 <b>${age == null ? '받은 적 없음' : `${fixed(age, 1)} s 전`}</b></span>`
        + `<span>각속도 <b>${fixed(r.speed ? r.speed.angular : null)} rad/s</b></span>`;
    }
  }

  // ---- 표 ------------------------------------------------------------------
  function logRows(items) {
    if (!items.length) return '<tr class="empty"><td colspan="7">아직 끝난 작업이 없습니다.</td></tr>';
    return items.map((t) => `<tr>
      <td class="num dim">${clock(t.finished)}</td>
      <td><span class="who"><i class="swatch" style="background:${robotColor(t.robot)}"></i>${esc(t.robot)}</span></td>
      <td>${ACTION[t.action]}</td>
      <td class="mono">${esc(t.target)}</td>
      <td><span class="result" data-state="${t.state}">${RESULT[t.state]}</span></td>
      <td class="detail hide-sm">${esc(t.detail) || '<span class="dim">—</span>'}</td>
      <td class="num">${seconds(t.elapsed_s)}</td>
    </tr>`).join('');
  }
  const LOG_HEAD = `<thead><tr><th class="num">끝난 시각</th><th>로봇</th><th>행동</th><th>대상</th><th>결과</th>
    <th class="hide-sm">사유</th><th class="num">걸린 시간</th></tr></thead>`;

  function renderLog() {
    $('#log-table').innerHTML = LOG_HEAD + `<tbody>${logRows(state.log.slice(0, LOG_ROWS))}</tbody>`;
    $('#log-count').textContent = `${state.log.length}건 · 전체 보기`;
    if (!$('#view-log').hidden) {
      $('#log-full').innerHTML = LOG_HEAD + `<tbody>${logRows(state.log)}</tbody>`;
      const ok = state.log.filter((t) => t.state === 'succeeded').length;
      const bad = state.log.filter((t) => t.state === 'failed').length;
      $('#log-summary').textContent = state.log.length
        ? `최근 ${state.log.length}건 중 완료 ${ok}건, 실패 ${bad}건, 취소 ${state.log.length - ok - bad}건`
        : '아직 끝난 작업이 없습니다.';
    }
  }

  function slotOccupant(name, slot) {
    for (const r of state.robots) {
      if (busy(r) && r.task.action === 'park' && r.task.target === name) return `${r.name} 들어오는 중`;
      if (r.pose && Math.hypot(r.pose.x - slot.x, r.pose.y - slot.y) < 0.25 && !busy(r)) return `${r.name} 주차됨`;
    }
    return null;
  }

  function renderLayout() {
    const slots = Object.entries(state.layout.slots);
    $('#slots-table').innerHTML = `<thead><tr><th>칸</th><th class="num">x</th><th class="num">y</th><th class="num">방향</th><th>상태</th></tr></thead><tbody>${
      slots.length ? slots.map(([n, s]) => {
        const who = slotOccupant(n, s);
        return `<tr><td class="mono">${esc(n)}</td><td class="num">${fixed(s.x)}</td><td class="num">${fixed(s.y)}</td>
          <td class="num">${deg(s.yaw)}°</td><td class="${who ? '' : 'dim'}">${who || '비어 있음'}</td></tr>`;
      }).join('') : '<tr class="empty"><td colspan="5">주차 칸이 없습니다 (slots.yaml).</td></tr>'
    }</tbody>`;
    // 경로는 경유점마다 한 줄: 주행 중인 로봇이 다음에 갈 점을 표시한다
    const routes = Object.entries(state.layout.routes);
    $('#routes-table').innerHTML = `<thead><tr><th>경로</th><th class="num">경유점</th><th class="num">x</th><th class="num">y</th><th>상태</th></tr></thead><tbody>${
      routes.length ? routes.map(([n, pts]) => pts.map((p, i) => {
        const who = state.robots.filter((r) => busy(r) && r.task.action === 'lap' && r.task.target === n && r.task.waypoint_index === i)
          .map((r) => `${r.name} 다음 목표`);
        return `<tr><td class="mono ${i ? 'dim' : ''}">${i ? '' : esc(n)}</td><td class="num">${i + 1}</td>
          <td class="num">${fixed(p.x)}</td><td class="num">${fixed(p.y)}</td>
          <td class="${who.length ? '' : 'dim'}">${who.length ? who.join(', ') : '—'}</td></tr>`;
      }).join('')).join('') : '<tr class="empty"><td colspan="5">경로가 없습니다 (route.yaml).</td></tr>'
    }</tbody>`;
  }

  function renderLegend() {
    $('#legend').innerHTML = state.robots.map((r) => `<li><i style="background:${robotColor(r.name)}"></i>${esc(r.name)}</li>`).join('')
      + '<li><i class="slot-mark"></i>주차 칸</li>'
      + '<li><i class="floor-mark"></i>바닥</li><li><i class="wall-mark"></i>벽</li>';
  }

  // ---- 배치도 --------------------------------------------------------------
  const canvas = $('#map');
  const ctx = canvas.getContext('2d');
  let view = null;
  let cellLayer = null;

  function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

  function layoutMap() {
    const m = state.map;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    if (!m || !rect.width) { view = null; return; }
    const W = m.width * m.resolution;
    const H = m.height * m.resolution;
    const pad = { l: 40, r: 8, t: 6, b: 26 };
    const s = Math.min((rect.width - pad.l - pad.r) / W, (rect.height - pad.t - pad.b) / H);
    view = {
      dpr, s, W, H, w: rect.width, h: rect.height, x0: m.origin.x, y0: m.origin.y,
      ox: pad.l + (rect.width - pad.l - pad.r - W * s) / 2,
      oy: pad.t + (rect.height - pad.t - pad.b - H * s) / 2,
    };
    // 칸은 크기가 바뀔 때만 다시 그린다
    cellLayer = document.createElement('canvas');
    cellLayer.width = canvas.width; cellLayer.height = canvas.height;
    const c = cellLayer.getContext('2d');
    c.scale(dpr, dpr);
    const colors = { f: css('--map-free'), o: css('--map-wall') };
    const cs = m.resolution * s;
    for (let row = 0; row < m.height; row += 1) {
      for (let col = 0; col < m.width; col += 1) {
        const kind = m.cells[row * m.width + col];
        if (kind === 'u') continue;
        c.fillStyle = colors[kind];
        c.fillRect(view.ox + col * cs, view.oy + row * cs, Math.ceil(cs), Math.ceil(cs));
      }
    }
  }

  const toPx = (x, y) => [view.ox + (x - view.x0) * view.s, view.oy + (view.H - (y - view.y0)) * view.s];
  const toWorld = (px, py) => ({ x: view.x0 + (px - view.ox) / view.s, y: view.y0 + view.H - (py - view.oy) / view.s });

  function drawMap() {
    if (!view) return;
    const { dpr } = view;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(cellLayer, 0, 0);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const mono = `11px ${css('--mono')}`;
    const sans = `12px ${css('--font')}`;

    // 0.5 m 격자와 1 m 눈금
    const [left, top] = toPx(view.x0, view.y0 + view.H);
    const [right, bottom] = toPx(view.x0 + view.W, view.y0);
    ctx.lineWidth = 1;
    ctx.strokeStyle = css('--border');
    ctx.fillStyle = css('--muted');
    ctx.font = mono;
    for (let x = Math.ceil(view.x0 * 2) / 2; x <= view.x0 + view.W; x += 0.5) {
      const [px] = toPx(x, 0);
      ctx.beginPath(); ctx.moveTo(Math.round(px) + 0.5, top); ctx.lineTo(Math.round(px) + 0.5, bottom); ctx.stroke();
      if (Number.isInteger(x)) { ctx.textAlign = 'center'; ctx.fillText(`${minus(String(x))} m`, px, bottom + 17); }
    }
    for (let y = Math.ceil(view.y0 * 2) / 2; y <= view.y0 + view.H; y += 0.5) {
      const [, py] = toPx(0, y);
      ctx.beginPath(); ctx.moveTo(left, Math.round(py) + 0.5); ctx.lineTo(right, Math.round(py) + 0.5); ctx.stroke();
      if (Number.isInteger(y)) { ctx.textAlign = 'right'; ctx.fillText(minus(String(y)), left - 8, py + 4); }
    }

    // 경로: 옅은 점선과 경유점
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = css('--control');
    for (const pts of Object.values(state.layout.routes)) {
      ctx.beginPath();
      pts.forEach((p, i) => { const [px, py] = toPx(p.x, p.y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.fillStyle = css('--control');
    for (const pts of Object.values(state.layout.routes)) {
      for (const p of pts) { const [px, py] = toPx(p.x, p.y); ctx.beginPath(); ctx.arc(px, py, 2.5, 0, Math.PI * 2); ctx.fill(); }
    }

    // 주차 칸: 로봇 크기의 테두리 (0.24 x 0.20 m)
    for (const [name, slot] of Object.entries(state.layout.slots)) {
      const [px, py] = toPx(slot.x, slot.y);
      const target = state.robots.some((r) => busy(r) && r.task.action === 'park' && r.task.target === name);
      ctx.save();
      ctx.translate(px, py); ctx.rotate(-slot.yaw);
      ctx.strokeStyle = target ? css('--text') : css('--muted');
      ctx.strokeRect(-0.12 * view.s, -0.10 * view.s, 0.24 * view.s, 0.20 * view.s);
      ctx.restore();
      ctx.fillStyle = target ? css('--text') : css('--muted');
      ctx.font = mono; ctx.textAlign = 'center';
      ctx.fillText(name, px, py - 0.10 * view.s - 6);
    }

    // 자취, 목표, 로봇
    for (const r of state.robots) {
      const color = robotColor(r.name);
      const trail = state.trails[r.name] || [];
      if (trail.length > 1) {
        ctx.globalAlpha = 0.35; ctx.strokeStyle = color; ctx.lineWidth = 1.5;
        ctx.beginPath();
        trail.forEach((p, i) => { const [px, py] = toPx(p.x, p.y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
        ctx.stroke(); ctx.globalAlpha = 1; ctx.lineWidth = 1;
      }
      if (busy(r) && r.pose) {
        const g = r.task.goal;
        const [gx, gy] = toPx(g.x, g.y);
        const [rx, ry] = toPx(r.pose.x, r.pose.y);
        ctx.strokeStyle = color; ctx.setLineDash([3, 4]);
        ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(gx, gy); ctx.stroke(); ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(gx, gy, 6, 0, Math.PI * 2); ctx.stroke();
      }
    }
    for (const r of state.robots) {
      if (!r.pose) continue;
      const [px, py] = toPx(r.pose.x, r.pose.y);
      const rad = Math.max(7, 0.07 * view.s);
      ctx.fillStyle = robotColor(r.name);
      ctx.beginPath(); ctx.arc(px, py, rad, 0, Math.PI * 2); ctx.fill();
      // 앞쪽 표시
      ctx.strokeStyle = css('--surface'); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(r.pose.yaw) * rad, py - Math.sin(r.pose.yaw) * rad); ctx.stroke(); ctx.lineWidth = 1;
      if (r.name === state.selected) {
        ctx.strokeStyle = css('--accent');
        ctx.beginPath(); ctx.arc(px, py, rad + 4, 0, Math.PI * 2); ctx.stroke();
      }
      ctx.fillStyle = css('--text'); ctx.font = sans; ctx.textAlign = 'left';
      ctx.fillText(r.name, px + rad + 8, py + 4);
    }

    // 지도를 눌러 정한 이동 목표
    if (state.pending) {
      const [px, py] = toPx(state.pending.x, state.pending.y);
      ctx.strokeStyle = css('--accent');
      const sel = state.robots.find((r) => r.name === state.selected);
      if (sel && sel.pose) {
        const [rx, ry] = toPx(sel.pose.x, sel.pose.y);
        ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(px, py); ctx.stroke(); ctx.setLineDash([]);
      }
      ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(px - 3, py); ctx.lineTo(px + 3, py); ctx.moveTo(px, py - 3); ctx.lineTo(px, py + 3); ctx.stroke();
    }
  }

  canvas.addEventListener('click', (event) => {
    if (!view || !state.selected) return;
    const rect = canvas.getBoundingClientRect();
    const p = toWorld(event.clientX - rect.left, event.clientY - rect.top);
    if (p.x < view.x0 || p.y < view.y0 || p.x > view.x0 + view.W || p.y > view.y0 + view.H) return;
    const sel = state.robots.find((r) => r.name === state.selected);
    const yaw = sel && sel.pose ? Math.atan2(p.y - sel.pose.y, p.x - sel.pose.x) : 0;
    state.pending = { x: +p.x.toFixed(2), y: +p.y.toFixed(2), yaw: +yaw.toFixed(3) };
    renderAll();
  });

  function renderMapBar() {
    const bar = $('#map-bar');
    const sel = state.robots.find((r) => r.name === state.selected);
    const key = JSON.stringify([state.selected, state.pending, sel && busy(sel), state.connected, sel && sel.nav2_ready, locked()]);
    if (bar.dataset.key === key) return;
    bar.dataset.key = key;
    if (!sel) { bar.textContent = '로봇을 기다리는 중입니다.'; return; }
    if (!state.pending) {
      bar.textContent = `지도를 누르면 ${sel.name}의 이동 목표가 됩니다. 다른 로봇은 카드에서 이름을 눌러 고릅니다.`;
      return;
    }
    const p = state.pending;
    const blocked = locked() || !state.connected || !sel.nav2_ready || busy(sel);
    bar.innerHTML = `<span class="target"><b>${esc(sel.name)}</b> → <span class="mono">x ${fixed(p.x)} · y ${fixed(p.y)} · ${deg(p.yaw)}°</span></span>
      <span class="push"><button class="btn btn-quiet" type="button" data-a="clear">지우기</button>
      <button class="btn btn-go" type="button" data-a="go" ${blocked ? 'disabled title="지금은 보낼 수 없습니다"' : ''}>이동 보내기</button></span>`;
    bar.querySelector('[data-a="clear"]').onclick = () => { state.pending = null; renderAll(); };
    bar.querySelector('[data-a="go"]').onclick = async () => {
      const target = state.pending; const name = sel.name;
      state.pending = null; renderAll();
      await send(name, 'goto', target);
    };
  }

  // ---- 화면 전환 (#log) ----------------------------------------------------
  function route() {
    const view = location.hash === '#log' ? 'log' : 'board';
    $('#view-board').hidden = view !== 'board';
    $('#view-log').hidden = view !== 'log';
    document.querySelectorAll('.rail-item[data-view]').forEach((a) => {
      if (a.dataset.view === view) a.setAttribute('aria-current', 'page'); else a.removeAttribute('aria-current');
    });
    if (view === 'board') requestAnimationFrame(() => { layoutMap(); drawMap(); });
    renderAll();
  }

  function renderAll() {
    renderLock(); renderFresh(); renderFinding(); renderCards(); renderMapBar(); renderLegend();
    renderLog(); renderLayout(); drawMap();
  }

  // ---- 시작 ----------------------------------------------------------------
  $('#stop-all').addEventListener('click', async () => {
    try { await api('POST', '/api/stop'); } catch (error) { $('#finding').textContent = `전체 정지 실패: ${error.message}`; }
  });
  $('#lock').addEventListener('click', () => setPassword(''));
  $('#unlock').addEventListener('submit', async (event) => {
    event.preventDefault();
    const value = $('#password').value;
    $('#unlock-error').textContent = '';
    if (!value) { $('#unlock-error').textContent = '비밀번호를 넣으세요.'; return; }
    try {
      await api('POST', '/api/auth/check', null, value);
      $('#password').value = '';
      setPassword(value);
    } catch (error) {
      $('#unlock-error').textContent = error.message;
    }
  });
  window.addEventListener('hashchange', route);
  new ResizeObserver(() => { layoutMap(); drawMap(); }).observe(canvas);
  setInterval(renderFresh, 1000);

  Promise.all([api('GET', '/api/map').catch(() => null), api('GET', '/api/layout').catch(() => null),
    api('GET', '/api/auth').catch(() => ({ required: false }))])
    .then(([map, layout, auth]) => {
      state.auth.required = Boolean(auth && auth.required);
      state.map = map;
      if (layout) state.layout = layout;
      if (map) $('#map-meta').textContent = `pinky_factory.world · ${(map.width * map.resolution).toFixed(2)} × ${(map.height * map.resolution).toFixed(2)} m · 칸 ${Math.round(map.resolution * 100)} cm`;
      else $('#map-meta').textContent = '지도를 받지 못했습니다';
      if (document.fonts) document.fonts.ready.then(() => { layoutMap(); drawMap(); });
      layoutMap();
      route();
      connect();
    });
})();
