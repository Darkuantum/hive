(() => {
  'use strict';

  const CONTROL_HZ = 10;
  const STATE_POLL_MS = 500;

  const KEY_MAP = { w: 'fwd', s: 'back', a: 'left', d: 'right', q: 'yawL', e: 'yawR' };
  const active = new Set();

  function computeSticks() {
    let x = 0, y = 0, r = 0;
    if (active.has('fwd')) x += 1;
    if (active.has('back')) x -= 1;
    if (active.has('right')) y += 1;
    if (active.has('left')) y -= 1;
    if (active.has('yawR')) r += 1;
    if (active.has('yawL')) r -= 1;
    return { x, y, r };
  }

  function updateButtonVisuals() {
    document.querySelectorAll('#dpad button, #yawpad button').forEach((btn) => {
      const name = KEY_MAP[btn.dataset.key];
      btn.classList.toggle('active', active.has(name));
    });
  }

  function press(key) {
    const name = KEY_MAP[key];
    if (!name) return;
    active.add(name);
    updateButtonVisuals();
  }

  function release(key) {
    const name = KEY_MAP[key];
    if (!name) return;
    active.delete(name);
    updateButtonVisuals();
  }

  // ---------------- keyboard ----------------
  window.addEventListener('keydown', (e) => {
    const key = e.key.toLowerCase();
    if (!KEY_MAP[key] || e.repeat) return;
    press(key);
  });
  window.addEventListener('keyup', (e) => release(e.key.toLowerCase()));
  window.addEventListener('blur', () => { active.clear(); updateButtonVisuals(); });

  // ---------------- on-screen buttons (mouse + touch) ----------------
  document.querySelectorAll('#dpad button, #yawpad button').forEach((btn) => {
    const key = btn.dataset.key;
    const start = (e) => { e.preventDefault(); press(key); };
    const end = (e) => { e.preventDefault(); release(key); };
    btn.addEventListener('mousedown', start);
    btn.addEventListener('touchstart', start, { passive: false });
    btn.addEventListener('mouseup', end);
    btn.addEventListener('mouseleave', end);
    btn.addEventListener('touchend', end);
    btn.addEventListener('touchcancel', end);
  });

  // ---------------- control push loop ----------------
  const stickReadout = document.getElementById('stick-readout');
  setInterval(() => {
    const { x, y, r } = computeSticks();
    stickReadout.textContent = `x=${x.toFixed(2)} y=${y.toFixed(2)} r=${r.toFixed(2)}`;
    fetch('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, r }),
    }).catch(() => {});
  }, 1000 / CONTROL_HZ);

  // ---------------- arm / disarm ----------------
  document.getElementById('arm-btn').addEventListener('click', () => {
    fetch('/api/arm', { method: 'POST' }).catch(() => {});
  });
  document.getElementById('disarm-btn').addEventListener('click', () => {
    active.clear();
    updateButtonVisuals();
    fetch('/api/disarm', { method: 'POST' }).catch(() => {});
  });

  // ---------------- state polling ----------------
  function setBadge(el, ok, textOk, textBad) {
    el.classList.remove('good', 'warning', 'critical');
    el.classList.add(ok ? 'good' : 'critical');
    el.lastChild.textContent = ok ? textOk : textBad;
  }

  function fmt(value, digits, suffix) {
    if (value === null || value === undefined || Number.isNaN(value)) return '--';
    return value.toFixed(digits) + (suffix || '');
  }

  function setField(container, field, text, isNa) {
    const el = container.querySelector(`[data-field="${field}"]`);
    if (!el) return;
    el.textContent = text;
    el.classList.toggle('na', !!isNa);
  }

  const badgeMavlink = document.getElementById('badge-mavlink');
  const badgeCamera = document.getElementById('badge-camera');
  const badgeArmed = document.getElementById('badge-armed');
  const badgeMarker = document.getElementById('badge-marker');
  const telemetryGrid = document.getElementById('telemetry-grid');
  const poseGrid = document.getElementById('pose-grid');
  const tiltFill = document.getElementById('tilt-fill');
  const watchdogEl = document.getElementById('watchdog');

  async function pollState() {
    try {
      const res = await fetch('/api/state');
      const data = await res.json();
      const m = data.mavlink || {};
      const cam = data.camera || {};
      const pose = data.pose;

      setBadge(badgeMavlink, m.connected, 'mavlink', m.error ? 'mavlink: ' + m.error : 'mavlink down');
      setBadge(badgeCamera, cam.connected, 'camera', cam.error ? 'camera: ' + cam.error : 'camera down');

      badgeArmed.classList.remove('good', 'warning', 'critical');
      badgeArmed.classList.add(m.armed ? 'good' : 'warning');
      badgeArmed.lastChild.textContent = m.armed ? 'armed' : 'disarmed';

      badgeMarker.classList.remove('good', 'warning', 'critical');
      badgeMarker.classList.add(cam.marker_detected ? 'good' : 'warning');
      badgeMarker.lastChild.textContent = cam.marker_detected ? 'marker locked' : 'no marker';

      setField(telemetryGrid, 'roll_deg', fmt(m.roll_deg, 1, '°'), m.roll_deg == null);
      setField(telemetryGrid, 'pitch_deg', fmt(m.pitch_deg, 1, '°'), m.pitch_deg == null);
      setField(telemetryGrid, 'yaw_deg', fmt(m.yaw_deg, 1, '°'), m.yaw_deg == null);
      setField(telemetryGrid, 'depth', fmt(m.depth, 2, ' m'), m.depth == null);
      setField(telemetryGrid, 'pressure_abs', fmt(m.pressure_abs, 1, ' hPa'), m.pressure_abs == null);
      setField(telemetryGrid, 'pressure_int', fmt(m.pressure_int, 1, ' hPa'), m.pressure_int == null);
      setField(telemetryGrid, 'mode', m.mode || '--', !m.mode);
      const servos = ['servo1', 'servo2', 'servo3', 'servo4'].map((k) => m[k]);
      const anyServo = servos.some((v) => v != null);
      setField(telemetryGrid, 'servo', anyServo ? servos.map((v) => v ?? '-').join(' / ') : '--', !anyServo);

      const tilt = m.tilt_deg;
      if (tilt != null) {
        const pct = Math.min(100, (tilt / 30) * 100); // 30deg == full track, matches stability_tolerance context
        tiltFill.style.width = pct + '%';
        tiltFill.classList.toggle('warning', tilt > 10 && tilt <= 20);
        tiltFill.classList.toggle('critical', tilt > 20);
      } else {
        tiltFill.style.width = '0%';
      }

      if (pose) {
        setField(poseGrid, 'x', fmt(pose.x, 3, ' m'));
        setField(poseGrid, 'y', fmt(pose.y, 3, ' m'));
        setField(poseGrid, 'z', fmt(pose.z, 3, ' m'));
        setField(poseGrid, 'yaw', fmt(pose.yaw * 180 / Math.PI, 1, '°'));
      } else {
        ['x', 'y', 'z', 'yaw'].forEach((f) => setField(poseGrid, f, '--', true));
      }

      const age = m.control_age_s;
      watchdogEl.textContent = age != null
        ? `last command: ${age.toFixed(1)}s ago`
        : 'last command: none yet';
      watchdogEl.classList.toggle('tripped', !!m.watchdog_tripped);
    } catch (err) {
      // transient network hiccup -- next poll will retry
    }
  }

  setInterval(pollState, STATE_POLL_MS);
  pollState();
})();
