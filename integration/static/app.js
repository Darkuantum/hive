(() => {
  'use strict';

  const CONTROL_HZ = 10;
  const STATE_POLL_MS = 500;

  const KEY_MAP = { w: 'fwd', s: 'back', a: 'left', d: 'right', q: 'yawL', e: 'yawR' };
  const active = new Set();

  // Server is the source of truth for control_mode (in case another
  // browser tab/device changes it) -- this just mirrors it so the
  // manual send-loop and keyboard handlers know whether to act.
  let currentMode = 'manual';

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
    if (currentMode !== 'manual') return;
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

  // ---------------- manual control push loop ----------------
  // Runs regardless of mode -- harmless while in auto, since the
  // server only acts on this in 'manual' (see hardware.py) -- but we
  // skip it in auto anyway so the watchdog display isn't misleading.
  const stickReadout = document.getElementById('stick-readout');
  setInterval(() => {
    if (currentMode !== 'manual') return;
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

  // ---------------- manual power slider ----------------
  // Scales manual-mode thruster output (e.g. 50 -> half PWM range).
  // Enforced server-side in hardware.py, this just controls it. While
  // the user is actively dragging, the periodic state poll won't
  // overwrite the slider out from under their thumb (userIsAdjustingPower).
  const powerSlider = document.getElementById('power-slider');
  const powerValue = document.getElementById('power-value');
  let userIsAdjustingPower = false;

  function sendPower(pct) {
    fetch('/api/power', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ power: pct }),
    }).catch(() => {});
  }

  powerSlider.addEventListener('input', () => {
    userIsAdjustingPower = true;
    powerValue.textContent = powerSlider.value;
  });
  powerSlider.addEventListener('change', () => {
    sendPower(Number(powerSlider.value));
    userIsAdjustingPower = false;
  });

  // ---------------- manual / auto mode toggle ----------------
  const manualPanel = document.getElementById('manual-panel');
  const autoPanel = document.getElementById('auto-panel');
  const modeManualBtn = document.getElementById('mode-manual-btn');
  const modeAutoBtn = document.getElementById('mode-auto-btn');

  function applyModeToUI(mode) {
    currentMode = mode;
    modeManualBtn.classList.toggle('active', mode === 'manual');
    modeAutoBtn.classList.toggle('active', mode === 'auto');
    manualPanel.style.display = mode === 'manual' ? '' : 'none';
    autoPanel.style.display = mode === 'auto' ? '' : 'none';
    if (mode !== 'manual') {
      active.clear();
      updateButtonVisuals();
    }
  }

  function requestModeChange(mode) {
    // Optimistic UI update -- reverted on the next poll if the server
    // disagrees (e.g. request failed), so this never gets stuck lying.
    applyModeToUI(mode);
    fetch('/api/control_mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    }).catch(() => {});
  }

  modeManualBtn.addEventListener('click', () => requestModeChange('manual'));
  modeAutoBtn.addEventListener('click', () => requestModeChange('auto'));

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
  const badgeLeak = document.getElementById('badge-leak');
  const telemetryGrid = document.getElementById('telemetry-grid');
  const externalGrid = document.getElementById('external-grid');
  const poseGrid = document.getElementById('pose-grid');
  const tiltFill = document.getElementById('tilt-fill');
  const watchdogEl = document.getElementById('watchdog');
  const autoStickReadout = document.getElementById('auto-stick-readout');
  const yawDebugReadout = document.getElementById('yaw-debug-readout');
  const yawSaturatedHint = document.getElementById('yaw-saturated-hint');

  function renderParams(params) {
    const container = document.getElementById('param-list');
    if (!params || params.length === 0) {
      container.innerHTML = '<div class="param-empty">Verifying on next connect...</div>';
      return;
    }
    let html = '';
    for (const p of params) {
      const dotClass = p.ok ? 'good' : 'critical';
      const actualStr = p.actual != null ? p.actual : '--';
      const expectedStr = p.expected != null ? p.expected : '--';
      const errorStr = p.error ? ` <span style="color:var(--color-critical,#d44);font-size:11px;">(${p.error})</span>` : '';
      html += `<div class="param-row">` +
        `<span class="dot ${dotClass}" style="display:inline-block;width:8px;height:8px;border-radius:50;margin-right:6px;flex-shrink:0;"></span>` +
        `<span style="font-weight:600;">${p.name}</span>` +
        `<span style="color:var(--text-muted,#999);margin-left:6px;font-size:12px;">${p.description}</span>` +
        `<div style="font-size:12px;margin-top:2px;color:var(--text-muted,#999);padding-left:14px;">expected ${expectedStr}, got ${actualStr}${errorStr}</div>` +
        `</div>`;
    }
    container.innerHTML = html;
  }

  async function pollState() {
    try {
      const res = await fetch('/api/state');
      const data = await res.json();

      // Fetch parameter verification status in parallel
      fetch('/api/params').then(r => r.json()).then(params => {
        renderParams(params);
      }).catch(() => {});
      const m = data.mavlink || {};
      const cam = data.camera || {};
      const pose = data.pose;
      const ext = data.external || {};

      setBadge(badgeMavlink, m.connected, 'mavlink', m.error ? 'mavlink: ' + m.error : 'mavlink down');
      setBadge(badgeCamera, cam.connected, 'camera', cam.error ? 'camera: ' + cam.error : 'camera down');

      badgeArmed.classList.remove('good', 'warning', 'critical');
      badgeArmed.classList.add(m.armed ? 'good' : 'warning');
      badgeArmed.lastChild.textContent = m.armed ? 'armed' : 'disarmed';

      badgeMarker.classList.remove('good', 'warning', 'critical');
      badgeMarker.classList.add(cam.marker_detected ? 'good' : 'warning');
      badgeMarker.lastChild.textContent = cam.marker_detected ? 'marker locked' : 'no marker';

      badgeLeak.classList.remove('good', 'warning', 'critical');
      if (!ext.connected) {
        badgeLeak.classList.add('warning');
        badgeLeak.lastChild.textContent = 'leak sensor down';
      } else {
        badgeLeak.classList.add(ext.leak ? 'critical' : 'good');
        badgeLeak.lastChild.textContent = ext.leak ? 'LEAK!' : 'dry';
      }

      if (m.control_mode && m.control_mode !== currentMode) {
        applyModeToUI(m.control_mode);
      }

      if (!userIsAdjustingPower && m.manual_power != null) {
        const serverPct = Math.round(m.manual_power * 100);
        if (Number(powerSlider.value) !== serverPct) {
          powerSlider.value = serverPct;
          powerValue.textContent = serverPct;
        }
      }

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

      const accelKnown = m.accel_x != null && m.accel_y != null && m.accel_z != null;
      setField(
        telemetryGrid, 'accel',
        accelKnown ? `${m.accel_x.toFixed(2)} / ${m.accel_y.toFixed(2)} / ${m.accel_z.toFixed(2)}` : '--',
        !accelKnown
      );

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

      const auto = m.auto || {};
      const autoContainer = document.getElementById('auto-panel');
      setField(autoContainer, 'auto-state', auto.state || '--', !auto.state);
      setField(autoContainer, 'auto-controlling', auto.controlling ? 'yes' : 'no', auto.controlling == null);
      if (auto.stick) {
        autoStickReadout.textContent =
          `x=${auto.stick.x.toFixed(2)} y=${auto.stick.y.toFixed(2)} r=${auto.stick.r.toFixed(2)}`;
      }
      if (auto.yaw_debug) {
        yawDebugReadout.textContent =
          `yaw: cam=${auto.yaw_debug.yaw_cam_deg.toFixed(1)}\u00b0 ` +
          `body=${auto.yaw_debug.yaw_body_deg.toFixed(1)}\u00b0`;
        yawSaturatedHint.style.display = auto.yaw_debug.yaw_saturated ? 'block' : 'none';
      } else {
        yawDebugReadout.textContent = 'yaw: cam=-- body=--';
        yawSaturatedHint.style.display = 'none';
      }

      if (currentMode === 'manual') {
        const age = m.control_age_s;
        watchdogEl.textContent = age != null
          ? `last command: ${age.toFixed(1)}s ago`
          : 'last command: none yet';
        watchdogEl.classList.toggle('tripped', !!m.watchdog_tripped);
      } else {
        watchdogEl.textContent = 'auto mode active';
        watchdogEl.classList.remove('tripped');
      }

      setField(externalGrid, 'ext-roll', fmt(ext.roll_deg, 1, '°'), ext.roll_deg == null);
      setField(externalGrid, 'ext-pitch', fmt(ext.pitch_deg, 1, '°'), ext.pitch_deg == null);
      setField(externalGrid, 'ext-yaw', fmt(ext.yaw_deg, 1, '°'), ext.yaw_deg == null);
      setField(
        externalGrid, 'ext-leak',
        ext.connected ? (ext.leak ? 'LEAK!' : 'dry') : '--',
        !ext.connected
      );
      const extAccelKnown = ext.accel_x != null && ext.accel_y != null && ext.accel_z != null;
      setField(
        externalGrid, 'ext-accel',
        extAccelKnown ? `${ext.accel_x.toFixed(2)} / ${ext.accel_y.toFixed(2)} / ${ext.accel_z.toFixed(2)}` : '--',
        !extAccelKnown
      );
    } catch (err) {
      // transient network hiccup -- next poll will retry
    }
  }

  applyModeToUI(currentMode);
  setInterval(pollState, STATE_POLL_MS);
  pollState();
})();
