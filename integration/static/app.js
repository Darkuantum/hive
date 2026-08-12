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

  // ---------------- LED brightness slider ----------------
  const ledSlider = document.getElementById('led-slider');
  const ledValue = document.getElementById('led-value');
  ledSlider.addEventListener('input', () => {
    ledValue.textContent = ledSlider.value;
  });
  ledSlider.addEventListener('change', () => {
    fetch('/api/led', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ brightness: parseFloat(ledSlider.value) }),
    });
  });

  // ---------------- camera refocus button ----------------
  const refocusBtn = document.getElementById('refocus-btn');
  const refocusStatusEl = document.getElementById('refocus-status');
  refocusBtn.addEventListener('click', () => {
    refocusStatusEl.textContent = 'requesting...';
    fetch('/api/camera/refocus', { method: 'POST' }).catch(() => {
      refocusStatusEl.textContent = 'request failed';
    });
  });

  // ---------------- ArduSub flight mode ----------------
  // This is ArduSub's OWN flight mode (MANUAL/STABILIZE/etc), separate
  // from this app's manual/auto control_mode toggle above.
  const flightModeSelect = document.getElementById('flight-mode-select');
  const flightModeSetBtn = document.getElementById('flight-mode-set-btn');
  let userIsPickingFlightMode = false;

  flightModeSelect.addEventListener('focus', () => { userIsPickingFlightMode = true; });
  flightModeSelect.addEventListener('blur', () => { userIsPickingFlightMode = false; });

  flightModeSetBtn.addEventListener('click', () => {
    fetch('/api/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: flightModeSelect.value }),
    }).catch(() => {});
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

      if (refocusStatusEl) {
        if (cam.refocus_pending) {
          refocusStatusEl.textContent = 'refocusing...';
        } else if (cam.refocus) {
          const age = Date.now() / 1000 - cam.refocus.ts;
          if (age < 5) {
            refocusStatusEl.textContent = cam.refocus.ok ? 'focused' : 'did not converge';
          } else {
            refocusStatusEl.textContent = '';
          }
        }
      }

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
      setField(telemetryGrid, 'output_bank', m.output_bank || '--', !m.output_bank || m.output_bank === 'UNKNOWN');

      const accelKnown = m.accel_x != null && m.accel_y != null && m.accel_z != null;
      setField(
        telemetryGrid, 'accel',
        accelKnown ? `${m.accel_x.toFixed(2)} / ${m.accel_y.toFixed(2)} / ${m.accel_z.toFixed(2)}` : '--',
        !accelKnown
      );

      // Battery
      const bv = m.battery_voltage;
      const bc = m.battery_current;
      const br = m.battery_remaining;
      let batteryText = '--';
      if (bv != null && bv >= 0) {
        const parts = [bv.toFixed(1) + 'V'];
        if (bc != null && bc >= 0) parts.push(bc.toFixed(1) + 'A');
        if (br != null && br >= 0) parts.push(br + '%');
        batteryText = parts.join(' / ');
      }
      setField(telemetryGrid, 'battery', batteryText, bv == null || bv < 0);

      // STATUSTEXT
      setField(telemetryGrid, 'statustext', m.statustext || '--', !m.statustext);
      if (m.statustext && m.statustext_severity != null) {
        const stEl = telemetryGrid.querySelector('[data-field="statustext"]');
        if (stEl) {
          if (m.statustext_severity <= 3) {
            stEl.style.color = m.statustext_severity <= 2 ? 'var(--color-critical,#d44)' : '#e0a030';
          } else {
            stEl.style.color = '';
          }
        }
      }

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

      setField(
        externalGrid, 'ext-leak',
        ext.connected ? (ext.leak ? 'LEAK!' : 'dry') : '--',
        !ext.connected
      );
    } catch (err) {
      // transient network hiccup -- next poll will retry
    }
  }

  applyModeToUI(currentMode);
  setInterval(pollState, STATE_POLL_MS);
  pollState();

  // ================================================================
  // Calibration module
  // ================================================================
  (() => {
    'use strict';

    const STEP_POLL_MS = 500; // 2 Hz

    // --- DOM refs ---
    const axisSelect     = document.getElementById('cal-axis');
    const amplitudeInput = document.getElementById('cal-amplitude');
    const durationInput  = document.getElementById('cal-duration');
    const runNameInput   = document.getElementById('cal-run-name');
    const runBtn         = document.getElementById('cal-run-btn');
    const abortBtn       = document.getElementById('cal-abort-btn');
    const stepStatusEl   = document.getElementById('cal-step-status');
    const stepStatusText = document.getElementById('cal-step-status-text');
    const completionEl   = document.getElementById('cal-completion');
    const stepErrorEl    = document.getElementById('cal-step-error');

    const csvPathInput   = document.getElementById('cal-csv-path');
    const idAxisSelect   = document.getElementById('cal-id-axis');
    const identifyBtn    = document.getElementById('cal-identify-btn');
    const applyBtn       = document.getElementById('cal-apply-btn');
    const modelResultsEl = document.getElementById('cal-model-results');
    const mK  = document.getElementById('cal-m-K');
    const mTau = document.getElementById('cal-m-tau');
    const mL   = document.getElementById('cal-m-L');
    const mR2  = document.getElementById('cal-m-R2');
    const gKp  = document.getElementById('cal-g-Kp');
    const gKi  = document.getElementById('cal-g-Ki');
    const gKd  = document.getElementById('cal-g-Kd');
    const gTauCl = document.getElementById('cal-g-tau-cl');
    const idWarnEl  = document.getElementById('cal-id-warn');
    const idErrorEl = document.getElementById('cal-id-error');

    const runsBody = document.getElementById('cal-runs-body');

    // Keep the last identification gains for the apply button
    let lastGains = null;

    // --- Update run button label based on axis ---
    function updateRunLabel() {
      const axis = axisSelect.value;
      runBtn.textContent = 'Run ' + axis.charAt(0).toUpperCase() + axis.slice(1) + ' Step';
    }
    axisSelect.addEventListener('change', updateRunLabel);

    // --- Step runner: run ---
    runBtn.addEventListener('click', async () => {
      const amplitude = parseFloat(amplitudeInput.value);
      const stepDuration = parseFloat(durationInput.value);
      if (isNaN(amplitude) || amplitude < -0.5 || amplitude > 0.5) {
        stepErrorEl.textContent = 'Amplitude must be between -0.5 and 0.5.';
        return;
      }
      if (isNaN(stepDuration) || stepDuration < 1) {
        stepErrorEl.textContent = 'Step duration must be at least 1s.';
        return;
      }
      stepErrorEl.textContent = '';
      completionEl.style.display = 'none';
      runBtn.disabled = true;
      abortBtn.disabled = false;

      try {
        const res = await fetch('/api/calibrate/step/run', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            axis: axisSelect.value,
            amplitude: amplitude,
            step_duration: stepDuration,
          }),
        });
        const data = await res.json();
        if (!res.ok) {
          stepErrorEl.textContent = data.error || 'Failed to start step run.';
          runBtn.disabled = false;
          abortBtn.disabled = true;
          return;
        }
        // Start polling status
        startStepPoll();
      } catch (err) {
        stepErrorEl.textContent = 'Network error starting step run.';
        runBtn.disabled = false;
        abortBtn.disabled = true;
      }
    });

    // --- Step runner: abort ---
    abortBtn.addEventListener('click', async () => {
      abortBtn.disabled = true;
      try {
        await fetch('/api/calibrate/step/abort', { method: 'POST' });
      } catch (err) {
        stepErrorEl.textContent = 'Network error sending abort.';
      }
    });

    // --- Step status polling ---
    let stepPollTimer = null;

    function startStepPoll() {
      if (stepPollTimer) return;
      pollStepStatus();
      stepPollTimer = setInterval(pollStepStatus, STEP_POLL_MS);
    }

    function stopStepPoll() {
      if (stepPollTimer) {
        clearInterval(stepPollTimer);
        stepPollTimer = null;
      }
    }

    async function pollStepStatus() {
      try {
        const res = await fetch('/api/calibrate/step/status');
        const data = await res.json();
        const status = data.status || 'idle';
        stepStatusEl.setAttribute('data-status', status);

        if (status === 'running') {
          stepStatusText.textContent = 'Running\u2026';
          runBtn.disabled = true;
          abortBtn.disabled = false;
        } else if (status === 'done') {
          stepStatusText.textContent = 'Done';
          runBtn.disabled = false;
          abortBtn.disabled = true;
          stopStepPoll();
          // Show completion details
          const s = data.summary || {};
          let html = '';
          if (s.run_id)        html += 'Run ID: <code>' + esc(s.run_id) + '</code><br>';
          if (s.csv_path)      html += 'CSV: <code>' + esc(s.csv_path) + '</code><br>';
          if (s.video_path)    html += 'Video: <code>' + esc(s.video_path) + '</code><br>';
          if (s.duration != null) html += 'Duration: ' + s.duration.toFixed(1) + 's';
          if (html) {
            completionEl.innerHTML = html;
            completionEl.style.display = 'block';
            // Auto-populate CSV path in Panel 2
            if (s.csv_path) {
              csvPathInput.value = s.csv_path;
            }
          }
          // Refresh runs table
          loadRuns();
        } else if (status === 'error') {
          stepStatusText.textContent = 'Error';
          runBtn.disabled = false;
          abortBtn.disabled = true;
          stopStepPoll();
          stepErrorEl.textContent = data.error || 'Step run failed.';
        } else {
          stepStatusText.textContent = 'Idle';
          runBtn.disabled = false;
          abortBtn.disabled = true;
          stopStepPoll();
        }
      } catch (err) {
        // transient — next poll will retry
      }
    }

    // --- Identify ---
    identifyBtn.addEventListener('click', async () => {
      const csvPath = csvPathInput.value.trim();
      if (!csvPath) {
        idErrorEl.textContent = 'Enter a CSV path (click a row in Past Runs to fill).';
        return;
      }
      idErrorEl.textContent = '';
      idWarnEl.style.display = 'none';
      modelResultsEl.style.display = 'none';
      lastGains = null;
      applyBtn.disabled = true;
      identifyBtn.disabled = true;

      try {
        const res = await fetch('/api/calibrate/identify', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ csv_path: csvPath, axis: idAxisSelect.value }),
        });
        const data = await res.json();
        identifyBtn.disabled = false;

        if (!res.ok) {
          idErrorEl.textContent = data.error || 'Identification failed.';
          return;
        }

        // Render model
        const model = data.model || {};
        mK.textContent   = model.K != null ? model.K.toPrecision(4) : '--';
        mTau.textContent = model.tau != null ? model.tau.toPrecision(4) : '--';
        mL.textContent   = model.L != null ? model.L.toPrecision(4) : '--';

        // Color-code R²
        const r2 = model.R_squared;
        if (r2 != null) {
          mR2.textContent = r2.toFixed(4);
          mR2.className = 'value';
          if (r2 >= 0.9) {
            mR2.classList.add('cal-r2-good');
          } else if (r2 >= 0.7) {
            mR2.classList.add('cal-r2-ok');
          } else {
            mR2.classList.add('cal-r2-bad');
          }
        } else {
          mR2.textContent = '--';
          mR2.className = 'value na';
        }

        // Render tuning
        const tuning = data.tuning || {};
        gKp.textContent  = tuning.Kp != null ? tuning.Kp.toPrecision(4) : '--';
        gKi.textContent  = tuning.Ki != null ? tuning.Ki.toPrecision(4) : '--';
        gKd.textContent  = tuning.Kd != null ? tuning.Kd.toPrecision(4) : '--';
        gTauCl.textContent = tuning.tau_cl != null ? tuning.tau_cl.toPrecision(4) : '--';

        modelResultsEl.style.display = 'block';

        // Store gains for apply
        lastGains = data.gains || tuning;
        applyBtn.disabled = false;

        // Warnings
        if (data.warnings && data.warnings.length) {
          idWarnEl.textContent = data.warnings.join(' ');
          idWarnEl.style.display = 'block';
        }
      } catch (err) {
        idErrorEl.textContent = 'Network error during identification.';
        identifyBtn.disabled = false;
      }
    });

    // --- Apply gains ---
    applyBtn.addEventListener('click', async () => {
      applyBtn.disabled = true;
      idErrorEl.textContent = '';
      try {
        const res = await fetch('/api/calibrate/gains/save', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(lastGains || {}),
        });
        const data = await res.json();
        if (!res.ok) {
          idErrorEl.textContent = data.error || 'Failed to save gains.';
          applyBtn.disabled = false;
          return;
        }
        // Brief confirmation
        applyBtn.textContent = 'Saved';
        setTimeout(() => {
          applyBtn.textContent = 'Apply gains';
          applyBtn.disabled = false;
        }, 2000);
      } catch (err) {
        idErrorEl.textContent = 'Network error saving gains.';
        applyBtn.disabled = false;
      }
    });

    // --- Past runs table ---
    async function loadRuns() {
      try {
        const res = await fetch('/api/calibrate/runs');
        const data = await res.json();
        const runs = data.runs || [];

        if (runs.length === 0) {
          runsBody.innerHTML = '<div class="cal-runs-empty">No runs recorded yet.</div>';
          return;
        }

        let html = '<table class="cal-runs-table"><thead><tr>' +
          '<th>Run</th><th>CSV</th><th>Video</th><th>Size</th>' +
          '</tr></thead><tbody>';

        for (const r of runs) {
          const csvIcon  = r.has_csv  ? '<span class="cal-icon cal-icon-present"></span>' : '<span class="cal-icon cal-icon-missing"></span>';
          const vidIcon  = r.has_video ? '<span class="cal-icon cal-icon-present"></span>' : '<span class="cal-icon cal-icon-missing"></span>';
          let sizeStr = '';
          const parts = [];
          if (r.csv_size) parts.push(fmtSize(r.csv_size));
          if (r.video_size) parts.push(fmtSize(r.video_size));
          if (parts.length) sizeStr = parts.join(' / ');

          html += '<tr data-run-id="' + esc(r.run_id) + '">' +
            '<td>' + esc(r.run_id) + '</td>' +
            '<td>' + csvIcon + (r.has_csv ? 'yes' : 'no') + '</td>' +
            '<td>' + vidIcon + (r.has_video ? 'yes' : 'no') + '</td>' +
            '<td>' + (sizeStr || '--') + '</td>' +
            '</tr>';
        }

        html += '</tbody></table>';
        runsBody.innerHTML = html;

        // Click handler: populate CSV path
        runsBody.querySelectorAll('tr[data-run-id]').forEach((tr) => {
          tr.addEventListener('click', () => {
            const runId = tr.getAttribute('data-run-id');
            // Construct plausible CSV path from run_id
            // The server will have the actual path, but we can guess based on convention
            // The identify endpoint needs the full csv_path, so we store the run_id
            // and let the user confirm or adjust
            csvPathInput.value = runId + '.csv';
            csvPathInput.focus();
          });
        });
      } catch (err) {
        runsBody.innerHTML = '<div class="cal-runs-empty">Failed to load runs.</div>';
      }
    }

    function fmtSize(bytes) {
      if (bytes < 1024) return bytes + ' B';
      if (bytes < 1048576) return (bytes / 1024).toFixed(1) + ' KB';
      return (bytes / 1048576).toFixed(1) + ' MB';
    }

    function esc(s) {
      const d = document.createElement('div');
      d.textContent = s;
      return d.innerHTML;
    }

    // --- Init ---
    updateRunLabel();
    loadRuns();
    // Also poll step status on load in case a step was already running
    pollStepStatus();
  })();
})();
