// L-Band Watch - reads the overnight recorder's log via /api/status.
// No external resources: the page must render even with no network.
(function () {
  const $ = (id) => document.getElementById(id);
  const fmt = (n) => (n === null || n === undefined ? '–' : n.toLocaleString());

  function setBadge(state, text) {
    $('dot').className = 'dot ' + state;
    $('badge-text').textContent = text;
  }

  function drawSpectrum(bins) {
    const c = $('spec'), ctx = c.getContext('2d');
    const w = (c.width = c.clientWidth * (window.devicePixelRatio || 1));
    const h = (c.height = 190 * (window.devicePixelRatio || 1));
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(document.documentElement);
    const line = css.getPropertyValue('--line').trim() || '#26313d';
    const accent = css.getPropertyValue('--accent').trim() || '#6fa8dc';
    ctx.strokeStyle = line; ctx.lineWidth = 1;
    for (let i = 1; i < 5; i++) {
      const y = (h / 5) * i;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
    if (!bins) return;
    const f = Object.keys(bins).map(Number).sort((a, b) => a - b);
    if (!f.length) return;
    const v = f.map((k) => bins[String(k)]);
    const lo = Math.min(...v), hi = Math.max(...v), rng = hi - lo || 1;
    ctx.beginPath();
    f.forEach((k, i) => {
      const x = (i / (f.length - 1)) * w;
      const y = h - ((v[i] - lo) / rng) * (h - 12) - 6;
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.strokeStyle = accent; ctx.lineWidth = 1.6 * (window.devicePixelRatio || 1);
    ctx.stroke();
    // shade the Iridium simplex band we are watching
    const i0 = f.findIndex((k) => k >= 1626), i1 = f.findIndex((k) => k >= 1627);
    if (i0 >= 0 && i1 > i0) {
      ctx.fillStyle = 'rgba(63,191,159,.16)';
      ctx.fillRect((i0 / (f.length - 1)) * w, 0, ((i1 - i0) / (f.length - 1)) * w, h);
    }
    $('spec-axis').innerHTML = `<span>${f[0]} MHz</span><span>Iridium 1626 MHz</span><span>${f[f.length - 1]} MHz</span>`;
  }

  function drawChannels(chan) {
    const el = $('chan');
    const e = Object.entries(chan || {}).sort((a, b) => b[1] - a[1]).slice(0, 12);
    if (!e.length) { el.innerHTML = '<div class="note">no bursts recorded yet</div>'; return; }
    const max = e[0][1];
    el.innerHTML = e.map(([f, n]) =>
      `<div class="row"><div class="f">${f}</div>
       <div class="bar"><div class="fill" style="width:${(n / max) * 100}%"></div></div>
       <div class="n">${n}</div></div>`).join('');
  }

  function drawGnss(g) {
    const el = $('gnss');
    if (!g) { el.innerHTML = '<div class="note">no GNSS check yet</div>'; return; }
    if (g.error) { el.innerHTML = `<div class="note">${g.error}</div>`; return; }
    const acq = (g.acquired || []).length;
    const rows = [
      ['Crossings this check', acq],
      ['Best detection metric', g.max_metric != null ? g.max_metric.toFixed(2) : '–'],

      ['ADC level (σ)', g.std != null ? g.std : '–'],
    ];
    let html = rows.map(([k, v]) =>
      `<div class="line"><span class="lab">${k}</span><span class="val">${v}</span></div>`).join('');
    html += (g.best || []).slice(0, 3).map((b) =>
      `<div class="line"><span class="lab">PRN ${b.prn}</span><span class="val">${b.m.toFixed(2)} @ ${b.dopp.toFixed(0)} Hz</span></div>`).join('');
    if (!acq) html += '<div class="note" style="display:block;margin-top:8px">No PRN above threshold in this check. GPS here is intermittent and near the detection floor; confirmation comes from the Doppler ramp across many checks, shown in the tile above.</div>';
    el.innerHTML = html;
  }

  function drawBursts(list) {
    const tb = document.querySelector('#bursts tbody');
    if (!list || !list.length) { tb.innerHTML = '<tr><td colspan="10">no bursts yet</td></tr>'; return; }
    tb.innerHTML = list.map((b) => {
      const t = (b.utc || '').replace('T', ' ').replace('+00:00', '');
      const lock = b.lock >= 0.25 ? `<span class="hi">${b.lock.toFixed(3)}</span>` : b.lock.toFixed(3);
      const sync = b.sync ? '<span class="pill yes">SYNC</span>' : '<span class="pill no">—</span>';
      return `<tr><td>${t}</td><td>${b.cycle}</td><td>${b.t}</td><td>${b.dur_ms}</td>
        <td>${b.freq_mhz.toFixed(4)}</td><td>${lock}</td><td>${b.x4_db}</td>
        <td>${b.nsym}</td><td>${sync}</td><td class="sym">${(b.sym || '').slice(0, 34)}</td></tr>`;
    }).join('');
  }

  async function tick() {
    let d;
    try {
      const r = await fetch('/api/status', { cache: 'no-store' });
      d = await r.json();
    } catch (e) {
      setBadge('off', 'server unreachable');
      return;
    }
    if (d.recorder !== 'running') {
      setBadge('off', 'recorder not running');
      $('foot').textContent = d.hint || '';
      return;
    }
    const age = (Date.now() - Date.parse(d.last_utc)) / 1000;
    setBadge(age < 180 ? 'live' : 'stale',
      age < 180 ? `live · last cycle ${Math.round(age)}s ago` : `stale · ${Math.round(age / 60)} min since last cycle`);

    $('t-cycles').textContent = fmt(d.cycles);
    $('t-bursts').textContent = fmt(d.totals.bursts);
    $('t-locked').textContent = fmt(d.totals.frames);
    $('t-sync').textContent = fmt(d.totals.frame_sync);
    $('t-snip').textContent = fmt(d.snippets);
    // confirmed = satellites whose Doppler swept like a real pass, not merely
    // PRNs that crossed the threshold once (which noise does)
    const V = d.gnss_verdict || {};
    const conf = V.confirmed || 0;
    $('t-gnss').textContent = conf;
    $('t-gnss').className = 'v ' + (conf ? 'ok' : 'bad');
    $('t-gnss-s').textContent = conf
      ? 'confirmed by Doppler ramp: PRN ' + (V.confirmed_prns || []).join(', ')
      : (V.crossings ? V.crossings + ' crossings, none confirmed' : 'nothing above threshold');
    if (d.first_utc) {
      const mins = Math.round((Date.parse(d.last_utc) - Date.parse(d.first_utc)) / 60000);
      $('t-span').textContent = `over ${mins} min`;
    }
    drawSpectrum((d.sweep || {}).bins);
    drawChannels(d.channels);
    drawGnss(d.gnss);
    drawBursts(d.recent_bursts);
    $('foot').textContent = `${d.receiver} · ${d.band} · updated ${new Date().toLocaleTimeString()}`;
  }

  tick();
  setInterval(tick, 5000);
  window.addEventListener('resize', tick);
})();
