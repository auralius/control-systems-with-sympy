# [HTML/JavaScript] Interactive PID-Controlled Second-Order Plant

This section demonstrates the step response of a second-order plant controlled using a PID controller. The goal is to observe how the controller gains $K_p$, $K_i$, and $K_d$ affect the closed-loop response.

The plant is modeled as a standard second-order system with these parameters:

- $\zeta$ is the damping ratio,
- $\omega_n$ is the natural frequency.

The notebook then plots the response $c(t)$ to a unit-step input. 

The notebook also calculates common time-domain response characteristics, including:

- rise time,
- peak time,
- maximum overshoot,
- settling time using the 5% band,
- settling time using the 2% band,
- closed-loop pole locations.

```{raw} html
<div id="pid-so-widget" style="max-width:900px;margin:auto;">
  <h3>Interactive PID-Controlled Second-Order Plant</h3>

  <div style="display:grid;grid-template-columns:120px 1fr 80px;gap:8px;align-items:center;">
    <label>Kp</label><input id="kp" type="range" min="0" max="50" step="0.1" value="10"><span id="kpv"></span>
    <label>Ki</label><input id="ki" type="range" min="0" max="50" step="0.1" value="3"><span id="kiv"></span>
    <label>Kd</label><input id="kd" type="range" min="0" max="20" step="0.1" value="1"><span id="kdv"></span>
    <label>zeta</label><input id="zeta" type="number" step="0.05" value="1"><span></span>
    <label>wn</label><input id="wn" type="number" step="0.05" value="1"><span></span>
    <label>t final</label><input id="tfinal" type="number" step="1" value="20"><span></span>
  </div>

  <div id="pidplot" style="height:480px;"></div>
  <pre id="pidinfo" style="background:#f7f7f7;padding:10px;overflow:auto;"></pre>
</div>

<script src="https://cdn.jsdelivr.net/npm/mathjs@13.2.0/lib/browser/math.js"></script>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>

<script>
(function () {
  const $ = (id) => document.getElementById(id);

  function C(x) {
    return math.complex(x);
  }

  function cAdd(...xs) {
    return xs.reduce((a, b) => math.add(a, b), C(0));
  }

  function cMul(...xs) {
    return xs.reduce((a, b) => math.multiply(a, b), C(1));
  }

  function cDiv(a, b) {
    return math.divide(a, b);
  }

  function cSub(a, b) {
    return math.subtract(a, b);
  }

  function cExp(z) {
    return math.exp(z);
  }

  function real(z) {
    return math.re(z);
  }

  function rootsDistinct(values, tol = 1e-7) {
    for (let i = 0; i < values.length; i++) {
      for (let j = i + 1; j < values.length; j++) {
        if (math.abs(math.subtract(values[i], values[j])) <= tol) {
          return false;
        }
      }
    }
    return true;
  }

  function firstCrossing(t, y, level) {
    for (let k = 0; k < t.length - 1; k++) {
      const f1 = y[k] - level;
      const f2 = y[k + 1] - level;

      if (f1 === 0) return t[k];

      if (f1 * f2 < 0) {
        const ratio = Math.abs(f1) / (Math.abs(f1) + Math.abs(f2));
        return t[k] + ratio * (t[k + 1] - t[k]);
      }
    }
    return null;
  }

  function settlingTime(t, y, yFinal, tol) {
    const lower = (1 - tol) * yFinal;
    const upper = (1 + tol) * yFinal;

    for (let k = 0; k < t.length; k++) {
      let ok = true;
      for (let j = k; j < t.length; j++) {
        if (y[j] < lower || y[j] > upper) {
          ok = false;
          break;
        }
      }
      if (ok) return t[k];
    }
    return null;
  }

  function closedFormPID(t, Kp, Ki, Kd, a) {
    const a1 = a[0], a2 = a[1], a3 = a[2];

    const cInf = cDiv(C(Ki), cMul(a1, a2, a3));

    const termA3 = cDiv(
      cMul(
        C(-1),
        cAdd(cMul(C(Kd), a3, a3), C(Ki), cMul(C(-Kp), a3)),
        cExp(cMul(C(-t), a3))
      ),
      cMul(a3, cSub(a1, a3), cSub(a2, a3))
    );

    const termA2 = cDiv(
      cMul(
        cAdd(cMul(C(Kd), a2, a2), C(Ki), cMul(C(-Kp), a2)),
        cExp(cMul(C(-t), a2))
      ),
      cMul(a2, cSub(a1, a2), cSub(a2, a3))
    );

    const termA1 = cDiv(
      cMul(
        C(-1),
        cAdd(cMul(C(Kd), a1, a1), C(Ki), cMul(C(-Kp), a1)),
        cExp(cMul(C(-t), a1))
      ),
      cMul(a1, cSub(a1, a2), cSub(a1, a3))
    );

    return real(cAdd(cInf, termA3, termA2, termA1));
  }

  function closedFormPD(t, Kp, Kd, b) {
    const b1 = b[0], b2 = b[1];

    const cInf = cDiv(C(Kp), cMul(b1, b2));

    const termB1 = cDiv(
      cMul(cSub(C(Kp), cMul(C(Kd), b1)), cExp(cMul(C(-t), b1))),
      cMul(b1, cSub(b1, b2))
    );

    const termB2 = cDiv(
      cMul(cSub(C(Kp), cMul(C(Kd), b2)), cExp(cMul(C(-t), b2))),
      cMul(b2, cSub(b2, b1))
    );

    return real(cAdd(cInf, termB1, termB2));
  }

  function update() {
    const Kp = parseFloat($("kp").value);
    const Ki = parseFloat($("ki").value);
    const Kd = parseFloat($("kd").value);
    const zeta = parseFloat($("zeta").value);
    const wn = parseFloat($("wn").value);
    const tFinal = parseFloat($("tfinal").value);

    $("kpv").textContent = Kp.toFixed(1);
    $("kiv").textContent = Ki.toFixed(1);
    $("kdv").textContent = Kd.toFixed(1);

    if (wn <= 0 || zeta <= 0 || Ki < 0 || tFinal <= 0) {
      $("pidinfo").textContent = "Use wn > 0, zeta > 0, Ki >= 0, and t final > 0.";
      return;
    }

    const n = 1200;
    const t = [];
    const y = [];

    let poles, constants, den, cInf, controllerType;

    if (Ki > 0) {
      controllerType = Kd > 0 ? "PID" : "PI";

      const b2 = Kd + 2 * zeta * wn;
      const b1 = Kp + wn * wn;
      const b0 = Ki;

      den = [1, b2, b1, b0];

      // math.polynomialRoot uses coefficients from constant to highest order:
      // b0 + b1*s + b2*s^2 + 1*s^3
      poles = math.polynomialRoot(b0, b1, b2, 1);
      constants = poles.map(p => math.multiply(C(-1), p));

      if (!rootsDistinct(constants)) {
        $("pidinfo").textContent = "Poles are repeated or too close. The partial-fraction equation assumes distinct poles.";
        return;
      }

      cInf = real(cDiv(C(Ki), cMul(constants[0], constants[1], constants[2])));

      for (let i = 0; i < n; i++) {
        const ti = (tFinal * i) / (n - 1);
        t.push(ti);
        y.push(closedFormPID(ti, Kp, Ki, Kd, constants));
      }

    } else {
      controllerType = Kd > 0 ? "PD" : "P";

      const b1 = Kd + 2 * zeta * wn;
      const b0 = Kp + wn * wn;

      den = [1, b1, b0];

      // b0 + b1*s + 1*s^2
      poles = math.polynomialRoot(b0, b1, 1);
      constants = poles.map(p => math.multiply(C(-1), p));

      if (!rootsDistinct(constants)) {
        $("pidinfo").textContent = "Poles are repeated or too close. The PD/P equation assumes distinct poles.";
        return;
      }

      cInf = real(cDiv(C(Kp), cMul(constants[0], constants[1])));

      for (let i = 0; i < n; i++) {
        const ti = (tFinal * i) / (n - 1);
        t.push(ti);
        y.push(closedFormPD(ti, Kp, Kd, constants));
      }
    }

    const tr = firstCrossing(t, y, 1.0);
    const t95 = firstCrossing(t, y, 0.95 * cInf);
    const t98 = firstCrossing(t, y, 0.98 * cInf);
    const Ts5 = settlingTime(t, y, cInf, 0.05);
    const Ts2 = settlingTime(t, y, cInf, 0.02);

    let imax = 0;
    for (let i = 1; i < y.length; i++) {
      if (y[i] > y[imax]) imax = i;
    }

    const cPeak = y[imax];
    const tPeak = t[imax];
    const Mp = cPeak - cInf;
    const OS = Math.abs(cInf) > 1e-12 ? 100 * Mp / cInf : NaN;

    const traces = [
      {
        x: t,
        y: y,
        mode: "lines",
        name: "c(t)"
      },
      {
        x: [0, tFinal],
        y: [1, 1],
        mode: "lines",
        name: "unit reference",
        line: { dash: "dash" }
      },
      {
        x: [0, tFinal],
        y: [1.05 * cInf, 1.05 * cInf],
        mode: "lines",
        name: "+5%",
        line: { dash: "dash" }
      },
      {
        x: [0, tFinal],
        y: [0.95 * cInf, 0.95 * cInf],
        mode: "lines",
        name: "-5%",
        line: { dash: "dash" }
      },
      {
        x: [0, tFinal],
        y: [1.02 * cInf, 1.02 * cInf],
        mode: "lines",
        name: "+2%",
        line: { dash: "dot" }
      },
      {
        x: [0, tFinal],
        y: [0.98 * cInf, 0.98 * cInf],
        mode: "lines",
        name: "-2%",
        line: { dash: "dot" }
      },
      {
        x: [tPeak],
        y: [cPeak],
        mode: "markers",
        name: "peak"
      }
    ];

    if (tr !== null) {
      traces.push({ x: [tr], y: [1], mode: "markers", name: "tr" });
    }

    Plotly.react("pidplot", traces, {
      title: controllerType + "-controlled second-order plant",
      xaxis: { title: "Time, t" },
      yaxis: { title: "Response, c(t)" },
      margin: { t: 50, r: 20, b: 50, l: 60 }
    }, { responsive: true });

    const stable = poles.every(p => math.re(p) < 0);

    $("pidinfo").textContent =
      `Controller type: ${controllerType}\n` +
      `Stable? ${stable}\n\n` +
      `Denominator coefficients: ${den.map(v => Number(v).toFixed(6)).join(", ")}\n\n` +
      `Closed-loop poles p_i:\n` +
      poles.map((p, i) => `p${i+1} = ${math.format(p, {precision: 6})}`).join("\n") +
      `\n\nConstants used in equation:\n` +
      constants.map((a, i) => `${Ki > 0 ? "a" : "b"}${i+1} = ${math.format(a, {precision: 6})}`).join("\n") +
      `\n\nc(infinity) = ${cInf.toFixed(6)}\n` +
      `max c(t)   = ${cPeak.toFixed(6)}\n` +
      `t_peak     = ${tPeak.toFixed(6)}\n` +
      `Mp         = ${Mp.toFixed(6)}\n` +
      `%OS        = ${OS.toFixed(6)} %\n\n` +
      `tr, c(t)=1 = ${tr === null ? "not found" : tr.toFixed(6)}\n` +
      `t95        = ${t95 === null ? "not found" : t95.toFixed(6)}\n` +
      `t98        = ${t98 === null ? "not found" : t98.toFixed(6)}\n` +
      `Ts 5%      = ${Ts5 === null ? "not settled" : Ts5.toFixed(6)}\n` +
      `Ts 2%      = ${Ts2 === null ? "not settled" : Ts2.toFixed(6)}\n`;
  }

  ["kp", "ki", "kd", "zeta", "wn", "tfinal"].forEach(id => {
    $(id).addEventListener("input", update);
    $(id).addEventListener("change", update);
  });

  update();
})();
</script>
```
