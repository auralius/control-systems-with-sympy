# [HTML/JavaScript] Interactive PID-Controlled Second-Order Plant

## Interactive PID Response of a Second-Order Plant

This section demonstrates the step response of a first-order plant controlled using a PID controller. The goal is to observe how the controller gains $K_p$, $K_i$, and $K_d$ affect the closed-loop response.

The plant is modeled as a standard first-order system with a time constant of $\tau$.

The notebook then plots the response $c(t)$ to a unit-step input. 

The notebook also calculates common time-domain response characteristics, including:

- rise time,
- peak time,
- maximum overshoot,
- settling time using the 5% band,
- settling time using the 2% band,
- closed-loop pole locations.

```{raw} html
# Interactive PID-Controlled First-Order Plant

This interactive plot runs directly in the browser using JavaScript, so it works on GitHub Pages without Binder, Colab, or a live Python kernel.

```{raw} html
<div id="pid-fo-widget" style="max-width:900px;margin:auto;">
  <h3>Interactive PID-Controlled First-Order Plant</h3>

  <p>
    Plant:
    <span style="font-family:serif;">G(s) = 1 / (&tau;s + 1)</span>
  </p>

  <div style="display:grid;grid-template-columns:120px 1fr 80px;gap:8px;align-items:center;">
    <label>Kp</label><input id="fo-kp" type="range" min="0" max="50" step="0.1" value="10"><span id="fo-kpv"></span>
    <label>Ki</label><input id="fo-ki" type="range" min="0" max="50" step="0.1" value="3"><span id="fo-kiv"></span>
    <label>Kd</label><input id="fo-kd" type="range" min="0" max="20" step="0.1" value="1"><span id="fo-kdv"></span>
    <label>tau</label><input id="fo-tau" type="number" step="0.05" value="1"><span></span>
    <label>t final</label><input id="fo-tfinal" type="number" step="1" value="20"><span></span>
  </div>

  <div id="fo-pidplot" style="height:480px;"></div>
  <pre id="fo-pidinfo" style="background:#f7f7f7;padding:10px;overflow:auto;"></pre>
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

  function cSub(a, b) {
    return math.subtract(a, b);
  }

  function cDiv(a, b) {
    return math.divide(a, b);
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

  function quadraticRoots(A, B, Cc) {
    const disc = math.subtract(C(B * B), C(4 * A * Cc));
    const sqrtDisc = math.sqrt(disc);

    const p1 = cDiv(cAdd(C(-B), sqrtDisc), C(2 * A));
    const p2 = cDiv(cSub(C(-B), sqrtDisc), C(2 * A));

    return [p1, p2];
  }

  function closedFormPIDFO(t, Kp, Ki, Kd, tau, a) {
    /*
      First-order plant with PID/PI, Ki > 0:

      G(s) = 1/(tau*s + 1)

      C(s)/R(s) =
          (Kd*s^2 + Kp*s + Ki)
          -------------------------------------------
          (tau+Kd)*s^2 + (1+Kp)*s + Ki

      For unit-step input:

      C(s) =
          (Kd*s^2 + Kp*s + Ki)
          ----------------------------------------
          s*(tau+Kd)*(s+a1)*(s+a2)

      where a_i = -p_i and p_i are the closed-loop poles.
    */
    const a1 = a[0], a2 = a[1];
    const A = tau + Kd;

    const cInf = cDiv(C(Ki), cMul(C(A), a1, a2));

    const termA1 = cDiv(
      cMul(
        cAdd(cMul(C(Kd), a1, a1), C(Ki), cMul(C(-Kp), a1)),
        cExp(cMul(C(-t), a1))
      ),
      cMul(C(A), a1, cSub(a1, a2))
    );

    const termA2 = cDiv(
      cMul(
        cAdd(cMul(C(Kd), a2, a2), C(Ki), cMul(C(-Kp), a2)),
        cExp(cMul(C(-t), a2))
      ),
      cMul(C(A), a2, cSub(a2, a1))
    );

    return real(cAdd(cInf, termA1, termA2));
  }

  function closedFormPDFO(t, Kp, Kd, tau, b) {
    /*
      First-order plant with PD/P, Ki = 0:

      C(s)/R(s) =
          (Kd*s + Kp)
          ---------------------------
          (tau+Kd)*s + (1+Kp)

      For unit-step input:

      C(s) =
          (Kd*s + Kp)
          -----------------------
          s*(tau+Kd)*(s+b)

      where b = (1+Kp)/(tau+Kd).
    */
    const A = tau + Kd;
    const B = 1 + Kp;
    const cInf = Kp / B;

    const coeff = (Kd * b - Kp) / B;

    return cInf + coeff * Math.exp(-b * t);
  }

  function update() {
    const Kp = parseFloat($("fo-kp").value);
    const Ki = parseFloat($("fo-ki").value);
    const Kd = parseFloat($("fo-kd").value);
    const tau = parseFloat($("fo-tau").value);
    const tFinal = parseFloat($("fo-tfinal").value);

    $("fo-kpv").textContent = Kp.toFixed(1);
    $("fo-kiv").textContent = Ki.toFixed(1);
    $("fo-kdv").textContent = Kd.toFixed(1);

    if (tau <= 0 || Ki < 0 || tFinal <= 0) {
      $("fo-pidinfo").textContent = "Use tau > 0, Ki >= 0, and t final > 0.";
      return;
    }

    const n = 1200;
    const t = [];
    const y = [];

    let poles = [];
    let constants = [];
    let den = [];
    let cInf;
    let controllerType;

    if (Ki > 0) {
      controllerType = Kd > 0 ? "PID" : "PI";

      const A = tau + Kd;
      const B = 1 + Kp;
      const Cc = Ki;

      den = [A, B, Cc];

      poles = quadraticRoots(A, B, Cc);
      constants = poles.map(p => math.multiply(C(-1), p));

      if (!rootsDistinct(constants)) {
        $("fo-pidinfo").textContent =
          "Closed-loop poles are repeated or too close. The partial-fraction equation assumes distinct poles.";
        return;
      }

      cInf = real(cDiv(C(Ki), cMul(C(A), constants[0], constants[1])));

      for (let i = 0; i < n; i++) {
        const ti = (tFinal * i) / (n - 1);
        t.push(ti);
        y.push(closedFormPIDFO(ti, Kp, Ki, Kd, tau, constants));
      }

    } else {
      controllerType = Kd > 0 ? "PD" : "P";

      const A = tau + Kd;
      const B = 1 + Kp;
      const b = B / A;

      den = [A, B];

      poles = [C(-b)];
      constants = [b];

      cInf = Kp / B;

      for (let i = 0; i < n; i++) {
        const ti = (tFinal * i) / (n - 1);
        t.push(ti);
        y.push(closedFormPDFO(ti, Kp, Kd, tau, b));
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
      traces.push({
        x: [tr],
        y: [1],
        mode: "markers",
        name: "tr"
      });
    }

    Plotly.react("fo-pidplot", traces, {
      title: controllerType + "-controlled first-order plant",
      xaxis: { title: "Time, t" },
      yaxis: { title: "Response, c(t)" },
      margin: { t: 50, r: 20, b: 50, l: 60 }
    }, { responsive: true });

    const stable = poles.every(p => math.re(p) < 0);

    $("fo-pidinfo").textContent =
      `Controller type: ${controllerType}\n` +
      `Stable? ${stable}\n\n` +
      `Plant time constant tau = ${tau.toFixed(6)}\n\n` +
      `Denominator coefficients:\n` +
      (Ki > 0
        ? `(tau+Kd)*s^2 + (1+Kp)*s + Ki\n${den.map(v => Number(v).toFixed(6)).join(", ")}`
        : `(tau+Kd)*s + (1+Kp)\n${den.map(v => Number(v).toFixed(6)).join(", ")}`) +
      `\n\nClosed-loop poles p_i:\n` +
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

  ["fo-kp", "fo-ki", "fo-kd", "fo-tau", "fo-tfinal"].forEach(id => {
    $(id).addEventListener("input", update);
    $(id).addEventListener("change", update);
  });

  update();
})();
</script>
```