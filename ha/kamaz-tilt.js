/* Kamaz Leveler — зеркальный датчик наклона (крен/тангаж).
 * Цвета симметричны относительно 0°: зелёный [−g…+g], жёлтый до ±y, далее красный.
 * Конфиг: entity, label, max (по умолчанию 10), green (0.5), yellow (1.5), unit.
 */
class KamazTilt extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this._config = {};
    this._hass = null;
    this._raf = 0;
  }

  setConfig(config) {
    this._config = config || {};
    this._render();
  }

  set hass(hass) {
    this._hass = hass;
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => {
      this._raf = 0;
      this._render();
    });
  }

  getCardSize() {
    return 3;
  }

  _pt(cx, cy, r, deg) {
    const a = (deg * Math.PI) / 180;
    return [cx + r * Math.cos(a), cy - r * Math.sin(a)];
  }

  _arc(cx, cy, r, v1, v2, max) {
    const a1 = 180 - ((Math.max(-max, Math.min(max, v1)) + max) / (2 * max)) * 180;
    const a2 = 180 - ((Math.max(-max, Math.min(max, v2)) + max) / (2 * max)) * 180;
    const [x1, y1] = this._pt(cx, cy, r, a1);
    const [x2, y2] = this._pt(cx, cy, r, a2);
    const large = Math.abs(a1 - a2) > 180 ? 1 : 0;
    return `M ${x1.toFixed(2)} ${y1.toFixed(2)} A ${r} ${r} 0 ${large} 1 ${x2.toFixed(2)} ${y2.toFixed(2)}`;
  }

  _render() {
    const ss = this.shadowRoot;
    const c = this._config;
    const h = this._hass;
    const ent = c.entity || "sensor.kamaz_angle_x";
    const st = h && h.states ? h.states[ent] : undefined;
    let v = st ? parseFloat(st.state) : NaN;
    if (!Number.isFinite(v)) v = 0;
    const max = Number(c.max != null ? c.max : 10);
    const g = Number(c.green != null ? c.green : 0.5);
    const y = Number(c.yellow != null ? c.yellow : 1.5);
    const unit = c.unit || "°";
    const label = c.label || (ent.endsWith("_x") ? "Крен" : "Тангаж");

    const W = 200,
      H = 128,
      cx = W / 2,
      cy = 104,
      r = 82,
      sw = 14;

    const colGreen = "var(--success-color, #43a047)";
    const colYellow = "var(--warning-color, #ffa600)";
    const colRed = "var(--error-color, #db4437)";
    const colTrack = "var(--divider-color, #e0e0e0)";
    const colText = "var(--primary-text-color, #212121)";
    const colDim = "var(--secondary-text-color, #727272)";

    const seg = (a, b, col) =>
      `<path d="${this._arc(cx, cy, r, a, b, max)}" fill="none" stroke="${col}" stroke-width="${sw}" stroke-linecap="butt"/>`;

    const zones = [];
    // track
    zones.push(seg(-max, max, colTrack));
    // mirrored zones
    zones.push(seg(-g, g, colGreen));
    zones.push(seg(-y, -g, colYellow));
    zones.push(seg(g, y, colYellow));
    zones.push(seg(-max, -y, colRed));
    zones.push(seg(y, max, colRed));

    // needle
    const ang = 180 - ((Math.max(-max, Math.min(max, v)) + max) / (2 * max)) * 180;
    const [nx, ny] = this._pt(cx, cy, r - 10, ang);
    const needle = `<line x1="${cx}" y1="${cy}" x2="${nx.toFixed(2)}" y2="${ny.toFixed(2)}" stroke="${colText}" stroke-width="3" stroke-linecap="round"/>
      <circle cx="${cx}" cy="${cy}" r="6" fill="${colText}"/>`;

    ss.innerHTML = `
      <style>
        :host { display:block; }
        ha-card, .card {
          background: var(--ha-card-background, var(--card-background-color, #fff));
          border-radius: var(--ha-card-border-radius, 12px);
          box-shadow: var(--ha-card-box-shadow, none);
          padding: 8px 10px 6px;
        }
        .lbl { font: 500 14px var(--paper-font-body1_-_font-family, Roboto, sans-serif); color: ${colDim}; text-align:center; }
        .val { font: 600 22px var(--paper-font-body1_-_font-family, Roboto, sans-serif); color: ${colText}; text-align:center; margin-top:-4px; }
        .wrap { display:flex; flex-direction:column; align-items:center; }
      </style>
      <div class="card">
        <div class="wrap">
          <svg viewBox="0 0 ${W} ${H}" width="100%" height="auto" role="img" aria-label="${label}">
            <defs></defs>
            ${zones.join("")}
            ${needle}
            <text x="${cx}" y="${cy - 6}" text-anchor="middle" font-size="18" font-weight="600"
                  fill="${colText}" font-family="Roboto, sans-serif">${v.toFixed(2).replace(".", ",")}${unit}</text>
          </svg>
          <div class="lbl">${label}</div>
        </div>
      </div>`;
  }
}

customElements.define("kamaz-tilt", KamazTilt);
window.customCards = window.customCards || [];
window.customCards.push({
  type: "kamaz-tilt",
  name: "Kamaz Tilt",
  description: "Зеркальный датчик крена/тангажа с цветами от нуля",
});
