// Тесты для TARS 2 metrics panel — API: setPanelUrl / clear / setState,
// колбэк onUrlChanged, getter'ы (currentUrl, state).
//
// Three.js Mesh мы не проверяем — он покрыт другими тестами, jsdom не
// даёт WebGL. Здесь — контракт handle'а и триггеры колбэков.
//
// ``HTMLCanvasElement.prototype.getContext`` в jsdom не реализован —
// ставим mock в beforeAll (см. tars1_text_panel.test.ts: комментарий).

import { describe, it, expect, beforeAll, beforeEach, vi } from "vitest";
import {
  createTars2MetricsPanel,
  formatValue,
  type Tars2MetricsPanelHandle,
  type Tars2UrlListener
} from "../src/scene/tars2_metrics_panel";

beforeAll(() => {
  type AnyCtx2D = CanvasRenderingContext2D;
  const stubCtx: AnyCtx2D = {
    fillStyle: "",
    strokeStyle: "",
    font: "",
    textBaseline: "",
    lineWidth: 1,
    lineJoin: "round",
    fillRect: () => {},
    fillText: () => {},
    measureText: (text: string) => ({ width: text.length * 7 }),
    clearRect: () => {},
    // issue #2184: панель рисует настоящий график — без этих примитивов
    // drawChart упал бы на первой же линии.
    beginPath: () => {},
    moveTo: () => {},
    lineTo: () => {},
    stroke: () => {},
    arc: () => {},
    fill: () => {},
  } as unknown as AnyCtx2D;
  HTMLCanvasElement.prototype.getContext = function (
    _contextId: "2d",
    _options?: CanvasRenderingContext2DSettings
  ): AnyCtx2D | null {
    return stubCtx;
  } as typeof HTMLCanvasElement.prototype.getContext;
});

describe("tars2_metrics_panel", () => {
  let panel: Tars2MetricsPanelHandle;

  beforeEach(() => {
    panel = createTars2MetricsPanel({ canvasWidth: 256, canvasHeight: 96 });
  });

  it("idle-состояние по умолчанию", () => {
    expect(panel.currentUrl).toBe(null);
    expect(panel.state).toBe("idle");
  });

  it("setPanelUrl → state=loading, currentUrl задан", () => {
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    expect(panel.currentUrl).toBe(
      "http://prometheus.lan/grafana/d/panel?query=foo"
    );
    expect(panel.state).toBe("loading");
  });

  it("повторный setPanelUrl с тем же URL — игнор (нет двойного loading)", () => {
    const url = "http://prometheus.lan/grafana/d/panel?query=foo";
    panel.setPanelUrl(url);
    panel.setPanelUrl(url);
    // currentUrl остался, state — loading (один раз)
    expect(panel.currentUrl).toBe(url);
    expect(panel.state).toBe("loading");
  });

  it("setPanelUrl с разными URL — state снова loading", () => {
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=bar");
    expect(panel.state).toBe("loading");
    expect(panel.currentUrl).toContain("bar");
  });

  it("setPanelUrl с пустой строкой — игнор", () => {
    const before = panel.currentUrl;
    const warn = vi.spyOn(console, "warn").mockImplementation(() => {});
    panel.setPanelUrl("");
    warn.mockRestore();
    expect(panel.currentUrl).toBe(before);
  });

  it("clear() сбрасывает URL и state в idle", () => {
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    panel.clear();
    expect(panel.currentUrl).toBe(null);
    expect(panel.state).toBe("idle");
  });

  it("setState меняет состояние (ok/error)", () => {
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    panel.setState("ok");
    expect(panel.state).toBe("ok");
    panel.setState("error");
    expect(panel.state).toBe("error");
  });

  it("колбэк onUrlChanged вызывается при setPanelUrl", () => {
    const listener = vi.fn<Tars2UrlListener>();
    panel.setOnUrlChanged(listener);
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    expect(listener).toHaveBeenCalledWith({
      kind: "url",
      url: "http://prometheus.lan/grafana/d/panel?query=foo",
    });
  });

  it("колбэк onUrlChanged вызывается при clear", () => {
    const listener = vi.fn<Tars2UrlListener>();
    panel.setOnUrlChanged(listener);
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    listener.mockClear();
    panel.clear();
    expect(listener).toHaveBeenCalledWith({ kind: "clear" });
  });

  it("setOnUrlChanged(null) снимает колбэк", () => {
    const listener = vi.fn<Tars2UrlListener>();
    panel.setOnUrlChanged(listener);
    panel.setOnUrlChanged(null);
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    expect(listener).not.toHaveBeenCalled();
  });

  it("повторный setPanelUrl с тем же URL — колбэк НЕ вызывается (дедуп)", () => {
    const url = "http://prometheus.lan/grafana/d/panel?query=foo";
    const listener = vi.fn<Tars2UrlListener>();
    panel.setOnUrlChanged(listener);
    panel.setPanelUrl(url);
    listener.mockClear();
    panel.setPanelUrl(url);
    expect(listener).not.toHaveBeenCalled();
  });

  it("getStats возвращает текущее состояние", () => {
    panel.setPanelUrl("http://prometheus.lan/grafana/d/panel?query=foo");
    panel.setState("ok");
    const stats = panel.getStats();
    expect(stats.url).toContain("foo");
    expect(stats.state).toBe("ok");
  });

  it("dispose() не падает", () => {
    expect(() => panel.dispose()).not.toThrow();
  });
});

// ── issue #2184: реальные данные вместо ссылки ──────────────────────

describe("tars2_metrics_panel · setPanelData", () => {
  let panel: Tars2MetricsPanelHandle;

  const seriesPayload = {
    status: "ok" as const,
    datasource: "prometheus",
    query: "rate(process_cpu_seconds_total[5m])",
    note: "«cpu» → rate(process_cpu_seconds_total[5m])",
    summary: "Вывел на TARS 2 — voice-assistant: 0.04",
    series: [
      {
        name: "voice-assistant",
        labels: { instance: "10.1.1.11:9100" },
        points: [
          [1788893900, 0.03],
          [1788893960, 0.04]
        ] as [number, number][]
      }
    ],
    lines: [],
    url: "http://10.1.1.249:3000/explore?orgId=1&left=%7B%7D"
  };

  beforeEach(() => {
    panel = createTars2MetricsPanel({ canvasWidth: 1280, canvasHeight: 720 });
  });

  it("ряды Prometheus рисуются и попадают в getStats", () => {
    panel.setPanelData(seriesPayload);
    expect(panel.state).toBe("ok");
    const stats = panel.getStats();
    expect(stats.seriesCount).toBe(1);
    expect(stats.pointCount).toBe(2);
  });

  it("плоский ряд (все значения равны) не делит на ноль", () => {
    expect(() =>
      panel.setPanelData({
        ...seriesPayload,
        series: [
          {
            name: "up",
            labels: {},
            points: [
              [1788893900, 1],
              [1788893960, 1]
            ] as [number, number][]
          }
        ]
      })
    ).not.toThrow();
    expect(panel.state).toBe("ok");
  });

  it("одна точка в ряду не ломает шкалу времени", () => {
    expect(() =>
      panel.setPanelData({
        ...seriesPayload,
        series: [
          { name: "up", labels: {}, points: [[1788893900, 1]] as [number, number][] }
        ]
      })
    ).not.toThrow();
  });

  it("status=empty → state=empty (не error: тракт цел, данных нет)", () => {
    panel.setPanelData({
      status: "empty",
      query: "rate(network_latency_ms[5m])",
      series: [],
      available: ["up", "voice_llm_request_total"]
    });
    expect(panel.state).toBe("empty");
    expect(panel.getStats().seriesCount).toBe(0);
  });

  it("status=error → state=error", () => {
    panel.setPanelData({
      status: "error",
      query: "up",
      error: "connection refused",
      series: []
    });
    expect(panel.state).toBe("error");
  });

  it("логи Loki рисуются без падения", () => {
    expect(() =>
      panel.setPanelData({
        status: "ok",
        datasource: "loki",
        query: '{container="voice-assistant"}',
        series: [],
        lines: [
          { ts: 1788893960, line: "hello  world", labels: {} },
          { ts: 1788893900, line: "older", labels: {} }
        ]
      })
    ).not.toThrow();
    expect(panel.state).toBe("ok");
  });

  it("setPanelUrl после данных сбрасывает график (ссылка данных не несёт)", () => {
    panel.setPanelData(seriesPayload);
    panel.setPanelUrl("http://10.1.1.249:3000/explore?orgId=1&left=%7B%22a%22%3A1%7D");
    expect(panel.currentData).toBe(null);
    expect(panel.getStats().seriesCount).toBe(0);
    expect(panel.state).toBe("loading");
  });

  it("clear() убирает и данные, и URL", () => {
    panel.setPanelData(seriesPayload);
    panel.clear();
    expect(panel.currentData).toBe(null);
    expect(panel.currentUrl).toBe(null);
    expect(panel.state).toBe("idle");
  });

  it("setPanelData подхватывает url из payload", () => {
    panel.setPanelData(seriesPayload);
    expect(panel.currentUrl).toContain("/explore?");
  });
});

describe("formatValue", () => {
  it("сжимает большие числа и держит точность у малых", () => {
    expect(formatValue(199266304)).toBe("199.27M");
    expect(formatValue(1500)).toBe("1.50k");
    expect(formatValue(0.0358)).toBe("0.0358");
    expect(formatValue(0)).toBe("0");
    expect(formatValue(NaN)).toBe("—");
  });
});