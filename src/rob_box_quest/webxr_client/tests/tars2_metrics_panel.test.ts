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
  type Tars2MetricsPanelHandle,
  type Tars2UrlListener
} from "../src/scene/tars2_metrics_panel";

beforeAll(() => {
  type AnyCtx2D = CanvasRenderingContext2D;
  const stubCtx: AnyCtx2D = {
    fillStyle: "",
    font: "",
    textBaseline: "",
    fillRect: () => {},
    fillText: () => {},
    measureText: (text: string) => ({ width: text.length * 7 }),
    clearRect: () => {},
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