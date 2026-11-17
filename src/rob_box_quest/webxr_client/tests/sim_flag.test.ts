import { describe, it, expect } from "vitest";
import { isSimRequested, simLatencyMs } from "../src/dev/sim_flag";

describe("sim flag (#3149)", () => {
  it("is off without ?sim — production path unaffected", () => {
    expect(isSimRequested("")).toBe(false);
    expect(isSimRequested("?pin=123456")).toBe(false);
    expect(isSimRequested("?sim=0")).toBe(false);
    expect(isSimRequested("?sim=no")).toBe(false);
  });

  it("is on for ?sim, ?sim=1, ?sim=true", () => {
    expect(isSimRequested("?sim")).toBe(true);
    expect(isSimRequested("?sim=1")).toBe(true);
    expect(isSimRequested("?x=2&sim=TRUE")).toBe(true);
  });

  it("latency is clamped and defaults to 0", () => {
    expect(simLatencyMs("?sim=1")).toBe(0);
    expect(simLatencyMs("?sim_latency=40")).toBe(40);
    expect(simLatencyMs("?sim_latency=-5")).toBe(0);
    expect(simLatencyMs("?sim_latency=99999")).toBe(2000);
    expect(simLatencyMs("?sim_latency=abc")).toBe(0);
  });
});
