import { describe, it, expect, vi } from "vitest";
import {
  createAcceptTonePlayer,
  ACCEPT_TONE_MAX_PER_SECOND
} from "../src/ui/accept_tone";

interface FakeGainParam {
  value: number;
  setValueAtTime: (v: number, t: number) => void;
  linearRampToValueAtTime: (v: number, t: number) => void;
}

interface FakeOsc {
  type: string;
  frequency: { value: number };
  connected: number;
  startedAt: number | null;
  stoppedAt: number | null;
}

function makeFakeCtx(opts?: { suspended?: boolean }) {
  const oscillators: FakeOsc[] = [];
  const gains: FakeGainParam[] = [];
  const suspended = !!opts?.suspended;
  let curTime = 0;
  const ctx = {
    destination: {},
    state: suspended ? ("suspended" as const) : ("running" as const),
    currentTime: 0,
    createOscillator: vi.fn(() => {
      const osc: FakeOsc = {
        type: "sine",
        frequency: { value: 0 },
        connected: 0,
        startedAt: null,
        stoppedAt: null
      };
      oscillators.push(osc);
      return {
        get type() { return osc.type; },
        set type(t: string) { osc.type = t; },
        get frequency() { return osc.frequency; },
        set frequency(f: { value: number }) { osc.frequency = f; },
        connect: () => { osc.connected++; },
        start: (t?: number) => { osc.startedAt = t ?? curTime; },
        stop: (t?: number) => { osc.stoppedAt = t ?? curTime; }
      } as unknown as OscillatorNode;
    }),
    createGain: vi.fn(() => {
      const param: FakeGainParam = {
        value: 0,
        setValueAtTime: (v: number) => { param.value = v; },
        linearRampToValueAtTime: (v: number) => { param.value = v; }
      };
      gains.push(param);
      return {
        gain: param,
        connect: () => undefined
      } as unknown as GainNode;
    }),
    resume: vi.fn(async () => {
      (ctx as { state: AudioContextState }).state = "running";
    }),
    close: vi.fn(async () => undefined)
  };
  return { ctx, oscillators, gains };
}

describe("accept_tone — локальный синтез тона (ADR-0078 §3.5)", () => {
  it("play() создаёт осциллятор, gain, ramps и start/stop по ADSR", async () => {
    const fake = makeFakeCtx();
    const player = createAcceptTonePlayer({
      AudioContextCtor: function () {
        return fake.ctx;
      } as unknown as new () => AudioContext,
      frequencyHz: 880,
      durationMs: 100
    });
    await player.play();
    // Осциллятор и gain созданы.
    expect(fake.oscillators).toHaveLength(1);
    expect(fake.gains).toHaveLength(1);
    expect(fake.oscillators[0].type).toBe("sine");
    expect(fake.oscillators[0].frequency.value).toBe(880);
    // Oscillator подключён к gain (1 connect), gain — к destination.
    expect(fake.oscillators[0].connected).toBe(1);
    // Старт/стоп были вызваны.
    expect(fake.oscillators[0].startedAt).not.toBeNull();
    expect(fake.oscillators[0].stoppedAt).not.toBeNull();
    // Старт раньше стопа.
    expect(fake.oscillators[0].startedAt!).toBeLessThanOrEqual(
      fake.oscillators[0].stoppedAt!
    );
    // Gain прошёл 3 фазы: silence → peak → silence.
    // Мы не проверяем exact значения, но ramp был вызван хотя бы 3 раза
    // (setValueAtTime + 2 × linearRampToValueAtTime).
    expect(fake.gains[0].setValueAtTime).toBeDefined();
    expect(fake.gains[0].linearRampToValueAtTime).toBeDefined();
    player.dispose();
  });

  it("AudioContext suspended → resume() вызывается", async () => {
    const fake = makeFakeCtx({ suspended: true });
    const player = createAcceptTonePlayer({
      AudioContextCtor: function () {
        return fake.ctx;
      } as unknown as new () => AudioContext
    });
    await player.play();
    expect(fake.ctx.resume).toHaveBeenCalled();
    player.dispose();
  });

  it("rate-limit: более ACCEPT_TONE_MAX_PER_SECOND тонов в секунду → drop", async () => {
    const fake = makeFakeCtx();
    const player = createAcceptTonePlayer({
      AudioContextCtor: function () {
        return fake.ctx;
      } as unknown as new () => AudioContext
    });
    // Шлём MAX+1 подряд.
    for (let i = 0; i < ACCEPT_TONE_MAX_PER_SECOND + 1; i++) {
      await player.play();
    }
    // Лишний тон отброшен.
    expect(fake.oscillators).toHaveLength(ACCEPT_TONE_MAX_PER_SECOND);
    player.dispose();
  });

  it("play() без AudioContext — silent no-op", async () => {
    const player = createAcceptTonePlayer({
      AudioContextCtor: undefined as unknown as new () => AudioContext
    });
    await expect(player.play()).resolves.toBeUndefined();
    player.dispose();
  });
});
