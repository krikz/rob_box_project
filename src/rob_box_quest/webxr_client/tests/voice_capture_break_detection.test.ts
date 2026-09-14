// voice_capture_break_detection: задача kanban t_d4ce35e3.
//
// Проверка детектирования обрыва getUserMedia-потока. В вердикте карточки
// перечислены признаки обрыва: «зависание видео, отвал трекинга, freeze
// браузера». На уровне voice_capture.ts эквиваленты этих признаков:
//
//   - track.getEnded → onError (микрофон отвалился посреди сессии);
//   - addModule бросил на N-ой попытке → onError;
//   - worklet.onmessage перестал приходить → capture живёт, но поток мёртвый
//     (это надо ловить со стороны приложения, не voice_capture).
//
// Эти тесты фиксируют, что voice_capture:
//   1. **ловит** ошибки от addModule и getUserMedia и зовёт onError;
//   2. **не зовёт onError ложно** при штатном завершении (stop());
//   3. **не рвёт** поток сам по себе при работе (см. sustained_session).
//
// Без физического Quest эти тесты — единственный честный способ зафиксировать
// «getUserMedia не висит», потому что мы можем проверить только production-код
// voice_capture.ts. На железе e2e (после мержа) должен подтвердить, что эти
// инварианты выживают в реальном AudioContext+AudioWorklet (см. ADR-0027 §14.2).

import { describe, it, expect, vi } from "vitest";
import {
  createVoiceCapture,
  type AudioWorkletNodeLike
} from "../src/input/voice_capture";

interface FakeDeps {
  track: { stop: ReturnType<typeof vi.fn> };
  getUserMedia: ReturnType<typeof vi.fn>;
  audioWorklet: { addModule: ReturnType<typeof vi.fn> };
  audioCtx: {
    sampleRate: number;
    audioWorklet: { addModule: ReturnType<typeof vi.fn> };
    createMediaStreamSource: ReturnType<typeof vi.fn>;
    close: ReturnType<typeof vi.fn>;
  };
  ports: Array<{
    onmessage: ((ev: { data: unknown }) => void) | null;
    postMessage: ReturnType<typeof vi.fn>;
    close: ReturnType<typeof vi.fn>;
  }>;
}

function makeFakeDeps(): FakeDeps {
  const track = { stop: vi.fn() };
  const stream = { getTracks: () => [track] } as unknown as MediaStream;
  const getUserMedia = vi.fn().mockResolvedValue(stream);
  const audioWorklet = { addModule: vi.fn().mockResolvedValue(undefined) };
  const audioCtx = {
    sampleRate: 48000,
    audioWorklet,
    createMediaStreamSource: vi
      .fn()
      .mockReturnValue({ connect: vi.fn(), disconnect: vi.fn() }),
    close: vi.fn().mockResolvedValue(undefined)
  } as unknown as FakeDeps["audioCtx"];
  return { track, getUserMedia, audioWorklet, audioCtx, ports: [] };
}

function makeCapture(
  deps: FakeDeps,
  onChunk: (pcm: Int16Array, channel: "ptt" | "wake") => void,
  onError?: (e: Error) => void
) {
  function FakeCtor(): AudioContext {
    return deps.audioCtx as unknown as AudioContext;
  }
  const createNode = (_ctx: AudioContext, _name: string): AudioWorkletNodeLike => {
    const port = {
      onmessage: null as ((ev: { data: unknown }) => void) | null,
      postMessage: vi.fn(),
      close: vi.fn()
    };
    deps.ports.push(port);
    return { port, connect: vi.fn(), disconnect: vi.fn() } as unknown as AudioWorkletNodeLike;
  };
  const cap = createVoiceCapture({
    onChunk,
    onError,
    deps: {
      getUserMedia: deps.getUserMedia,
      AudioContextCtor: FakeCtor as unknown as new () => AudioContext,
      audioWorkletModuleUrl: "blob:fake-worklet-url"
    },
    createAudioWorkletNode: createNode
  });
  cap.setPttEnabled(true);
  return cap;
}

describe("voice_capture — break / failure detection (kanban t_d4ce35e3)", () => {
  it("calls onError when addModule fails after getUserMedia resolved", async () => {
    // getUserMedia прошёл (юзер дал доступ к микрофону), но worklet-модуль
    // не загрузился (Chromium-крэш, битый blob URL). Это типичная причина
    // "freeze браузера" в поле — capture вроде бы стартанул, но потока нет.
    const deps = makeFakeDeps();
    deps.audioWorklet.addModule.mockRejectedValue(
      new Error("audio worklet blob failed")
    );
    const onError = vi.fn();
    const cap = makeCapture(deps, () => {}, onError);

    await cap.start();

    expect(onError).toHaveBeenCalledTimes(1);
    expect(onError.mock.calls[0][0]).toBeInstanceOf(Error);
    expect((onError.mock.calls[0][0] as Error).message).toMatch(/audio worklet/i);
    // Capture считает себя «не стартовавшим».
    expect(cap.isCapturing()).toBe(false);
    // getUserMedia уже вернул stream — клиент сам его не закроет, это ответственность
    // приложения (см. main.ts:dispose). Проверяем, что capture не оставил
    // неиспользуемых ссылок (worklet-фабрика не дёрнута).
    expect(deps.ports.length).toBe(0);
  });

  it("does NOT call onError on a clean stop() (stop() is not a failure)", async () => {
    const deps = makeFakeDeps();
    const onError = vi.fn();
    const cap = makeCapture(deps, () => {}, onError);

    await cap.start();
    expect(cap.isCapturing()).toBe(true);
    cap.stop();
    expect(cap.isCapturing()).toBe(false);

    expect(onError).not.toHaveBeenCalled();
    expect(deps.track.stop).toHaveBeenCalledTimes(1);
    expect(deps.audioCtx.close).toHaveBeenCalledTimes(1);
  });

  it("remains in capturing=true if worklet goes silent (defensive — UI watches the chunk stream)", async () => {
    // На физическом Quest freeze выглядит так: capture.start() вернул
    // resolved, audioWorklet.addModule() прошёл, но реальный микрофон шлёт
    // тишину / не шлёт ничего. voice_capture.ts не должен сам «падать»
    // при отсутствии чанков — это ответственность приложения (heartbeat
    // чанков в main.ts). Проверяем: isCapturing остаётся true.
    const deps = makeFakeDeps();
    const cap = makeCapture(
      deps,
      () => {},
      (e) => {
        throw new Error(`unexpected onError: ${e.message}`);
      }
    );

    await cap.start();
    expect(cap.isCapturing()).toBe(true);

    // 5 «секунд» без единого worklet.onmessage.
    await new Promise((r) => setTimeout(r, 10));
    expect(cap.isCapturing()).toBe(true);
    expect(deps.track.stop).not.toHaveBeenCalled();
    expect(deps.audioCtx.close).not.toHaveBeenCalled();
  });

  it("start() after a failed start is a clean retry (no leaked half-state)", async () => {
    // Если первый start провалился (например, getUserMedia отказал из-за
    // permission), второй start должен отработать чисто: getUserMedia зовётся
    // заново, addModule — заново, capture переходит в capturing=true.
    const deps = makeFakeDeps();
    let getUserMediaCalls = 0;
    deps.getUserMedia.mockImplementation(() => {
      getUserMediaCalls++;
      if (getUserMediaCalls === 1) {
        return Promise.reject(new Error("permission denied"));
      }
      const stream = { getTracks: () => [deps.track] } as unknown as MediaStream;
      return Promise.resolve(stream);
    });
    const onError = vi.fn();
    const cap = makeCapture(deps, () => {}, onError);

    await cap.start();
    expect(onError).toHaveBeenCalledTimes(1);
    expect(cap.isCapturing()).toBe(false);

    await cap.start();
    expect(onError).toHaveBeenCalledTimes(1); // второй start без ошибок
    expect(cap.isCapturing()).toBe(true);
    expect(deps.audioWorklet.addModule).toHaveBeenCalledTimes(1);
  });
});
