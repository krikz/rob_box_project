// Менеджер подписок капитанского мостика (issue #3150).
//
// Оператор может сидеть не в LAN, а в интернете: ему нужно выбрать, на что
// подписываться и с какой частотой (SUBSCRIBE.max_hz, сервер режет кадры
// у себя — server/stream_rate.py), и видеть, сколько ест каждый поток.
//
// Чистая логика без DOM/Three.js/WS: транспорт инжектится
// (`SubscriptionTransport`), время — параметром, storage — интерфейсом.
// Сцена (streams_panel.ts) только рисует `view()` и сообщает клики.
//
// Профили:
//   «LAN»      — всё, полная частота;
//   «Интернет» — главная камера ~5 Гц, лидар 5 Гц, карта, статус, голос;
//                остальные камеры выключены;
//   «Минимум»  — без видео: статус, лидар 2 Гц, карта, голос;
//   «Свой»     — то, что оператор накрутил руками (любая ручная правка
//                переводит сюда).

export type ProfileId = "lan" | "internet" | "minimum" | "custom";

export const PROFILE_IDS: readonly ProfileId[] = ["lan", "internet", "minimum", "custom"];

export const PROFILE_LABELS: Record<ProfileId, string> = {
  lan: "LAN",
  internet: "Интернет",
  minimum: "Минимум",
  custom: "Свой"
};

/** `maxHz === null` — без лимита (поле max_hz в SUBSCRIBE не шлём). */
export interface StreamConfig {
  enabled: boolean;
  maxHz: number | null;
}

/** Шаги частоты, по которым крутит кнопка «rate» на панели. */
export const RATE_STEPS: readonly (number | null)[] = [null, 15, 10, 5, 2, 1];

export interface SubscriptionTransport {
  subscribe(topic: string, maxHz: number | null): void;
  unsubscribe(topic: string): void;
}

/** Минимальный кусок `Storage`, который нужен менеджеру. */
export interface SubscriptionStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

export const SUBSCRIPTIONS_STORAGE_KEY = "robbox.quest.subscriptions.v1";

export interface StreamView {
  topic: string;
  video: boolean;
  enabled: boolean;
  maxHz: number | null;
  /** Входящий трафик потока, кбит/с (EMA). */
  kbps: number;
  /** Кадров в секунду (EMA). */
  fps: number;
}

export interface SubscriptionsView {
  profile: ProfileId;
  streams: StreamView[];
  totalKbps: number;
}

export interface SubscriptionManagerOptions {
  /** Известные потоки, порядок = порядок строк на панели. */
  topics: readonly string[];
  /** Главная камера (для профиля «Интернет»). */
  mainVideoTopic: string;
  /** `null` — без персиста (тесты). */
  storage?: SubscriptionStorage | null;
  /** Вес нового замера в EMA (0..1]. */
  emaAlpha?: number;
}

export function isVideoTopic(topic: string): boolean {
  return topic.startsWith("camera_");
}

/** Конфиг потока в профиле. `custom` здесь не бывает. */
export function profileConfig(
  profile: Exclude<ProfileId, "custom">,
  topic: string,
  mainVideoTopic: string
): StreamConfig {
  if (profile === "lan") return { enabled: true, maxHz: null };
  if (isVideoTopic(topic)) {
    const on = profile === "internet" && topic === mainVideoTopic;
    return { enabled: on, maxHz: on ? 5 : null };
  }
  if (topic === "lidar_2d") return { enabled: true, maxHz: profile === "internet" ? 5 : 2 };
  return { enabled: true, maxHz: null };
}

/** Следующий шаг частоты по кругу (неизвестное значение → «без лимита»). */
export function nextRate(current: number | null): number | null {
  const i = RATE_STEPS.indexOf(current);
  return RATE_STEPS[(i + 1) % RATE_STEPS.length];
}

interface Meter {
  bytes: number;
  frames: number;
  kbps: number;
  fps: number;
}

interface Persisted {
  profile: ProfileId;
  custom: Record<string, StreamConfig>;
}

function sanitizeConfig(raw: unknown): StreamConfig | null {
  if (!raw || typeof raw !== "object") return null;
  const r = raw as Record<string, unknown>;
  if (typeof r.enabled !== "boolean") return null;
  const hz = r.maxHz;
  const maxHz = typeof hz === "number" && Number.isFinite(hz) && hz > 0 ? hz : null;
  return { enabled: r.enabled, maxHz };
}

export function parsePersisted(text: string | null): Persisted | null {
  if (!text) return null;
  let obj: unknown;
  try {
    obj = JSON.parse(text);
  } catch {
    return null;
  }
  if (!obj || typeof obj !== "object") return null;
  const o = obj as Record<string, unknown>;
  const profile = PROFILE_IDS.find((p) => p === o.profile);
  if (!profile) return null;
  const custom: Record<string, StreamConfig> = {};
  if (o.custom && typeof o.custom === "object") {
    for (const [topic, cfg] of Object.entries(o.custom as Record<string, unknown>)) {
      const c = sanitizeConfig(cfg);
      if (c) custom[topic] = c;
    }
  }
  return { profile, custom };
}

export class SubscriptionManager {
  private readonly order: string[] = [];
  private readonly configs = new Map<string, StreamConfig>();
  private readonly meters = new Map<string, Meter>();
  /** Что, по нашему мнению, уже подписано на сервере: topic → maxHz. */
  private readonly applied = new Map<string, number | null>();
  private readonly listeners = new Set<() => void>();
  private readonly mainVideoTopic: string;
  private readonly storage: SubscriptionStorage | null;
  private readonly alpha: number;
  private transport: SubscriptionTransport | null = null;
  private currentProfile: ProfileId = "lan";
  private lastTickMs: number | null = null;

  constructor(opts: SubscriptionManagerOptions) {
    this.mainVideoTopic = opts.mainVideoTopic;
    this.storage = opts.storage ?? null;
    this.alpha = opts.emaAlpha ?? 0.5;
    for (const t of opts.topics) this.addTopic(t);
    this.restore();
  }

  // ─────────────── чтение ───────────────

  profile(): ProfileId {
    return this.currentProfile;
  }

  topics(): string[] {
    return [...this.order];
  }

  config(topic: string): StreamConfig | undefined {
    const c = this.configs.get(topic);
    return c ? { ...c } : undefined;
  }

  view(): SubscriptionsView {
    const streams: StreamView[] = this.order.map((topic) => {
      const c = this.configs.get(topic)!;
      const m = this.meters.get(topic)!;
      return { topic, video: isVideoTopic(topic), enabled: c.enabled, maxHz: c.maxHz, kbps: m.kbps, fps: m.fps };
    });
    const totalKbps = streams.reduce((acc, s) => acc + s.kbps, 0);
    return { profile: this.currentProfile, streams, totalKbps };
  }

  totalKbps(): number {
    return this.view().totalKbps;
  }

  onChange(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  // ─────────────── управление ───────────────

  applyProfile(profile: ProfileId): void {
    this.currentProfile = profile;
    if (profile !== "custom") {
      for (const t of this.order) this.configs.set(t, profileConfig(profile, t, this.mainVideoTopic));
    }
    this.commit();
  }

  setEnabled(topic: string, enabled: boolean): void {
    this.editCustom(topic, (c) => ({ ...c, enabled }));
  }

  toggle(topic: string): void {
    this.editCustom(topic, (c) => ({ ...c, enabled: !c.enabled }));
  }

  setMaxHz(topic: string, maxHz: number | null): void {
    this.editCustom(topic, (c) => ({ ...c, maxHz }));
  }

  cycleRate(topic: string): void {
    this.editCustom(topic, (c) => ({ ...c, maxHz: nextRate(c.maxHz) }));
  }

  /**
   * Панель сменила стрим через меню: новый topic наследует настройки
   * старого (оператор настраивал «слот», а не имя топика). Старый остаётся,
   * только если его ещё показывает другая панель.
   */
  replaceTopic(oldTopic: string, newTopic: string, keepOld: boolean): void {
    const inherited = this.configs.get(oldTopic) ?? { enabled: true, maxHz: null };
    if (!this.configs.has(newTopic)) this.addTopic(newTopic, oldTopic);
    this.configs.set(newTopic, { ...inherited, enabled: true });
    if (!keepOld) this.removeTopic(oldTopic);
    this.commit();
  }

  // ─────────────── транспорт ───────────────

  /** Новый сокет (после WELCOME): сервер ничего не помнит — подписываем заново. */
  attach(transport: SubscriptionTransport): void {
    this.transport = transport;
    this.applied.clear();
    this.sync();
  }

  /** Сокет упал: подписки на сервере пропали вместе с сессией. */
  detach(): void {
    this.transport = null;
    this.applied.clear();
  }

  // ─────────────── счётчик трафика ───────────────

  recordFrame(topic: string, bytes: number): void {
    const m = this.meters.get(topic);
    if (!m) return;
    m.bytes += bytes;
    m.frames += 1;
  }

  /** Раз в ~секунду: пересчитать EMA по накопленному с прошлого тика. */
  tick(nowMs: number): void {
    const prev = this.lastTickMs;
    this.lastTickMs = nowMs;
    if (prev === null || nowMs <= prev) return;
    const dt = (nowMs - prev) / 1000;
    for (const m of this.meters.values()) {
      const kbps = (m.bytes * 8) / 1000 / dt;
      const fps = m.frames / dt;
      m.kbps = this.alpha * kbps + (1 - this.alpha) * m.kbps;
      m.fps = this.alpha * fps + (1 - this.alpha) * m.fps;
      m.bytes = 0;
      m.frames = 0;
    }
    this.emit();
  }

  // ─────────────── внутреннее ───────────────

  private addTopic(topic: string, after?: string): void {
    if (this.configs.has(topic)) return;
    const idx = after !== undefined ? this.order.indexOf(after) : -1;
    if (idx >= 0) this.order.splice(idx + 1, 0, topic);
    else this.order.push(topic);
    const base = this.currentProfile === "custom" ? "lan" : this.currentProfile;
    this.configs.set(topic, profileConfig(base, topic, this.mainVideoTopic));
    this.meters.set(topic, { bytes: 0, frames: 0, kbps: 0, fps: 0 });
  }

  private removeTopic(topic: string): void {
    const i = this.order.indexOf(topic);
    if (i >= 0) this.order.splice(i, 1);
    this.configs.delete(topic);
    this.meters.delete(topic);
  }

  private editCustom(topic: string, fn: (c: StreamConfig) => StreamConfig): void {
    const c = this.configs.get(topic);
    if (!c) return;
    this.configs.set(topic, fn(c));
    this.currentProfile = "custom";
    this.commit();
  }

  private commit(): void {
    this.sync();
    this.persist();
    this.emit();
  }

  private sync(): void {
    const t = this.transport;
    if (!t) return;
    for (const topic of [...this.applied.keys()]) {
      const c = this.configs.get(topic);
      if (c && c.enabled) continue;
      t.unsubscribe(topic);
      this.applied.delete(topic);
    }
    for (const topic of this.order) {
      const c = this.configs.get(topic)!;
      if (!c.enabled) continue;
      if (this.applied.has(topic) && this.applied.get(topic) === c.maxHz) continue;
      t.subscribe(topic, c.maxHz);
      this.applied.set(topic, c.maxHz);
    }
  }

  private emit(): void {
    for (const l of this.listeners) {
      try {
        l();
      } catch {
        // слушатель UI не должен ломать менеджер подписок
      }
    }
  }

  private persist(): void {
    if (!this.storage) return;
    const custom: Record<string, StreamConfig> = {};
    for (const [t, c] of this.configs) custom[t] = { ...c };
    const data: Persisted = { profile: this.currentProfile, custom };
    try {
      this.storage.setItem(SUBSCRIPTIONS_STORAGE_KEY, JSON.stringify(data));
    } catch {
      // квота / приватный режим — живём без персиста
    }
  }

  private restore(): void {
    if (!this.storage) return;
    let text: string | null = null;
    try {
      text = this.storage.getItem(SUBSCRIPTIONS_STORAGE_KEY);
    } catch {
      return;
    }
    const saved = parsePersisted(text);
    if (!saved) return;
    if (saved.profile !== "custom") {
      this.currentProfile = saved.profile;
      for (const t of this.order) this.configs.set(t, profileConfig(saved.profile, t, this.mainVideoTopic));
      return;
    }
    this.currentProfile = "custom";
    for (const t of this.order) {
      const c = saved.custom[t];
      if (c) this.configs.set(t, c);
    }
  }
}
