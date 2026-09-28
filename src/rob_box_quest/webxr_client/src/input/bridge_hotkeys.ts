// Глобальные хоткеи мостика (AV-25 / AV-27 / #3151 / #3150) — чистое
// сопоставление «клавиша → действие», чтобы раскладку можно было проверить
// тестом и держать в синхроне с help_overlay.ts (DEFAULT_HOTKEYS).
//
//   R          — сброс раскладки панелей;
//   V          — TTS picker;
//   G          — взвести/разрядить прицел nav-цели, Shift+G — отменить цель;
//   P          — показать/скрыть панель «ПОТОКИ».
//
// Заняты в других модулях (сюда не попадают): WASD/Shift — ходьба
// (desktop_walk.ts), стрелки+Space и E — телеоп (desktop_teleop.ts),
// M — панель режимов, H/Esc — справка (help_overlay.ts).
//
// G и P — по `ev.code` (физическая клавиша: в русской раскладке это «П» и
// «З»), R и V — исторически по `ev.key`.

export type BridgeHotkey = "reset_layout" | "tts_picker" | "nav_aim" | "nav_cancel" | "streams_panel";

export interface HotkeyEventLike {
  key: string;
  code: string;
  shiftKey?: boolean;
  ctrlKey?: boolean;
  metaKey?: boolean;
  altKey?: boolean;
  repeat?: boolean;
}

export function bridgeHotkey(ev: HotkeyEventLike): BridgeHotkey | null {
  if (ev.repeat) return null;
  if (ev.key === "r" || ev.key === "R") return "reset_layout";
  if (ev.key === "v" || ev.key === "V") return "tts_picker";
  if (ev.code === "KeyG") return ev.shiftKey ? "nav_cancel" : "nav_aim";
  if (ev.code === "KeyP" && !ev.ctrlKey && !ev.metaKey && !ev.altKey) return "streams_panel";
  return null;
}
