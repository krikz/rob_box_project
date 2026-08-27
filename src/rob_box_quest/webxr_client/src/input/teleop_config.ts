// src/input/teleop_config.ts
//
// Декларативный маппинг кнопок/осей XR-контроллеров Meta Quest (профиль
// "xr-standard"). Это ЕДИНСТВЕННОЕ место, где описано «какая кнопка за что
// отвечает». Чтобы поменять биндинг — правим DEFAULT_BINDINGS и гоняем
// `npm test` (tests/xr_teleop.test.ts).
//
// Справочник индексов (Meta docs, Quest Touch):
//   buttons: 0=trigger, 1=squeeze(grip), 2=thumbstick press,
//            3=A/X, 4=B/Y, 5=thumbrest
//   axes:    0/1=touchpad, 2/3=thumbstick (x, y)

export const GAMEPAD_BUTTONS = {
  trigger: 0,
  squeeze: 1, // grip (боковая кнопка-хват)
  thumbstickPress: 2,
  aX: 3,
  bY: 4,
  thumbrest: 5
} as const;

export const GAMEPAD_AXES = {
  touchpadX: 0,
  touchpadY: 1,
  thumbstickX: 2,
  thumbstickY: 3
} as const;

export interface TeleopBindings {
  /** Кнопка arm/disarm: клик стика тогглит активацию телеопа
   *  (нажал — ARM, нажал ещё раз — DISARM). */
  armButton: number;
  /** Рука, на которой живёт arm-кнопка (правая — thumbstick press). */
  armHandedness: "left" | "right" | "none";
  /** Кнопка emergency-stop (edge-triggered, шлётся один раз). */
  emergencyButton: number;
  /** Ось линейного хода (стик вперёд/назад). */
  linearAxis: number;
  /** Ось поворота (стик влево/вправо). */
  angularAxis: number;
  /** Инвертировать знак линейного хода. */
  invertLinear: boolean;
  /** Инвертировать знак поворота. */
  invertAngular: boolean;
  /** Deadzone: |v| < deadzone → 0 (допустимо 0 < deadzone < 1). */
  deadzone: number;
  /** Кнопка PTT (рация): нажатие → voice_ptt_start (edge-triggered). */
  pttButton: number;
  /** Рука PTT: правая = рация, левая = робот-голос. */
  pttHandedness: "left" | "right" | "none";
  /** Кнопка PTT робот-голоса: левый grip → STT → LLM → TTS голосом робота. */
  robotPttButton: number;
  /** Рука PTT робот-голоса (по умолчанию левая). */
  robotPttHandedness: "left" | "right" | "none";
}

export const DEFAULT_BINDINGS: TeleopBindings = {
  armButton: GAMEPAD_BUTTONS.thumbstickPress, // клик правого стика
  armHandedness: "right",
  emergencyButton: GAMEPAD_BUTTONS.bY, // B/Y
  linearAxis: GAMEPAD_AXES.thumbstickY, // стик вперёд/назад
  angularAxis: GAMEPAD_AXES.thumbstickX, // стик влево/вправо
  invertLinear: false,
  // Стик вправо (tx>0) → angular<0 → поворот направо.
  invertAngular: true,
  deadzone: 0.12,
  pttButton: GAMEPAD_BUTTONS.squeeze, // grip (рация)
  pttHandedness: "right",
  robotPttButton: GAMEPAD_BUTTONS.squeeze, // grip (робот-голос)
  robotPttHandedness: "left"
};
