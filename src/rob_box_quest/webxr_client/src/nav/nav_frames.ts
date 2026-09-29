// Системы координат навигационного слоя мостика (issue #3151).
//
// ЕДИНСТВЕННОЕ место, где живёт математика map ↔ base_link ↔ сцена. Путь
// Nav2, след одометрии, пин цели и сама цель, которую оператор тыкает лучом
// в пол, — всё проходит через эти функции. Карта на полу доворачивается
// через `mapPlaneTransform` (scene/map_payload.ts) — это то же
// преобразование, записанное как поворот группы; тест
// `nav_frames.test.ts` сверяет их между собой, чтобы путь не «уплыл» с
// карты при смене соглашений в одном из мест.
//
// Соглашения (REP-103 + сцена мостика):
//   map        — x восток, y север (как OccupancyGrid);
//   base_link  — x вперёд робота, y влево робота; yaw робота в map — от +x
//                против часовой;
//   сцена      — начало = base_link, ЖЁСТКО (эгоцентрично): оператор стоит
//                внутри робота; вперёд робота = −Z сцены, влево = −X,
//                вверх = +Y. Модели робота в сцене нет (решение Шифу).
//
// Поза робота берётся из map_2d (`robot_x/robot_y/robot_yaw`, ~5 Гц).

export interface Pose2D {
  /** Позиция робота в `map`, м. */
  x: number;
  y: number;
  /** Курс робота в `map`, рад (от +x против часовой). */
  yaw: number;
}

/** Точка в плоскости `map` или `base_link` (x, y по REP-103). */
export interface Xy {
  x: number;
  y: number;
}

/** Точка на полу сцены (Y сцены — высота, здесь не участвует). */
export interface SceneXz {
  x: number;
  z: number;
}

/** map → base_link: куда точка карты попадает относительно робота. */
export function mapToBaseLink(pose: Pose2D, p: Xy): Xy {
  const dx = p.x - pose.x;
  const dy = p.y - pose.y;
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
}

/** base_link → map: обратное к `mapToBaseLink`. */
export function baseLinkToMap(pose: Pose2D, b: Xy): Xy {
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  return { x: pose.x + c * b.x - s * b.y, y: pose.y + s * b.x + c * b.y };
}

/** base_link → пол сцены: вперёд = −Z, влево = −X. */
export function baseLinkToScene(b: Xy): SceneXz {
  // `0 - v` вместо `-v`: не плодим −0 (toEqual в тестах их различает).
  return { x: 0 - b.y, z: 0 - b.x };
}

/** Пол сцены → base_link. */
export function sceneToBaseLink(s: SceneXz): Xy {
  return { x: 0 - s.z, y: 0 - s.x };
}

export function mapToScene(pose: Pose2D, p: Xy): SceneXz {
  return baseLinkToScene(mapToBaseLink(pose, p));
}

export function sceneToMap(pose: Pose2D, s: SceneXz): Xy {
  return baseLinkToMap(pose, sceneToBaseLink(s));
}

/**
 * Направление на полу сцены → курс в base_link (рад, 0 = вперёд робота,
 * +π/2 = влево).
 */
export function sceneDirToBaseYaw(dx: number, dz: number): number {
  return Math.atan2(-dx, -dz);
}

/**
 * Курс в `map` → поворот объекта сцены вокруг +Y. Объект, построенный
 * «носом» в −Z (вперёд робота), после `rotation.y = mapYawToSceneRotY(...)`
 * смотрит по этому курсу.
 */
export function mapYawToSceneRotY(pose: Pose2D, mapYaw: number): number {
  return mapYaw - pose.yaw;
}

/** Угол в (−π, π]. */
export function wrapAngle(a: number): number {
  const w = Math.atan2(Math.sin(a), Math.cos(a));
  return w === -Math.PI ? Math.PI : w;
}
