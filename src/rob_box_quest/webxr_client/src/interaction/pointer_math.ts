// Математика указателя: куда уехала панель, когда оператор тянет её лучом.
//
// Чистая логика без Three.js и DOM — считаем на простых векторах, чтобы
// тесты гоняли её без сцены (как teleop_fsm / panel_manager).
//
// Модель перетаскивания. Панели мостика живут на сфере вокруг оператора:
// позиция задаёт направление, `facing` всегда смотрит в центр. Поэтому
// драг не двигает панель «в плоскости экрана», а катает её по сфере
// постоянного радиуса: куда светит луч — туда панель и едет, расстояние
// до оператора при этом не меняется. Так панель не может уехать в стену
// или в лицо и всегда остаётся читаемой.

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Vec2 {
  x: number;
  z: number;
}

/** Границы по высоте, чтобы панель не улетела под пол или под потолок. */
export const PANEL_MIN_Y = 0.8;
export const PANEL_MAX_Y = 2.6;

export function length(v: Vec3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

export function normalize(v: Vec3): Vec3 {
  const len = length(v);
  if (len < 1e-9) return { x: 0, y: 0, z: -1 };
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

/**
 * Пересечение луча со сферой (центр `center`, радиус `radius`).
 * Возвращает дальнюю точку пересечения или `null`, если луч мимо.
 *
 * Дальнюю, а не ближнюю: оператор стоит В центре сферы, поэтому «назад»
 * пересечение всегда за спиной — брать надо то, куда он смотрит.
 */
export function intersectRaySphere(
  origin: Vec3,
  direction: Vec3,
  center: Vec3,
  radius: number
): Vec3 | null {
  const d = normalize(direction);
  const ox = origin.x - center.x;
  const oy = origin.y - center.y;
  const oz = origin.z - center.z;
  const b = 2 * (d.x * ox + d.y * oy + d.z * oz);
  const c = ox * ox + oy * oy + oz * oz - radius * radius;
  const disc = b * b - 4 * c;
  if (disc < 0) return null;
  const sq = Math.sqrt(disc);
  const t = (-b + sq) / 2; // дальний корень
  if (t <= 0) return null;
  return {
    x: origin.x + d.x * t,
    y: origin.y + d.y * t,
    z: origin.z + d.z * t
  };
}

/**
 * Новая позиция панели при драге: точка на сфере радиуса `radius` вокруг
 * `center`, в которую светит луч. Если луч сферу не задел (оператор
 * смотрит совсем мимо) — берём направление луча и кладём панель на сферу
 * по этому направлению, чтобы панель не «залипала».
 */
export function dragTargetPosition(
  rayOrigin: Vec3,
  rayDirection: Vec3,
  center: Vec3,
  radius: number
): Vec3 {
  const hit = intersectRaySphere(rayOrigin, rayDirection, center, radius);
  const p =
    hit ??
    (() => {
      const d = normalize(rayDirection);
      return {
        x: center.x + d.x * radius,
        y: center.y + d.y * radius,
        z: center.z + d.z * radius
      };
    })();
  return { ...p, y: clampPanelY(p.y) };
}

export function clampPanelY(y: number): number {
  return Math.max(PANEL_MIN_Y, Math.min(PANEL_MAX_Y, y));
}

/**
 * Горизонтальный радиус позиции относительно центра — именно он держится
 * постоянным при драге (панели стоят на полукруге радиуса 2 м).
 */
export function horizontalRadius(position: Vec3, center: Vec3): number {
  const dx = position.x - center.x;
  const dz = position.z - center.z;
  const r = Math.hypot(dx, dz);
  return r < 1e-6 ? 1e-6 : r;
}

/** Нормаль панели: всегда смотрит в центр (на оператора). */
export function facingToward(position: Vec3, center: Vec3): Vec2 {
  const fx = center.x - position.x;
  const fz = center.z - position.z;
  const len = Math.hypot(fx, fz);
  if (len < 1e-6) return { x: 0, z: 1 };
  return { x: fx / len, z: fz / len };
}

/**
 * Сдвинулся ли указатель настолько, что это уже перетаскивание, а не клик.
 * Порог в метрах по дуге — 4 см на радиусе 2 м это ~1°.
 */
export const CLICK_SLOP_M = 0.04;

export function isDrag(from: Vec3, to: Vec3, slop: number = CLICK_SLOP_M): boolean {
  return length({ x: to.x - from.x, y: to.y - from.y, z: to.z - from.z }) > slop;
}

/**
 * Углы панели (для хит-теста на угловые «ручки» resize). Панель стоит
 * на сфере вокруг оператора лицом к нему. UV-координаты Three.js для
 * PlaneGeometry: u ∈ [0,1] слева направо, v ∈ [0,1] снизу вверх. Углы:
 *   bl=(0,0), br=(1,0), tr=(1,1), tl=(0,1).
 */
export type PanelCorner = "br" | "tr" | "tl" | "bl";

/** Размер угловой hit-зоны в долях от UV. 0.12 = 12% по каждой оси. */
export const RESIZE_CORNER_UV_FRACTION = 0.12;

/**
 * Найти угол панели по UV координатам точки пересечения луча с
 * плоскостью панели. `null` — попадание не в угловую зону (центральная
 * часть панели — клик/драг).
 */
export function cornerFromUv(
  uv: { x: number; y: number },
  fraction: number = RESIZE_CORNER_UV_FRACTION
): PanelCorner | null {
  const { x: u, y: v } = uv;
  if (!Number.isFinite(u) || !Number.isFinite(v)) return null;
  if (u <= fraction && v <= fraction) return "bl";
  if (u >= 1 - fraction && v <= fraction) return "br";
  if (u >= 1 - fraction && v >= 1 - fraction) return "tr";
  if (u <= fraction && v >= 1 - fraction) return "tl";
  return null;
}

/**
 * Вычислить новый размер панели при ресайзе за угол. Семантика:
 * «противоположный» угол остаётся на месте, схваченный угол переезжает
 * на `cornerPosition`. Размер = удвоенная проекция вектора между
 * ними на оси панели.
 *
 * Параметры:
 *   `position`       — текущий центр панели,
 *   `size`           — старые width/height,
 *   `cornerPosition` — где сейчас угол (на сфере вокруг оператора),
 *   `facing`         — нормаль панели (направлена к оператору).
 *
 * Возвращает предложенный размер ДО клампа по min/max (клампит
 * PanelManager.resize — здесь честная геометрия).
 */
export function resizeSize(
  position: Vec3,
  size: { width: number; height: number },
  cornerPosition: Vec3,
  facing: Vec2,
  corner: PanelCorner
): { width: number; height: number } {
  // «Право» панели в мире: cross(forward, +Y). forward = -facing
  // (направление взгляда оператора К панели). forward лежит в плоскости
  // пола, up = (0,1,0). cross даёт горизонтальный вектор, перпендикулярный
  // обоим — это «правая» сторона панели со стороны оператора.
  const forwardX = -facing.x;
  const forwardZ = -facing.z;
  const rightX = -forwardZ;
  const rightZ = forwardX;
  // Знак угла: +1 = br/tr (правая сторона), -1 = bl/tl (левая сторона).
  // По вертикали: +1 = tr/tl (верх), -1 = br/bl (низ).
  // «Противоположный» угол имеет противоположные знаки — он остаётся
  // на месте при ресайзе.
  const sx = corner === "tr" || corner === "br" ? 1 : -1;
  const sy = corner === "tr" || corner === "tl" ? 1 : -1;
  const sxOpp = -sx;
  const syOpp = -sy;
  // Старый «противоположный» угол = центр ± половинки.
  const oppX = position.x + sxOpp * (size.width / 2) * rightX;
  const oppY = position.y + syOpp * (size.height / 2);
  const oppZ = position.z + sxOpp * (size.width / 2) * rightZ;
  // Вектор от противоположного угла к схваченному.
  const vx = cornerPosition.x - oppX;
  const vy = cornerPosition.y - oppY;
  const vz = cornerPosition.z - oppZ;
  // Размеры: проекция вектора на оси панели (без удвоения —
  // расстояние между противоположными углами и есть размер).
  const width = Math.abs(vx * rightX + vz * rightZ);
  const height = Math.abs(vy);
  return { width, height };
}

/**
 * Квадрат расстояния от точки `p` до центра `c` (без sqrt — для
 * горячего hit-теста на каждом кадре).
 */
export function distSq(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}
