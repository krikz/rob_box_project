// Минимальный MessagePack-декодер для payload'ов сервера.
//
// Зачем свой, а не библиотека: сервер шлёт ровно два msgpack-стрима
// (robot_status 0x1201 и voice_state 0x1202, см. meta-quest-api.md §4) —
// плоские map'ы из строк, чисел, bool и null. Полный декодер (ext-типы,
// timestamp, bin64) здесь не нужен, а лишняя runtime-зависимость в
// WebXR-бандле стоит дороже сорока строк кода.
//
// Зеркало для encode-стороны: rob_box_quest/protocol/topics.py
// (`msgpack.packb(..., use_bin_type=True)`).
//
// Неподдерживаемые типы (ext, fixext) бросают — молчаливое возвращение
// мусора хуже, чем видимая ошибка в консоли.

export type MsgpackValue =
  | null
  | boolean
  | number
  | string
  | Uint8Array
  | MsgpackValue[]
  | { [key: string]: MsgpackValue };

class Reader {
  private readonly view: DataView;
  private offset = 0;

  constructor(private readonly bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }

  get done(): boolean {
    return this.offset >= this.bytes.length;
  }

  read(): MsgpackValue {
    const b = this.u8();
    // positive fixint / negative fixint
    if (b <= 0x7f) return b;
    if (b >= 0xe0) return b - 0x100;
    // fixmap / fixarray / fixstr
    if (b >= 0x80 && b <= 0x8f) return this.map(b & 0x0f);
    if (b >= 0x90 && b <= 0x9f) return this.array(b & 0x0f);
    if (b >= 0xa0 && b <= 0xbf) return this.str(b & 0x1f);

    switch (b) {
      case 0xc0:
        return null;
      case 0xc2:
        return false;
      case 0xc3:
        return true;
      case 0xc4:
        return this.bin(this.u8());
      case 0xc5:
        return this.bin(this.u16());
      case 0xc6:
        return this.bin(this.u32());
      case 0xca:
        return this.f32();
      case 0xcb:
        return this.f64();
      case 0xcc:
        return this.u8();
      case 0xcd:
        return this.u16();
      case 0xce:
        return this.u32();
      case 0xcf:
        return this.u64();
      case 0xd0:
        return this.i8();
      case 0xd1:
        return this.i16();
      case 0xd2:
        return this.i32();
      case 0xd3:
        return this.i64();
      case 0xd9:
        return this.str(this.u8());
      case 0xda:
        return this.str(this.u16());
      case 0xdb:
        return this.str(this.u32());
      case 0xdc:
        return this.array(this.u16());
      case 0xdd:
        return this.array(this.u32());
      case 0xde:
        return this.map(this.u16());
      case 0xdf:
        return this.map(this.u32());
      default:
        throw new Error(`msgpack: unsupported type 0x${b.toString(16)}`);
    }
  }

  private need(n: number): void {
    if (this.offset + n > this.bytes.length) {
      throw new Error("msgpack: truncated payload");
    }
  }

  private u8(): number {
    this.need(1);
    return this.view.getUint8(this.offset++);
  }

  private u16(): number {
    this.need(2);
    const v = this.view.getUint16(this.offset);
    this.offset += 2;
    return v;
  }

  private u32(): number {
    this.need(4);
    const v = this.view.getUint32(this.offset);
    this.offset += 4;
    return v;
  }

  private u64(): number {
    this.need(8);
    // Значения из robot_status (ts_ms) укладываются в Number.MAX_SAFE_INTEGER
    // до 287396 года — BigInt не нужен.
    const v = this.view.getBigUint64(this.offset);
    this.offset += 8;
    return Number(v);
  }

  private i8(): number {
    this.need(1);
    return this.view.getInt8(this.offset++);
  }

  private i16(): number {
    this.need(2);
    const v = this.view.getInt16(this.offset);
    this.offset += 2;
    return v;
  }

  private i32(): number {
    this.need(4);
    const v = this.view.getInt32(this.offset);
    this.offset += 4;
    return v;
  }

  private i64(): number {
    this.need(8);
    const v = this.view.getBigInt64(this.offset);
    this.offset += 8;
    return Number(v);
  }

  private f32(): number {
    this.need(4);
    const v = this.view.getFloat32(this.offset);
    this.offset += 4;
    return v;
  }

  private f64(): number {
    this.need(8);
    const v = this.view.getFloat64(this.offset);
    this.offset += 8;
    return v;
  }

  private str(len: number): string {
    this.need(len);
    const slice = this.bytes.subarray(this.offset, this.offset + len);
    this.offset += len;
    return new TextDecoder().decode(slice);
  }

  private bin(len: number): Uint8Array {
    this.need(len);
    const slice = this.bytes.slice(this.offset, this.offset + len);
    this.offset += len;
    return slice;
  }

  private array(len: number): MsgpackValue[] {
    const out: MsgpackValue[] = [];
    for (let i = 0; i < len; i += 1) out.push(this.read());
    return out;
  }

  private map(len: number): { [key: string]: MsgpackValue } {
    const out: { [key: string]: MsgpackValue } = {};
    for (let i = 0; i < len; i += 1) {
      const key = this.read();
      const value = this.read();
      out[String(key)] = value;
    }
    return out;
  }
}

/** Декодировать один msgpack-объект из начала буфера. */
export function decodeMsgpack(bytes: Uint8Array): MsgpackValue {
  return new Reader(bytes).read();
}

/**
 * Декодировать msgpack-map. Возвращает `null`, если payload битый или это
 * не map — вызывающий код (роутинг BINARY_FRAME) не должен падать из-за
 * одного испорченного кадра.
 */
export function decodeMsgpackMap(bytes: Uint8Array): { [key: string]: MsgpackValue } | null {
  try {
    const value = decodeMsgpack(bytes);
    if (value === null || typeof value !== "object" || Array.isArray(value) || value instanceof Uint8Array) {
      return null;
    }
    return value;
  } catch {
    return null;
  }
}

// =====================================================================
// Encoder (минимальный, зеркало server-side msgpack.packb с теми же
// типами, что покрывает декодер выше: map / array / str / uint / int /
// bool / null). Поддержка ext / bin / float намеренно НЕ добавляется:
// клиент AV-17 не отправляет такие поля в payload'ах supervisor-
// команд (msgpack `{client_id, mode/floor}`), а добавлять «запас на
// будущее» = лишний код без покрытия тестами.
//
// Совместим с `msgpack.packb(..., use_bin_type=True)` на стороне сервера
// (см. `src/rob_box_quest/rob_box_quest/protocol/topics.py`). Round-trip
// покрыт в tests/msgpack.test.ts ("encode/decode round-trip" suite).
// =====================================================================

export type MsgpackInput =
  | null
  | boolean
  | number
  | string
  | MsgpackInput[]
  | { [key: string]: MsgpackInput };

class Writer {
  private readonly chunks: number[] = [];

  push(bytes: Uint8Array | number[]): void {
    if (bytes instanceof Uint8Array) {
      for (let i = 0; i < bytes.length; i += 1) this.chunks.push(bytes[i]);
    } else {
      for (const b of bytes) this.chunks.push(b);
    }
  }

  u8(v: number): void {
    this.chunks.push(v & 0xff);
  }

  u16(v: number): void {
    // msgpack — big-endian (декодер читает через DataView.getUint16 без LE-флага).
    this.chunks.push((v >>> 8) & 0xff, v & 0xff);
  }

  u32(v: number): void {
    this.chunks.push((v >>> 24) & 0xff, (v >>> 16) & 0xff, (v >>> 8) & 0xff, v & 0xff);
  }

  u64(v: number): void {
    // BigInt не нужен: ts_ms укладывается в 2^53 до 287396 года.
    const hi = Math.floor(v / 0x1_0000_0000);
    const lo = v >>> 0;
    this.u32(hi);
    this.u32(lo);
  }

  i64(v: number): void {
    // Двух-комплемент big-endian через DataView — без new ArrayBuffer в hot-path.
    const view = new DataView(new ArrayBuffer(8));
    view.setBigInt64(0, BigInt(Math.trunc(v)));
    for (let i = 0; i < 8; i += 1) this.chunks.push(view.getUint8(i));
  }

  toBytes(): Uint8Array {
    return new Uint8Array(this.chunks);
  }
}

function writeStr(w: Writer, s: string): void {
  // UTF-8 length может быть > char count — замеряем байты.
  const bytes = new TextEncoder().encode(s);
  const len = bytes.length;
  if (len <= 31) {
    w.u8(0xa0 | len);
  } else if (len <= 0xff) {
    w.u8(0xd9);
    w.u8(len);
  } else if (len <= 0xffff) {
    w.u8(0xda);
    w.u16(len);
  } else {
    w.u8(0xdb);
    w.u32(len);
  }
  w.push(bytes);
}

function writeUint(w: Writer, v: number): void {
  if (!Number.isInteger(v) || v < 0) {
    throw new RangeError(`msgpack encode: unsigned integer expected, got ${v}`);
  }
  if (v <= 0x7f) {
    w.u8(v); // positive fixint
  } else if (v <= 0xff) {
    w.u8(0xcc);
    w.u8(v);
  } else if (v <= 0xffff) {
    w.u8(0xcd);
    w.u16(v);
  } else if (v <= 0xffffffff) {
    w.u8(0xce);
    w.u32(v);
  } else {
    w.u8(0xcf);
    w.u64(v);
  }
}

function writeInt(w: Writer, v: number): void {
  if (!Number.isInteger(v)) {
    throw new RangeError(`msgpack encode: integer expected, got ${v}`);
  }
  if (v >= 0) return writeUint(w, v);
  if (v >= -32) {
    w.u8(v & 0xff); // negative fixint
    return;
  }
  if (v >= -0x80) {
    w.u8(0xd0);
    w.u8(v & 0xff);
  } else if (v >= -0x8000) {
    w.u8(0xd1);
    w.u16(v & 0xffff);
  } else if (v >= -0x8000_0000) {
    w.u8(0xd2);
    w.u32(v >>> 0);
  } else {
    w.u8(0xd3);
    w.i64(v);
  }
}

function writeValue(w: Writer, v: unknown): void {
  if (v === null) {
    w.u8(0xc0);
    return;
  }
  if (v === false) {
    w.u8(0xc2);
    return;
  }
  if (v === true) {
    w.u8(0xc3);
    return;
  }
  if (typeof v === "number") {
    if (Number.isInteger(v)) {
      writeInt(w, v);
    } else {
      // float64 — для ts_ms/length мы integer, но encode-функция
      // остаётся полной (на случай будущих команд с float-полями).
      w.u8(0xcb);
      const view = new DataView(new ArrayBuffer(8));
      view.setFloat64(0, v);
      for (let i = 0; i < 8; i += 1) w.u8(view.getUint8(i));
    }
    return;
  }
  if (typeof v === "string") {
    writeStr(w, v);
    return;
  }
  if (Array.isArray(v)) {
    if (v.length <= 15) {
      w.u8(0x90 | v.length);
    } else if (v.length <= 0xffff) {
      w.u8(0xdc);
      w.u16(v.length);
    } else {
      w.u8(0xdd);
      w.u32(v.length);
    }
    for (const item of v) writeValue(w, item);
    return;
  }
  if (typeof v === "object") {
    // Plain object → map. Uint8Array и массивы уже обработаны выше.
    const entries = Object.entries(v as Record<string, MsgpackInput>);
    if (entries.length <= 15) {
      w.u8(0x80 | entries.length);
    } else if (entries.length <= 0xffff) {
      w.u8(0xde);
      w.u16(entries.length);
    } else {
      w.u8(0xdf);
      w.u32(entries.length);
    }
    for (const [k, val] of entries) {
      writeStr(w, k);
      writeValue(w, val);
    }
    return;
  }
  throw new TypeError(`msgpack encode: unsupported value ${typeof v}`);
}

/** Закодировать значение в msgpack. */
export function encodeMsgpack(value: MsgpackInput): Uint8Array {
  const w = new Writer();
  writeValue(w, value);
  return w.toBytes();
}

/**
 * Удобный хелпер: map → msgpack. В AV-17 это payload для supervisor-
 * команд (`{client_id, floor}`, `{client_id, mode}`).
 */
export function encodeMsgpackMap(map: { [key: string]: MsgpackInput }): Uint8Array {
  return encodeMsgpack(map);
}
