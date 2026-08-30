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
