# SIGILL `controller_server`: статический разбор корки (#5181)

Дата разбора: 2026-11-23. Образ не пересобирался и не выкачивался заново.

Правило, по которому написан этот файл (уточнение Шифу через слот №6, #5180):
файл и смещение называются только если есть адрес падения и карта загрузки
библиотек **того же процесса**. Совпадение по размеру сегмента выводом не является.

## Что доказано

Процесс — тот же, что упал с SIGILL. Карта взята не из другого прогона.

- Корка: `/tmp/stand5180/runs/B-diag/qemu_controller_server.core`, 336674816 байт.
  `pr_cursig = 4` (SIGILL). Поток pid 211.
- `AT_EXECFN` = `/opt/ros/humble/lib/nav2_controller/controller_server`.
- PC = `0x7ca8a42a2a7c`, x30 = `0x7ca8a4051d20`.
- Карта загрузчика прочитана из этой же корки: `AT_BASE = 0x7ca8acc10000`,
  `_r_debug` интерпретатора по `0x3d1d0` → гостевой `0x7ca8acc4d1d0`,
  `r_version = 1`, `r_map = 0x7ca8acc4d370`. В списке 71 запись `link_map`.

| регистр | адрес | `l_addr` из `link_map` | файл | смещение |
|---|---|---|---|---|
| PC | `0x7ca8a42a2a7c` | `0x7ca8a4110000` | `/opt/ros/humble/lib/librclcpp.so` | `0x192a7c` |
| x30 | `0x7ca8a4051d20` | `0x7ca8a3f80000` | `/opt/ros/humble/lib/libcontroller_server_core.so` | `0xd1d20` |

Оба смещения попадают в единственный `PT_LOAD` с флагами R-X своего файла
(`p_offset == p_vaddr` у первого LOAD, поэтому файловое смещение равно
смещению от `l_addr`).

Образ, из которого взяты байты: `10.1.1.249:5000/krikz/rob_box:nav2-humble-dev-72a212d`,
Id `sha256:886ebe8e8b60a559af51cfb92a7512d08e7615b0fd36eb1dd0716ac4e1903d07`, arm64.

| файл | BuildID | sha256 | размер |
|---|---|---|---|
| `librclcpp.so` | `36c1c0dfb5612c5e5b371a5eec4b1fb166283632` | `f92ff09b667762a56213cfd9bbd3ce81212b302f8d0a759edd292f11f41a5472` | 1929952 |
| `libcontroller_server_core.so` | `a2d7fedf0ff287c759f71714c59d54bf04978a01` | `271b38e17788d56be66ddc6c74fb587a9f6f11d831dcc40264c63b0ab422ebf8` | 2278752 |

`readelf -A` обоих файлов пуст: заметки атрибутов ISA нет, только
`NT_GNU_BUILD_ID`. Флагов SVE/BTI/MTE/PAuth в заголовке нет.

## Байт, на котором упал PC

Смещение `0x192a7c` лежит в `.rodata` (`addr = 0x181fa8`, `size = 0x184c6`),
не в `.text` (`.text` кончается на `0x181f90`).

Покрывающий символ динамической таблицы:

```
_ZTSZN6rclcpp12experimental28SubscriptionIntraProcessBase21set_on_ready_callbackESt8functionIFvmiEEEUlmE_
value = 0x192a70, size = 0x66
```

Это typeinfo-имя (`_ZTS`) лямбды внутри
`rclcpp::experimental::SubscriptionIntraProcessBase::set_on_ready_callback`.
Само имя, 101 байт, начинается на `0x192a70`:

```
ZN6rclcpp12experimental28SubscriptionIntraProcessBase21set_on_ready_callbackESt8functionIFvmiEEEUlmE_
```

PC стоит на 12-м байте этой строки. Слово по адресу падения:

| | |
|---|---|
| байты (LE) | `78 70 65 72` |
| ASCII | `xper` |
| слово | `0x72657078` |

`aarch64-linux-gnu-objdump -D -b binary -m aarch64` помечает это слово как
`.inst 0x72657078 ; undefined`. Соседние слова той же строки — тоже
`.inst … ; undefined` либо случайно декодируемые `ands`/`adr`/`umin`/`ldpsw`
из ASCII. Это не инструкция SVE, SVE2, LSE, RCpc, PAuth, BTI, MTE или crypto.
Класс: данные, которые начали исполнять.

Байтов исполняемого сегмента в корке нет (`filesz = 0` у R-X, покрывающего PC).
Строка `ZN6rclcpp12experimental28Subscri` в дампах корки не найдена. Байт
взят из файла образа по смещению, которое дала карта того же процесса, а не
из самой корки.

## Откуда вернулись (x30), не место падения

Смещение `0xd1d20` — `.text` `libcontroller_server_core.so`
(`.text` `0xc0050`…`0x145e30`). Дизассемблер того же образа:

```
d1d18: aa1303e0  mov x0, x19
d1d1c: 97ffb201  bl  be520 <_ZNK16rclcpp_lifecycle13LifecycleNode13get_namespaceEv@plt>
d1d20: aa0003e1  mov x1, x0
d1d24: aa1703e0  mov x0, x23
d1d28: 97ffc326  bl  c29c0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
```

Символ: `nav2_controller::ControllerServer::ControllerServer(rclcpp::NodeOptions const&)+0xb50`.
Инструкции базового AArch64. Это адрес возврата из `LifecycleNode::get_namespace()`,
не слово, на котором пришёл SIGILL.

## С чем сравнивать CPU

Main Pi и Vision Pi по `docs/architecture/HARDWARE.md:88` и `:122`:
Raspberry Pi 5, 4× Cortex-A76. Cortex-A76 — Armv8.2-A, без SVE.

QEMU в этом прогоне объявил гостю больше, чем есть у Cortex-A76.
Из `NT_AUXV` той же корки:

- `AT_HWCAP = 0x1effffffb` — выставлены в том числе `SVE`, `PACA`, `PACG`.
- `AT_HWCAP2 = 0xbff47fc77fff` — выставлены в том числе `SVE2`, `BTI`, `MTE`, `SME`.

Это возможности, которые QEMU **сообщил** процессу, а не возможности Main Pi.
На слово по PC они не влияют: `0x72657078` не является инструкцией ни
Cortex-A76, ни qemu-user. Неопределённая инструкция даёт SIGILL на обоих.

Вывод, который из этого следует: причина этого SIGILL — не отсутствующее
расширение и не флаг `-march`. Причина — переход на typeinfo-строку в
`.rodata` `librclcpp.so`. Почему указатель инструкций туда попал, этим
разбором не установлено (порча адреса, битый возврат, переход по данным).
Код из-за этого не менялся.

## Что не доказано и что не перепроверялось

- Свежей корки SIGILL с одновременно снятой картой нет. Два прогона стенда
  после исходной корки SIGILL не повторили: один закончился SIGSEGV, другой —
  SIGABRT (`guard_condition implementation does not match rmw implementation
  'rmw_zenoh_cpp'`, затем `corrupted size vs. prev_size`). Адреса тех прогонов
  другие (`0x779d…`), к PC `0x7ca8a42a2a7c` они не прикладываются. Это другие
  падения, не причина этого SIGILL.
- Совпадение по размеру сегмента (`0x1cb000` / `0x172000`) в вывод не шло.
  Имя файла взято из `link_map` той же корки; размер только совпал с ним
  задним числом.
- Соседний RO-сегмент корки (phdr 6637, `va = 0x7ca8a42ea000`) содержит
  0xb02 байт, совпадающих с `.gcc_except_table` `librclcpp.so` по файловому
  смещению `0x1ca000`. До виртуального адреса это не доведено: при базе
  `0x7ca8a4110000` эта страница должна лежать на `0x7ca8a42da000`, а корка
  подписывает её `0x7ca8a42ea000` (сдвиг на `0x10000`, квант `p_align`).
  На вывод это не опирается.
- `AT_HWCAP` робота не снимался: робот не трогался. Модель CPU — из
  `docs/architecture/HARDWARE.md`, не из `/proc/cpuinfo` живой платы.
- Пересчёт `udf`/`brk` по коду контроллера не делался. Предыдущие комментарии
  к #5181 это уже мерили на том же digest; здесь это не новое измерение.
- Почему PC оказался в `.rodata`, не установлено. Фикс не предлагается.
