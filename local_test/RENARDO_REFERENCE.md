# Renardo / FoxDot — Полная справка для LLM

## ОСНОВЫ

### Создание звука
```python
# 2-символьные переменные зарезервированы для Player()
p1 >> pluck()               # играет C5 бесконечно
p1 >> pluck([0,2,4])        # мелодия (degree)
p1 >> pluck(degree=[0,2,4]) # то же самое

# Остановка
p1.stop()
Clock.clear()   # CTRL+. — остановить всё
```

### Синтезаторы (SynthDef)
```python
print(SynthDefs)  # все доступные синтезаторы
# Примеры: pluck, bass, pads, piano, blip, dirt, space, arpy, sitar, karp,
#          pulse, donk, marimba, gong, fuzz, varsaw, dub, rave, feel,
#          ambi, faim, quin, bell, pianovel, wobblebass, зайди в print(SynthDefs)
```

---

## АТРИБУТЫ PLAYER

### Универсальные
```python
p1 >> pluck([0,2,4],
    degree=[0,2,4],    # нота (позиция в гамме, 0=тоника)
    oct=5,             # октава (default=5, в обычной теории = 3!)
    dur=1,             # длительность (доли)
    sus=1,             # сустейн (default=dur)
    amp=1,             # громкость итоговая
    amplify=1,         # умножает amp (удобно для fade)
    pan=0,             # панорама (-1=лево, 1=право)
    delay=0,           # задержка перед нотой (в долях)
    rate=1,            # скорость воспроизведения (для семплов)
    sample=0,          # индекс семпла в папке
    room=0, mix=0.1,   # реверб (room=размер, mix=баланс 0-1)
    hpf=0, hpr=1,      # high pass filter + резонанс
    lpf=0, lpr=1,      # low pass filter + резонанс
    echo=0, decay=0.5, # эхо
    vib=0, vibdepth=0.2, # вибрато
    slide=0,           # глайд (portamento)
    shape=0,           # overdrive/дистрит
    formant=0,         # формант-фильтр
    chop=0,            # "чоппер" LFO
    pshift=0,          # питч-шифт в полутонах (для семплов)
    spread=0,          # стерео-расширение
    spin=0,            # вращение в стерео
    cut=0,             # обрезка конца ноты
    bend=0,            # бенд питча
    coarse=0,          # огрубление звука
    bits=0, crush=8,   # битдепт/битркраш
    striate=0,         # нарезка файла (для loop)
)

# Просмотр
print(Player.get_attributes())
print(Player("wobblebass").get_extra_attributes())
print(Player.get_fxs())
print(FxList)
```

### Рестановка атрибутов после создания
```python
p1 >> pluck([0,2], oct=5)
p1.oct = 4
p1.amp = 0.5
```

### Референсирование атрибутов
```python
pitches = P[0,1,2,3,4]
p1 >> pluck(pitches)
p2 >> star(dur=0.5).follow(p1) + 2    # следовать за p1

# Условные amp
p1 >> pluck([0,1,2,3], amp=(p1.degree==1)*4)
p1 >> pluck([0,1,2,3], amp=p1.degree.map({1:4, 2:1}))

# Ссылка на degree другого игрока
b1 >> bass(p1.degree[0])     # только root-нота
p2 >> star(p1.pitch) + 2
```

---

## SAMPLE PLAYER

```python
print(Samples)  # таблица символов → звуки

# Базовые паттерны
d1 >> play("x-o-")       # x=kick, o=snare, -=hihat
d1 >> play("x  x  ")     # пробелы = тишина
d1 >> play("x..x..")     # точки = тишина

# Скобки
d1 >> play("(x-)(-x)o-")   # () чередуют символы по кругу
d1 >> play("x-o[---]")     # [] = все в 1 бит (быстрая очередь)
d1 >> play("x-o{-=[--][-o]}")  # {} = случайный выбор
d1 >> play("<X   ><-   >")     # <> = слои одновременно

# Выбор семпла прямо в строке
p1 >> play("x-|o2|-")      # o с sample=2
p1 >> play("x-|o(12)|-")   # o чередует sample 1 и 2
p1 >> play("x-|o[23]|-")   # o играет 2 и 3 быстро

# Атрибут sample
d1 >> play("x-o-", sample=1)
d1 >> play("x-o-", sample=[0,1,2])  # список — по очереди
d1 >> play("x-o-", sample=(0,3))    # tuple — одновременно

# Слои через P[]
d1 >> play(P["x-o-"] & P[" ** "])
d1 >> play(P["x-o-"] & P[" ** "], room=0.5)
```

### Таблица символов (выборка)
```
Kick:     A v V x X W
Snare:    D i I o O t u
HiHat:    : = - a n N
Clap:     \ * h H
Cymbal:   / # e E
Tom:      m M p P w
Percussion: & + d f l r R y
SoundFX:  b F k L Q Y z Z
Voice:    1 2 3 4 ! < ? c C
Bell:     T
Noise:    @ %
Shaker:   s S
```

---

## LOOP PLAYER

```python
s1 >> loop('foxdot')                      # встроенный семпл
s1 >> loop('foxdot', dur=4)
s1 >> loop('/path/to/file.wav')
s1 >> loop('/path/to/folder', sample=1)
s1 >> loop('snare/*', sample=2)           # wildcard

# Позиция воспроизведения
l1 >> loop('my_file', P[:4], dur=1)       # первые 4 доли по порядку
l1 >> loop('my_file', P[:4].shuffle(), dur=1)  # случайный порядок

# Темп-мэтчинг (растяжение под Clock.bpm)
l1 >> loop('my_file', P[:4], dur=1, tempo=135)

# Стриейт (нарезка без питч-изменений)
l1 >> loop('my_file', dur=4, striate=100)
l1 >> loop('my_file', dur=4, beat_stretch=True)
```

---

## CLOCK

```python
Clock.bpm = 120         # темп
Clock.clear()           # остановить всё (= CTRL+.)
print(Clock)            # что играет
print(Clock.latency)    # задержка
print(Clock.mod(32))    # начало следующего 32-битного цикла
Clock.now()             # текущий бит

# Расписание
Clock.future(4, lambda: print("hello"))        # через 4 бита
Clock.schedule(lambda: print("hi"), Clock.now() + 4)
Clock.every(4, lambda: print("tick"))          # каждые 4 бита

# ⚠️ ИЗМЕНЕНИЕ BPM ЧЕРЕЗ Clock.future() — ТОЛЬКО setattr ИЛИ ИМЕННАЯ ФУНКЦИЯ!
# НЕЛЬЗЯ: Clock.future(8, lambda: Clock.bpm = 170)  — SyntaxError (нельзя присваивать в lambda)
# НЕЛЬЗЯ: Clock.future(8, lambda: Clock.bpm.set(170))  — AttributeError (bpm это int)
# ПРАВИЛЬНО — вариант 1: setattr
Clock.future(8, lambda: setattr(Clock, 'bpm', 170))
# ПРАВИЛЬНО — вариант 2: именная функция
def go_dnb():
    Clock.bpm = 170
Clock.future(8, go_dnb)
# ПРАВИЛЬНО — изменение Scale/Root через setattr в lambda тоже работает:
Clock.future(16, lambda: setattr(Scale, 'default', 'minor'))
Clock.future(16, lambda: setattr(Root, 'default', 4))

# nextBar декоратор
nextBar(Clock.clear)
@nextBar
def change():
    Root.default = 4
    Scale.default = "minor"

# Кастомный PlayerMethod
@PlayerMethod
def test(self):
    print(self.degree)

p1 >> pluck([0,4]).every(3, "test")
p1.never("test")
```

---

## МЕТОДЫ PLAYER

```python
# .every(n, method, *args, **kwargs)
p1 >> pads([0,2,4,6]).every(4, "reverse")    # реверс каждые 4
p1.every(4, "rotate")                         # сдвиг
p1.every(4, "shuffle")                        # рандомный порядок
p1.every(4, "stutter", 4, oct=4, pan=[-1,1]) # повтор с эффектами
p1.never("reverse")                           # отмена

# Следование
p2 >> star(dur=0.5).follow(p1) + 2
p3 >> pluck(s1.degree + 2)

# Управление
p1.solo()      # солировать (мьют остальных)
p1.solo(0)     # отключить соло
p1.only()      # остановить всех кроме него

# Модификаторы degree
p1 >> pads([0,1,2,3]) + [0,0,0,2]   # поднять 4-ю ноту на 2
p1 >> pads([0,1,2,3]) + [0,0,2]     # каждую 3-ю на 2
```

### Fade & Eclipse (новые методы)
```python
# Fade in/out
b1 >> pluck([0,3,0,4], dur=.5).fadein(16)
b1 >> pluck([0,3,0,4], dur=.5).fadeout(16)    # автостоп при =0
b1 >> pluck([0,3,0,4], dur=.5).fadeout(16, autostop=False)

# Постепенное изменение громкости
b1 >> pluck([0,3,0,4], dur=.5, amplify=0).fade(16, fvol=.5)
b1.fade(dur=4, fvol=.8)
b1.fade(dur=4, fvol=0, autostop=False)

# Eclipse — автоматические паузы
b1 >> pluck([0,3,0,4], dur=.5).eclipse(4, 32)        # 4 бита тишины каждые 32
b1 >> pluck([0,3,0,4], dur=.5).eclipse(dur=4, total=32, leftshift=16)  # пауза с 16-го бита
```

### Rests
```python
p1 >> pads([0,1,2,3,4], dur=[1,1,1,1,rest(4)])  # пауза 4 бита вместо ноты
```

---

## ПАТТЕРНЫ (Pattern)

```python
# Базовые операции
P[1,2,3] * 2      # P[2,4,6]
P[1,2,3] + 100    # P[101,102,103]
P[1,2,3] + [3,4]  # P[4,6,6,5,5,7] — все комбинации

# Срезы
P[:8]             # P[0,1,2,3,4,5,6,7]
P[0,1,2,3:20]
P[2:15:3]

# Pipe (конкатенация)
PRange(4) | [1,7,6]   # P[0,1,2,3,1,7,6]

# PGroup (аккорд)
P(0,2,4) + 2           # играть ноты одновременно
p1 >> pluck(P(0,2,4))  # аккорд

# Специальные множители
p1 >> pluck(P*(0,2,4), dur=1)   # ноты разбросаны по dur
p1 >> pluck(P/(0,2,4), dur=1)   # через раз
p1 >> pluck(P+(0,2,4), dur=2, sus=3)  # по sus
p1 >> pluck(P^(0,2,4,0.5), dur=0.5)  # с пробелами

# Методы Pattern
P[:8].shuffle()    # случайный порядок
P[:8].palindrome() # + реверс в конце
P[:8].rotate(3)    # сдвиг на 3
P[:8].stretch(12)  # растянуть до 12 элементов
P[:8].reverse()
P[:8].loop(2)      # повторить 2 раза
P[:8].offadd(5)    # добавить смещённую копию
P[:8].offmul(5)
P[:8].stutter(5)   # каждый элемент × 5
P["x-o-"].amen()   # знаменитый amen-ритм
P["x-o-"].bubble()
```

### Генераторы паттернов
```python
PRange(10)            # P[0..9]
PTri(5)               # P[0,1,2,3,4,3,2,1] — треугольник
PTri(3,10)            # от 3 до 10 и обратно
PEuclid(3,8)          # эвклидов ритм: P[1,0,0,1,0,0,1,0]
PDur(3,8)             # таймингп по эвклиду: P[0.75,0.75,0.5]
PDur(5,8)             # 5 пульсов / 8 шагов
PSine(16)             # значения синус-волны (16 частей)
PStep(4,1)            # каждый 4-й = 1, остальные = 0
PStep(8,6,4)          # каждый 8-й = 6, остальные = 4
PSum(3,8)             # 3 эл-та, сумма = 8 → P[3,3,2]
PSum(5,4)             # 5 эл-тов, сумма = 4

# Случайные
PRand(8)              # случайное целое 0..8 (бесконечно)
PRand(8,16)           # между 8 и 16
PRand([1,2,3])        # случайный из списка
PRand([1,2,3], seed=5)
PxRand([1,2,3])       # без повторов
PwRand([a,b,c],[2,1,3]) # взвешенная случайность
PWhite(0,1)           # случайные float
PWalk(7,1)            # случайная прогулка ±max
PWalk(max=2)
PBern(16,0.5)         # случайные 0/1 по вероятности
PChain({0:[2,4], 2:[0,3], 4:[0]})  # цепь Маркова
PJoin([pat1,pat2])    # объединить паттерны
PAlt(p1,p2,p3)        # чередовать элементы из нескольких
PPairs([0,4,2], lambda n: var([n*3,n-1],[12,4]))  # пары
PStutter([0,1,2], 3)  # каждый элемент × 3
PStretch([0,1,2], 5)  # P[0,1,2,0,1] — растянуть до 5
PZip([0,1,2],[3,4])   # P[(0,3),(1,4),(2,3),...]
PFibMod()             # числа Фибоначчи
P10(16)               # 16 случайных 0/1
PShuf([0,1,3,4,-1])   # перемешать один раз
PDur([3,5],8)         # список PDur-ов
PRhythm([2,(3,8)])    # tuple → PDur-тайминги
PQuicken(dur=2,stepsize=2,steps=3)  # задержки убывают
```

### Pvar (паттерн во времени)
```python
d = Pvar([[0,1,2,3,4,5,6,7], [0,1,2,3,4,5,4,3,2,1]], 8)
p1 >> pads(a, dur=0.25) + d

# Смена гаммы каждые 16 бит
Scale.default = Pvar([Scale.major, Scale.minor], 16)
```

---

## TIMEVARS

### var() — переключение между значениями
```python
a = var([0,3], 4)          # 0 → 3 → 0... каждые 4 бита
a = var([0,3], [4,2])      # 0 на 4 бита, 3 на 2 бита

# Именованные var
var.chords = var([0,4,5,4], 4)
b1 >> pluck(var.chords)
var.chords = var([0,1,5,3], 4)  # обновляет везде

# Прогрессия с var
a = var([0,4,5,3], 4)
b1 >> bass(a, dur=PDur(3,8))
p1 >> pads(a + (0,2), dur=PDur(7,16))

# inf — остановиться на последнем значении
x = var([0,1,2,3], [4,4,4,inf])
```

### linvar / sinvar / expvar — плавное изменение
```python
c = linvar([0,1], 16)           # линейно 0..1 за 16 бит
c = sinvar([0,1], 8)            # синусоидально
c = expvar([0,1], 8)            # экспоненциально

# Фильтр нарастает за 32 бита
p1 >> play("x-o-", hpf=linvar([0,4000],[32,0]))

# Offset — старт с конкретного места
linvar([0,1], 8, start=Clock.mod(32))  # со старта следующего 32-битного цикла

# Вложенные: HPF нарастает только в последние 4 из 32 бит
p1 >> dirt(dur=0.25, hpf=var([0, expvar([0,4000],[4,0])], [28,4]))
```

### Аккорд-прогрессии
```python
# Синтаксис
a = var([0,4,5,3], 4)
b1 >> bass(a, dur=2) + var([0,1],[3,1])
b = a + var([0,10], 8)  # зависимый var обновляется автоматически
```

---

## ГАММЫ И ТОНАЛЬНОСТЬ

```python
# Просмотр
print(Scale.names())           # все гаммы
print(Scale.default.pentatonic)

# Установить гамму
Scale.default = "minor"
Scale.default = Scale.major
Scale.default = [0,2,4,5,7,9,11]  # вручную через полутона

# Установить тонику
Root.default = 0        # C
Root.default = "C#"     # C#
Root.default = 4        # E
Root.default = var([1,2], 32)  # меняется каждые 32 бита

# Для отдельного игрока
p1 >> pads([0,1,2], scale=Scale.minor, root=2)

# Все ноты гаммы
Scale.default = Scale.chromatic
steps = len(Scale.default)
p1 >> pluck(P[:steps])
```

---

## ГРУППЫ (Groups)

```python
# Создать группу
Piano = Group(s1, s2, s3)
Piano.amp = 0               # всем Volume = 0
Piano.amp = var([1,0], 4)   # вкл/выкл каждые 4
Piano.stop()

# Авто-группы по именам
s1 >> pads([0,4,-2,3], dur=4)
s2 >> pluck([0,1,3,4], dur=0.25)
s_all.hpf = 500             # всем s1..s9 HPF
s_all.amp = 0
s_all.stop()
s_all.solo()
s_all.only()

# Структурные функции
def verse():
    b1 >> bass([0,3], dur=4)
    p1 >> pluck([0,4], dur=0.5)
    d1 >> play("x--x--x-")
    Clock.future(16, chorus)

def chorus():
    b1 >> bass([0,4,5,3], dur=4)
    p1 >> pluck([0,4,7,9], dur=0.25)
    d1 >> play("x-o-")
    Clock.future(16, verse)

verse()
```

---

## ПРОДВИНУТЫЕ ПРИМЕРЫ

### Drum n Bass
```python
Clock.bpm = 170
b1 >> play("V....V..VV...V..", rate=0.8, sample=2, amplify=0.6)
b4 >> play(Pvar(["..o.","..o[.o.]"], [12,2]), sample=2, amplify=0.4)
b7 >> play("s", rate=0.8, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))
b8 >> play("-", rate=1.4, pshift=linvar([0,16], 8), sample=2, shape=0.3, amplify=1.2)
```

### House
```python
Clock.bpm = 128
b1 >> play("X.", rate=0.8, sample=2, amplify=0.6)
b4 >> play("..*.", sample=3, amplify=0.4)
b7 >> play(".-", rate=0.8, sample=3, delay=PRand([0,PWhite(-0.5,0.5)]), amp=0.6)
b8 >> play("#", rate=1.2, dur=16, sus=8, amplify=0.8)
```

### Аккорды billy jean стиль
```python
Clock.bpm = 117
Root.default = "E"
Scale.default = Scale.minor
chords = [(0,2,4),(0,1,3,5),(0,2,4,6),(0,1,3,5)]
s1 >> pluck(chords, oct=3, dur=[1.5,5/2], sus=2)
b1 >> play("<V....V..V...[VV]V..><..o.><---->")
```

### Клавишные с динамикой
```python
Scale.default = "minor"
Root.default.set(var([1,2], 32))
Clock.bpm = 105
a1 >> pianovel(
    (P[2,6,4,-2], P[var([0,2,-2,2],16),4,8]),
    amp=(0.4 * var([linvar([1,0.2],0.25),1,PBern(16,0.9)],16), var([0.4,0.6],4)),
    dur=(1,2), oct=(3,6),
    vib=0.5, vibdepth=0.5,
    lpf=(var([0,600],32), linvar([400,4000],64)),
    chop=(linvar([0,4],64), 0),
    pan=(expvar([0,-0.5],12), expvar([0,0.5],16))
)
```

### Buildup / Drop с группами
```python
# BuildUp
c1 >> play("V.", dur=Pvar([1,0.5,0.25,0.1],[16,8,4,4]), amplify=Pvar([0.6,0],[30,2]))
c2 >> play("X.", dur=Pvar([1,0.5,0.25,0.1],[16,8,4,4]), amplify=Pvar([0.6,0],[30,2]))
gC = Group(c1,c2)

# Drop
gB.amp = var([1,0],[64,32])
gC.amp = var([0,1],[64,32])
```

### Transition с HPF
```python
gBeats = Group(b1,b2,b3,b4,b5)
# Рамп HPF + drop
gBeats.hpf = linvar([0,5000],[12,0], start=Clock.mod(4))
gBeats.amp = var([0,1],[4,inf], start=Clock.mod(4))
```

### follow + условные ноты
```python
b1 >> bass([0,2,3,4], dur=4)
s2 >> pluck(dur=0.5).follow(b1) + (0,2,4)  # трезвучие поверх баса

# Условие на degree
p1 >> pluck([0,1,2,3], amp=p1.degree.map({1:4, 2:1}))
d1 >> play("x[--]xu[--]x", sample=(d1.degree=="x")*2 + (d1.degree=="-")*5)
```

### Случайные семплы
```python
h2 >> play("--------", sample=PRand([0,1,2]), amplify=[0.3,0.6])
```

### Эффекты на ходу
```python
# HPF нарастает линейно
p1 >> pluck(P[:8], dur=0.5, hpf=linvar([0,4000],[8,0]), hpr=0.5)

# HPF с резонансом linvar
p1 >> pluck(P[:8], dur=0.5,
    hpf=linvar([0,4000],[8,0]),
    hpr=linvar([0.1,1],12))

# Реверб
d1 >> play("x-o-", room=0.5, mix=0.5)

# Пример крутого drum-паттерна
Clock.bpm = 142
brks = [1]*28 + [0]*4
k1 >> play("A", sample=var([0,2],64), dur=2, delay=[0,0.5], amplify=0.75*P[brks], amp=1)
s1 >> play("O", sample=var([0,2],[32,16]), dur=2, delay=1, room=0.66, mix=0.5, amplify=0.7*P[brks])
h1 >> play(":", sample=var([0,1],32), dur=1, delay=0.5, amplify=5/6)
```

---

## СОЗДАНИЕ СОБСТВЕННЫХ SYNTHDEFS

```python
from SCLang import *

example = SynthDef("example")
example.osc = SinOsc.ar(example.freq)
example.env = Env.perc()
example.add()

# Или через контекст-менеджер
with SynthDef("pads") as pads:
    pads.osc = SinOsc.ar(pads.freq)
    pads.env = Env.perc()

# Использовать уже существующий SC SynthDef
mySynth = SynthDef("mySynth")
```

---

## СОВЕТЫ ДЛЯ КРУТОЙ МУЗЫКИ

1. **Layering** — накладывать несколько Player с разными ролями (бас, пэды, арпеджио, ударные)
2. **follow()** — синтезатор автоматически играет аккорды поверх быса: `p2 >> pads().follow(b1) + (0,2,4)`
3. **Структура** — verse/chorus через `Clock.future()` и `@nextBar`
4. **Динамика** — `linvar` для HPF/LPF + `amplify=var([1,0],[28,4])` для брейков
5. **Euclidean ритмы** — `PDur(3,8)`, `PDur(5,8)`, `PEuclid(5,7)` = не квадратные ритмы
6. **Swing / groove** — `delay=PRand([0,PWhite(-0.1,0.1)])` на хай-хэт
7. **Аккордовые прогрессии** — `var([0,4,5,3], 4)` менять гармонию каждые 4 бита
8. **Эффект "дроп"** — `amplify=var([1,0],[28,4])` — 28 бит музыки, 4 бита тишины
9. **HPF sweep** — `hpf=linvar([0,4000],[32,0], start=Clock.mod(4))` — фильтр в начале следующего такта
10. **Stutter** — `p1.every(4,"stutter",3,pan=[-1,1])` — заикание с эффектом

---

## БЫСТРЫЕ ШАБЛОНЫ

### Minimal techno
```python
Clock.bpm = 135
Scale.default = "minor"
b1 >> play("X...X...", sample=2, amplify=0.8)
b2 >> play("..o...o.", sample=1, amplify=0.6)
b3 >> play("--------", dur=0.5, sample=3, amplify=0.4)
s1 >> bass([0,-2,0,-3], dur=var([1,0.5],[12,4]), oct=3, amp=0.9)
s2 >> pads((0,2,4), dur=8, amp=0.4, room=0.7, mix=0.5)
```

### Lo-fi hip-hop
```python
Clock.bpm = 80
Scale.default = "minor"
Root.default = "A"
b1 >> play("X..X....",rate=var([0.8,1],8), sample=5, amplify=1.3)
b4 >> play("..i.", rate=0.75, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))
b7 >> play("--.-", rate=0.75, sample=3, amplify=0.4)
s1 >> piano([0,4,3,2], oct=4, dur=2, room=0.5, mix=0.4, amp=0.7)
s2 >> bass([0,-3,0,-2], dur=4, oct=3, amp=0.8)
```

### Ambient
```python
Clock.bpm = 70
Scale.default = "lydian"
s1 >> space((0,2,4,7), dur=var([4,8],[32,16]), oct=5, room=0.9, mix=0.7, amp=0.5)
s2 >> pads([0,5,3,4], dur=8, oct=4, vib=0.3, room=0.8, mix=0.5).fadein(32)
b1 >> arpy(var([0,4,5,3],8), dur=0.5, oct=6, pan=sinvar([-0.5,0.5],16), amp=0.4)
```
