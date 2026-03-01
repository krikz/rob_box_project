-- voice_memory fixture: 85 turns from real robot session
-- Topics: songs (raccoon, kitten, mammoth), jokes, space stories, AI, navigation
-- Generated from live robot DB for integration tests

PRAGMA journal_mode=WAL;

CREATE TABLE IF NOT EXISTS voice_turns (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id  TEXT NOT NULL,
    role        TEXT NOT NULL CHECK(role IN ('user','assistant','system')),
    content     TEXT NOT NULL,
    timestamp   REAL NOT NULL DEFAULT (unixepoch('now','subsec'))
);
CREATE INDEX IF NOT EXISTS idx_vt_session   ON voice_turns(session_id);
CREATE INDEX IF NOT EXISTS idx_vt_timestamp ON voice_turns(timestamp);

CREATE TABLE IF NOT EXISTS voice_facts (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    fact        TEXT NOT NULL,
    category    TEXT NOT NULL DEFAULT 'general',
    created_at  REAL NOT NULL DEFAULT (unixepoch('now','subsec')),
    updated_at  REAL NOT NULL DEFAULT (unixepoch('now','subsec'))
);

CREATE TABLE IF NOT EXISTS voice_memory_meta (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

BEGIN TRANSACTION;
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_151148', 'user', 'спой песенку про енотов', 1771600477.0003734);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_151148', 'user', 'а какие-то звуки может воспроизводить', 1771600581.59041);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_151148', 'user', 'робот расскажи анекдот', 1771601089.8992522);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'с спой пожалуйста песенку маленького енотика', 1771602017.7003293);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'теперь спой гимн россии', 1771602136.1847386);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'Робокс да хочу', 1771602158.549136);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'а о чем мы с тобой говорили в прошлый раз', 1771602225.6902165);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'я хочу чтобы ты гангстер рэп зачитал', 1771602278.8604803);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'расскажи историю про космический корабль', 1771602355.3018894);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'странная у тебя история получилась про космический корабль расскажи побольше', 1771602492.6378832);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_153904', 'user', 'странная история расскажи больше', 1771602539.2927368);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_161948', 'user', 'расскажи о чем мы там с тобой болтали в прошлый раз', 1771604460.2818427);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'с с песенку про мамонтенка спой пожалуйста', 1771606542.936126);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'а какие звуки ты можешь воспроизводить', 1771606725.6086404);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'я хочу услышать 5 случайных звуков из всех категорий', 1771606773.515687);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'с воспроизведи самый длинный звук', 1771606809.0686512);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'воспроизведи свой любимый звук', 1771606888.070935);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'да покукарекай 10 раз пожалуйста', 1771607370.4534843);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'са спой песенку про котенка', 1771607482.1265435);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'вот ты тряпка', 1771607556.4294865);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_165410', 'user', 'ставь псу чечетку', 1771608240.192764);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'расскажи о вреде ковыряния в носу', 1771609453.1515925);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Ответь на следующие вопросы:
1. с а что если козюли есть
2. Рома что ли плохо', 1771609504.8485951);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Да подожди а ты на 1 вопрос не ответил', 1771609530.7053099);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Я имел в виду есть козюль это кушать козюли', 1771609563.8172717);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'а теперь спой песенку про козюли', 1771609599.572974);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Робот напукарекай мне', 1771609675.2499545);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'о чем мы с тобой говорили', 1771609726.8456137);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Робот а сколько сейчас времени', 1771609759.5589101);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'с так что сколько времени', 1771609823.1961346);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'а какой день недели сейчас', 1771609881.2606926);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Ответь на следующие вопросы:
1. Робот посмотри у тебя в инструментах есть что нибудь
2. Робот провел дни недели', 1771609912.80601);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'спой песенку из шелдона купера про маленького котенка', 1771610058.8465278);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'представь что ты полиграф полиграфович что бы ты сказал', 1771610123.6831696);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'я имел ввиду этот персонаж из собачьего сердца', 1771610164.233391);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'представь что ты шариков общайся со мной как будто ты шариков', 1771610228.0825915);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'Шариков по бугорекой', 1771610363.1683974);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_174256', 'user', 'как там твой шариков', 1771610425.7558599);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'теперь ты шариков говори от имени шарикова со мной понял', 1771610840.5429144);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робот расскажи анекдот', 1771610885.5855544);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робот ты теперь шариков не забывай', 1771610916.3159928);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робот я тебе говорю ты шариков из собачьего сердца', 1771610948.2471757);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робот сколько времени', 1771611008.0333703);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'насколько ты серьезный', 1771611083.6126013);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'будь сладким пирожочком', 1771611133.070378);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'расскажи про инференс', 1771611210.8814828);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'проиграй 5 мелодий расскажи анекдот расскажи сказку и еще раз 5 мелодий проиграй', 1771611253.3154616);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'а станцу еще чечеточку', 1771611340.209078);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робокс танцующий чечетку', 1771611393.311514);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'Робокс ты не танцевал станцуй', 1771611423.6231842);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'с твой любимый цвет', 1771611483.9794834);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'о чем мы с тобой говорили', 1771611534.0297198);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260220_180607', 'user', 'что тебе больше всего понравилось из нашей предыдущей беседы', 1771611577.6100926);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'о чем мы с тобой говорили', 1771681298.3793795);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'А помнишь я у тебя просил чтобы ты от шарикова со мной общался', 1771681347.5200298);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Да я хочу чтобы ты общался как шариков', 1771681383.1098306);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Робот да хочу', 1771681416.0452316);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Робот а что ты про слонов знаешь', 1771681470.7616317);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Я хочу чтобы ты как шариков про слонов мне сказал', 1771681515.5496316);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Эх ты неправильно шариков по другому говорил', 1771681553.6189396);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Робокс не очень поищи про шарикова как он общается про слонов', 1771681604.6283467);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Ты же говорил от шарикова ты как шариков должен со мной общаться не забыл', 1771681647.132246);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'И что и что', 1771681688.7280154);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Ответь на следующие вопросы:
1. Чуть чуть у + значит надо без этого ходить ты что охренел совсем что ли
2. Тебе по жопе дать', 1771681700.2783868);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Поругая кирилла он ходит по улице без куртки', 1771681711.4647768);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Почему ребята ты с ребятами пошел гулять', 1771681743.999022);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Ты дома они сказали ты пошел что то разбирать что ты будешь поразбирать', 1771681765.6981518);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Робот вся покукарекай 5 раз как шариков', 1771681810.3885503);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Робот что ты сейчас сделал', 1771681902.4353092);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'Ответь на следующие вопросы:
1. Робот сколько время
2. Робот сколько время
3. Робот сколько время
4. Робот сколько время
5. Робот сколько время', 1771682036.882327);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'с так что там у тебя', 1771682211.3878193);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_134013', 'user', 'покукарекай 5 раз', 1771682615.430588);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_141405', 'user', 'с о чем мы с тобой говорили', 1771683316.374826);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_141405', 'user', 'Робот на какую тему хочешь поговорить', 1771683365.2436502);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_141405', 'user', 'А ты на какую хочешь поговорить тему', 1771683408.033797);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_141405', 'user', 'Ответь на следующие вопросы:
1. Так о чем ты хочешь поговорить со мной
2. Робот о чем ты хочешь со мной поговорить
3. Робот спой песенку
4. Робот спой песенку
5. Робот спой песенку', 1771683490.539687);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_141405', 'user', 'Робот расскажи песенку', 1771683624.5898066);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'о чем мы с тобой говорили в прошлый раз', 1771684128.580364);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'Робот предлагай мне тему поговорим с тобой', 1771684165.0997446);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'Давай о будущем искусственного интеллекта', 1771684196.4457994);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'Ответь на следующие вопросы:
1. Я думаю что это неплохо может расширить человеческое
2. Робот я думаю что он неплохо поможет человеку
3. Робот ты тут', 1771684282.4102023);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'Ответь на следующие вопросы:
1. Робот что дальше
2. Робот что дальше
3. Покажи анимацию полиции робот покажи анимацию полиции
4. Робот проиграй анимацию любимую
5. Робот проиграй любимую анимацию', 1771684481.5962179);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'ну и что там', 1771684608.860159);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'Искусственный белый велосипед', 1771684924.5928047);
INSERT INTO voice_turns(session_id, role, content, timestamp) VALUES('20260221_142737', 'user', 'я всегда люблю вот чтоб случайно', 1771685051.6780536);

INSERT INTO voice_facts(fact, category, created_at, updated_at) VALUES('Пользователь напомнил про шарики в контексте анекдотов', 'general', 1771610932.6257544, 1771610932.6257544);

COMMIT;
