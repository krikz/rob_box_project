# Замер PDMX для ADR-0155 — сырой вывод `scripts/music/pdmx_coverage.py`

Сгенерировано 2026-10-06 11:23 UTC; CSV `PDMX.csv`; время 44 с.

Строк в PDMX.csv: **254077**; в `subset:no_license_conflict`: **222856** (87.7 %).

## 1. Колонки PDMX.csv и заполненность (не NA/пусто)

| колонка | все | no_license_conflict |
|---|---|---|
| `path` | 100.0 % | 100.0 % |
| `metadata` | 100.0 % | 100.0 % |
| `mxl` | 100.0 % | 100.0 % |
| `pdf` | 100.0 % | 100.0 % |
| `mid` | 100.0 % | 100.0 % |
| `version` | 100.0 % | 100.0 % |
| `is_user_pro` | 100.0 % | 100.0 % |
| `is_user_publisher` | 100.0 % | 100.0 % |
| `is_user_staff` | 100.0 % | 100.0 % |
| `has_paywall` | 100.0 % | 100.0 % |
| `is_rated` | 100.0 % | 100.0 % |
| `is_official` | 100.0 % | 100.0 % |
| `is_original` | 100.0 % | 100.0 % |
| `is_draft` | 100.0 % | 100.0 % |
| `has_custom_audio` | 100.0 % | 100.0 % |
| `has_custom_video` | 100.0 % | 100.0 % |
| `n_comments` | 100.0 % | 100.0 % |
| `n_favorites` | 100.0 % | 100.0 % |
| `n_views` | 100.0 % | 100.0 % |
| `n_ratings` | 100.0 % | 100.0 % |
| `rating` | 100.0 % | 100.0 % |
| `license` | 100.0 % | 100.0 % |
| `license_url` | 100.0 % | 100.0 % |
| `license_conflict` | 100.0 % | 100.0 % |
| `genres` | 32.6 % | 26.2 % |
| `groups` | 2.5 % | 2.1 % |
| `tags` | 8.1 % | 6.5 % |
| `song_name` | 93.9 % | 94.9 % |
| `title` | 100.0 % | 100.0 % |
| `subtitle` | 15.5 % | 14.1 % |
| `artist_name` | 93.9 % | 94.9 % |
| `composer_name` | 35.8 % | 27.3 % |
| `publisher` | 0.0 % | 0.0 % |
| `complexity` | 100.0 % | 100.0 % |
| `n_tracks` | 100.0 % | 100.0 % |
| `tracks` | 100.0 % | 100.0 % |
| `song_length` | 100.0 % | 100.0 % |
| `song_length.seconds` | 100.0 % | 100.0 % |
| `song_length.bars` | 100.0 % | 100.0 % |
| `song_length.beats` | 100.0 % | 100.0 % |
| `n_notes` | 100.0 % | 100.0 % |
| `notes_per_bar` | 100.0 % | 100.0 % |
| `n_annotations` | 100.0 % | 100.0 % |
| `has_annotations` | 100.0 % | 100.0 % |
| `n_lyrics` | 100.0 % | 100.0 % |
| `has_lyrics` | 100.0 % | 100.0 % |
| `n_tokens` | 100.0 % | 100.0 % |
| `pitch_class_entropy` | 100.0 % | 100.0 % |
| `scale_consistency` | 100.0 % | 100.0 % |
| `groove_consistency` | 100.0 % | 100.0 % |
| `best_path` | 100.0 % | 100.0 % |
| `is_best_path` | 100.0 % | 100.0 % |
| `best_arrangement` | 100.0 % | 100.0 % |
| `is_best_arrangement` | 100.0 % | 100.0 % |
| `best_unique_arrangement` | 100.0 % | 100.0 % |
| `is_best_unique_arrangement` | 100.0 % | 100.0 % |
| `subset:all` | 100.0 % | 100.0 % |
| `subset:rated` | 100.0 % | 100.0 % |
| `subset:deduplicated` | 100.0 % | 100.0 % |
| `subset:rated_deduplicated` | 100.0 % | 100.0 % |
| `subset:no_license_conflict` | 100.0 % | 100.0 % |
| `subset:all_valid` | 100.0 % | 100.0 % |

### Топ `genres` в no_license_conflict

| значение | записей |
|---|---|
| classical | 41395 |
| folk | 9619 |
| soundtrack | 2210 |
| rock | 878 |
| pop | 624 |
| classical-soundtrack | 510 |
| religiousmusic | 305 |
| jazz | 274 |
| electronic | 234 |
| rock-pop | 192 |
| rbfunksoul | 150 |
| worldmusic | 148 |
| pop-rbfunksoul | 144 |
| rock-folk | 128 |
| hiphop | 128 |
| jazz-classical | 92 |
| classical-experimental | 90 |
| classical-folk | 86 |
| metal | 79 |
| pop-soundtrack | 74 |

### Топ `license` в no_license_conflict

| значение | записей |
|---|---|
| publicdomain | 189714 |
| cc-zero | 33142 |

### Топ `tags` в no_license_conflict

| значение | записей |
|---|---|
| copyrightfreeoldjazz-creolejazz | 187 |
| disney | 138 |
| polskamuzykaw-polonica-pienipatriotyczne-muzykawokalna-piewniki-wroclawuniversitylibrary-publicdomain-digitallibrary-polish | 83 |
| piano | 83 |
| muzykainstrumentalna-niemcymuzykaw-wroclawuniversitylibrary-digitallibrary-publicdomain-polish-harp | 75 |
| keresztny | 50 |
| muzykainstrumentalna-niemcymuzykaw-wroclawuniversitylibrary-digitallibrary-publicdomain-harp | 37 |
| modifiedtranscription | 37 |
| muzykawokalna-jerzyrudolfksilegnicki-niemcymuzykaw-wroclawuniversitylibrary-publicdomain-digitallibrary-polish | 29 |
| jazz | 26 |
| ragtime-tombrier | 24 |
| bach-openwtc-piano-harpsichord | 24 |
| christmas | 22 |
| openwtc | 22 |
| trombone-accompaniment | 20 |
| piano-etude-study-lemoine | 20 |
| classicalguitar | 19 |
| guitar-classicalguitar-luigi-legnani-luigilegnani-caprice-capriccio | 19 |
| muzykainstrumentalna-niemcymuzykaw-wroclawuniversitylibrary-digitallibrary-publicdomain-strings | 18 |
| llc | 18 |

### Топ `composer_name` в no_license_conflict

| значение | записей |
|---|---|
| anon. | 4244 |
| William Marshall | 1185 |
| Trad. | 1013 |
| Traditional | 808 |
| Trad | 707 |
| Tradicional | 626 |
| J. S. Bach | 540 |
| Composer | 502 |
| Alexander Walker | 343 |
| Urheber unbekanntDatum in der hier transkribierten schriftlichen Quelle: 1767 | 326 |
| J. Scott Skinner | 316 |
| Teton Sioux band 1911-1914 | 309 |
| traditionell | 283 |
| Johann Sebastian Bach | 274 |
| Niel Gow | 248 |
| Nathaniel Gow | 215 |
| after Chief F. O'Neill | 210 |
| Unattributed | 210 |
| Tielman Susato | 193 |
| trad | 183 |

В no_license_conflict с рейтингом ≥ 4 и ≥ 3 оценок (порог В2 ADR-0154): **10763**.

## 2. Ключи metadata.tar.gz (выборка 20000 JSON)

| ключ | заполнено |
|---|---|
| `data` | 100.0 % |
| `data.as_pro` | 100.0 % |
| `data.comments` | 100.0 % |
| `data.comments.comments_total` | 100.0 % |
| `data.complexity` | 100.0 % |
| `data.count_comments` | 100.0 % |
| `data.count_favorites` | 100.0 % |
| `data.count_views` | 100.0 % |
| `data.disable_hidden_url` | 100.0 % |
| `data.dispute_hidden` | 100.0 % |
| `data.hidden` | 100.0 % |
| `data.isAddedToFavorite` | 100.0 % |
| `data.isAddedToSpotlight` | 100.0 % |
| `data.is_author_blocked_you` | 100.0 % |
| `data.is_banned_user` | 100.0 % |
| `data.is_blocked` | 100.0 % |
| `data.is_can_rate_score` | 100.0 % |
| `data.is_download_limited` | 100.0 % |
| `data.is_ogg_supported` | 100.0 % |
| `data.is_public_domain` | 100.0 % |
| `data.is_similar_scores_more` | 100.0 % |
| `data.is_user_follow` | 100.0 % |
| `data.is_waiting_for_moderate` | 100.0 % |
| `data.license_string` | 100.0 % |
| `data.license_url` | 100.0 % |
| `data.limit_download_count` | 100.0 % |
| `data.opened_dispute` | 100.0 % |
| `data.pr_show` | 100.0 % |
| `data.privacy_string` | 100.0 % |
| `data.score` | 100.0 % |
| `data.score.can_manage_score` | 100.0 % |
| `data.score.comments_count` | 100.0 % |
| `data.score.complexity` | 100.0 % |
| `data.score.date_created` | 100.0 % |
| `data.score.date_updated` | 100.0 % |
| `data.score.duration` | 100.0 % |
| `data.score.favorite_count` | 100.0 % |
| `data.score.has_custom_audio` | 100.0 % |
| `data.score.has_custom_video` | 100.0 % |
| `data.score.hits` | 100.0 % |
| `data.score.id` | 100.0 % |
| `data.score.instrumentation_id` | 100.0 % |
| `data.score.is_blocked` | 100.0 % |
| `data.score.is_downloadable` | 100.0 % |
| `data.score.is_draft` | 100.0 % |
| `data.score.is_official` | 100.0 % |
| `data.score.is_origin` | 100.0 % |
| `data.score.is_original` | 100.0 % |
| `data.score.is_private` | 100.0 % |
| `data.score.is_public_domain` | 100.0 % |
| `data.score.keysig` | 100.0 % |
| `data.score.license` | 100.0 % |
| `data.score.license_id` | 100.0 % |
| `data.score.license_version` | 100.0 % |
| `data.score.measures` | 100.0 % |
| `data.score.pages_count` | 100.0 % |
| `data.score.parts` | 100.0 % |
| `data.score.processing` | 100.0 % |
| `data.score.rating` | 100.0 % |
| `data.score.revision_id` | 100.0 % |
| `data.score.revisions_count` | 100.0 % |
| `data.score.share` | 100.0 % |
| `data.score.thumbnails` | 100.0 % |
| `data.score.title` | 100.0 % |
| `data.score.url` | 100.0 % |
| `data.score.user` | 100.0 % |
| `data.score_blocked_by_country` | 100.0 % |
| `data.score_type` | 100.0 % |
| `data.score_user_count` | 100.0 % |
| `score` | 100.0 % |
| `status_code` | 100.0 % |
| `data.score.instrumentations` | 100.0 % |
| `data.score.instruments` | 100.0 % |
| `data.score.file_score_title` | 99.9 % |
| `data.score.parts_names` | 99.2 % |
| `data.score.artist_name` | 93.8 % |
| `data.score.song_name` | 93.8 % |
| `data.song` | 93.8 % |
| `data.song.artist` | 93.8 % |
| `data.song.id` | 93.8 % |
| `data.song.name` | 93.8 % |
| `data.score.body` | 92.6 % |
| `data.score.truncated_description` | 92.6 % |
| `data.similar_scores` | 55.4 % |
| `data.score.composer_name` | 36.6 % |
| `data.genres` | 33.1 % |
| `data.composer` | 15.8 % |
| `data.composer.featured` | 15.8 % |
| `data.composer.id` | 15.8 % |
| `data.composer.name` | 15.8 % |
| `data.composer.uri` | 15.8 % |
| `data.composer.url` | 15.8 % |
| `data.score.subtitle` | 15.7 % |
| `data.sets` | 14.0 % |
| `data.score.tags` | 8.4 % |
| `data.comments.comments` | 6.6 % |
| `data.score.description` | 4.8 % |
| `data.groups` | 2.5 % |
| `data.official_score` | 0.8 % |
| `data.official_score.price` | 0.8 % |
| `data.official_score.url` | 0.8 % |

## 3. Покрытие RTTTL-библиотеки партитурами PDMX

Уровни: `exact_artist` — нормализованное название совпало и артист/композитор пересёкся; `exact` — только название; `fuzzy` — вложение токенов или difflib ≥ 0.88; `generic` — название без значимых слов («Theme»); `any_nlc` — найденная партитура в no_license_conflict.

| срез | n | exact_artist | exact | fuzzy | generic | none | any | any_nlc | any % |
|---|---|---|---|---|---|---|---|---|---|
| all | 10461 | 433 | 1339 | 1164 | 419 | 7106 | 2936 | 2451 | 28.1 % |
| tag:tv | 636 | 3 | 64 | 78 | 227 | 264 | 145 | 126 | 22.8 % |
| tag:movie | 190 | 1 | 49 | 32 | 39 | 69 | 82 | 77 | 43.2 % |
| tag:game | 102 | 13 | 24 | 16 | 14 | 35 | 53 | 40 | 52.0 % |
| tag:classical | 80 | 24 | 16 | 10 | 1 | 29 | 50 | 44 | 62.5 % |
| tag:anthem | 165 | 0 | 43 | 38 | 19 | 65 | 81 | 76 | 49.1 % |
| tag:christmas | 118 | 35 | 36 | 12 | 1 | 34 | 83 | 63 | 70.3 % |
| tag:folk | 43 | 11 | 14 | 1 | 0 | 17 | 26 | 23 | 60.5 % |
| with_artist | 9190 | 433 | 1174 | 1057 | 418 | 6108 | 2664 | 2218 | 29.0 % |
| no_artist | 1271 | 0 | 165 | 107 | 1 | 998 | 272 | 233 | 21.4 % |
| theme:Марио | 23 | 0 | 9 | 6 | 3 | 5 | 15 | 5 | 65.2 % |
| theme:Тетрис | 5 | 1 | 2 | 0 | 2 | 0 | 3 | 3 | 60.0 % |
| theme:Контра | 4 | 0 | 2 | 0 | 0 | 2 | 2 | 2 | 50.0 % |
| theme:Зельда | 14 | 0 | 3 | 4 | 4 | 3 | 7 | 6 | 50.0 % |
| theme:Аладдин | 5 | 0 | 0 | 1 | 2 | 2 | 1 | 1 | 20.0 % |
| theme:Интерстеллар | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 % |
| theme:Терминатор | 3 | 0 | 0 | 0 | 2 | 1 | 0 | 0 | 0.0 % |
| theme:Попкорн | 9 | 0 | 0 | 7 | 1 | 1 | 7 | 7 | 77.8 % |
| theme:Моцарт 40 | 10 | 3 | 3 | 1 | 0 | 3 | 7 | 6 | 70.0 % |
| theme:Чайковский | 3 | 1 | 1 | 0 | 0 | 1 | 2 | 1 | 66.7 % |
| theme:Вивальди | 1 | 0 | 1 | 0 | 0 | 0 | 1 | 1 | 100.0 % |
| theme:Бах | 24 | 11 | 2 | 1 | 1 | 9 | 14 | 11 | 58.3 % |

## 3a. Выборка сопоставлений для ручной проверки точности (сид 20261006, по 20 на уровень)

| уровень | запись RTTTL | → партитура PDMX | жанры | NLC |
|---|---|---|---|---|
| exact_artist | `whatawon` What A Wonderful World / Louis Armstrong | What a Wonderful World (SATB) / arr. Derrick Kempster | pop-jazz | True |
| exact_artist | `crazyinl_3` Crazy In Love / Beyonce ft Jay Z | Crazy (in Love) / Gnarls Barkley Beyoncé arr. Oliver Buck | NA | False |
| exact_artist | `paintitb_2` Paint It Black / Rolling Stones | Paint It Black (Advanced Piano Solo) / Composed by The Rolling Stones & Ramin DjawadiPiano arrangement by Nicolas Del GalloFull playthrough and more athttps://www.youtube.com/c/NDGmusicIf you want to donate please check out my Patreon âºhttps://www.patreon.com/ndg | rock | True |
| exact_artist | `septembe` September / Earth Wind & Fire | September / NA | rbfunksoul | True |
| exact_artist | `septembe_2` September / Earth Wind And Fire | September / NA | rbfunksoul | True |
| exact_artist | `withouty` Without You / Mariah Carey | without you ÙØ¹ ØªØÙØØªÙØÙØ ØÙØºÙØÙ / NA | pop-rbfunksoul | False |
| exact_artist | `littledr` Little Drummer Boy / Christmas Carols | Drummer Boy / NA | soundtrack | False |
| exact_artist | `lacucara_3` La Cucaracha / Traditional | La Cucaracha / NA | folk | False |
| exact_artist | `theenter` The Entertainer / Misc | The Entertainer / Scott Joplin 1902 arranged Colin Hume | NA | True |
| exact_artist | `sonicthe_7` Sonic The Hedgehog 3 - Battery Zone / Computer Games | Sonic - Labyrinth Zone / Masato Nakamura | soundtrack | True |
| exact_artist | `entersan` Enter Sandman 1 / Metallica | Enter Sandman / MetallicaArr: Anders Thue | metal | True |
| exact_artist | `purplera` Purple Rain / Prince | Purple Rain - For Concert Band / Music by - Prince (arr. Matt Kelley) | rock | False |
| exact_artist | `taintedl_3` Tainted Love 2 / Soft Cell | Tainted Love / Soft Cell | rock-electronic | True |
| exact_artist | `madworld_2` Mad World / Michael andrews ft Gary Jules | Mad World - Melodica Duet / Gary Jules | NA | True |
| exact_artist | `runaway_5` Runaway / The Corrs | Runaway / Arr. Lakhvir Kumar | rock-pop-folk | True |
| exact_artist | `boysdont` Boys Dont Cry / The Cure | Boys Don't Cry - Drum Transcription / New Wave | rock | False |
| exact_artist | `forevery_2` Forever Young / Alphaville | Forever Young / Alphaville | electronic | True |
| exact_artist | `jinglebe` Jingle Bell Rock / Bobby Helms | Jingle Bell Rock / Hoang Nguyen | rock-soundtrack | True |
| exact_artist | `gimmegim` Gimme Gimme Gimme / Abba | Dod man dod man dod man ar bungÄm! / NA | pop | True |
| exact_artist | `sonicthe_8` Sonic The Hedgehog 3 - Bonus Stage / Computer Games | Sonic - Labyrinth Zone / Masato Nakamura | soundtrack | True |
| exact | `dreams2` Dreams 2 / Corrs | Dreams / The CranberriesArranged by Alex Shellans | rock | True |
| exact | `therisin` The Rising / Bruce Springsteen | THE RISING / Melencio Guerrero | NA | True |
| exact | `happyday_3` Happy Days /  | Happy Days / NA | NA | True |
| exact | `oldrugge` Old Rugged Cross / Religious | Old Rugged Cross / George Bernard - 1913 | NA | True |
| exact | `motherea` Mother Earth / Tom Swerts | Mother: Mother Earth - Piano Solo / NA | soundtrack | True |
| exact | `baabaabl` Baa Baa Black Sheep / Nursery Rhymes | Baa Baa Black Sheep Kyrie - Oliver Hayes / NA | folk | True |
| exact | `beautifu_15` Beautiful / Matt Darey feat. Marcella Woods | Beautiful - Bazzi feat. Camilla Cabello String Quartet / NA | NA | True |
| exact | `startrek_5` Star Trek - Voyager 1 / Films And Tv | Star Trek INTO DARKNESS / Michael Giacchinoarr. Rémi | soundtrack | True |
| exact | `getready` Get Ready / 2Unlimited | Get Ready / NA | soundtrack | True |
| exact | `smile` Smile / Butterfly | Smile by BENNIK - Piano Version / Song byBENNIK | NA | False |
| exact | `silence_4` Silence / Dj Tiesto | Silence / The Dark Lord Zarden | NA | True |
| exact | `happy_3` Happy / Sita | HAPPY - Pharell WILLIAMS - The Bottom 40 Band cover / Yaume | rbfunksoul | True |
| exact | `vaderjac` Vader Jacob / Danny | Vader Jacob / Arr. Michiel de Boer | NA | True |
| exact | `scoobydo` Scooby Doo / Films And Tv | scooby doo / Written by David MookArranged by Philthathrill | soundtrack | True |
| exact | `girls_3` Girls / D12 | girl in red - girls / girl in redarrangement by _lesbabe_ | rock-pop | True |
| exact | `hardknoc` Hard Knock Life / Dr Evil | Hard Knock Life / Composed byCharles Strouse | soundtrack | True |
| exact | `we_rock_57` smoke /  | Smoke / NA | NA | True |
| exact | `awakenin_2` Awakening / Rank 1 | Awakening / NA | NA | True |
| exact | `hypnotiz` Hypnotize / Biggy | Hypnotize / System of a Down | metal | True |
| exact | `fantasy` Fantasy / Appleton | Fantasy final / NA | NA | True |
| fuzzy | `crimesce` Crime Scene Investigation (Csi) /  | Crime scene / Young Eun Kim | NA | False |
| fuzzy | `scotland_2` Scotland National Anthem /  | National Anthem / National Anthem | rock | True |
| fuzzy | `mary_sbo` Mary's Boy Child / Bony M | Mary s Boy child SWING / Jesper Hairston Arranged by Elena | NA | False |
| fuzzy | `finalfan_2` Final Fantasy Victory /  | Fantasy final / NA | NA | True |
| fuzzy | `somewher` Somewhere Only We Know V2 / Keane | Somewhere Only We Know / Arr. Lakhvir Kumar | NA | True |
| fuzzy | `shooting` Shooting Star / DJ Hixxy | Shooting Stars / Arr. J. Morgan | rock | True |
| fuzzy | `whatifi` What If I / Pennywise | What happens if I... / Joey Conway | NA | True |
| fuzzy | `longnigh` Long Night / The Corrs | The Long Night / NA | NA | True |
| fuzzy | `frog` Frog / In Development | The Frog / NA | NA | True |
| fuzzy | `addamsfa_3` Addams Family 3 / Films And Tv | The Addams Family theme / NA | soundtrack | True |
| fuzzy | `hey_hey` Hey, Hey, My, My / Neil Young | Traditional music - Hey my Nanny / traditional highland pipe tune | NA | True |
| fuzzy | `classica` Classical / In Development | The Classical / NA | NA | True |
| fuzzy | `shapeofm` Shape Of My Heart / Backstreet Boys | A song in my heart - Florence W. Williams / Florence W. Williams | classical | False |
| fuzzy | `hereweco` Here We Come / Timbaland | Here We Come A-Wassailing / NA | folk | True |
| fuzzy | `bigbroth_4` Big Brother / Theme | Little Brother Big Sisters / NA | NA | True |
| fuzzy | `tearin_u` Tearin' Up My Heart / 'N Sync | A song in my heart - Florence W. Williams / Florence W. Williams | classical | False |
| fuzzy | `ibeginto_3` I Begin To Wonder / JCA | I Wonder / NA | NA | True |
| fuzzy | `lasvegas` Las Vegas (Hills Of Donegal) / Goats Dont Shave | Las Vegas - George Coles Stebbins / George Coles Stebbins 1878 | classical | False |
| fuzzy | `wherever_2` Wherever We Go / Crisis Crew | We Go! / One Piece | NA | True |
| fuzzy | `basstune` Bass Tune /  | Bass / NA | NA | True |

## 4. Темы Шифу напрямую в PDMX (подстрока в названии/композиторе/исполнителе)

| тема | партитур всего | в no_license_conflict | примеры (NLC) |
|---|---|---|---|
| Марио | 244 | 217 | Super Mario Athletic Theme [publicdomain; r=0.0]; Preludio No. 1 [cc-zero; r=0.0]; That One Annoying Super Mario Level [cc-zero; r=0.0] |
| Тетрис | 32 | 28 | Tetris: Theme C (with the Trio) (Clarinet Trio) [cc-zero; r=0.0]; Tetris Theme A Fast [cc-zero; r=0.0]; Tetris Theme [cc-zero; r=0.0] |
| Контра | 113 | 102 | choir-contralto [cc-zero; r=0.0]; Music Theory Composition [cc-zero; r=0.0]; The Contradiction Reel [publicdomain; r=0.0] |
| Зельда | 131 | 114 | Inside a House (Zelda) [cc-zero; r=0.0]; Legend of Zelda Overworld for Percussion Ensemble [cc-zero; r=0.0]; Molduga Battle [cc-zero; r=4.83] |
| Аладдин | 8 | 8 | MARCH IN ALADDIN. [publicdomain; r=0.0]; Friend Like Me (from Aladdin) [cc-zero; r=0.0]; Aladdin (Genesis) - Cave Of Wonders - Tommy Tallarico [cc-zero; r=0.0] |
| Интерстеллар | 9 | 8 | Interstellar Main Theme [cc-zero; r=4.9]; Drum Corps Ballad [cc-zero; r=0.0]; Where We're Going - Interstellar [cc-zero; r=4.66] |
| Терминатор | 1 | 1 | Love Scene from The Terminator [cc-zero; r=4.89] |
| Попкорн | 4 | 3 | Popcorn Behaviour [publicdomain; r=0.0]; The Popcorn [publicdomain; r=0.0]; The Popcorn [publicdomain; r=0.0] |
| Моцарт 40 | 482 | 423 | Mozart: Tuba Mirum [publicdomain; r=4.49]; the Blacksmith. [publicdomain; r=0.0]; KV 80 Minuetto [publicdomain; r=4.83] |
| Чайковский | 178 | 156 | Casse-Noisette Danse de la Fée Dragée [publicdomain; r=4.79]; Pas de deux from The Nutcracker- cello and piano [cc-zero; r=4.72]; The Nutcracker 12d. Russian Dance (Trepak Piano) [cc-zero; r=4.81] |
| Вивальди | 126 | 101 | Siciliano from L'Estro Armonico Op. 3 No. 11 [cc-zero; r=0.0]; Sonata No. 5 [cc-zero; r=0.0]; Sposa son disprezzata -joss [cc-zero; r=4.45] |
| Бах | 1550 | 1360 | Allt y Caethiwed bach [publicdomain; r=0.0]; BWV 862 The Well-Tempered Clavier Part I Fuga XVII [cc-zero; r=0.0]; BWV244 O Haupt Voll Blut Und Wunden [publicdomain; r=4.33] |

## 4a. Целое слово в названии/композиторе: лицензии и конфликт (кто реально лежит в no_license_conflict)

| фраза | всего | NLC | первые строки |
|---|---|---|---|
| super mario | 128 | 111 | Super Mario Athletic Theme — Koji Kondo / xMrPianox (A [publicdomain, conflict=False, r=0.0/0]<br>That One Annoying Super Mario Level — NA [cc-zero, conflict=False, r=0.0/0]<br>Tema de batalla de New Super Mario Bros — Shiho Fujiiarr. Jordi Hor [cc-zero, conflict=False, r=4.83/3]<br>Title Screen New Super Mario Bros. (DS) — from Asuka Ota Hajime Wak [cc-zero, conflict=False, r=4.87/13]<br>Super Mario World Overworld Theme (big band wip) — NA [cc-zero, conflict=False, r=0.0/0]<br>Super Mario Medley — Matt Nash [cc-zero, conflict=True, r=0.0/0]<br>Super Mario Sunshine: Dolpic Town — Koji Kondo arr. G. Oyenar [cc-zero, conflict=False, r=4.83/3]<br>Super Mario 2 Cello+Bass Duet — Composed by Koji KondoTra [cc-zero, conflict=False, r=4.37/5] |
| korobeiniki | 10 | 8 | Korobeiniki — NA [publicdomain, conflict=False, r=0.0/0]<br>Korobeiniki — NA [publicdomain, conflict=False, r=0.0/0]<br>A Theme — Traditional Russian Folk  [cc-zero, conflict=False, r=0.0/0]<br>Korobeiniki — Russian Folk Song 19th Ce [publicdomain, conflict=False, r=0.0/0]<br>Korobeiniki with ThisIsMeYes' arrangement — Russian Folk Song [publicdomain, conflict=True, r=0.0/0]<br>Tetris Theme (Korobeiniki) — Tobin Waling [cc-zero, conflict=False, r=0.0/0]<br>Korobeiniki (Tetris Theme Song) — Russian Folk Song 19th Ce [cc-zero, conflict=True, r=0.0/0]<br>Tetris Theme — arr. Tamara Sanyshyn Pete [cc-zero, conflict=False, r=4.25/13] |
| tetris | 27 | 24 | Tetris: Theme C (with the Trio) (Clarinet Trio) — J. S. Bach Transposed to  [cc-zero, conflict=False, r=0.0/0]<br>Tetris Theme A Fast — Tchaikovsky (1840 - 1893) [cc-zero, conflict=False, r=0.0/0]<br>Tetris Theme — Arranged By Braden WelshC [cc-zero, conflict=False, r=0.0/0]<br>Tetris — Composer [cc-zero, conflict=False, r=0.0/0]<br>Tetris 4 — Koof Ibi [cc-zero, conflict=False, r=0.0/0]<br>Drunk Tetris — soif [cc-zero, conflict=False, r=0.0/0]<br>Death By Harpsichord Extended (or Tetris Type Ra) — NA [cc-zero, conflict=False, r=0.0/0]<br>Tetris — NA [publicdomain, conflict=False, r=4.87/5] |
| contra | 12 | 12 | Danish Contra — NA [publicdomain, conflict=False, r=0.0/0]<br>Contra Danza — Author anonimo [cc-zero, conflict=False, r=4.83/3]<br>a CONTRA DANCE — NA [publicdomain, conflict=False, r=0.0/0]<br>Contra Blues (Contrabass Clarinet) — NA [cc-zero, conflict=False, r=0.0/0]<br>The MUSES. (A Contra Dance.) — NA [publicdomain, conflict=False, r=0.0/0]<br>Contra G Bugle Fingering Chart — SDSM&T [cc-zero, conflict=False, r=0.0/0]<br>Contra- Moon Hooch — NA [cc-zero, conflict=False, r=4.9/8]<br>Klezmer Contra — NA [publicdomain, conflict=False, r=0.0/0] |
| zelda | 124 | 109 | Inside a House (Zelda) — Koji Kondo [cc-zero, conflict=False, r=0.0/0]<br>Legend of Zelda Overworld for Percussion Ensemble — Arranged by Noah Walters [cc-zero, conflict=False, r=0.0/0]<br>Molduga Battle — Manaka Kataoka Yasuaki Iw [cc-zero, conflict=False, r=4.83/15]<br>The Legend of Zelda Arrangement — Koji Kondo [publicdomain, conflict=False, r=0.0/0]<br>The Legend of Zelda Medley for Brass Quartet — Arr. Dominic Francis [cc-zero, conflict=False, r=0.0/0]<br>Great Fairy Fountain (The Legend of Zelda: Breath of the Wil — Manaka Kataoka Yasuaki Iw [cc-zero, conflict=False, r=4.79/165]<br>A Zelda Medley — NA [publicdomain, conflict=False, r=0.0/0]<br>Kaepora_Gaebora_Zelda_Ocarina_of_Time V.2 — NA [cc-zero, conflict=False, r=0.0/0] |
| aladdin | 8 | 8 | MARCH IN ALADDIN. — NA [publicdomain, conflict=False, r=0.0/0]<br>Friend Like Me (from Aladdin) — NA [cc-zero, conflict=False, r=0.0/0]<br>Aladdin (Genesis) - Cave Of Wonders - Tommy Tallarico — NA [cc-zero, conflict=False, r=0.0/0]<br>Aladdin Medley for Brass Quintet — NA [cc-zero, conflict=False, r=3.83/3]<br>Friend Like Me — Arranged Alex Nam [cc-zero, conflict=False, r=0.0/0]<br>Friend Like Me — NA [cc-zero, conflict=False, r=0.0/0]<br>LE REVE BLEU de Aladdin — Arrgt 4 mains : S.Hermand [cc-zero, conflict=False, r=4.84/10]<br>Speechless From Disney's Aladdin — ALAN MENKENBENJ PASEKJUST [cc-zero, conflict=False, r=0.0/0] |
| interstellar | 7 | 6 | Interstellar Main Theme — Han ZimerVersion By Ricar [cc-zero, conflict=False, r=4.9/8]<br>Drum Corps Ballad — NA [cc-zero, conflict=False, r=0.0/0]<br>Interstellar for low brass Quartet — Hans Zimmer [cc-zero, conflict=False, r=4.82/19]<br>Interstellar — Hans Zimmer [cc-zero, conflict=False, r=4.33/3]<br>Interstellar — Hans Zimmer Arreglo DPMF [cc-zero, conflict=True, r=4.16/3]<br>Interstellar- No Time For Caution — NA [cc-zero, conflict=False, r=0.0/0]<br>Interstellar — Composed by Hans ZimmerAr [cc-zero, conflict=False, r=0.0/0] |
| terminator | 1 | 1 | Love Scene from The Terminator — Composed by: Brad Fiedel [cc-zero, conflict=False, r=4.89/15] |
| popcorn | 4 | 3 | The Popcorn Man (1910) — by JEAN SCHWARTZ [cc-zero, conflict=True, r=4.83/9]<br>Popcorn Behaviour — NA [publicdomain, conflict=False, r=0.0/0]<br>The Popcorn — NA [publicdomain, conflict=False, r=0.0/0]<br>The Popcorn — NA [publicdomain, conflict=False, r=0.0/0] |
| symphony 40 | 0 | 0 |  |
| nutcracker | 41 | 36 | Pas de deux from The Nutcracker- cello and piano — P. I. Tchaikovsky(arr. T. [cc-zero, conflict=False, r=4.72/47]<br>The Nutcracker 12d. Russian Dance (Trepak Piano) — Hinci [cc-zero, conflict=False, r=4.81/13]<br>Dance of the Merlitons — P. TchaikovskyAdapted for [publicdomain, conflict=False, r=4.88/6]<br>The Nutcracker 12c. Chinese Dance (Tea Piano) — Hinci [cc-zero, conflict=False, r=4.76/35]<br>Dance of the Sugar Plum Fairy — Piotr Iljicz Czajkowski [publicdomain, conflict=False, r=4.6/108]<br>Dance Of The Sugar Plum Fairy — Arr. Jeremiah Edmiston [cc-zero, conflict=False, r=4.81/8]<br>The Nutcracker Children's Galop — Peter Illyich Tchaikovsky [cc-zero, conflict=True, r=0.0/0]<br>Arabian Dance for clarinet quartet from The Nutcracker by P. — P. I. Tsjaikovski [cc-zero, conflict=True, r=4.75/9] |
| four seasons | 28 | 24 | The Four Seasons THO4.053 — NA [publicdomain, conflict=False, r=0.0/0]<br>The Four Seasons (Complete) — Antonio Vivaldi (1678-174 [publicdomain, conflict=True, r=4.75/768]<br>The Four Seasons — NA [publicdomain, conflict=False, r=0.0/0]<br>Spring from Vivaldi's Four Seasons — Antonio Vivaldi (1678-174 [cc-zero, conflict=True, r=4.62/5]<br>Four Seasons. VWMLa.077 — NA [publicdomain, conflict=False, r=0.0/0]<br>Four Seasons — anon. [publicdomain, conflict=False, r=0.0/0]<br>Bb Simplified Vivaldi Four Seasons Complete Trumpet Clarinet — Antonio Vivaldi (1678-174 [cc-zero, conflict=True, r=4.66/3]<br>The Four Seasons — NA [publicdomain, conflict=False, r=0.0/0] |
| toccata | 41 | 31 | Lanquetuit Toccata in D Major — Marcel Lanquetuit [cc-zero, conflict=True, r=4.49/3]<br>Toccata on Thaxted — Ben Yanke [publicdomain, conflict=False, r=4.85/4]<br>Prelude and Toccata — Vinzenz Lachner [cc-zero, conflict=False, r=0.0/0]<br>Frescobaldi: Toccata per l'Elevatione (1635) - La Messa dell — Girolamo Frescobaldi [cc-zero, conflict=True, r=0.0/0]<br>Toccata in Seven — John Rutter [cc-zero, conflict=False, r=0.0/0]<br>Ellipsis — NA [cc-zero, conflict=False, r=0.0/0]<br>Toccata in Seven — John RutterArr. by Corina [cc-zero, conflict=False, r=0.0/0]<br>Toccata — Bruno de Lorenzo.Niterói  [cc-zero, conflict=False, r=0.0/0] |

## 5. Лицензии PDMX

`license` по всем строкам: [('publicdomain', 210364), ('cc-zero', 43713)]; `license_conflict`: [('False', 222856), ('True', 31221)]; в NLC `is_original`: [('False', 215306), ('True', 7550)]; NLC ∩ `subset:deduplicated`: 77354; NLC ∩ `subset:rated_deduplicated`: 10271.

---

## Приложение A. Как получено (воспроизводимость)

- katana (`ros2-Katana-GF66-11UD`, Python 3.10.12), каталог `~/pdmx/`. Скачано с Zenodo `10.5281/zenodo.15571083` (API `files`): `PDMX.csv` 225 399 738 байт (md5 `30392ccf38bb63ce70e7afae70f9c88c`), `metadata.tar.gz` 159 444 765 (md5 `5bc79445090dd2fe5e96cffa77a3461c`), `subset_paths.tar.gz` 29 258 714 (md5 `092eee416ece8060f77d08575b94a43d`). Остальные файлы записи (`mxl.tar.gz` 1.89 ГБ, `data.tar.gz` 2.24 ГБ, `mid.tar.gz` 214 МБ, `pdf.tar.gz` 9.6 ГБ) **не качались** (В3 ADR-0154 — по слову Шифу).
- Команда: `PYTHONIOENCODING=utf-8 python3 pdmx_coverage.py --csv PDMX.csv --metadata metadata.tar.gz --meta-sample 20000 --lib rtttl_melodies.jsonl.gz --out pdmx_coverage.md --matches pdmx_matches.jsonl`; stderr: `csv: 254077 rows, nlc 222856, 3s` / `metadata: 20000 files, 8s` / `coverage: {'n': 10461, 'exact_artist': 433, 'exact': 1339, 'fuzzy': 1164, 'generic': 419, 'none': 7106, 'any': 2936, 'any_nlc': 2451}, 40s`. md5 отчёта на katana `b4b596cb22bef3618c72dc35aa9b0982`; разделы 1–5 выше — копия без правок.
- `subset_paths.tar.gz`: `all.txt`, `rated.txt`, `deduplicated.txt`, `rated_deduplicated.txt`, `no_license_conflict.txt`, `all_valid.txt` — те же флаги, что колонки `subset:*` CSV (списки путей).
- Первый прогон (до ужесточения правил): `any` 3513 (33.6 %) — «Unknown» ↔ «Unknown» ×28 и вложение любых двух токенов (гимны XIX века вбирали «Not yet», «In My Head»). Правила ужесточены: `GENERIC_TITLES`, `CONTAIN_SLACK = 2`; числа выше — после.
- Лога частоты запросов тем (топ-100) в репо не нашёл (`scripts/music/live_dj/runs.sh` — 5 фиксированных тем); срез «узнаваемые» — теги архива + список тем Шифу из брифа.

## Приложение B. Точность сопоставления RTTTL → PDMX (ручной подсчёт по §3a — мои глаза, не Шифу)

Считал «то же произведение» (аранжировка/версия допускается), «не то» и «не уверен» по 20 парам каждого уровня из §3a. Критерий — название + автор/контекст, ноты не сверял.

| уровень | всего | то же произведение | не уверен | не то | оценка точности |
|---|---|---|---|---|---|
| `exact_artist` | 433 | 14 (`whatawon`, `paintitb_2`, `lacucara_3`, `theenter`, `entersan`, `purplera`, `taintedl_3`, `madworld_2`, `boysdont`, `forevery_2`, `jinglebe`, `littledr`, `septembe`, `septembe_2`) | 2 (`withouty`, `runaway_5`) | 4 (`crazyinl_3` → Gnarls Barkley; `sonicthe_7`/`sonicthe_8` → другая зона той же игры; `gimmegim` → шведская пародия) | **≈ 0.7–0.8** |
| `exact` | 1339 | 5 (`oldrugge`, `baabaabl`, `vaderjac`, `scoobydo`, `hardknoc`) | 4 (`therisin`, `happyday_3`, `getready`, `awakenin_2`) | 11 (одноимённые чужие песни: `dreams2` Cranberries, `hypnotiz` SoaD, `girls_3`, `smile`, `silence_4`, `happy_3`, `beautifu_15`, `startrek_5` другой фильм, `motherea`, `we_rock_57`, `fantasy`) | **≈ 0.25–0.45** |
| `fuzzy` | 1164 | 3 (`mary_sbo`, `somewher`, `addamsfa_3`) | 1 (`longnigh`) | 16 | **≈ 0.15–0.2** |

Ожидаемое число записей RTTTL, у которых в PDMX есть **та же** пьеса: 433×0.75 + 1339×0.35 + 1164×0.17 ≈ **325 + 470 + 200 ≈ 1 000 (≈ 9–10 % библиотеки)**; верхняя граница по названию — 2 936 (28.1 %). Для тегов `christmas`/`classical`/`folk`/`game` доля по названию выше (§3: 52–70 %), для `tv` — 227 из 636 записей называются «Theme» и по названию не сопоставимы вовсе.

Вывод замера: связь «запись RTTTL ↔ партитура PDMX» по названию ненадёжна и покрывает ≈ 1/10 библиотеки; **полезный путь — поиск партитуры по теме запроса напрямую в PDMX** (§4/§4a: Марио 111 целословно, Зельда 109, Щелкунчик 36, Тетрис/Коробейники 24+8, Интерстеллар 6, Аладдин 8; Терминатор — 1, и это «Love Scene», не главная тема; Попкорн (Kingsley) — 0, три «The Popcorn» — народные пьесы; Контра — 0, двенадцать «contra» — контрдансы).

## Приложение C. Что из полей закрывает PDMX (по §1–§2)

| поле | PDMX.csv (NLC) | metadata JSON | примечание |
|---|---|---|---|
| название | `title` 100 %, `song_name` 94.9 % | `score.title` 100 % | `title` часто «Название - Композитор», `song_name` чище |
| композитор | `composer_name` 27.3 % | `score.composer_name` 36.7 % | топ: `anon.`, `Trad.`, `Composer` — значимых меньше |
| исполнитель | `artist_name` 94.9 % | `score.artist_name` 93.2 % | часто `Misc tunes`, `Misc Traditional`, `Misc Computer Games` — категория, как в RTTTL |
| жанр | `genres` 26.2 % | `data.genres` 32.4 % | classical 41 395, folk 9 619, soundtrack 2 210, rock 878, pop 624 … |
| теги | `tags` 6.5 % | `score.tags` 9.1 % | свободный текст |
| тональность (знаки) | — | `score.keysig` **100 %** («G major, E minor» — пара знаков, лад не различён) | лад — из нот |
| размер, темп | — | — | только в MXL/`data` JSON (не качали) |
| год произведения | — | — (`date_created` — дата загрузки) | дыра |
| рейтинг/популярность | `rating`, `n_ratings`, `n_views` 100 % | те же | rating ≥ 4 ∧ n_ratings ≥ 3 в NLC: **10 763**; `subset:rated_deduplicated` ∩ NLC: 10 271 |
| лицензия | `license` ∈ {publicdomain 210 364, cc-zero 43 713}; `license_conflict` True 31 221 | `is_public_domain` | **метка загрузившего**: в NLC лежат «Interstellar Main Theme» (Циммер 2014, cc-zero), «Super Mario Athletic Theme» (Кондо, publicdomain), «Love Scene from The Terminator» (Фидель) — PDMX-флаг не равен юридическому PD |
| длина, сложность, число нот | 100 % | `score.measures`, `duration`, `complexity` | фильтры импорта |

