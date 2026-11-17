"""Планировщик ресурсов (ADR-0150 §5.6): кто занял голос/лицо/SFX/музыку/движение и что делать с новым действием.

* :mod:`.resources` — :class:`ResourceState` по событиям владельцев (статус задачи — только от владельца);
* :mod:`.composition` — :func:`compose` строго по ``knowledge.COMPOSITION`` + Tier-1 + ``when`` LLM;
* :mod:`.plan_events` — ``PlanEvent`` (``queued{after, eta}``, ``rejected``) и фраза «Поставлю после …» кодом;
* :mod:`.estimate` — ETA речи по ``SpeechEvent``;
* :mod:`.perform` — выступление (§5.6.3): сегменты, пред-синтез, подложка, правка, «хватит»/«стой»;
  :mod:`.segments` — реплики выступления и граница сегмента (О7), :mod:`.backing` — подложка, :mod:`.sync` — старт
  на такте.

Чистый Python: транспорт (ROS) — у хоста.
"""
