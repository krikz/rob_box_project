"""Планировщик ресурсов (ADR-0150 §5.6): кто занял голос/лицо/SFX/музыку/движение и что делать с новым действием.

* :mod:`.resources` — :class:`ResourceState` по событиям владельцев (статус задачи — только от владельца);
* :mod:`.composition` — :func:`compose` строго по ``knowledge.COMPOSITION`` + Tier-1 + ``when`` LLM;
* :mod:`.plan_events` — ``PlanEvent`` (``queued{after, eta}``, ``rejected``) и фраза «Поставлю после …» кодом;
* :mod:`.estimate` — ETA речи по ``SpeechEvent``.

Чистый Python: транспорт (ROS) — у хоста.
"""
