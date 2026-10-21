"""Остаток пакета планировщика v1: только пред-синтез речи :mod:`.pregen` (ADR-0056), его удаляет PR-7c (ADR-0150 §5.6.3).

Каналы VOICE/MUSIC/ANIM (``TaskScheduler``), ``SchedulerToolExecutor``, тул правки сегментов и ``quick_decide`` удалены в
PR-7b (#3699): порядок и одновременность действий решает ``rob_box_dialog.plan`` по таблице ``knowledge.COMPOSITION``,
статус задачи — событие владельца ресурса.
"""
