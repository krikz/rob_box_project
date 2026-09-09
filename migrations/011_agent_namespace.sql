-- ============================================================================
-- Migration: 011_agent_namespace.sql
-- Purpose:   ADR-0083 §2.4 / ADR-0055 Phase 2 — unified memory namespace.
--
-- After this migration, both dialogue_node (personality) and supervisor_node
-- (operator/ТАРС) write into the SAME SQLite file `/data/harness_voice.db`
-- and the `agent` column on every `facts` row identifies the owner.
-- The previous third database `/data/operator_memory.db` is removed by the
-- supervisor_node change (issue #2111 closure).
--
-- This file is a marker (mirroring 010_voice_memory_unify.sql): the actual
-- DDL is applied idempotently inside ``SQLiteVoiceMemory.init()`` via
-- ``PRAGMA table_info(facts)`` checks + ``ALTER TABLE ... ADD COLUMN``.
-- That keeps the migration available to the schema_migrations framework
-- (for forensics / downgrade tooling) while guaranteeing that an existing
-- DB opened by an older binary is upgraded without forcing the operator
-- to run a separate ``data-migrate`` step.
--
-- Backfill:
--   * ``scope = 'mcp:legacy'`` (VoiceMemoryAdapter Phase 1 rows) → agent='personality'
--   * ``scope LIKE 'operator.%'``                                 → agent='operator'
--   * everything else                                              → agent='personality'
--
-- The default ('default') is left for legacy rows that pre-date both
-- adapters; a future shrink-wrap script may re-classify them once Shifu
-- is sure no legacy writer is still around. ADR-0083 §2.4 carries this
-- note verbatim.
-- ============================================================================

INSERT OR IGNORE INTO voice_memory_meta (key, value)
VALUES ('migration_011_applied_at', strftime('%s', 'now'));

-- Marker only. The actual column ADD + index creation happens in
-- ``SQLiteVoiceMemory.init()`` (idempotent inline). See ADR-0083 §2.4.
