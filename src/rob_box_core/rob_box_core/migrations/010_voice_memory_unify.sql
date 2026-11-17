-- ============================================================================
-- Migration: 010_voice_memory_unify.sql
-- Purpose:   ADR-0055 Phase 1 marker — path consolidation.
--
-- After this migration runs against /data/voice_memory.db, MCP tools
-- (mcp_server.py, waypoint_store.py) write to /data/harness_voice.db
-- through VoiceMemoryAdapter, not /data/voice_memory.db. The old file
-- stays on the volume read-only until Shifu explicitly removes it.
--
-- This file does NOT execute data-migration. data-migration = Phase 2,
-- separate card after merge of AgentCore (step 03) which will add the
-- `agent` namespace column.
--
-- Why a migration file if it's only a marker?
--   The schema_migrations framework (migrations/001_init.sql etc.) requires
--   a row per applied step. The marker keeps the framework aware that
--   the path was switched, so:
--     * downgrade tooling skips the marker gracefully,
--     * evidence reports (agent-flow) see the step recorded,
--     * forensic on /data/voice_memory.db can prove when the legacy
--       writes stopped.
-- ============================================================================

INSERT OR IGNORE INTO voice_memory_meta (key, value)
VALUES ('migration_010_applied_at', strftime('%s', 'now'));

-- Marker only. No DDL changes. See ADR-0055 §2.5.