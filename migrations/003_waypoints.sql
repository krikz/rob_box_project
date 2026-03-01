-- ============================================================================
-- Migration: 003_waypoints.sql
-- Purpose:   Dynamic waypoint management with map association.
--            Replaces hardcoded WAYPOINTS dict in navigation.py / command_node.py.
--
-- Tables:
--   maps       — each mapping session gets a unique ID; one is "active"
--   waypoints  — named poses (x, y, theta) tied to a specific map
--
-- Stored in the same SQLite file as voice_memory (VOICE_MEMORY_DB_PATH).
--
-- Version: 3  (follows 002_voice_memory.sql)
-- ============================================================================

-- ----------------------------------------------------------------------------
-- Maps — one row per mapping session
-- ----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS maps (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id     TEXT    NOT NULL UNIQUE,            -- UUID v4
    name       TEXT,                               -- optional human-friendly name ("квартира")
    created_at REAL    NOT NULL,                   -- Unix epoch (float)
    is_active  INTEGER NOT NULL DEFAULT 0          -- 1 = current active map
);

CREATE INDEX IF NOT EXISTS idx_maps_active ON maps(is_active);

-- ----------------------------------------------------------------------------
-- Waypoints — named locations on a specific map
-- ----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS waypoints (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id     TEXT    NOT NULL,                   -- FK → maps.map_id
    name       TEXT    NOT NULL,                   -- human name ("кухня", "зал")
    x          REAL    NOT NULL,                   -- position in map frame (metres)
    y          REAL    NOT NULL,
    theta      REAL    NOT NULL DEFAULT 0.0,       -- orientation (radians)
    created_at REAL    NOT NULL,
    updated_at REAL    NOT NULL,
    UNIQUE(map_id, name),                          -- no duplicate names per map
    FOREIGN KEY (map_id) REFERENCES maps(map_id) ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_wp_map ON waypoints(map_id);
