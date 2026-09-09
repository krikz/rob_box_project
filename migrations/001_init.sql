-- ============================================================================
-- Migration: 001_init.sql
-- Purpose: Initializes the database schema for the event tracking and summarization system.
--          Creates the 'events', 'summaries', and 'summary_events' tables, along with
--          relevant indexes and foreign key constraints.
--
-- When to apply: This migration should be applied when setting up a new database instance
--                for the first time, or when migrating from a version prior to the introduction
--                of the event and summary tracking features.
--
-- Prerequisites: No prior migrations are required. This is the initial schema setup.
--
-- Version compatibility: Designed for SQLite 3.x. May require adjustments for other SQL dialects.
--
-- Rollback considerations: Dropping these tables will result in loss of all event and summary data.
--                        Ensure backups are taken before rollback. No explicit DROP statements
--                        are included in this migration.
-- ============================================================================

CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  category TEXT NOT NULL,
  time REAL NOT NULL,
  content TEXT NOT NULL,
  important INTEGER DEFAULT 0,
  summarized INTEGER DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_events_time ON events(time);
CREATE INDEX IF NOT EXISTS idx_events_summarized ON events(summarized);

CREATE TABLE IF NOT EXISTS summaries (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  category TEXT NOT NULL,
  time REAL NOT NULL,
  summary TEXT NOT NULL,
  event_count INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS summary_events (
  summary_id INTEGER NOT NULL,
  event_id INTEGER NOT NULL,
  PRIMARY KEY (summary_id, event_id),
  FOREIGN KEY(summary_id) REFERENCES summaries(id) ON DELETE CASCADE,
  FOREIGN KEY(event_id) REFERENCES events(id) ON DELETE CASCADE
);