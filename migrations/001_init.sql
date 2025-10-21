-- 001_init.sql
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