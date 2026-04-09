-- ============================================================================
-- Migration: 005_faq.sql
-- Purpose:   Event FAQ knowledge store for FAQ mode in the voice assistant.
--            Stores question/answer pairs per event with FTS5 indexing.
--            Vector table is created lazily by faq_store.py when embeddings are available.
-- ============================================================================

CREATE TABLE IF NOT EXISTS faq_items (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id   TEXT    NOT NULL,
    question   TEXT    NOT NULL,
    answer     TEXT    NOT NULL,
    category   TEXT    NOT NULL DEFAULT 'general',
    source     TEXT    NOT NULL DEFAULT '',
    indexed_at TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_faq_items_event_id ON faq_items(event_id);
CREATE INDEX IF NOT EXISTS idx_faq_items_category ON faq_items(category);

CREATE VIRTUAL TABLE IF NOT EXISTS faq_items_fts USING fts5(
    question,
    answer,
    category,
    source,
    content       = faq_items,
    content_rowid = id,
    tokenize      = 'unicode61'
);

CREATE TRIGGER IF NOT EXISTS faq_items_ai
AFTER INSERT ON faq_items BEGIN
    INSERT INTO faq_items_fts(rowid, question, answer, category, source)
    VALUES (new.id, new.question, new.answer, new.category, new.source);
END;

CREATE TRIGGER IF NOT EXISTS faq_items_au
AFTER UPDATE OF question, answer, category, source ON faq_items BEGIN
    INSERT INTO faq_items_fts(faq_items_fts, rowid, question, answer, category, source)
        VALUES ('delete', old.id, old.question, old.answer, old.category, old.source);
    INSERT INTO faq_items_fts(rowid, question, answer, category, source)
        VALUES (new.id, new.question, new.answer, new.category, new.source);
END;

CREATE TRIGGER IF NOT EXISTS faq_items_ad
AFTER DELETE ON faq_items BEGIN
    INSERT INTO faq_items_fts(faq_items_fts, rowid, question, answer, category, source)
        VALUES ('delete', old.id, old.question, old.answer, old.category, old.source);
END;