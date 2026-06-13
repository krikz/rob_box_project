-- ============================================================================
-- Migration: 009_cleanup_presets.sql
-- Purpose:   Remove the 7 genre presets added by 006 (now deleted from source).
--            Keep only: csm_132_full_track (004) + club_energy_128bpm (008).
--
-- Version: 9  (follows 008_*.sql)
-- ============================================================================

DELETE FROM music_tracks WHERE name IN (
    'star_wars_march',
    'jazz_groove_96bpm',
    'acid_techno_136bpm',
    'ambient_dreamy_88bpm',
    'hiphop_boom_bap',
    'folk_fiddle_reel',
    'electro_funk_115bpm'
);
