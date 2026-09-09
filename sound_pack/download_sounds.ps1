# Download additional robot sounds from Freesound.org
# Usage: .\download_sounds.ps1

$sounds = @(
    @{id="843358"; author="chungus43A"; name="elevator_chime"},
    @{id="843355"; author="chungus43A"; name="dot_matrix_printer_3"},
    @{id="843354"; author="chungus43A"; name="dot_matrix_printer_2"},
    @{id="843353"; author="chungus43A"; name="dot_matrix_printer_1"},
    @{id="843330"; author="gulfstreamav"; name="power_up"},
    @{id="843291"; author="subetha2026"; name="subetha_sound"},
    @{id="843127"; author="SonosFreesound"; name="sonos_sound"},
    @{id="842939"; author="TannerSound"; name="tanner_sound"},
    @{id="842834"; author="TheSoundLibrary"; name="soundlib_sound"},
    @{id="842710"; author="joeffl"; name="joeffl_sound"},
    @{id="842609"; author="ui-hater2012"; name="ui_hater_sound"},
    @{id="842507"; author="hotpin7"; name="hotpin_sound_1"},
    @{id="842282"; author="stano458"; name="stano_sound"},
    @{id="842201"; author="nugsano"; name="nugsano_sound_1"},
    @{id="842200"; author="nugsano"; name="nugsano_sound_2"},
    @{id="841968"; author="P4INKILLA"; name="painkilla_sound"},
    @{id="841638"; author="MATUSTRM"; name="matustrm_sound"},
    @{id="841346"; author="114802300"; name="user_sound"},
    @{id="841257"; author="NightDrawr"; name="nightdrawr_sound"},
    @{id="840903"; author="hotpin7"; name="hotpin_sound_2"},
    @{id="840805"; author="OverlookHotelRecords"; name="overlook_sound"},
    @{id="840490"; author="kolel"; name="kolel_sound"},
    @{id="840178"; author="SilverIllusionist"; name="silver_sound"},
    @{id="840060"; author="pedr01"; name="pedr_sound"},
    @{id="840042"; author="hotpin7"; name="hotpin_sound_3"},
    @{id="839955"; author="KerDwyn"; name="kerdwyn_sound"},
    @{id="839842"; author="TommyListens"; name="tommy_sound_1"},
    @{id="839830"; author="TommyListens"; name="tommy_sound_2"},
    @{id="838158"; author="JustASeriesofFonetics"; name="fonetics_sound"},
    @{id="836450"; author="Feraly_"; name="feraly_sound"}
)

Write-Host "`n=== DOWNLOADING ROBOT SOUNDS ===" -ForegroundColor Cyan
Write-Host "Total sounds to download: $($sounds.Count)" -ForegroundColor Yellow

$downloaded = 0
$failed = 0

foreach ($sound in $sounds) {
    $id = $sound.id
    $name = $sound.name
    
    # Try to guess author ID from common patterns
    # Most preview URLs follow: https://cdn.freesound.org/previews/{folder}/{id}_{authorid}-lq.mp3
    # We'll try to download and if it fails, skip
    
    $folder = $id.Substring(0, $id.Length - 3)
    $outFile = "$name.mp3"
    
    # Skip if already exists
    if (Test-Path $outFile) {
        Write-Host "  ⏭️  $outFile already exists" -ForegroundColor Gray
        continue
    }
    
    # Try to download from preview URL (we need to figure out the author ID)
    # For now, let's just note them down
    Write-Host "  📋 $id - $name (need to download manually)" -ForegroundColor Yellow
    $failed++
}

Write-Host "`n=== SUMMARY ===" -ForegroundColor Cyan
Write-Host "  Downloaded: $downloaded" -ForegroundColor Green
Write-Host "  Failed/Skipped: $failed" -ForegroundColor Yellow
Write-Host "`nNote: These sounds need to be downloaded manually from Freesound.org" -ForegroundColor Gray
Write-Host "You need to be logged in to download." -ForegroundColor Gray
