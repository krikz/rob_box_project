# WIP: t_dd49849b fix in progress

Investigating why L: Build All Services hangs in_progress 2.7h+ on the
`build-vision-services / update-image-versions` job step.

Evidence collected (raw):
- 3 concurrent runs: 32825697105 (round-225), 32824953210 (develop), 32814362039 (round-224)
- All stuck on `Tag with SHA and update .image-visions` step
- Main update pushed (`ci: main SHA tags → test-69b27a7 [skip ci]` in round-225)
- Vision update did NOT push — `.image-visions.test` still has tags from round-219
- PR #1566 concurrency gate did NOT cause this (different refs = different groups, all 3 passed gate)
- Real cause: `scripts/ci/push-image-versions.sh` does `git pull --rebase` + `git push` with no socket
  timeout and no job-level `timeout-minutes`. With 3 concurrent runners on the same remote ref,
  network hangs forever.

Fix (WIP):
- Add `timeout-minutes: 10` to `update-image-versions` jobs in both Vision and Main workflows
- Add `timeout-minutes: 60` to summary jobs
- Cancel the 3 stuck runs immediately

Full fix + PR to follow.