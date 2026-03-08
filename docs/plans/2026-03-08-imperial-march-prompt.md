# Imperial March Prompt Update Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Improve the music prompt so DJ transitions can keep `Imperial March` recognizable beyond the opening motif.

**Architecture:** Update the music skill prompt only. Replace the current short `Imperial March` snippet with a section-based recipe that teaches the model to build a recognizable form (`A → A' → bridge → B`) and to preserve the motif during Star Wars DJ transitions.

**Tech Stack:** Prompt engineering, FoxDot / Renardo, dialogue-driven DJ transitions.

---

### Task 1: Replace the Imperial March guidance

**Files:**
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Step 1: Inspect current melody-library section**

Locate the existing `Imperial March` entry and the nearby recognizable-melody rules.

**Step 2: Write the new prompt content**

Add:
- anti-loop rule (`don't replay only the first phrase`)
- fixed form (`A → A' → chromatic bridge → B`)
- separate `DJ recognisable arrangement` guidance
- separate `extended / concert-like` guidance
- pizzicato / arco transition guidance

**Step 3: Keep the guidance compact**

Ensure the prompt remains practical for the model:
- no full score dump
- short motif arrays only where needed
- preserve existing style and surrounding examples

**Step 4: Verify the section reads clearly**

Re-read the updated block and confirm it gives the model a deterministic path for Star Wars requests.

**Step 5: Commit**

Commit message:

```bash
git commit -m "fix(voice): expand Imperial March prompt guidance"
```

### Task 2: Verify and deploy prompt change

**Files:**
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Step 1: Diff the prompt**

Confirm the new `Imperial March` block replaced the short motif-only version.

**Step 2: Push branch**

```bash
git pull --rebase
git push
```

**Step 3: Deploy to robot**

```bash
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project && git pull && docker restart voice-assistant'
```

**Step 4: Smoke-check logs**

Confirm the container restarted successfully.
