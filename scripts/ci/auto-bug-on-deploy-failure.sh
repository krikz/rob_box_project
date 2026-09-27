#!/usr/bin/env bash
set -o pipefail

# Evidence collector for a Docker Compose crash during deployment.
# Usage:
#   auto-bug-on-deploy-failure.sh RUN_ID NODE SERVICE_NAME CONTAINER_ID [RUN_STARTED_AT] [FAILED_AT] [RUN_URL]
# SERVICE_NAME and CONTAINER_ID may be "-" to auto-detect failed compose containers.

if [[ $# -lt 4 ]]; then
  echo "usage: $0 RUN_ID NODE SERVICE_NAME CONTAINER_ID [RUN_STARTED_AT] [FAILED_AT] [RUN_URL]" >&2
  exit 2
fi

RUN_ID="$1"
NODE="$2"
REQUESTED_SERVICE="$3"
REQUESTED_CONTAINER="$4"
RUN_STARTED_AT="unknown"
FAILED_AT="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
RUN_URL="https://github.com/$GITHUB_REPOSITORY/actions/runs/$RUN_ID"
[[ $# -ge 5 ]] && RUN_STARTED_AT="$5"
[[ $# -ge 6 ]] && FAILED_AT="$6"
[[ $# -ge 7 ]] && RUN_URL="$7"

REPO="$GITHUB_REPOSITORY"
SSH_OPTS='-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3'
[[ -n "$SSH_OPTS_OVERRIDE" ]] && SSH_OPTS="$SSH_OPTS_OVERRIDE"

case "$NODE" in
  vision_pi)
    NODE_IP="$VISION_PI_IP"
    COMPOSE_DIR='$HOME/rob_box_project/docker/vision'
    PROJECT_NAME="vision"
    ;;
  main_pi)
    NODE_IP="$MAIN_PI_IP"
    COMPOSE_DIR='$HOME/rob_box_project/docker/main'
    PROJECT_NAME="main"
    ;;
  *)
    echo "Unsupported node: $NODE" >&2
    exit 2
    ;;
esac

ssh_remote() {
  sshpass -p "$SSHPASS" ssh $SSH_OPTS "ros2@$NODE_IP" "$@"
}

ensure_label() {
  local name="$1"
  local color="$2"
  local description="$3"
  gh label create "$name" --repo "$REPO" --color "$color" --description "$description" --force >/dev/null 2>&1 || true
}

ensure_label "bug" "E99695" "Something isn't working"
ensure_label "deploy-failure" "B60205" "Automatic deployment failure"
ensure_label "auto-generated" "C5DEF5" "Created automatically by CI"
ensure_label "priority:high" "B60205" "High priority"
ensure_label "agent:devops" "5319E7" "DevOps agent work"

run_start_epoch=0
if [[ "$RUN_STARTED_AT" != "unknown" ]]; then
  run_start_epoch="$(date -d "$RUN_STARTED_AT" +%s 2>/dev/null || echo 0)"
fi

CONTAINERS="$(ssh_remote "docker ps -aq --filter label=com.docker.compose.project=$PROJECT_NAME" 2>/dev/null || true)"
if [[ -n "$REQUESTED_CONTAINER" && "$REQUESTED_CONTAINER" != "-" ]]; then
  CONTAINERS="$REQUESTED_CONTAINER"
fi

PS_ALL="$(ssh_remote "docker ps -a --no-trunc" 2>&1 || true)"
[[ -n "$PS_ALL" ]] || PS_ALL="[docker ps -a failed: SSH/remote Docker unavailable]"

created_any=0

while IFS= read -r container_id; do
  [[ -n "$container_id" ]] || continue

  meta="$(ssh_remote "docker inspect --type container '$container_id' --format '{{.Id}}|{{.Name}}|{{.Config.Image}}|{{.State.Status}}|{{.State.ExitCode}}|{{.State.FinishedAt}}|{{.State.Error}}|{{.State.OOMKilled}}'" 2>/dev/null || true)"
  [[ -n "$meta" ]] || continue

  IFS='|' read -r full_id container_name image_ref state exit_code finished_at state_error oom_killed <<< "$meta"
  service="$(ssh_remote "docker inspect --type container '$container_id' --format '{{index .Config.Labels \"com.docker.compose.service\"}}'" 2>/dev/null || true)"
  [[ -n "$service" ]] || service="$REQUESTED_SERVICE"
  [[ -n "$service" && "$service" != "-" ]] || service="unknown-service"

  [[ "$REQUESTED_SERVICE" == "-" || "$service" == "$REQUESTED_SERVICE" ]] || continue
  [[ "$state" != "running" ]] || continue
  [[ "$exit_code" =~ ^[0-9]+$ ]] || exit_code=0
  (( exit_code != 0 )) || continue

  finished_epoch=0
  if [[ "$finished_at" != "0001-01-01T00:00:00Z" ]]; then
    finished_epoch="$(date -d "$finished_at" +%s 2>/dev/null || echo 0)"
  fi
  if (( run_start_epoch > 0 && finished_epoch > 0 && finished_epoch < run_start_epoch - 60 )); then
    continue
  fi

  image_sha="$(ssh_remote "docker image inspect '$image_ref' --format '{{json .RepoDigests}}'" 2>/dev/null | jq -r '.[0] // empty' 2>/dev/null | sed 's/.*@//' || true)"
  [[ -n "$image_sha" ]] || image_sha="$(ssh_remote "docker inspect --type container '$container_id' --format '{{.Image}}'" 2>/dev/null || true)"
  [[ -n "$image_sha" ]] || image_sha="unavailable"

  dedup_key="$image_sha:$service"
  dedup_marker="<!-- deploy-dedup-key: $dedup_key -->"

  existing="$(gh issue list --repo "$REPO" --state open --label deploy-failure --limit 200 --json number,body,url 2>/dev/null |
    jq -r --arg key "$dedup_marker" '.[] | select(.body | contains($key)) | [.number,.url] | @tsv' |
    head -1 || true)"

  logs="$(ssh_remote "docker logs --tail 50 '$container_id' 2>&1" 2>&1 || true)"
  [[ -n "$logs" ]] || logs="[docker logs unavailable]"

  signal="none"
  if (( exit_code >= 128 && exit_code <= 255 )); then
    signal="SIG$((exit_code - 128)) (derived from exit code)"
  fi

  reason="$state_error"
  [[ -n "$reason" ]] || reason="container exited with code $exit_code"

  oom_value="$oom_killed"
  [[ -n "$oom_value" ]] || oom_value="false"

  body_file="$(mktemp)"
  {
    echo "## Deploy run #$RUN_ID"
    echo
    echo "$dedup_marker"
    echo "- Workflow: L-Deploy and Verify"
    echo "- Started: $RUN_STARTED_AT"
    echo "- Failed at: $FAILED_AT"
    echo "- Run: $RUN_URL"
    echo "- Node: $NODE ($NODE_IP)"
    echo "- Service: $service"
    echo "- Container: $full_id"
    echo "- Image: $image_ref"
    echo "- Image SHA: $image_sha"
    echo "- Exit code: $exit_code"
    echo "- Signal: $signal"
    echo "- OOMKilled: $oom_value"
    echo "- State error: $reason"
    echo
    echo "## Docker logs (last 50)"
    echo
    echo '~~~text'
    printf '%s\n' "$logs"
    echo '~~~'
    echo
    echo "## docker ps -a"
    echo
    echo '~~~text'
    printf '%s\n' "$PS_ALL"
    echo '~~~'
    echo
    echo "## Previous successful run"
    previous_run="$(gh run list --repo "$REPO" --workflow "L-Deploy and Verify.yml" --status success --limit 50 --json databaseId,url,headBranch |
      jq -r --arg branch "$GITHUB_REF_NAME" 'map(select(.headBranch == $branch)) | .[0].url // "not found"' 2>/dev/null || true)"
    echo "$previous_run"
    echo
    echo "## Repro"
    echo
    echo '~~~bash'
    echo "ssh ros2@$NODE_IP"
    echo "cd $COMPOSE_DIR"
    echo "docker compose up -d $service"
    echo '~~~'
    echo
    echo "## Dedup"
    echo
    echo "Key: $dedup_key"
  } > "$body_file"

  title="[deploy-run-$RUN_ID] $service crash on $NODE: $reason"
  if [[ -n "$existing" ]]; then
    issue_number="$(cut -f1 <<< "$existing")"
    issue_url="$(cut -f2 <<< "$existing")"
    comment_file="$(mktemp)"
    {
      echo "## Repeated deploy failure — run #$RUN_ID"
      echo
      echo "$dedup_marker"
      echo "- Node: $NODE ($NODE_IP)"
      echo "- Service: $service"
      echo "- Container: $full_id"
      echo "- Image SHA: $image_sha"
      echo "- Exit code: $exit_code"
      echo "- Run: $RUN_URL"
      echo
      echo "New Docker logs (last 50):"
      echo
      echo '~~~text'
      printf '%s\n' "$logs"
      echo '~~~'
    } > "$comment_file"
    gh issue comment "$issue_number" --repo "$REPO" --body-file "$comment_file" >/dev/null 2>&1 || true
    echo "Existing deploy-failure issue #$issue_number updated: $issue_url"
    rm -f "$comment_file"
  else
    gh issue create --repo "$REPO" --title "$title" --body-file "$body_file" \
      --label bug --label deploy-failure --label auto-generated --label priority:high --label agent:devops >/dev/null
    echo "Created deploy-failure issue for $service on $NODE"
  fi

  created_any=1
  rm -f "$body_file"
done <<< "$CONTAINERS"

if (( created_any == 0 )); then
  dedup_key="unavailable:$NODE:compose-up"
  dedup_marker="<!-- deploy-dedup-key: $dedup_key -->"
  existing="$(gh issue list --repo "$REPO" --state open --label deploy-failure --limit 200 --json number,body,url 2>/dev/null |
    jq -r --arg key "$dedup_marker" '.[] | select(.body | contains($key)) | [.number,.url] | @tsv' |
    head -1 || true)"

  fallback_service="$REQUESTED_SERVICE"
  [[ "$fallback_service" != "-" ]] || fallback_service="unknown"
  fallback_container="$REQUESTED_CONTAINER"
  [[ "$fallback_container" != "-" ]] || fallback_container="unknown"

  body_file="$(mktemp)"
  {
    echo "## Deploy run #$RUN_ID"
    echo
    echo "$dedup_marker"
    echo "- Workflow: L-Deploy and Verify"
    echo "- Started: $RUN_STARTED_AT"
    echo "- Failed at: $FAILED_AT"
    echo "- Run: $RUN_URL"
    echo "- Node: $NODE ($NODE_IP)"
    echo "- Service: $fallback_service"
    echo "- Container: $fallback_container"
    echo "- Image SHA: unavailable"
    echo
    echo "## docker ps -a"
    echo
    echo '~~~text'
    printf '%s\n' "$PS_ALL"
    echo '~~~'
    echo
    echo "Docker container metadata could not identify a non-zero-exit container."
    echo "The deploy step itself failed; this issue preserves that fact for triage."
  } > "$body_file"

  if [[ -n "$existing" ]]; then
    issue_number="$(cut -f1 <<< "$existing")"
    gh issue comment "$issue_number" --repo "$REPO" --body-file "$body_file" >/dev/null 2>&1 || true
    echo "Updated fallback deploy-failure issue #$issue_number"
  else
    gh issue create --repo "$REPO" --title "[deploy-run-$RUN_ID] deploy compose failure on $NODE: no container evidence" \
      --body-file "$body_file" --label bug --label deploy-failure --label auto-generated --label priority:high --label agent:devops >/dev/null
    echo "Created fallback deploy-failure issue for $NODE"
  fi
  rm -f "$body_file"
fi

exit 0
