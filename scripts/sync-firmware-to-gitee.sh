#!/usr/bin/env bash

# Sync firmware assets from a GitHub Release to a Gitee Release, then publish
# the stable firmware-latest.json consumed by Gyro ELRS Configurator.
#
# Required environment variable:
#   GITEE_TOKEN  Gitee personal access token with write access to the target.
#
# Usage:
#   GITEE_TOKEN=... ./scripts/sync-firmware-to-gitee.sh \
#     HumpbackLab/Gyro-ELRS \
#     ncer/Gyro-ELRS \
#     v0.9.0_e364
#
# Optional environment variables:
#   GITHUB_TOKEN                 GitHub token for private repositories/API limits.
#   GITEE_FIRMWARE_BRANCH        Destination branch; default: elrs_fc.
#   GITEE_FIRMWARE_MANIFEST_PATH Stable manifest path; default:
#                                updater/firmware-latest.json.
#   DELETE_LEGACY_FIRMWARE       Delete the old repository-hosted binary after
#                                publishing the Release manifest; default: 1.
#   GITEE_FIRMWARE_PATH          Legacy repository firmware path to delete.

set -Eeuo pipefail

on_error() {
  local status=$?
  local line=${BASH_LINENO[0]}
  echo "error: command failed at line $line (exit $status)" >&2
  exit "$status"
}

trap on_error ERR

usage() {
  cat <<'EOF'
Usage:
  GITEE_TOKEN=... sync-firmware-to-gitee.sh <github-owner/repo> <gitee-owner/repo> [tag]

Arguments:
  github-owner/repo  Source GitHub repository.
  gitee-owner/repo   Destination Gitee repository.
  tag                Release tag to sync. Defaults to "latest".

The GitHub Release must contain firmware-latest.json and the single firmware
asset named by its firmwares[0].filename entry. Both assets are mirrored to a
Gitee Release. Only the stable manifest is committed to the Gitee repository,
after its firmware attachment is available.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

[[ ${1:-} != -h && ${1:-} != --help ]] || { usage; exit 0; }
[[ $# -ge 2 && $# -le 3 ]] || { usage >&2; exit 2; }

require_command curl
require_command jq
require_command base64
require_command sha256sum
require_command stat
require_command sed
require_command tr
require_command cut
require_command mktemp

github_repo=$1
gitee_repo=$2
requested_tag=${3:-latest}

[[ $github_repo == */* && $github_repo != */*/* ]] || die "invalid GitHub repository: $github_repo"
[[ $gitee_repo == */* && $gitee_repo != */*/* ]] || die "invalid Gitee repository: $gitee_repo"
[[ -n ${GITEE_TOKEN:-} ]] || die "GITEE_TOKEN is required"

gitee_branch=${GITEE_FIRMWARE_BRANCH:-elrs_fc}
manifest_path=${GITEE_FIRMWARE_MANIFEST_PATH:-updater/firmware-latest.json}
legacy_firmware_path=${GITEE_FIRMWARE_PATH:-updater/firmware/firmware-latest.bin}
delete_legacy_firmware=${DELETE_LEGACY_FIRMWARE:-1}

[[ $gitee_branch =~ ^[A-Za-z0-9._/-]+$ ]] || die "GITEE_FIRMWARE_BRANCH contains unsupported characters"
[[ $manifest_path =~ ^[A-Za-z0-9._/-]+$ ]] || die "GITEE_FIRMWARE_MANIFEST_PATH contains unsupported characters"
[[ $legacy_firmware_path =~ ^[A-Za-z0-9._/-]+$ ]] || die "GITEE_FIRMWARE_PATH contains unsupported characters"
[[ $delete_legacy_firmware == 0 || $delete_legacy_firmware == 1 ]] || die "DELETE_LEGACY_FIRMWARE must be 0 or 1"

tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT

umask 077
auth_file="$tmp_dir/gitee-auth-header"
printf 'Authorization: token %s\n' "$GITEE_TOKEN" > "$auth_file"
gitee_auth=(--header @"$auth_file")

github_headers=(
  -H "Accept: application/vnd.github+json"
  -H "X-GitHub-Api-Version: 2022-11-28"
  -H "User-Agent: firmware-github-to-gitee-sync"
)
if [[ -n ${GITHUB_TOKEN:-} ]]; then
  github_headers+=(-H "Authorization: Bearer $GITHUB_TOKEN")
fi

if [[ $requested_tag == latest ]]; then
  github_release_url="https://api.github.com/repos/$github_repo/releases/latest"
else
  encoded_requested_tag=$(jq -rn --arg value "$requested_tag" '$value | @uri')
  github_release_url="https://api.github.com/repos/$github_repo/releases/tags/$encoded_requested_tag"
fi

echo "Fetching GitHub release: $github_repo ($requested_tag)"
curl --fail-with-body --silent --show-error --location \
  "${github_headers[@]}" "$github_release_url" > "$tmp_dir/github-release.json"

[[ $(jq -r '.draft' "$tmp_dir/github-release.json") != true ]] || die "draft GitHub releases cannot be synced"

tag=$(jq -er '.tag_name' "$tmp_dir/github-release.json")
release_name=$(jq -r '.name // .tag_name' "$tmp_dir/github-release.json")
release_body=$(jq -r '.body // ""' "$tmp_dir/github-release.json")
prerelease=$(jq -r '.prerelease // false' "$tmp_dir/github-release.json")
manifest_url=$(jq -er '.assets[] | select(.name == "firmware-latest.json") | .browser_download_url' \
  "$tmp_dir/github-release.json")
manifest_file="$tmp_dir/firmware-latest.json"

echo "Downloading GitHub asset: firmware-latest.json"
curl --fail-with-body --silent --show-error --location \
  "${github_headers[@]}" "$manifest_url" --output "$manifest_file"

jq -e '
  .schema == 1
  and (.version | type == "string")
  and (.firmwares | type == "array" and length == 1)
  and (.firmwares[0].filename | type == "string" and length > 0)
  and (.firmwares[0].size | type == "number" and . > 0 and . == floor and . <= 8388608)
  and (.firmwares[0].sha256 | type == "string" and test("^[0-9A-Fa-f]{64}$"))
  and (.firmwares[0].sources.gitee | type == "string" and length > 0)
' "$manifest_file" >/dev/null || die "firmware-latest.json is invalid or does not contain exactly one firmware"

manifest_version=$(jq -r '.version' "$manifest_file")
[[ $manifest_version == "$tag" ]] || die "manifest version $manifest_version does not match release tag $tag"

firmware_name=$(jq -r '.firmwares[0].filename' "$manifest_file")
[[ $firmware_name != */* && $firmware_name != . && $firmware_name != .. ]] || die "invalid firmware filename: $firmware_name"
firmware_url=$(jq -er --arg name "$firmware_name" \
  '.assets[] | select(.name == $name) | .browser_download_url' "$tmp_dir/github-release.json")
firmware_file="$tmp_dir/$firmware_name"

echo "Downloading GitHub asset: $firmware_name"
curl --fail-with-body --silent --show-error --location \
  "${github_headers[@]}" "$firmware_url" --output "$firmware_file"

expected_size=$(jq -r '.firmwares[0].size' "$manifest_file")
expected_sha256=$(jq -r '.firmwares[0].sha256 | ascii_downcase' "$manifest_file")
actual_size=$(stat -c%s "$firmware_file")
actual_sha256=$(sha256sum "$firmware_file" | cut -d' ' -f1)

[[ $actual_size == "$expected_size" ]] || die "firmware size mismatch: expected $expected_size, got $actual_size"
[[ $actual_sha256 == "$expected_sha256" ]] || die "firmware SHA-256 mismatch"

gitee_api="https://gitee.com/api/v5/repos/$gitee_repo"
encoded_tag=$(jq -rn --arg value "$tag" '$value | @uri')

gitee_status=$(curl --silent --show-error --output "$tmp_dir/gitee-release.json" \
  --write-out '%{http_code}' "${gitee_auth[@]}" \
  "$gitee_api/releases/tags/$encoded_tag")
if [[ $gitee_status == 200 && $(jq -r 'type' "$tmp_dir/gitee-release.json") != object ]]; then
  gitee_status=404
fi

release_form=(
  --data-urlencode "tag_name=$tag"
  --data-urlencode "name=$release_name"
  --data-urlencode "body=$release_body"
  --data-urlencode "prerelease=$prerelease"
  --data-urlencode "target_commitish=$gitee_branch"
)

case $gitee_status in
  200)
    release_id=$(jq -er '.id' "$tmp_dir/gitee-release.json")
    echo "Updating Gitee release: $gitee_repo ($tag)"
    curl --fail-with-body --silent --show-error --request PATCH \
      "${gitee_auth[@]}" "${release_form[@]}" "$gitee_api/releases/$release_id" > "$tmp_dir/gitee-release.json"
    ;;
  404)
    echo "Creating Gitee release: $gitee_repo ($tag)"
    curl --fail-with-body --silent --show-error --request POST \
      "${gitee_auth[@]}" "${release_form[@]}" "$gitee_api/releases" > "$tmp_dir/gitee-release.json"
    release_id=$(jq -er '.id' "$tmp_dir/gitee-release.json")
    ;;
  *)
    cat "$tmp_dir/gitee-release.json" >&2
    die "Gitee release lookup returned HTTP $gitee_status"
    ;;
esac

curl --fail-with-body --silent --show-error --get \
  "${gitee_auth[@]}" \
  --data-urlencode "per_page=100" \
  "$gitee_api/releases/$release_id/attach_files" > "$tmp_dir/gitee-assets.json"

upload_release_asset() {
  local source_file=$1
  local asset_name=$2
  local existing_id

  while IFS= read -r existing_id; do
    [[ -z $existing_id ]] || {
      echo "Replacing existing Gitee release asset: $asset_name"
      curl --fail-with-body --silent --show-error --request DELETE \
        "${gitee_auth[@]}" \
        "$gitee_api/releases/$release_id/attach_files/$existing_id" >/dev/null
    }
  done < <(jq -r --arg name "$asset_name" '.[] | select(.name == $name) | .id' "$tmp_dir/gitee-assets.json")

  echo "Uploading Gitee release asset: $asset_name"
  curl --fail-with-body --silent --show-error --request POST \
    "${gitee_auth[@]}" \
    --form "file=@$source_file;filename=$asset_name" \
    "$gitee_api/releases/$release_id/attach_files" >/dev/null
}

encoded_firmware_name=$(jq -rn --arg value "$firmware_name" '$value | @uri')
gitee_firmware_url="https://gitee.com/$gitee_repo/releases/download/$encoded_tag/$encoded_firmware_name"
jq --arg gitee_url "$gitee_firmware_url" \
  '.firmwares[0].sources.gitee = $gitee_url' \
  "$manifest_file" > "$tmp_dir/firmware-latest.gitee.json"
manifest_file="$tmp_dir/firmware-latest.gitee.json"

# Keep this order: the manifest must never point at a missing attachment.
upload_release_asset "$firmware_file" "$firmware_name"
upload_release_asset "$manifest_file" "firmware-latest.json"

publish_file() {
  local source_file=$1
  local destination_path=$2
  local commit_message=$3
  local encoded_path content_url content_status content_sha base64_file

  encoded_path=$(jq -rn --arg value "$destination_path" '$value | @uri' | sed 's/%2F/\//g')
  content_url="$gitee_api/contents/$encoded_path"
  content_status=$(curl --silent --show-error --output "$tmp_dir/gitee-content.json" \
    --write-out '%{http_code}' --get "${gitee_auth[@]}" \
    --data-urlencode "ref=$gitee_branch" \
    "$content_url")
  if [[ $content_status == 200 && $(jq -r 'type' "$tmp_dir/gitee-content.json") != object ]]; then
    content_status=404
  fi

  base64_file="$tmp_dir/content.base64"
  base64 < "$source_file" | tr -d '\n' > "$base64_file"
  content_form=(
    --data-urlencode "content@$base64_file"
    --data-urlencode "message=$commit_message"
    --data-urlencode "branch=$gitee_branch"
  )

  case $content_status in
    200)
      content_sha=$(jq -er '.sha' "$tmp_dir/gitee-content.json")
      content_form+=(--data-urlencode "sha=$content_sha")
      curl --fail-with-body --silent --show-error --request PUT \
        "${gitee_auth[@]}" "${content_form[@]}" "$content_url" >/dev/null
      ;;
    404)
      curl --fail-with-body --silent --show-error --request POST \
        "${gitee_auth[@]}" "${content_form[@]}" "$content_url" >/dev/null
      ;;
    *)
      cat "$tmp_dir/gitee-content.json" >&2
      die "Gitee content lookup for $destination_path returned HTTP $content_status"
      ;;
  esac

  echo "Published Gitee file: $destination_path"
}

publish_file "$manifest_file" "$manifest_path" "Update firmware manifest for $tag"

delete_repository_file() {
  local destination_path=$1
  local encoded_path content_url content_status content_sha

  encoded_path=$(jq -rn --arg value "$destination_path" '$value | @uri' | sed 's/%2F/\//g')
  content_url="$gitee_api/contents/$encoded_path"
  content_status=$(curl --silent --show-error --output "$tmp_dir/gitee-content.json" \
    --write-out '%{http_code}' --get "${gitee_auth[@]}" \
    --data-urlencode "ref=$gitee_branch" \
    "$content_url")

  if [[ $content_status == 200 && $(jq -r 'type' "$tmp_dir/gitee-content.json") == object ]]; then
    content_sha=$(jq -er '.sha' "$tmp_dir/gitee-content.json")
    curl --fail-with-body --silent --show-error --request DELETE \
      "${gitee_auth[@]}" \
      --data-urlencode "sha=$content_sha" \
      --data-urlencode "message=Remove repository-hosted firmware after $tag release" \
      --data-urlencode "branch=$gitee_branch" \
      "$content_url" >/dev/null
    echo "Deleted legacy Gitee repository file: $destination_path"
  elif [[ $content_status != 404 && $content_status != 200 ]]; then
    cat "$tmp_dir/gitee-content.json" >&2
    die "Gitee legacy firmware lookup returned HTTP $content_status"
  fi
}

if [[ $delete_legacy_firmware == 1 ]]; then
  delete_repository_file "$legacy_firmware_path"
fi

echo "Gitee release: https://gitee.com/$gitee_repo/releases/tag/$encoded_tag"
echo "Stable firmware manifest: https://raw.giteeusercontent.com/$gitee_repo/raw/$gitee_branch/$manifest_path"
