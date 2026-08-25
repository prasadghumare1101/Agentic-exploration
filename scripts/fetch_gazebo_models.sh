#!/usr/bin/env bash
# Fetch the Gazebo models the shipped worlds need, into ~/.gazebo/models.
#
# These are NOT committed: baylands alone is 393 MB with three mesh files of 92-97 MB each,
# which would sit against GitHub's 100 MB per-file limit and make every clone slow forever.
# They come from the public model database instead, which serves exactly the same assets.
#
# Run once after cloning:   ./scripts/fetch_gazebo_models.sh
#
# The simulator runs with the online database DISABLED (Gazebo otherwise blocks for minutes
# trying to pull meshes mid-launch), so anything missing here renders as an EMPTY world
# rather than an error. That is why this is a separate, explicit step.
set -euo pipefail

DB="${GAZEBO_MODEL_DB:-http://models.gazebosim.org}"
DEST="${GAZEBO_MODEL_DEST:-${HOME}/.gazebo/models}"

# Every model referenced by the six shipped worlds that is not already part of PX4.
# iris_custom_slam is deliberately absent: it is this project's own model and is committed
# under assets/gazebo_models/, because the public database has no copy of it.
MODELS=(
    ambulance              # clear_follow
    baylands               # baylands            393 MB
    first_2015_trash_can   # warehouse
    grey_wall              # warehouse
    ksql_airport           # ksql_airport         54 MB
    mcmillan_airfield      # mcmillan_airfield    61 MB
    prius_hybrid           # clear_follow
)

mkdir -p "${DEST}"
echo "── fetching Gazebo models into ${DEST} ───────────────────────"

failed=0
for m in "${MODELS[@]}"; do
    if [[ -f "${DEST}/${m}/model.config" ]]; then
        printf "  %-24s already present\n" "${m}"
        continue
    fi
    printf "  %-24s downloading… " "${m}"
    tmp="$(mktemp -d)"
    if curl -fsSL --retry 3 --connect-timeout 20 "${DB}/${m}/model.tar.gz" -o "${tmp}/m.tar.gz" \
       && tar xzf "${tmp}/m.tar.gz" -C "${DEST}"; then
        echo "ok ($(du -sh "${DEST}/${m}" | cut -f1))"
    else
        echo "FAILED"
        failed=$((failed + 1))
    fi
    rm -rf "${tmp}"
done

# The project's own model ships in the repository — link it in alongside the fetched ones.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -d "${HERE}/assets/gazebo_models" ]]; then
    cp -rn "${HERE}/assets/gazebo_models/." "${DEST}/" 2>/dev/null || true
    echo "  committed models copied from assets/gazebo_models/"
fi

echo
if (( failed )); then
    echo "  ${failed} model(s) failed — those worlds will render empty. Re-run when online."
    exit 1
fi
echo "  all models present — every shipped world will render."
