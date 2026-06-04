#!/bin/bash
set -e
# ============================================================
# Batch evaluation of ORB-SLAM3 on EuRoC dataset
# Usage: ./script/batch_eval_euroc.sh [--no-evo]
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJ_DIR"

RUN_EVO=true
if [ "$1" == "--no-evo" ]; then
    RUN_EVO=false
fi

# ---- Config ----
VOCAB="./Vocabulary/ORBvoc.txt"
CONFIG="./Examples/Stereo-Inertial/EuRoC.yaml"
TIMESTAMPS_DIR="./Examples/Stereo-Inertial/EuRoC_TimeStamps"
EXEC="./Examples/Stereo-Inertial/stereo_inertial_euroc"
DATA_DIR="/home/xinxin/Data/EuRoc"
RESULTS_DIR="./results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS_DIR"

# Map dataset short name -> folder / timestamp file
declare -A DATASETS=(
    ["MH01"]="euroc-mh-01-easy:MH01"
    ["MH02"]="euroc-mh-02-easy:MH02"
    ["MH03"]="euroc-mh-03-medium:MH03"
    ["MH04"]="euroc-mh-04-difficult:MH04"
    ["MH05"]="euroc-mh-05-difficult:MH05"
    ["V101"]="euroc-v1-01-easy:V101"
    ["V102"]="euroc-v1-02-medium:V102"
    ["V103"]="euroc-v1-03-difficult:V103"
    ["V201"]="euroc-v2-01-easy:V201"
    ["V202"]="euroc-v2-02-medium:V202"
    ["V203"]="euroc-v2-03-difficult:V203"
)

# Check prerequisites
if [ ! -f "$EXEC" ]; then
    echo "[ERROR] Executable not found: $EXEC"
    echo "Please build ORB-SLAM3 first: ./build.sh"
    exit 1
fi

if ! command -v evo_ape &>/dev/null; then
    echo "[WARN] evo not found. Install: pip install evo"
    RUN_EVO=false
fi

TOTAL_START=$(date +%s)

# ---- Run each sequence ----
for short_name in "${!DATASETS[@]}"; do
    IFS=':' read -r folder tfile <<< "${DATASETS[$short_name]}"
    SEQ_DIR="$DATA_DIR/$folder"
    TS_FILE="$TIMESTAMPS_DIR/${tfile}.txt"
    GROUNDTRUTH="$SEQ_DIR/mav0/state_groundtruth_estimate0/data.csv"
    GROUNDTRUTH_TUM="data.tum"

    echo ""
    echo "========================================="
    echo "  Processing $short_name"
    echo "  Data  : $SEQ_DIR"
    echo "  Times : $TS_FILE"
    echo "========================================="

    if [ ! -d "$SEQ_DIR" ]; then
        echo "[SKIP] Data folder not found: $SEQ_DIR"
        continue
    fi
    if [ ! -f "$TS_FILE" ]; then
        echo "[SKIP] Timestamp file not found: $TS_FILE"
        continue
    fi

    SEQ_RESULTS="$RESULTS_DIR/$short_name"
    mkdir -p "$SEQ_RESULTS"

    # --- Run SLAM ---
    echo "[RUN] $EXEC $VOCAB $CONFIG $SEQ_DIR $TS_FILE"
    cd "$PROJ_DIR"
    ./Examples/Stereo-Inertial/stereo_inertial_euroc \
        "$VOCAB" "$CONFIG" "$SEQ_DIR" "$TS_FILE" \
        2>&1 | tee "$SEQ_RESULTS/slam_output.log"

    # --- Save trajectory ---
    if [ -f "CameraTrajectory.txt" ]; then
        cp "CameraTrajectory.txt" "$SEQ_RESULTS/"
        echo "[SAVE] CameraTrajectory.txt -> $SEQ_RESULTS/"
    fi
    if [ -f "KeyFrameTrajectory.txt" ]; then
        cp "KeyFrameTrajectory.txt" "$SEQ_RESULTS/"
        echo "[SAVE] KeyFrameTrajectory.txt -> $SEQ_RESULTS/"
    fi

    # --- Extract IMU bias from log ---
    grep -i "bias" "$SEQ_RESULTS/slam_output.log" > "$SEQ_RESULTS/bias_log.txt" || true
    grep -i "scale" "$SEQ_RESULTS/slam_output.log" > "$SEQ_RESULTS/scale_log.txt" || true

    # --- evo evaluation ---
    if [ "$RUN_EVO" = true ] && [ -f "CameraTrajectory.txt" ] && [ -f "$GROUNDTRUTH" ]; then
        echo "[EVO] Evaluating $short_name ..."

        # Convert groundtruth to TUM format
        evo_traj euroc "$GROUNDTRUTH" --save_as_tum

        # APE with SE(3) alignment #--plot --plot_mode xz \
        MPLBACKEND=Agg evo_ape tum "CameraTrajectory.txt" "$GROUNDTRUTH_TUM" -a \
            --save_results "$SEQ_RESULTS/${short_name}_ape.zip" \
            --save_plot "$SEQ_RESULTS/${short_name}_ape.pdf" \
            2>&1 | tee "$SEQ_RESULTS/eval_ape.log"

        # RPE (delta = 1m) #--plot --plot_mode xz \
        MPLBACKEND=Agg evo_rpe tum "CameraTrajectory.txt" "$GROUNDTRUTH_TUM" -a --delta 1 --delta_unit m \
            --save_results "$SEQ_RESULTS/${short_name}_rpe.zip" \
            --save_plot "$SEQ_RESULTS/${short_name}_rpe.pdf" \
            2>&1 | tee "$SEQ_RESULTS/eval_rpe.log"

        echo "[EVO] Done"
    else
        echo "[SKIP] evo evaluation (missing trajectory or groundtruth)"
    fi

    # --- Clean up SLAM output files ---
    rm -f "CameraTrajectory.txt" "KeyFrameTrajectory.txt" "$GROUNDTRUTH_TUM" 2>/dev/null

    echo "[DONE] $short_name"
done

# ---- Summary report ----
TOTAL_END=$(date +%s)
echo ""
echo "========================================="
echo "  Summary"
echo "  Results saved to: $RESULTS_DIR"
echo "  Total time: $(( (TOTAL_END - TOTAL_START) / 60 )) min"
echo "========================================="
echo ""

echo "Dataset   |   RMSE   |   Mean   |  Median  |   Std    "
echo "----------|----------|----------|----------|----------"
for short_name in "${!DATASETS[@]}"; do
    ape_log="$RESULTS_DIR/$short_name/eval_ape.log"
    if [ -f "$ape_log" ]; then
        rmse=$(awk '/^ *rmse/{print $2}' "$ape_log")
        mean=$(awk '/^ *mean/{print $2}' "$ape_log")
        median=$(awk '/^ *median/{print $2}' "$ape_log")
        std=$(awk '/^ *std/{print $2}' "$ape_log")
        printf "%-10s| %-9s| %-9s| %-9s| %-9s\n" "$short_name" "$rmse" "$mean" "$median" "$std"
    fi
done
