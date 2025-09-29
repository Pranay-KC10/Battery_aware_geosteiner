#!/bin/bash
# manual_iter.sh - Run N iterations of battery-aware GeoSteiner
# Usage: ./manual_iter.sh [iterations]

# Config
NUM_TERMINALS=20
BUDGET=1800000     # Adjust budget here
CHARGE=10.0
DEMAND=2.0
ITERATIONS=${1:-5}  # Default = 5 iterations if not specified

# Create output folder with timestamp
OUTPUT_DIR="manual_run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "=== Running $ITERATIONS iterations with budget=$BUDGET ==="
echo "Results will be saved in: $OUTPUT_DIR"
echo ""

# Iteration 1: random start
./rand_points $NUM_TERMINALS > "$OUTPUT_DIR/terminals_iter1.txt"
./efst < "$OUTPUT_DIR/terminals_iter1.txt" > "$OUTPUT_DIR/fsts_iter1.txt"
GEOSTEINER_BUDGET=$BUDGET ./bb < "$OUTPUT_DIR/fsts_iter1.txt" > "$OUTPUT_DIR/solution_iter1.txt" 2>&1
./simulate -t "$OUTPUT_DIR/terminals_iter1.txt" -f "$OUTPUT_DIR/fsts_iter1.txt" -r "$OUTPUT_DIR/solution_iter1.txt" -w "$OUTPUT_DIR/visualization_iter1.html" -v

# Subsequent iterations
for i in $(seq 2 $ITERATIONS); do
    prev=$((i-1))

    echo "=== Iteration $i ==="

    ./battery_wrapper -t "$OUTPUT_DIR/terminals_iter${prev}.txt" -s "$OUTPUT_DIR/solution_iter${prev}.txt" -o "$OUTPUT_DIR/terminals_iter${i}.txt" -c $CHARGE -d $DEMAND
    ./efst < "$OUTPUT_DIR/terminals_iter${i}.txt" > "$OUTPUT_DIR/fsts_iter${i}.txt"
    GEOSTEINER_BUDGET=$BUDGET ./bb < "$OUTPUT_DIR/fsts_iter${i}.txt" > "$OUTPUT_DIR/solution_iter${i}.txt" 2>&1
    ./simulate -t "$OUTPUT_DIR/terminals_iter${i}.txt" -f "$OUTPUT_DIR/fsts_iter${i}.txt" -r "$OUTPUT_DIR/solution_iter${i}.txt" -w "$OUTPUT_DIR/visualization_iter${i}.html" -v
done

echo ""
echo "✅ Finished $ITERATIONS iterations"
echo "📁 All results saved in: $OUTPUT_DIR"
