#!/bin/bash

# Battery-Aware Network Optimization - Automated 5-Iteration Runner
# This script runs the complete iterative pipeline with battery dynamics

set -e  # Exit on any error

# Configuration
NUM_TERMINALS=${1:-20}           # Number of terminals (default: 20)
BUDGET=${2:-800000}              # Budget constraint (default: 800000)
CHARGE_RATE=${3:-15.0}           # Charge rate for connected terminals (default: 15.0%)
DEMAND_RATE=${4:-3.0}            # Demand rate for all terminals (default: 3.0%)
OUTPUT_DIR=${5:-"auto-test"}     # Output directory (default: auto-test)

echo "🚀 Battery-Aware Network Optimization - 5 Iteration Runner"
echo "============================================================="
echo "Configuration:"
echo "  Terminals:    $NUM_TERMINALS"
echo "  Budget:       $BUDGET"
echo "  Charge rate:  $CHARGE_RATE%"
echo "  Demand rate:  $DEMAND_RATE%"
echo "  Output dir:   $OUTPUT_DIR"
echo "============================================================="
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"
echo "📁 Created output directory: $OUTPUT_DIR"

# Generate initial terminals
echo "🎲 Generating $NUM_TERMINALS random terminals..."
./rand_points $NUM_TERMINALS > "$OUTPUT_DIR/terminals_iter1.txt"
echo "✅ Generated terminals: $OUTPUT_DIR/terminals_iter1.txt"
echo ""

# Function to run a single iteration
run_iteration() {
    local iter=$1
    local prev_iter=$2

    echo "🚀 ITERATION $iter"
    echo "==============="

    # Step 1: Update batteries (skip for iteration 1)
    if [ $iter -gt 1 ]; then
        echo "Step 1: Updating battery levels..."
        ./battery_wrapper \
            -t "$OUTPUT_DIR/terminals_iter${prev_iter}.txt" \
            -s "$OUTPUT_DIR/solution_iter${prev_iter}.txt" \
            -o "$OUTPUT_DIR/terminals_iter${iter}.txt" \
            -c $CHARGE_RATE \
            -d $DEMAND_RATE \
            > "$OUTPUT_DIR/battery_update_iter${iter}.log" 2>&1
        echo "✅ Battery levels updated"
    fi

    # Step 2: Generate FSTs
    echo "Step 2: Generating Full Steiner Trees..."
    ./efst < "$OUTPUT_DIR/terminals_iter${iter}.txt" > "$OUTPUT_DIR/fsts_iter${iter}.txt"
    fst_count=$(wc -l < "$OUTPUT_DIR/fsts_iter${iter}.txt")
    echo "✅ Generated $fst_count FSTs"

    # Step 3: Solve optimization
    echo "Step 3: Solving budget-constrained optimization..."
    GEOSTEINER_BUDGET=$BUDGET ./bb < "$OUTPUT_DIR/fsts_iter${iter}.txt" > "$OUTPUT_DIR/solution_iter${iter}.txt" 2>&1
    echo "✅ Optimization completed"

    # Step 4: Generate visualization
    echo "Step 4: Creating visualization..."
    ./simulate \
        -t "$OUTPUT_DIR/terminals_iter${iter}.txt" \
        -f "$OUTPUT_DIR/fsts_iter${iter}.txt" \
        -r "$OUTPUT_DIR/solution_iter${iter}.txt" \
        -w "$OUTPUT_DIR/visualization_iter${iter}.html" \
        -v > "$OUTPUT_DIR/simulate_iter${iter}.log" 2>&1
    echo "✅ Visualization created: $OUTPUT_DIR/visualization_iter${iter}.html"
    echo ""
}

# Run all 5 iterations
for i in {1..5}; do
    prev_i=$((i-1))
    if [ $i -eq 1 ]; then
        run_iteration $i 0
    else
        run_iteration $i $prev_i
    fi
done

echo "📊 GENERATING COMPARISON REPORT"
echo "==============================="

# Create comparison report
cat > "$OUTPUT_DIR/battery_evolution_report.txt" << 'EOF'
🔋 BATTERY EVOLUTION ACROSS 5 ITERATIONS
==========================================

This report shows how battery levels evolve across iterations due to:
- Connected terminals: +15% charge, -3% demand = +12% net gain
- Disconnected terminals: 0% charge, -3% demand = -3% net loss
- Terminal 0 (source): Always maintained at 100%

EOF

echo "Terminal | Iter1 | Iter2 | Iter3 | Iter4 | Iter5 | Net Change | Trend" >> "$OUTPUT_DIR/battery_evolution_report.txt"
echo "---------|-------|-------|-------|-------|-------|------------|-------" >> "$OUTPUT_DIR/battery_evolution_report.txt"

for i in $(seq 0 $((NUM_TERMINALS-1))); do
    iter1=$(sed -n "$((i+1))p" "$OUTPUT_DIR/terminals_iter1.txt" | awk '{print $3}')
    iter2=$(sed -n "$((i+1))p" "$OUTPUT_DIR/terminals_iter2.txt" | awk '{print $3}')
    iter3=$(sed -n "$((i+1))p" "$OUTPUT_DIR/terminals_iter3.txt" | awk '{print $3}')
    iter4=$(sed -n "$((i+1))p" "$OUTPUT_DIR/terminals_iter4.txt" | awk '{print $3}')
    iter5=$(sed -n "$((i+1))p" "$OUTPUT_DIR/terminals_iter5.txt" | awk '{print $3}')

    net_change=$(echo "$iter5 - $iter1" | bc)

    if (( $(echo "$net_change > 10" | bc -l) )); then
        trend="🔋 CHARGING"
    elif (( $(echo "$net_change < -5" | bc -l) )); then
        trend="⚡ DRAINING"
    else
        trend="➖ STABLE"
    fi

    printf "T%-8d| %5.1f | %5.1f | %5.1f | %5.1f | %5.1f | %+10.1f | %s\n" \
        $i $iter1 $iter2 $iter3 $iter4 $iter5 $net_change "$trend" >> "$OUTPUT_DIR/battery_evolution_report.txt"
done

echo "" >> "$OUTPUT_DIR/battery_evolution_report.txt"
echo "📈 SUMMARY STATISTICS:" >> "$OUTPUT_DIR/battery_evolution_report.txt"
echo "=====================" >> "$OUTPUT_DIR/battery_evolution_report.txt"

# Calculate coverage statistics for each iteration
for i in {1..5}; do
    if [ -f "$OUTPUT_DIR/battery_update_iter$((i+1)).log" ]; then
        coverage=$(grep "Coverage Status:" "$OUTPUT_DIR/battery_update_iter$((i+1)).log" | awk '{print $3}')
    else
        # For iteration 1, we need to check solution
        coverage="Unknown"
    fi
    echo "Iteration $i: Coverage = $coverage" >> "$OUTPUT_DIR/battery_evolution_report.txt"
done

echo "" >> "$OUTPUT_DIR/battery_evolution_report.txt"
echo "📁 Generated Files:" >> "$OUTPUT_DIR/battery_evolution_report.txt"
echo "==================" >> "$OUTPUT_DIR/battery_evolution_report.txt"
ls -la "$OUTPUT_DIR/"*.{txt,html} | tail -n +2 >> "$OUTPUT_DIR/battery_evolution_report.txt"

echo "✅ Comparison report generated: $OUTPUT_DIR/battery_evolution_report.txt"

echo ""
echo "🎉 ALL 5 ITERATIONS COMPLETED SUCCESSFULLY!"
echo "============================================="
echo ""
echo "📁 Results saved in: $OUTPUT_DIR/"
echo "📊 View report: cat $OUTPUT_DIR/battery_evolution_report.txt"
echo "🌐 Open visualizations:"
for i in {1..5}; do
    echo "   Iteration $i: $OUTPUT_DIR/visualization_iter${i}.html"
done
echo ""
echo "🔄 To run again with different parameters:"
echo "   ./run_iterations.sh [terminals] [budget] [charge_rate] [demand_rate] [output_dir]"
echo ""
echo "Example:"
echo "   ./run_iterations.sh 30 1000000 20.0 5.0 big-test"