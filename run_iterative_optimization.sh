#!/bin/bash

#
# Iterative Battery-Aware Network Optimization Script
#
# This script runs the GeoSteiner pipeline iteratively, updating terminal
# battery levels after each optimization iteration based on coverage solutions.
#
# Author: Generated with Claude Code
# Date: 2025-09-28
#

set -e  # Exit on any error

# Ensure log and results directories exist before first log
mkdir -p logs iteration_results
touch logs/pipeline.log


# =============================================================================
# CONFIGURATION PARAMETERS
# =============================================================================

# Default values - can be overridden by command line arguments
NUM_ITERATIONS=5
NUM_TERMINALS=10
GEOSTEINER_BUDGET=500000
CHARGE_RATE=10.0
DEMAND_RATE=5.0
VERBOSE=true
GENERATE_VISUALIZATION=true
INITIAL_BATTERY_LEVEL=75.0

# File names
TERMINALS_FILE="terminals.txt"
FSTS_FILE="fsts.txt"
FSTS_DUMP_FILE="fsts_dump.txt"
SOLUTION_FILE="solution.txt"
UPDATED_TERMINALS_FILE="updated_terminals.txt"
VISUALIZATION_FILE="visualization.html"

# Output directories for iteration results
RESULTS_DIR="iteration_results"
LOGS_DIR="logs"

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

print_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Iterative Battery-Aware Network Optimization Pipeline

OPTIONS:
    -n, --iterations NUM     Number of iterations to run (default: $NUM_ITERATIONS)
    -t, --terminals NUM      Number of terminals to generate (default: $NUM_TERMINALS)
    -b, --budget AMOUNT      GeoSteiner budget constraint (default: $GEOSTEINER_BUDGET)
    -c, --charge RATE        Battery charge rate for covered terminals (default: $CHARGE_RATE)
    -d, --demand RATE        Battery demand rate for all terminals (default: $DEMAND_RATE)
    -i, --initial LEVEL      Initial battery level for generated terminals (default: $INITIAL_BATTERY_LEVEL)
    --no-viz                 Skip visualization generation
    --quiet                  Reduce output verbosity
    -h, --help              Show this help message

EXAMPLES:
    $0                                    # Run with defaults
    $0 -n 10 -b 750000 -c 15 -d 3       # 10 iterations, higher budget, custom rates
    $0 --terminals 20 --no-viz --quiet   # 20 terminals, no visualization, quiet mode

PIPELINE STAGES:
    1. Generate/Load terminals with battery levels
    2. Compute Full Steiner Trees (FSTs)
    3. Solve budget-constrained SMT with CPLEX
    4. Update terminal battery levels based on coverage
    5. Generate visualization (optional)
    6. Repeat for specified iterations

OUTPUT:
    - iteration_results/: Contains snapshots from each iteration
    - logs/: Contains detailed logs for each stage
    - Final results in current directory

EOF
}

log_message() {
    local level=$1
    local message=$2
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')

    if [[ "$VERBOSE" == "true" || "$level" == "ERROR" ]]; then
        echo "[$timestamp] [$level] $message" | tee -a "$LOGS_DIR/pipeline.log"
    fi
}

check_prerequisites() {
    log_message "INFO" "Checking prerequisites..."

    local missing_tools=()

    # Check for required executables
    for tool in rand_points efst bb simulate battery_wrapper; do
        if ! command -v ./$tool &> /dev/null && ! [ -x ./$tool ]; then
            missing_tools+=($tool)
        fi
    done

    if [ ${#missing_tools[@]} -ne 0 ]; then
        log_message "ERROR" "Missing required tools: ${missing_tools[*]}"
        log_message "ERROR" "Please ensure all GeoSteiner tools and battery_wrapper are compiled"
        exit 1
    fi

    log_message "INFO" "All required tools found"
}

setup_directories() {
    log_message "INFO" "Setting up output directories..."

    mkdir -p "$RESULTS_DIR" "$LOGS_DIR"

    # Clear previous logs
    > "$LOGS_DIR/pipeline.log"

    log_message "INFO" "Output directories ready"
}

generate_initial_terminals() {
    log_message "INFO" "Generating $NUM_TERMINALS initial terminals..."

    # Generate terminals with coordinates and add battery levels
    if ! ./rand_points $NUM_TERMINALS > temp_coords.txt; then
        log_message "ERROR" "Failed to generate terminal coordinates"
        exit 1
    fi

    # Add battery levels to coordinates (format: x y battery_level)
    awk -v battery="$INITIAL_BATTERY_LEVEL" '{print $1, $2, battery}' temp_coords.txt > "$TERMINALS_FILE"
    rm temp_coords.txt

    local terminal_count=$(wc -l < "$TERMINALS_FILE")
    log_message "INFO" "Generated $terminal_count terminals with initial battery level: $INITIAL_BATTERY_LEVEL%"
}

backup_iteration_files() {
    local iteration=$1
    local iter_dir="$RESULTS_DIR/iteration_$iteration"

    mkdir -p "$iter_dir"

    # Backup key files from this iteration
    [ -f "$TERMINALS_FILE" ] && cp "$TERMINALS_FILE" "$iter_dir/"
    [ -f "$FSTS_FILE" ] && cp "$FSTS_FILE" "$iter_dir/"
    [ -f "$SOLUTION_FILE" ] && cp "$SOLUTION_FILE" "$iter_dir/"
    [ -f "$VISUALIZATION_FILE" ] && cp "$VISUALIZATION_FILE" "$iter_dir/"

    log_message "INFO" "Backed up iteration $iteration files to $iter_dir"
}

# =============================================================================
# PIPELINE STAGE FUNCTIONS
# =============================================================================
compute_fsts() {
    local iteration=$1
    log_message "INFO" "Iteration $iteration: Computing Full Steiner Trees..."

    # Count only valid (non-comment, non-empty) lines
    local n=$(grep -v '^[#[:space:]]' "$TERMINALS_FILE" | wc -l)

    {
        echo "p $n"
        grep -v '^[#[:space:]]' "$TERMINALS_FILE"
    } | ./efst > "$FSTS_FILE" 2> "$LOGS_DIR/efst_iter_${iteration}.log"

    if [ ! -s "$FSTS_FILE" ]; then
        log_message "ERROR" "FST computation failed in iteration $iteration"
        cat "$LOGS_DIR/efst_iter_${iteration}.log"
        exit 1
    fi

    local fst_count=$(grep -c "^fs" "$FSTS_FILE" 2>/dev/null || echo "0")
    log_message "INFO" "Generated $fst_count Full Steiner Trees"
}




solve_optimization() {
    local iteration=$1
    log_message "INFO" "Iteration $iteration: Solving budget-constrained optimization (Budget: $GEOSTEINER_BUDGET)..."

    local solver_log="$LOGS_DIR/bb_iter_${iteration}.log"

    if ! GEOSTEINER_BUDGET=$GEOSTEINER_BUDGET ./bb < "$FSTS_FILE" > "$SOLUTION_FILE" 2> "$solver_log"; then
        log_message "ERROR" "Optimization solver failed in iteration $iteration"
        log_message "ERROR" "Check solver log: $solver_log"
        exit 1
    fi

    # Extract solution statistics
    local selected_fsts=$(grep -c "x\[.*\] = 1\.0" "$SOLUTION_FILE" 2>/dev/null || echo "0")
    log_message "INFO" "Solution selected $selected_fsts FSTs"
}

update_battery_levels() {
    local iteration=$1
    log_message "INFO" "Iteration $iteration: Updating battery levels (Charge: +$CHARGE_RATE%, Demand: -$DEMAND_RATE%)..."

    local battery_log="$LOGS_DIR/battery_iter_${iteration}.log"
    local battery_args="-i $TERMINALS_FILE -s $SOLUTION_FILE -o $UPDATED_TERMINALS_FILE -c $CHARGE_RATE -d $DEMAND_RATE"

    if [[ "$VERBOSE" == "true" ]]; then
        battery_args="$battery_args -v"
    fi

    if ! ./battery_wrapper $battery_args > "$battery_log" 2>&1; then
        log_message "ERROR" "Battery update failed in iteration $iteration"
        log_message "ERROR" "Check battery log: $battery_log"
        exit 1
    fi

    # Replace terminals file with updated version
    mv "$UPDATED_TERMINALS_FILE" "$TERMINALS_FILE"

    # Extract battery statistics from log
    local avg_battery=$(tail -n 20 "$battery_log" | grep -o "Average battery level: [0-9.]*" | awk '{print $4}' || echo "N/A")
    log_message "INFO" "Average battery level after update: $avg_battery%"
}

generate_visualization() {
    local iteration=$1

    if [[ "$GENERATE_VISUALIZATION" != "true" ]]; then
        return 0
    fi

    log_message "INFO" "Iteration $iteration: Generating visualization..."

    local viz_log="$LOGS_DIR/simulate_iter_${iteration}.log"
    local viz_file="visualization_iter_${iteration}.html"

    if ! ./simulate -t "$TERMINALS_FILE" -f "$FSTS_FILE" -r "$SOLUTION_FILE" -w "$viz_file" -v > "$viz_log" 2>&1; then
        log_message "WARN" "Visualization generation failed in iteration $iteration (non-critical)"
        return 0
    fi

    # Create symlink to latest visualization
    ln -sf "$viz_file" "$VISUALIZATION_FILE"
    log_message "INFO" "Visualization saved as $viz_file"
}

# =============================================================================
# MAIN PIPELINE EXECUTION
# =============================================================================

run_iteration() {
    local iteration=$1

    log_message "INFO" "=========================================="
    log_message "INFO" "STARTING ITERATION $iteration/$NUM_ITERATIONS"
    log_message "INFO" "=========================================="

    # Stage 1: Compute FSTs from current terminals
    compute_fsts $iteration

    # Stage 2: Solve optimization problem
    solve_optimization $iteration

    # Stage 3: Update battery levels based on solution
    update_battery_levels $iteration

    # Stage 4: Generate visualization (optional)
    generate_visualization $iteration

    # Stage 5: Backup iteration results
    backup_iteration_files $iteration

    log_message "INFO" "Iteration $iteration completed successfully"
}

# =============================================================================
# COMMAND LINE ARGUMENT PARSING
# =============================================================================

parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -n|--iterations)
                NUM_ITERATIONS="$2"
                shift 2
                ;;
            -t|--terminals)
                NUM_TERMINALS="$2"
                shift 2
                ;;
            -b|--budget)
                GEOSTEINER_BUDGET="$2"
                shift 2
                ;;
            -c|--charge)
                CHARGE_RATE="$2"
                shift 2
                ;;
            -d|--demand)
                DEMAND_RATE="$2"
                shift 2
                ;;
            -i|--initial)
                INITIAL_BATTERY_LEVEL="$2"
                shift 2
                ;;
            --no-viz)
                GENERATE_VISUALIZATION=false
                shift
                ;;
            --quiet)
                VERBOSE=false
                shift
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                print_usage
                exit 1
                ;;
        esac
    done
}

main() {
    # Parse command line arguments
    parse_arguments "$@"

    # Setup and validation
    setup_directories
    check_prerequisites

    log_message "INFO" "Starting iterative battery-aware network optimization"
    log_message "INFO" "Configuration:"
    log_message "INFO" "  Iterations: $NUM_ITERATIONS"
    log_message "INFO" "  Terminals: $NUM_TERMINALS"
    log_message "INFO" "  Budget: $GEOSTEINER_BUDGET"
    log_message "INFO" "  Charge Rate: $CHARGE_RATE%"
    log_message "INFO" "  Demand Rate: $DEMAND_RATE%"
    log_message "INFO" "  Initial Battery: $INITIAL_BATTERY_LEVEL%"
    log_message "INFO" "  Generate Visualization: $GENERATE_VISUALIZATION"

    # Generate initial terminals if they don't exist
    if [ ! -f "$TERMINALS_FILE" ]; then
        generate_initial_terminals
    else
        log_message "INFO" "Using existing terminals file: $TERMINALS_FILE"
    fi

    # Run optimization iterations
    local start_time=$(date +%s)

    for ((iteration=1; iteration<=NUM_ITERATIONS; iteration++)); do
        run_iteration $iteration
    done

    local end_time=$(date +%s)
    local duration=$((end_time - start_time))

    log_message "INFO" "=========================================="
    log_message "INFO" "OPTIMIZATION COMPLETED SUCCESSFULLY"
    log_message "INFO" "=========================================="
    log_message "INFO" "Total iterations: $NUM_ITERATIONS"
    log_message "INFO" "Total runtime: ${duration}s"
    log_message "INFO" "Results available in: $RESULTS_DIR/"
    log_message "INFO" "Logs available in: $LOGS_DIR/"

    if [[ "$GENERATE_VISUALIZATION" == "true" ]]; then
        log_message "INFO" "Latest visualization: $VISUALIZATION_FILE"
    fi
}

# Run main function with all arguments
main "$@"