# 🌐 GeoSteiner Budget-Constrained SMT Simulation Wrapper

## Overview

The `simulate` program provides an automated end-to-end pipeline for budget-constrained Steiner Minimum Tree (SMT) optimization using GeoSteiner. It integrates all components from data generation to visualization in a single command.

## Features

- **🎲 Automatic Data Generation**: Creates random terminal coordinates with realistic battery level distributions
- **🌳 FST Computation**: Automatically computes Full Steiner Trees using GeoSteiner's `efst`
- **🎯 Budget-Constrained Optimization**: Solves multi-objective SMT with source terminal constraints using `bb`
- **📊 Interactive Visualization**: Generates HTML visualizations showing network topology and solution metrics
- **⚙️ Source Terminal Constraint**: Enforces that terminal 0 (source) is always covered in solutions

## Installation

The simulation wrapper is built automatically with GeoSteiner:

```bash
make simulate
```

## Usage

The `simulate` program supports two distinct modes:

### 1. Full Simulation Mode

Generates data, computes FSTs, solves SMT, and creates visualization in one pipeline.

```bash
./simulate -n <num_terminals> -b <budget> [options]
```

**Required Arguments:**
- `-n N`: Number of terminals to generate (must be > 0)
- `-b BUDGET`: Budget constraint for SMT optimization

**Optional Arguments:**
- `-s SEED`: Random seed for reproducible terminal generation (default: current time)
- `-o OUTDIR`: Output directory (default: `simulation_output`)
- `-v`: Enable verbose output showing detailed progress
- `-h`: Show help message

### 2. Visualization-Only Mode ⭐ NEW

Creates visualization from existing solution files (CPLEX output).

```bash
./simulate -t <terminals> -f <fsts> -r <solution> -w <output> [options]
```

**Required Arguments:**
- `-t FILE`: Terminals file (coordinates and battery levels)
- `-f FILE`: FSTs file (Full Steiner Tree data)
- `-r FILE`: Solution file (CPLEX solver output)
- `-w FILE`: Output HTML file for visualization

**Optional Arguments:**
- `-v`: Enable verbose output
- `-h`: Show help message

### Examples

```bash
# Full simulation with 10 terminals and 1.5M budget
./simulate -n 10 -b 1500000

# Reproducible simulation with custom output directory
./simulate -n 8 -b 1200000 -s 42 -o my_test -v

# Large-scale simulation
./simulate -n 20 -b 3000000 -s 12345 -o large_network -v

# Visualization-only mode (from existing files)
./simulate -t terminals.txt -f fsts.txt -r solution.txt -w custom_viz.html -v

# Visualize results from different runs
./simulate -t run1/terminals.txt -f run1/fsts.txt -r run1/solution.txt -w run1_analysis.html
```

## Pipeline Stages

The simulation automatically executes four stages:

### 1. 📍 Terminal Generation
- Generates N random terminals with coordinates in [0,1] × [0,1]
- Assigns realistic battery levels using weighted distribution:
  - 20% low battery (10-40%)
  - 60% normal battery (40-80%)
  - 20% high battery (80-100%)
- Saves terminals to `terminals.txt`

### 2. 🌳 FST Computation
- Runs GeoSteiner's `efst` program to compute Full Steiner Trees
- Generates all possible FST topologies with Steiner points
- Saves FST data to `fsts.txt`

### 3. 🎯 Budget-Constrained SMT Solving
- Runs the enhanced `bb` solver with budget constraints
- **Enforces source terminal constraint**: `not_covered[0] = 0`
- Applies multi-objective optimization balancing:
  - Tree construction costs
  - Battery consumption costs
  - Terminal coverage penalties
- Saves detailed solution to `solution.txt`

### 4. 📊 Visualization Generation
- Creates interactive HTML visualization
- Shows network topology with FST selections
- Displays terminal coverage status and battery levels
- Includes solution metrics and constraint details
- Saves to `visualization.html`

## Output Files

Each simulation creates the following files in the output directory:

| File | Description |
|------|-------------|
| `terminals.txt` | Terminal coordinates and battery levels |
| `fsts.txt` | Full Steiner Tree topologies and costs |
| `solution.txt` | Detailed CPLEX solution with debug information |
| `visualization.html` | Interactive network visualization |

## Key Features

### Source Terminal Constraint
- **Guaranteed Coverage**: Terminal 0 is always covered in feasible solutions
- **Mathematical Formulation**: `not_covered[0] = 0`
- **Use Case**: Ensures network connectivity from a designated source node

### Multi-Objective Optimization
- **Primary**: Minimize tree construction costs
- **Secondary**: Minimize battery consumption (weighted by factor α=100.0)
- **Penalty**: Large penalty (β=1,000,000) for uncovered terminals

### Budget Enforcement
- **Hard Constraint**: Total tree costs ≤ specified budget
- **Practical Application**: Resource-constrained network deployment

## Performance

- **Small Networks** (≤10 terminals): Typically completes in seconds
- **Medium Networks** (10-20 terminals): Usually under 1 minute
- **Large Networks** (>20 terminals): May require several minutes
- **Timeout Protection**: 5-minute timeout prevents infinite runs

## Troubleshooting

### Common Issues

1. **"efst command not found"**
   - Ensure GeoSteiner is properly compiled: `make`
   - Run from GeoSteiner root directory

2. **"Permission denied"**
   - Make executable: `chmod +x simulate`

3. **Visualization not generated**
   - Check if Python 3 and `html_generator.py` are available
   - Basic HTML fallback is created if Python generator fails

4. **Solution timeout**
   - Reduce number of terminals or increase budget
   - Large instances may require more time than 5-minute limit

### Debug Information

Use `-v` flag for detailed progress information:
- Terminal generation details
- Command execution status
- File creation confirmation
- Error messages and warnings

## Integration with Other Tools

### With Python HTML Generator
```bash
# Manual visualization generation
python3 html_generator.py --terminals terminals.txt --fsts fsts.txt --solution solution.txt --output custom_viz.html
```

### With Custom Analysis
```bash
# Extract specific solution metrics
grep "Final solution" solution.txt
grep "not_covered\[0\]" solution.txt

# Use visualization-only mode for analysis
./simulate -t terminals.txt -f fsts.txt -r solution.txt -w analysis.html -v
```

## Example Session

```bash
$ ./simulate -n 8 -b 1200000 -s 42 -o demo -v

🌐 GeoSteiner Budget-Constrained SMT Simulation
================================================
Terminals:     8
Budget:        1200000
Seed:          42
Output Dir:    demo
Verbose:       Yes
================================================

📍 Step 1: Generating 8 random terminals...
   Terminal 0: (0.033, 0.330) battery=56.9%
   Terminal 1: (0.206, 0.250) battery=74.5%
   [... additional terminals ...]
   ✅ Terminals saved to: demo/terminals.txt

🌳 Step 2: Computing Full Steiner Trees...
   ✅ FSTs saved to: demo/fsts.txt

🎯 Step 3: Solving budget-constrained SMT (budget=1200000)...
   ✅ Solution saved to: demo/solution.txt

📊 Step 4: Generating HTML visualization...
   ✅ Visualization saved to: demo/visualization.html

🎉 Simulation completed successfully!
📁 All outputs available in: demo/
🌐 Open demo/visualization.html in a web browser to view results
```

## Technical Details

- **Language**: C (ANSI C compatible)
- **Dependencies**: GeoSteiner library, CPLEX (for optimization)
- **Memory Usage**: Scales with number of terminals and FST complexity
- **Thread Safety**: Single-threaded execution
- **Platform Support**: Linux, macOS, Windows (with appropriate compilation)

## Contributing

To extend the simulation wrapper:

1. **Add new objective functions**: Modify the cost calculation in `solve_smt()`
2. **Custom terminal distributions**: Extend `random_battery_level()`
3. **Additional output formats**: Enhance `generate_visualization()`
4. **Performance improvements**: Optimize command execution and file I/O

## License

This simulation wrapper follows the same license as GeoSteiner: Creative Commons Attribution-NonCommercial 4.0 International License.