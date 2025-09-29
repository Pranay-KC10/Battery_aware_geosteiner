# GeoSteiner with Budget Constraints - Usage Guide

## Multi-objective Optimization with Budget Constraints

This implementation adds budget-constrained multi-objective optimization to GeoSteiner using CPLEX integration.

### Environment Variables

- `GEOSTEINER_BUDGET=<budget_value>`: Enable budget-constrained mode with specified budget limit
  - Example: `GEOSTEINER_BUDGET=200000`
  - When set, the solver uses multi-objective optimization with soft cutset constraints

### Pipeline Usage

The correct pipeline for testing budget-constrained optimization:

```bash
# Generate test data and run optimization
./rand_points 4 | ./efst | GEOSTEINER_BUDGET=200000 ./bb

# Or step by step:
./rand_points 4 > test_4pts.in
./efst < test_4pts.in > test_4pts.fst
GEOSTEINER_BUDGET=200000 ./bb test_4pts.fst
```

### Mathematical Formulation

When `GEOSTEINER_BUDGET` is set, the solver uses:

**Objective Function:**
- Minimize: Σ(tree_cost[i] + α×battery_cost[i])×x[i] + β×Σ(not_covered[j])
- Where α = 0.1 (battery cost weight), β = 1000000 (penalty for uncovered terminals)

**Constraints:**
1. **Budget Constraint:** Σ tree_cost[i] × x[i] ≤ budget_limit
2. **Soft Cutset Constraints:** For each terminal j: Σ(x[i]: FST i covers terminal j) + not_covered[j] ≥ 1

**Variables:**
- x[i]: Binary variables for Full Steiner Trees (FSTs)
- not_covered[j]: Continuous variables for uncovered terminals (0 ≤ not_covered[j] ≤ 1)

### Debug Output

Set `GEOSTEINER_BUDGET` to see detailed debug information:
- LP variable values (FST and not_covered variables)
- Budget constraint coefficients
- Soft cutset constraint generation
- Terminal mapping and indexing

### Key Implementation Details

- **Variable Count:** LP uses `nedges + nterms` variables (FST variables + not_covered variables)
- **Constraint Pool:** Buffer allocation accounts for additional variables in multi-objective mode
- **Terminal Indexing:** Consistent 0-based indexing throughout (terminals and FSTs both start at 0)
- **Memory Management:** All arrays sized for `total_vars = nedges + num_terminals`

### Compilation

```bash
make clean
make
```

Requires CPLEX installation with proper library paths configured.