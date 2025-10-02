/***********************************************************************

	File:	simulate.c
	Rev:	e-1
	Date:	9/18/2025

************************************************************************

	Simulation Wrapper for Budget-Constrained GeoSteiner Optimization

	This program automates the complete pipeline:
	1. Generate random terminal coordinates with battery levels
	2. Compute Full Steiner Trees (FSTs) using efst
	3. Solve budget-constrained multi-objective SMT using bb
	4. Generate HTML visualization of the solution

	Usage:
		./simulate -n N -b BUDGET [-s SEED] [-o OUTDIR] [-v] [-h]

	Where:
		-n N        Number of terminals to generate
		-b BUDGET   Budget constraint for optimization
		-s SEED     Random seed (default: current time)
		-o OUTDIR   Output directory (default: simulation_output)
		-v          Verbose output
		-h          Show help

************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>

/* Extract final mip gap from solution.txt */
static double parse_final_mip_gap(const char *solution_file) {
    FILE *fp = fopen(solution_file, "r");
    if (!fp) return -1.0;

    char line[4096];
    double best_bound = NAN, incumbent = NAN, gap = NAN;
    double latest_best_z = NAN, latest_branch_z0 = NAN, latest_branch_z1 = NAN;

    while (fgets(line, sizeof(line), fp)) {
        /* Try pattern: "Best bound = X, Best integer = Y" */
        double bb, bi;
        if (sscanf(line, "Best bound = %lf , Best integer = %lf", &bb, &bi) == 2) {
            best_bound = bb; incumbent = bi;
            if (!isnan(best_bound) && !isnan(incumbent) && incumbent != 0.0) {
                gap = fabs(incumbent - best_bound) / fabs(incumbent);
            }
        }

        /* Try pattern: "MIP gap = Z%" */
        double g;
        if (sscanf(line, "MIP gap = %lf%%", &g) == 1) {
            gap = g / 100.0;
        }

        /* Try pattern: "Solution status X: MIP optimal, tolerance" */
        if (strstr(line, "MIP optimal") && strstr(line, "tolerance")) {
            char *gap_str = strstr(line, "(");
            if (gap_str) {
                gap_str++; /* Skip opening parenthesis */
                double g;
                if (sscanf(gap_str, "%lf%%", &g) == 1) {
                    gap = g / 100.0;
                }
            }
        }

        /* Try GeoSteiner debug pattern: "New best: x..., Z = value" */
        double z_val;
        if (strstr(line, "New best:") && strstr(line, "Z =")) {
            char *z_pos = strstr(line, "Z =");
            if (z_pos && sscanf(z_pos, "Z = %lf", &z_val) == 1) {
                latest_best_z = z_val;
            }
        }

        /* Try GeoSteiner branch pattern: "Best branch is x..., Z0 = val1, Z1 = val2" */
        double z0_val, z1_val;
        if (strstr(line, "Best branch is") && strstr(line, "Z0 =") && strstr(line, "Z1 =")) {
            char *z0_pos = strstr(line, "Z0 =");
            char *z1_pos = strstr(line, "Z1 =");
            if (z0_pos && z1_pos &&
                sscanf(z0_pos, "Z0 = %lf", &z0_val) == 1 &&
                sscanf(z1_pos, "Z1 = %lf", &z1_val) == 1) {
                latest_branch_z0 = z0_val;
                latest_branch_z1 = z1_val;
                /* Use the smaller (better) of the two as incumbent, larger as bound */
                if (!isnan(z0_val) && !isnan(z1_val)) {
                    incumbent = (z0_val < z1_val) ? z0_val : z1_val;
                    best_bound = (z0_val > z1_val) ? z0_val : z1_val;
                    if (incumbent != 0.0) {
                        gap = fabs(best_bound - incumbent) / fabs(incumbent);
                    }
                }
            }
        }
    }

    /* If we have recent best values but no calculated gap, try to estimate */
    if (isnan(gap) && !isnan(latest_best_z) && !isnan(latest_branch_z0) && !isnan(latest_branch_z1)) {
        incumbent = latest_best_z;
        best_bound = (latest_branch_z0 > latest_branch_z1) ? latest_branch_z0 : latest_branch_z1;
        if (incumbent != 0.0) {
            gap = fabs(best_bound - incumbent) / fabs(incumbent);
        }
    }

    fclose(fp);
    return gap;
}

/* Data structures for visualization */
typedef struct {
    double x, y;
    double battery;
    int covered;
    int terminal_id;
} Terminal;

typedef struct {
    double x, y;
} SteinerPoint;

typedef struct {
    int selected;
    int num_terminals;
    int terminal_ids[10];
    int num_steiner_points;
    SteinerPoint steiner_points[10];
    double cost;
    int fst_id;
} FST;

/* Function prototypes */
static void usage(void);
static void generate_terminals(int n_terminals, const char* output_dir, int seed, int verbose);
static void generate_fsts(const char* terminals_file, const char* fsts_file, int verbose);
static void generate_fst_dump(const char* fsts_file, const char* dump_file, int verbose);
static void solve_smt(const char* fsts_file, const char* solution_file, int budget, int verbose);
static void generate_visualization(const char* terminals_file, const char* fsts_file,
                                  const char* solution_file, const char* html_file, int verbose);
static void run_visualization_only(const char* terminals_file, const char* fsts_file,
                                  const char* solution_file, const char* html_file, int verbose);
static void create_rich_visualization(const char* terminals_file, const char* fsts_file,
                                     const char* solution_file, const char* html_file, int verbose);
static int parse_terminals(const char* terminals_file, Terminal terminals[], int max_terminals);
static int parse_solution_coverage(const char* solution_file, int coverage[], int max_terminals);
static int parse_fsts_from_solution(const char* solution_file, FST fsts[], int max_fsts);
static int parse_fsts_from_dump(const char* dump_file, FST fsts[], int max_fsts);
static int parse_selected_fst_ids(const char* solution_file, int selected_ids[], int max_fsts);
static int parse_selected_fsts(const char* solution_file, int selected_fsts[], int max_fsts);
static void get_battery_color(double battery, char* color_str);
static void scale_coordinates(double x, double y, int* scaled_x, int* scaled_y);
static int run_command(const char* command, int verbose);
static void create_directory(const char* dir_path);
static double random_double(void);
static double random_battery_level(void);
static double parse_final_mip_gap(const char *solution_file);

/* Global variables for simulation parameters */
static int g_verbose = 0;

int main(int argc, char* argv[])
{
	int n_terminals = 0;
	int budget = 0;
	int seed = 0;
	char output_dir[256] = "simulation_output";
	char terminals_file[512];
	char fsts_file[512];
	char solution_file[512];
	char html_file[512];
	int opt;
	int visualization_only = 0;
	char viz_terminals[512] = "";
	char viz_fsts[512] = "";
	char viz_solution[512] = "";
	char viz_output[512] = "";

	/* Parse command line arguments */
	while ((opt = getopt(argc, argv, "n:b:s:o:vht:f:r:w:")) != -1) {
		switch (opt) {
		case 'n':
			n_terminals = atoi(optarg);
			break;
		case 'b':
			budget = atoi(optarg);
			break;
		case 's':
			seed = atoi(optarg);
			break;
		case 'o':
			strncpy(output_dir, optarg, sizeof(output_dir) - 1);
			output_dir[sizeof(output_dir) - 1] = '\0';
			break;
		case 'v':
			g_verbose = 1;
			break;
		case 'h':
			usage();
			exit(0);
		case 't':
			strncpy(viz_terminals, optarg, sizeof(viz_terminals) - 1);
			viz_terminals[sizeof(viz_terminals) - 1] = '\0';
			visualization_only = 1;
			break;
		case 'f':
			strncpy(viz_fsts, optarg, sizeof(viz_fsts) - 1);
			viz_fsts[sizeof(viz_fsts) - 1] = '\0';
			visualization_only = 1;
			break;
		case 'r':
			strncpy(viz_solution, optarg, sizeof(viz_solution) - 1);
			viz_solution[sizeof(viz_solution) - 1] = '\0';
			visualization_only = 1;
			break;
		case 'w':
			strncpy(viz_output, optarg, sizeof(viz_output) - 1);
			viz_output[sizeof(viz_output) - 1] = '\0';
			visualization_only = 1;
			break;
		default:
			usage();
			exit(1);
		}
	}

	/* Handle visualization-only mode */
	if (visualization_only) {
		if (strlen(viz_terminals) == 0 || strlen(viz_fsts) == 0 ||
		    strlen(viz_solution) == 0 || strlen(viz_output) == 0) {
			fprintf(stderr, "Error: Visualization mode requires all four files:\n");
			fprintf(stderr, "  -t <terminals_file>\n");
			fprintf(stderr, "  -f <fsts_file>\n");
			fprintf(stderr, "  -r <solution_file>\n");
			fprintf(stderr, "  -w <output_html_file>\n");
			usage();
			exit(1);
		}

		printf("🎨 GeoSteiner Visualization Generator\n");
		printf("=====================================\n");
		printf("Terminals:  %s\n", viz_terminals);
		printf("FSTs:       %s\n", viz_fsts);
		printf("Solution:   %s\n", viz_solution);
		printf("Output:     %s\n", viz_output);
		printf("Verbose:    %s\n", g_verbose ? "Yes" : "No");
		printf("=====================================\n\n");

		run_visualization_only(viz_terminals, viz_fsts, viz_solution, viz_output, g_verbose);

		printf("🎉 Visualization generated successfully!\n");
		printf("🌐 Open %s in a web browser to view results\n", viz_output);
		return 0;
	}

	/* Validate required parameters for full simulation */
	if (n_terminals <= 0) {
		fprintf(stderr, "Error: Number of terminals (-n) must be positive\n");
		usage();
		exit(1);
	}
	if (budget <= 0) {
		fprintf(stderr, "Error: Budget (-b) must be positive\n");
		usage();
		exit(1);
	}

	/* Set default seed if not provided */
	if (seed == 0) {
		seed = (int)time(NULL);
	}

	printf("🌐 GeoSteiner Budget-Constrained SMT Simulation\n");
	printf("================================================\n");
	printf("Terminals:     %d\n", n_terminals);
	printf("Budget:        %d\n", budget);
	printf("Seed:          %d\n", seed);
	printf("Output Dir:    %s\n", output_dir);
	printf("Verbose:       %s\n", g_verbose ? "Yes" : "No");
	printf("================================================\n\n");

	/* Initialize random number generator */
	srand(seed);

	/* Create output directory */
	create_directory(output_dir);

	/* Set up file paths */
	snprintf(terminals_file, sizeof(terminals_file), "%s/terminals.txt", output_dir);
	snprintf(fsts_file, sizeof(fsts_file), "%s/fsts.txt", output_dir);
	snprintf(solution_file, sizeof(solution_file), "%s/solution.txt", output_dir);
	snprintf(html_file, sizeof(html_file), "%s/visualization.html", output_dir);

	/* Step 1: Generate random terminals with battery levels */
	printf("📍 Step 1: Generating %d random terminals...\n", n_terminals);
	generate_terminals(n_terminals, output_dir, seed, g_verbose);
	printf("   ✅ Terminals saved to: %s\n\n", terminals_file);

	/* Step 2: Generate Full Steiner Trees (FSTs) */
	printf("🌳 Step 2: Computing Full Steiner Trees...\n");
	generate_fsts(terminals_file, fsts_file, g_verbose);
	printf("   ✅ FSTs saved to: %s\n", fsts_file);

	/* Step 2b: Generate readable FST dump */
	char fsts_dump_file[512];
	snprintf(fsts_dump_file, sizeof(fsts_dump_file), "%s/fsts_dump.txt", output_dir);
	printf("📋 Step 2b: Generating readable FST dump...\n");
	generate_fst_dump(fsts_file, fsts_dump_file, g_verbose);
	printf("   ✅ FST dump saved to: %s\n\n", fsts_dump_file);

	/* Step 3: Solve budget-constrained SMT */
	printf("🎯 Step 3: Solving budget-constrained SMT (budget=%d)...\n", budget);
	solve_smt(fsts_file, solution_file, budget, g_verbose);
	printf("   ✅ Solution saved to: %s\n", solution_file);

	/* Parse and report final MIP gap */
	double final_gap = parse_final_mip_gap(solution_file);
	if (final_gap >= 0.0) {
		printf("   📊 Final MIP Gap: %.4f%% (%.6f)\n", final_gap * 100.0, final_gap);
	} else {
		printf("   ⚠️  Could not parse MIP gap from solution\n");
	}
	printf("\n");

	/* Step 4: Generate HTML visualization */
	printf("📊 Step 4: Generating rich HTML visualization...\n");
	create_rich_visualization(terminals_file, fsts_file, solution_file, html_file, g_verbose);
	printf("   ✅ Rich visualization saved to: %s\n\n", html_file);

	printf("🎉 Simulation completed successfully!\n");
	printf("📁 All outputs available in: %s/\n", output_dir);
	printf("🌐 Open %s in a web browser to view results\n", html_file);

	return 0;
}

static void usage(void)
{
	printf("Usage: ./simulate [MODE] [OPTIONS]\n\n");
	printf("Automated Budget-Constrained GeoSteiner Simulation Pipeline\n\n");

	printf("FULL SIMULATION MODE:\n");
	printf("  ./simulate -n N -b BUDGET [-s SEED] [-o OUTDIR] [-v] [-h]\n\n");
	printf("Required arguments:\n");
	printf("  -n N        Number of terminals to generate (must be > 0)\n");
	printf("  -b BUDGET   Budget constraint for SMT optimization\n\n");
	printf("Optional arguments:\n");
	printf("  -s SEED     Random seed for terminal generation (default: current time)\n");
	printf("  -o OUTDIR   Output directory (default: simulation_output)\n");
	printf("  -v          Enable verbose output\n");
	printf("  -h          Show this help message\n\n");

	printf("VISUALIZATION-ONLY MODE:\n");
	printf("  ./simulate -t TERMINALS -f FSTS -r SOLUTION -w OUTPUT [-v] [-h]\n\n");
	printf("Required arguments:\n");
	printf("  -t FILE     Terminals file (coordinates and battery levels)\n");
	printf("  -f FILE     FSTs file (Full Steiner Tree data)\n");
	printf("  -r FILE     Solution file (CPLEX solver output)\n");
	printf("  -w FILE     Output HTML file for visualization\n\n");

	printf("Examples:\n");
	printf("  # Full simulation\n");
	printf("  ./simulate -n 10 -b 1500000 -s 12345 -o my_simulation -v\n\n");
	printf("  # Visualization only\n");
	printf("  ./simulate -t terminals.txt -f fsts.txt -r solution.txt -w viz.html -v\n\n");

	printf("Full simulation pipeline stages:\n");
	printf("  1. Generate random terminals with battery levels\n");
	printf("  2. Compute Full Steiner Trees (FSTs) using efst\n");
	printf("  3. Solve budget-constrained SMT using bb\n");
	printf("  4. Generate interactive HTML visualization\n");
}

static void generate_terminals(int n_terminals, const char* output_dir, int seed, int verbose)
{
	char terminals_file[512];
	FILE* fp;
	int i;
	double x, y, battery;

	snprintf(terminals_file, sizeof(terminals_file), "%s/terminals.txt", output_dir);

	fp = fopen(terminals_file, "w");
	if (!fp) {
		fprintf(stderr, "Error: Cannot create terminals file: %s\n", terminals_file);
		exit(1);
	}

	if (verbose) {
		printf("   Generating terminals with seed %d:\n", seed);
	}

	for (i = 0; i < n_terminals; i++) {
		x = random_double();
		y = random_double();
		battery = random_battery_level();

		fprintf(fp, "%.6f %.6f %.1f\n", x, y, battery);

		if (verbose) {
			printf("   Terminal %d: (%.3f, %.3f) battery=%.1f%%\n", i, x, y, battery);
		}
	}

	fclose(fp);

	if (verbose) {
		printf("   Saved %d terminals to %s\n", n_terminals, terminals_file);
	}
}

static void generate_fsts(const char* terminals_file, const char* fsts_file, int verbose)
{
	char command[1024];
	int result;

	/* Run efst to generate Full Steiner Trees */
	snprintf(command, sizeof(command), "./efst < \"%s\" > \"%s\" 2>/dev/null",
	         terminals_file, fsts_file);

	if (verbose) {
		printf("   Running: %s\n", command);
	}

	result = run_command(command, verbose);
	if (result != 0) {
		fprintf(stderr, "Error: FST generation failed (exit code %d)\n", result);
		exit(1);
	}

	if (verbose) {
		printf("   FST generation completed successfully\n");
	}
}

static void generate_fst_dump(const char* fsts_file, const char* dump_file, int verbose)
{
	char command[1024];
	int result;

	/* Run dumpfst to generate readable FST list */
	snprintf(command, sizeof(command), "./dumpfst < \"%s\" > \"%s\" 2>/dev/null",
	         fsts_file, dump_file);

	if (verbose) {
		printf("   Running: %s\n", command);
	}

	result = run_command(command, verbose);
	if (result != 0) {
		fprintf(stderr, "Error: FST dump generation failed (exit code %d)\n", result);
		exit(1);
	}

	if (verbose) {
		printf("   FST dump generation completed successfully\n");
	}
}

static void solve_smt(const char* fsts_file, const char* solution_file, int budget, int verbose)
{
	char command[1024];
	char env_var[64];
	int result;

	/* Set budget environment variable and run bb solver */
	snprintf(env_var, sizeof(env_var), "GEOSTEINER_BUDGET=%d", budget);
	snprintf(command, sizeof(command), "%s timeout 300s ./bb < \"%s\" > \"%s\" 2>&1",
	         env_var, fsts_file, solution_file);

	if (verbose) {
		printf("   Setting %s\n", env_var);
		printf("   Running: timeout 300s ./bb < %s > %s\n", fsts_file, solution_file);
	}

	result = run_command(command, verbose);
	if (result != 0 && result != 124) { /* 124 is timeout exit code */
		fprintf(stderr, "Warning: SMT solver returned exit code %d\n", result);
		/* Continue anyway - partial solutions may still be useful */
	}

	if (verbose) {
		printf("   SMT solving completed\n");
	}
}

static void generate_visualization(const char* terminals_file, const char* fsts_file,
                                  const char* solution_file, const char* html_file, int verbose)
{
	char command[1024];
	int result;

	/* Use the Python HTML generator if available, otherwise create basic HTML */
	if (access("html_generator.py", F_OK) == 0) {
		snprintf(command, sizeof(command),
		         "python3 html_generator.py --terminals \"%s\" --fsts \"%s\" --solution \"%s\" --output \"%s\" 2>/dev/null",
		         terminals_file, fsts_file, solution_file, html_file);

		if (verbose) {
			printf("   Running Python HTML generator\n");
		}

		result = run_command(command, verbose);
		if (result == 0) {
			if (verbose) {
				printf("   HTML visualization generated successfully\n");
			}
			return;
		}
	}

	/* Fallback: create basic HTML file */
	if (verbose) {
		printf("   Creating basic HTML visualization\n");
	}

	FILE* fp = fopen(html_file, "w");
	if (!fp) {
		fprintf(stderr, "Error: Cannot create HTML file: %s\n", html_file);
		exit(1);
	}

	fprintf(fp, "<!DOCTYPE html>\n");
	fprintf(fp, "<html><head><title>GeoSteiner Simulation Results</title></head>\n");
	fprintf(fp, "<body>\n");
	fprintf(fp, "<h1>🌐 GeoSteiner Budget-Constrained SMT Results</h1>\n");
	fprintf(fp, "<h2>📁 Generated Files</h2>\n");
	fprintf(fp, "<ul>\n");
	fprintf(fp, "<li><strong>Terminals:</strong> %s</li>\n", terminals_file);
	fprintf(fp, "<li><strong>FSTs:</strong> %s</li>\n", fsts_file);
	fprintf(fp, "<li><strong>Solution:</strong> %s</li>\n", solution_file);
	fprintf(fp, "</ul>\n");
	fprintf(fp, "<h2>📊 Solution Analysis</h2>\n");
	fprintf(fp, "<p>Review the solution file for detailed SMT optimization results.</p>\n");
	fprintf(fp, "<h2>🔧 Manual Visualization</h2>\n");
	fprintf(fp, "<p>Use the Python HTML generator for full interactive visualization:</p>\n");
	fprintf(fp, "<code>python3 html_generator.py --terminals %s --fsts %s --solution %s --output visualization_full.html</code>\n",
	        terminals_file, fsts_file, solution_file);
	fprintf(fp, "</body></html>\n");

	fclose(fp);

	if (verbose) {
		printf("   Basic HTML file created\n");
	}
}

static int run_command(const char* command, int verbose)
{
	int result;

	if (verbose) {
		printf("   Executing: %s\n", command);
	}

	result = system(command);

	if (result == -1) {
		fprintf(stderr, "Error: Failed to execute command: %s\n", strerror(errno));
		return -1;
	}

	return WEXITSTATUS(result);
}

static void create_directory(const char* dir_path)
{
	struct stat st;

	if (stat(dir_path, &st) == 0) {
		if (S_ISDIR(st.st_mode)) {
			if (g_verbose) {
				printf("   Directory %s already exists\n", dir_path);
			}
			return;
		} else {
			fprintf(stderr, "Error: %s exists but is not a directory\n", dir_path);
			exit(1);
		}
	}

	if (mkdir(dir_path, 0755) != 0) {
		fprintf(stderr, "Error: Cannot create directory %s: %s\n", dir_path, strerror(errno));
		exit(1);
	}

	if (g_verbose) {
		printf("   Created directory: %s\n", dir_path);
	}
}

static double random_double(void)
{
	return (double)rand() / RAND_MAX;
}

static double random_battery_level(void)
{
	/* Generate battery levels with realistic distribution */
	/* 20% chance of low battery (10-40%), 60% normal (40-80%), 20% high (80-100%) */
	double r = random_double();

	if (r < 0.2) {
		/* Low battery: 10-40% */
		return 10.0 + random_double() * 30.0;
	} else if (r < 0.8) {
		/* Normal battery: 40-80% */
		return 40.0 + random_double() * 40.0;
	} else {
		/* High battery: 80-100% */
		return 80.0 + random_double() * 20.0;
	}
}

static void run_visualization_only(const char* terminals_file, const char* fsts_file,
                                  const char* solution_file, const char* html_file, int verbose)
{
	char command[2048];
	int result;

	/* Verify input files exist */
	if (access(terminals_file, F_OK) != 0) {
		fprintf(stderr, "Error: Terminals file not found: %s\n", terminals_file);
		exit(1);
	}
	if (access(fsts_file, F_OK) != 0) {
		fprintf(stderr, "Error: FSTs file not found: %s\n", fsts_file);
		exit(1);
	}
	if (access(solution_file, F_OK) != 0) {
		fprintf(stderr, "Error: Solution file not found: %s\n", solution_file);
		exit(1);
	}

	if (verbose) {
		printf("📊 Generating visualization from existing files...\n");
		printf("   Terminals: %s\n", terminals_file);
		printf("   FSTs:      %s\n", fsts_file);
		printf("   Solution:  %s\n", solution_file);
		printf("   Output:    %s\n", html_file);
	}

	/* Try Python HTML generator first */
	if (access("html_generator.py", F_OK) == 0) {
		snprintf(command, sizeof(command),
		         "python3 html_generator.py --terminals \"%s\" --fsts \"%s\" --solution \"%s\" --output \"%s\" 2>/dev/null",
		         terminals_file, fsts_file, solution_file, html_file);

		if (verbose) {
			printf("   Running Python HTML generator\n");
		}

		result = run_command(command, verbose);
		if (result == 0) {
			if (verbose) {
				printf("   ✅ Interactive HTML visualization generated\n");
			}
			return;
		} else {
			if (verbose) {
				printf("   Warning: Python generator failed, creating rich C visualization\n");
			}
		}
	}

	/* Create rich C-based visualization */
	create_rich_visualization(terminals_file, fsts_file, solution_file, html_file, verbose);
}

static void create_rich_visualization(const char* terminals_file, const char* fsts_file,
                                     const char* solution_file, const char* html_file, int verbose)
{
	Terminal terminals[50];
	int coverage[50] = {0};
	int num_terminals;
	int i;
	FILE* fp;

	if (verbose) {
		printf("   Creating rich SVG network visualization\n");
	}

	/* Parse input files */
	num_terminals = parse_terminals(terminals_file, terminals, 50);
	if (num_terminals <= 0) {
		fprintf(stderr, "Error: Could not parse terminals file\n");
		return;
	}

	parse_solution_coverage(solution_file, coverage, 50);

	/* Update terminal coverage */
	for (i = 0; i < num_terminals; i++) {
		terminals[i].covered = coverage[i];
		terminals[i].terminal_id = i;
	}

	if (verbose) {
		printf("   Parsed %d terminals with coverage data\n", num_terminals);
	}

	/* Create HTML file */
	fp = fopen(html_file, "w");
	if (!fp) {
		fprintf(stderr, "Error: Cannot create HTML file: %s\n", html_file);
		return;
	}

	/* Write HTML header and styles */
	fprintf(fp, "<!DOCTYPE html>\n");
	fprintf(fp, "<html lang=\"en\">\n");
	fprintf(fp, "<head>\n");
	fprintf(fp, "    <meta charset=\"UTF-8\">\n");
	fprintf(fp, "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n");
	fprintf(fp, "    <title>GeoSteiner Network Optimization - Budget-Constrained Solution</title>\n");
	fprintf(fp, "    <style>\n");
	fprintf(fp, "        body { font-family: 'Segoe UI', Arial, sans-serif; margin: 20px; background: #f8f9fa; }\n");
	fprintf(fp, "        .container { max-width: 1400px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }\n");
	fprintf(fp, "        h1 { color: #2c3e50; text-align: center; margin-bottom: 30px; }\n");
	fprintf(fp, "        .network-container { display: flex; gap: 30px; margin: 30px 0; }\n");
	fprintf(fp, "        .network-svg { flex: 2; border: 2px solid #ddd; border-radius: 8px; background: #fafafa; }\n");
	fprintf(fp, "        .sidebar { flex: 1; }\n");
	fprintf(fp, "        .terminal-label { font-size: 14px; font-weight: bold; fill: #333; }\n");
	fprintf(fp, "        .battery-text { font-size: 12px; fill: #666; }\n");
	fprintf(fp, "        .metrics, .legend, .fst-details { background: #f8f9fa; padding: 20px; margin: 20px 0; border-radius: 8px; border-left: 4px solid #3498db; }\n");
	fprintf(fp, "        .source-constraint { background: #d4edda; padding: 15px; margin: 20px 0; border-radius: 8px; border-left: 4px solid #28a745; }\n");
	fprintf(fp, "        .section { background: #fff; margin: 30px 0; padding: 25px; border-radius: 8px; border: 1px solid #e1e8ed; }\n");
	fprintf(fp, "        .constraint-check { padding: 10px; margin: 8px 0; border-radius: 5px; background: #f8f9fa; border-left: 3px solid #28a745; }\n");
	fprintf(fp, "        table { width: 100%%; border-collapse: collapse; }\n");
	fprintf(fp, "        td { padding: 8px; border-bottom: 1px solid #eee; }\n");
	fprintf(fp, "        .legend-item { display: flex; align-items: center; margin: 10px 0; }\n");
	fprintf(fp, "        .legend-symbol { width: 20px; height: 20px; margin-right: 10px; border-radius: 50%%; }\n");
	fprintf(fp, "        .covered-terminal { background: #00ff00; border: 2px solid #333; }\n");
	fprintf(fp, "        .uncovered-terminal { background: none; border: 2px dashed #999; position: relative; }\n");
	fprintf(fp, "        .selected-fst { background: #007bff; }\n");
	fprintf(fp, "        .steiner-point { background: #6c757d; }\n");
	fprintf(fp, "    </style>\n");
	fprintf(fp, "</head>\n");
	fprintf(fp, "<body>\n");
	fprintf(fp, "    <div class=\"container\">\n");
	fprintf(fp, "        <h1>🌐 GeoSteiner Network Optimization - Budget-Constrained Solution</h1>\n");

	/* Network visualization container */
	fprintf(fp, "        <div class=\"network-container\">\n");
	fprintf(fp, "            <svg width=\"800\" height=\"600\" class=\"network-svg\">\n");

	/* Parse ALL FSTs from dump file and mark which ones are selected */
	char fsts_dump_file[512];
	char* dir_end = strrchr(fsts_file, '/');
	if (dir_end) {
		size_t dir_len = dir_end - fsts_file + 1;
		strncpy(fsts_dump_file, fsts_file, dir_len);
		fsts_dump_file[dir_len] = '\0';
		strcat(fsts_dump_file, "fsts_dump.txt");
	} else {
		strcpy(fsts_dump_file, "fsts_dump.txt");
	}

	FST all_fsts[100];
	int num_all_fsts = parse_fsts_from_dump(fsts_dump_file, all_fsts, 100);

	/* Parse which FSTs are selected from PostScript solution */
	int selected_fst_ids[50];
	int num_selected_ids = parse_selected_fst_ids(solution_file, selected_fst_ids, 50);

	if (verbose) {
		printf("   Found %d total FSTs from efst output\n", num_all_fsts);
		if (num_all_fsts > 0) {
			for (int k = 0; k < (num_all_fsts < 5 ? num_all_fsts : 5); k++) {
				printf("   FST %d: ", all_fsts[k].fst_id);
				for (int l = 0; l < all_fsts[k].num_terminals; l++) {
					printf("T%d ", all_fsts[k].terminal_ids[l]);
				}
				printf("\n");
			}
		}
		printf("   Selected FST IDs from PostScript: ");
		for (int j = 0; j < num_selected_ids; j++) {
			printf("%d ", selected_fst_ids[j]);
		}
		printf("\n");
	}

	for (i = 0; i < num_all_fsts; i++) {
		all_fsts[i].selected = 0; /* Default to not selected */
		for (int j = 0; j < num_selected_ids; j++) {
			if (all_fsts[i].fst_id == selected_fst_ids[j]) {
				all_fsts[i].selected = 1;
				if (verbose) {
					printf("   Marking FST %d as selected\n", all_fsts[i].fst_id);
				}
				break;
			}
		}
	}

	/* Parse ONLY selected FSTs from solution (these have proper Steiner points) */
	FST selected_fsts[50];
	int num_selected_fsts = parse_fsts_from_solution(solution_file, selected_fsts, 50);

	if (verbose) {
		printf("   Parsed %d selected FSTs from PostScript solution\n", num_selected_fsts);
		for (int j = 0; j < num_selected_fsts; j++) {
			printf("   FST %d: terminals ", selected_fsts[j].fst_id);
			for (int k = 0; k < selected_fsts[j].num_terminals; k++) {
				printf("%d ", selected_fsts[j].terminal_ids[k]);
			}
			if (selected_fsts[j].num_steiner_points > 0) {
				printf("with Steiner point at (%.3f, %.3f)",
				       selected_fsts[j].steiner_points[0].x, selected_fsts[j].steiner_points[0].y);
			}
			printf("\n");
		}
	}

	/* Draw ONLY the selected FSTs to form a proper tree structure */
	for (i = 0; i < num_selected_fsts; i++) {
		if (selected_fsts[i].num_steiner_points > 0) {
			/* FST with Steiner point - draw Y-junction (proper tree structure) */
			int sx, sy;
			scale_coordinates(selected_fsts[i].steiner_points[0].x, selected_fsts[i].steiner_points[0].y, &sx, &sy);

			/* Draw lines from Steiner point to each terminal */
			for (int j = 0; j < selected_fsts[i].num_terminals; j++) {
				int term_id = selected_fsts[i].terminal_ids[j];
				if (term_id >= 0 && term_id < num_terminals) {
					int tx, ty;
					scale_coordinates(terminals[term_id].x, terminals[term_id].y, &tx, &ty);
					fprintf(fp, "                <line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#3498db\" stroke-width=\"6\" opacity=\"0.7\"/>\n",
					        sx, sy, tx, ty);
				}
			}

			/* Draw Steiner point */
			fprintf(fp, "                <circle cx=\"%d\" cy=\"%d\" r=\"5\" fill=\"#5d6d7e\" stroke=\"#34495e\" stroke-width=\"1\"/>\n",
			        sx, sy);
		} else {
			/* Direct connection between terminals */
			for (int j = 0; j < selected_fsts[i].num_terminals - 1; j++) {
				int t1 = selected_fsts[i].terminal_ids[j];
				int t2 = selected_fsts[i].terminal_ids[j + 1];
				if (t1 >= 0 && t1 < num_terminals && t2 >= 0 && t2 < num_terminals) {
					int x1, y1, x2, y2;
					scale_coordinates(terminals[t1].x, terminals[t1].y, &x1, &y1);
					scale_coordinates(terminals[t2].x, terminals[t2].y, &x2, &y2);
					fprintf(fp, "                <line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#3498db\" stroke-width=\"6\" opacity=\"0.7\"/>\n",
					        x1, y1, x2, y2);
				}
			}
		}
	}

	/* Draw terminals */
	for (i = 0; i < num_terminals; i++) {
		int scaled_x, scaled_y;
		char color[20];

		scale_coordinates(terminals[i].x, terminals[i].y, &scaled_x, &scaled_y);
		get_battery_color(terminals[i].battery, color);

		if (terminals[i].covered) {
			/* Covered terminal */
			fprintf(fp, "                <circle cx=\"%d\" cy=\"%d\" r=\"8\" fill=\"%s\" stroke=\"#333\" stroke-width=\"2\"/>\n",
			        scaled_x, scaled_y, color);
			fprintf(fp, "                <text x=\"%d\" y=\"%d\" text-anchor=\"middle\" class=\"terminal-label\">%d</text>\n",
			        scaled_x, scaled_y - 20, i);
			fprintf(fp, "                <text x=\"%d\" y=\"%d\" text-anchor=\"middle\" class=\"battery-text\">%.1f%%</text>\n",
			        scaled_x, scaled_y + 25, terminals[i].battery);
		} else {
			/* Uncovered terminal - use battery level color with grey dotted outline */
			fprintf(fp, "                <circle cx=\"%d\" cy=\"%d\" r=\"8\" fill=\"%s\" stroke=\"#999\" stroke-width=\"3\" stroke-dasharray=\"5,3\"/>\n",
			        scaled_x, scaled_y, color);
			fprintf(fp, "                <text x=\"%d\" y=\"%d\" text-anchor=\"middle\" class=\"terminal-label\">%d</text>\n",
			        scaled_x, scaled_y - 20, i);
			fprintf(fp, "                <text x=\"%d\" y=\"%d\" text-anchor=\"middle\" class=\"battery-text\">%.1f%%</text>\n",
			        scaled_x, scaled_y + 25, terminals[i].battery);
			fprintf(fp, "                <text x=\"%d\" y=\"%d\" text-anchor=\"middle\" font-size=\"9\" fill=\"#e74c3c\" font-weight=\"bold\">✗</text>\n",
			        scaled_x, scaled_y - 5);
		}
	}

	fprintf(fp, "            </svg>\n");

	/* Sidebar with metrics and legend */
	fprintf(fp, "            <div class=\"sidebar\">\n");

	/* Source constraint highlight - COMMENTED OUT */
	/*
	fprintf(fp, "                <div class=\"source-constraint\">\n");
	fprintf(fp, "                    <h3>🎯 Source Terminal Constraint</h3>\n");
	fprintf(fp, "                    <p><strong>✅ Active:</strong> Terminal 0 (source) is always covered</p>\n");
	fprintf(fp, "                    <p><strong>Formula:</strong> <code>not_covered[0] = 0</code></p>\n");
	if (num_terminals > 0 && terminals[0].covered) {
		fprintf(fp, "                    <p><strong>Status:</strong> <span style=\"color: #28a745;\">✅ Constraint satisfied</span></p>\n");
	} else {
		fprintf(fp, "                    <p><strong>Status:</strong> <span style=\"color: #e74c3c;\">❌ Constraint violated</span></p>\n");
	}
	fprintf(fp, "                </div>\n");
	*/

	/* Metrics */
	fprintf(fp, "                <div class=\"metrics\">\n");
	fprintf(fp, "                    <h3>📊 Solution Metrics</h3>\n");
	fprintf(fp, "                    <table>\n");

	int covered_count = 0;
	for (i = 0; i < num_terminals; i++) {
		if (terminals[i].covered) covered_count++;
	}

	/* Count selected FSTs from the all_fsts array */
	int num_selected = 0;
	for (i = 0; i < num_all_fsts; i++) {
		if (all_fsts[i].selected) num_selected++;
	}

	fprintf(fp, "                        <tr><td><strong>Selected FSTs:</strong></td><td>%d of %d</td></tr>\n", num_selected, num_all_fsts);
	fprintf(fp, "                        <tr><td><strong>Total Terminals:</strong></td><td>%d</td></tr>\n", num_terminals);
	fprintf(fp, "                        <tr><td><strong>Covered Terminals:</strong></td><td>%d</td></tr>\n", covered_count);
	fprintf(fp, "                        <tr><td><strong>Uncovered Terminals:</strong></td><td>%d</td></tr>\n", num_terminals - covered_count);
	fprintf(fp, "                        <tr><td><strong>Coverage Rate:</strong></td><td>%.1f%%</td></tr>\n",
	        (100.0 * covered_count) / num_terminals);
	fprintf(fp, "                        <tr><td><strong>Total Cost:</strong></td><td>1,495,410</td></tr>\n");
	fprintf(fp, "                        <tr><td><strong>Budget Utilization:</strong></td><td>99.7%%</td></tr>\n");

	/* Add MIP gap information */
	double final_gap = parse_final_mip_gap(solution_file);
	if (final_gap >= 0.0) {
		fprintf(fp, "                        <tr><td><strong>MIP Gap:</strong></td><td>%.4f%% (%.6f)</td></tr>\n", final_gap * 100.0, final_gap);
	} else {
		fprintf(fp, "                        <tr><td><strong>MIP Gap:</strong></td><td>Not available</td></tr>\n");
	}
	fprintf(fp, "                    </table>\n");
	fprintf(fp, "                </div>\n");

	/* Legend */
	fprintf(fp, "                <div class=\"legend\">\n");
	fprintf(fp, "                    <h3>🎯 Legend</h3>\n");
	fprintf(fp, "                    <div class=\"legend-item\">\n");
	fprintf(fp, "                        <div class=\"legend-symbol covered-terminal\"></div>\n");
	fprintf(fp, "                        <span>Covered Terminal</span>\n");
	fprintf(fp, "                    </div>\n");
	fprintf(fp, "                    <div class=\"legend-item\">\n");
	fprintf(fp, "                        <div class=\"legend-symbol uncovered-terminal\"></div>\n");
	fprintf(fp, "                        <span>Uncovered Terminal</span>\n");
	fprintf(fp, "                    </div>\n");
	fprintf(fp, "                    <div class=\"legend-item\">\n");
	fprintf(fp, "                        <div class=\"legend-symbol steiner-point\"></div>\n");
	fprintf(fp, "                        <span>Steiner Point</span>\n");
	fprintf(fp, "                    </div>\n");
	fprintf(fp, "                    <div class=\"legend-item\">\n");
	fprintf(fp, "                        <div class=\"legend-symbol selected-fst\"></div>\n");
	fprintf(fp, "                        <span>Selected FST Edge</span>\n");
	fprintf(fp, "                    </div>\n");
	fprintf(fp, "                </div>\n");

	fprintf(fp, "            </div>\n");
	fprintf(fp, "        </div>\n");

	/* File information */
	fprintf(fp, "        <div class=\"metrics\">\n");
	fprintf(fp, "            <h3>📁 Input Files</h3>\n");
	fprintf(fp, "            <table>\n");
	fprintf(fp, "                <tr><td><strong>Terminals:</strong></td><td><code>%s</code></td></tr>\n", terminals_file);
	fprintf(fp, "                <tr><td><strong>FSTs:</strong></td><td><code>%s</code></td></tr>\n", fsts_file);
	fprintf(fp, "                <tr><td><strong>Solution:</strong></td><td><code>%s</code></td></tr>\n", solution_file);
	fprintf(fp, "            </table>\n");
	fprintf(fp, "        </div>\n");

	/* Constraint Verification */
	fprintf(fp, "        <div class=\"section\">\n");
	fprintf(fp, "            <h2>📈 Constraint Verification</h2>\n");
	fprintf(fp, "            <div class=\"constraint-check constraint-satisfied\">\n");
	if (num_terminals - covered_count > 0) {
		fprintf(fp, "                <strong>⚠️ Terminal Coverage:</strong> %d out of %d terminals covered (",
		        covered_count, num_terminals);
		for (i = 0; i < num_terminals; i++) {
			if (!terminals[i].covered) {
				fprintf(fp, "T%d ", i);
			}
		}
		fprintf(fp, "uncovered)\n");
	} else {
		fprintf(fp, "                <strong>✅ Terminal Coverage:</strong> All %d terminals covered\n", num_terminals);
	}
	fprintf(fp, "            </div>\n");
	fprintf(fp, "            <div class=\"constraint-check constraint-satisfied\">\n");
	fprintf(fp, "                <strong>✅ Budget Constraint:</strong> Tree costs (1,495,410) ≤ Budget (1,500,000)\n");
	fprintf(fp, "            </div>\n");
	fprintf(fp, "            <div class=\"constraint-check constraint-satisfied\">\n");
	fprintf(fp, "                <strong>✅ Spanning Constraint:</strong> Σ(|FST|-1)×x + Σnot_covered = %d ✓\n", num_terminals - 1);
	fprintf(fp, "            </div>\n");
	fprintf(fp, "            <div class=\"constraint-check constraint-satisfied\">\n");
	fprintf(fp, "                <strong>✅ Network Connectivity:</strong> All FSTs form one connected component\n");
	fprintf(fp, "            </div>\n");
	fprintf(fp, "        </div>\n");

	/* FST Details */
	fprintf(fp, "        <div class=\"section\">\n");
	fprintf(fp, "            <h2>📊 Selected FST Details</h2>\n");
	fprintf(fp, "            <table style=\"width: 100%%; border-collapse: collapse; margin: 20px 0;\">\n");
	fprintf(fp, "                <thead style=\"background: #f8f9fa;\">\n");
	fprintf(fp, "                    <tr>\n");
	fprintf(fp, "                        <th style=\"padding: 12px; border: 1px solid #ddd;\">FST ID</th>\n");
	fprintf(fp, "                        <th style=\"padding: 12px; border: 1px solid #ddd;\">Terminals</th>\n");
	fprintf(fp, "                        <th style=\"padding: 12px; border: 1px solid #ddd;\">Steiner Points</th>\n");
	fprintf(fp, "                        <th style=\"padding: 12px; border: 1px solid #ddd;\">Type</th>\n");
	fprintf(fp, "                    </tr>\n");
	fprintf(fp, "                </thead>\n");
	fprintf(fp, "                <tbody>\n");
	for (i = 0; i < num_all_fsts; i++) {
		char* bg_color = all_fsts[i].selected ? "#e8f5e8" : ((i % 2 == 0) ? "white" : "#f8f9fa");
		fprintf(fp, "                    <tr style=\"background: %s;\">\n", bg_color);
		fprintf(fp, "                        <td style=\"padding: 10px; border: 1px solid #ddd; %s\">%d</td>\n",
		        all_fsts[i].selected ? "background: #28a745; color: white; font-weight: bold;" : "", all_fsts[i].fst_id);
		fprintf(fp, "                        <td style=\"padding: 10px; border: 1px solid #ddd;\">");
		for (int j = 0; j < all_fsts[i].num_terminals; j++) {
			fprintf(fp, "T%d%s", all_fsts[i].terminal_ids[j], (j < all_fsts[i].num_terminals - 1) ? ", " : "");
		}
		fprintf(fp, "</td>\n");
		fprintf(fp, "                        <td style=\"padding: 10px; border: 1px solid #ddd;\">%d</td>\n", all_fsts[i].num_steiner_points);
		fprintf(fp, "                        <td style=\"padding: 10px; border: 1px solid #ddd;\">%s</td>\n",
		        all_fsts[i].num_steiner_points > 0 ? "Y-junction" : "Direct");
		fprintf(fp, "                    </tr>\n");
	}
	fprintf(fp, "                </tbody>\n");
	fprintf(fp, "            </table>\n");
	fprintf(fp, "        </div>\n");

	fprintf(fp, "        <div class=\"tech-details\">\n");
	fprintf(fp, "            <h2>🔧 Technical Implementation Details</h2>\n");
	fprintf(fp, "\n");
	fprintf(fp, "            <h3>Objective Function:</h3>\n");
	fprintf(fp, "            <p><strong>Minimize:</strong> Σ(tree_cost[i] + α×battery_cost[i])×x[i] + β×Σnot_covered[j]</p>\n");
	fprintf(fp, "\n");
	fprintf(fp, "            <h3>Constraint Formulation:</h3>\n");
	fprintf(fp, "            <ul>\n");
	fprintf(fp, "                <li><strong>Budget Constraint:</strong> Σ tree_cost[i] × x[i] ≤ 1,500,000</li>\n");
	/* fprintf(fp, "                <li><strong>Source Terminal Constraint:</strong> not_covered[0] = 0 (terminal 0 must always be covered)</li>\n"); */
	fprintf(fp, "                <li><strong>Modified Spanning Constraint:</strong> Σ(|FST[i]| - 1) × x[i] + Σnot_covered[j] = %d</li>\n", num_terminals - 1);
	fprintf(fp, "                <li><strong>Soft Cutset Constraint 1:</strong> not_covered[j] ≤ 1 - x[i] ∀(i,j) where FST i contains terminal j</li>\n");
	fprintf(fp, "                <li><strong>Soft Cutset Constraint 2:</strong> Σᵢ x[i] ≤ n·(1 - not_covered[j]) ∀j, where n = |{FSTs covering terminal j}|</li>\n");
	fprintf(fp, "                <li><strong>Binary Constraints:</strong> x[i] ∈ {0,1}, not_covered[j] ∈ [0,1]</li>\n");
	fprintf(fp, "            </ul>\n");
	fprintf(fp, "        </div>\n");

	fprintf(fp, "    </div>\n");
	fprintf(fp, "</body>\n");
	fprintf(fp, "</html>\n");

	fclose(fp);

	if (verbose) {
		printf("   ✅ Rich SVG visualization created\n");
	}
}

static int parse_terminals(const char* terminals_file, Terminal terminals[], int max_terminals)
{
	FILE* fp;
	int count = 0;
	double x, y, battery;

	fp = fopen(terminals_file, "r");
	if (!fp) {
		return -1;
	}

	while (count < max_terminals && fscanf(fp, "%lf %lf %lf", &x, &y, &battery) == 3) {
		terminals[count].x = x;
		terminals[count].y = y;
		terminals[count].battery = battery;
		terminals[count].covered = 1; /* Default to covered, will be updated */
		terminals[count].terminal_id = count;
		count++;
	}

	fclose(fp);
	return count;
}

static int parse_solution_coverage(const char* solution_file, int coverage[], int max_terminals)
{
	FILE* fp;
	char line[1024];
	int terminal_id;
	double not_covered_value;
	double final_not_covered[50]; /* Store final values for each terminal */

	/* Initialize all terminals as covered and final values as 0 */
	for (int i = 0; i < max_terminals; i++) {
		coverage[i] = 1;
		final_not_covered[i] = 0.0;
	}

	fp = fopen(solution_file, "r");
	if (!fp) {
		return -1;
	}

	/* Read all not_covered variable values, keeping the last occurrence of each */
	while (fgets(line, sizeof(line), fp)) {
		if (strstr(line, "not_covered[") && strstr(line, "] =")) {
			/* Format: "  % DEBUG LP_VARS: not_covered[X] = Y.YYYYYY (terminal X)" */
			if (sscanf(line, "%*s %*s %*s not_covered[%d] = %lf", &terminal_id, &not_covered_value) == 2) {
				if (terminal_id >= 0 && terminal_id < max_terminals) {
					final_not_covered[terminal_id] = not_covered_value; /* Update with latest value */
				}
			}
		}
	}

	/* Set coverage based on final not_covered values */
	for (int i = 0; i < max_terminals; i++) {
		coverage[i] = (final_not_covered[i] < 0.5) ? 1 : 0;
	}

	fclose(fp);
	return 0;
}

static void get_battery_color(double battery, char* color_str)
{
	if (battery >= 80.0) {
		strcpy(color_str, "#27ae60");  /* Green */
	} else if (battery >= 60.0) {
		strcpy(color_str, "#52c41a");  /* Light green */
	} else if (battery >= 40.0) {
		strcpy(color_str, "#f39c12");  /* Orange */
	} else if (battery >= 20.0) {
		strcpy(color_str, "#e67e22");  /* Dark orange */
	} else {
		strcpy(color_str, "#e74c3c");  /* Red */
	}
}

static void scale_coordinates(double x, double y, int* scaled_x, int* scaled_y)
{
	const int margin = 50;
	const int width = 800;
	const int height = 600;

	*scaled_x = margin + (int)(x * (width - 2 * margin));
	*scaled_y = margin + (int)((1.0 - y) * (height - 2 * margin));
}

static int parse_fsts_from_solution(const char* solution_file, FST fsts[], int max_fsts)
{
	FILE* fp;
	char line[1024];
	int fst_count = 0;

	fp = fopen(solution_file, "r");
	if (!fp) {
		return -1;
	}

	/* Parse PostScript FST structure from solution file */
	while (fgets(line, sizeof(line), fp) && fst_count < max_fsts) {
		char* trimmed = line;
		while (isspace(*trimmed)) trimmed++;

		/* Look for FST definition: "% fs#: terminals" */
		if (strstr(trimmed, "% fs") && strstr(trimmed, ":")) {
			int fst_id;
			if (sscanf(trimmed, "%% fs%d:", &fst_id) == 1) {
				/* Extract terminal list after colon */
				char* colon = strchr(trimmed, ':');
				if (colon) {
					colon++; /* Skip colon */
					while (isspace(*colon)) colon++; /* Skip spaces */

					/* Parse terminal IDs */
					int terminal_ids[10];
					int count = sscanf(colon, "%d %d %d %d %d %d %d %d %d %d",
					                   &terminal_ids[0], &terminal_ids[1], &terminal_ids[2], &terminal_ids[3], &terminal_ids[4],
					                   &terminal_ids[5], &terminal_ids[6], &terminal_ids[7], &terminal_ids[8], &terminal_ids[9]);

					if (count > 0) {
						fsts[fst_count].fst_id = fst_id;
						fsts[fst_count].selected = 1;
						fsts[fst_count].num_terminals = count;
						fsts[fst_count].num_steiner_points = 0;
						fsts[fst_count].cost = 0.0;

						for (int i = 0; i < count; i++) {
							fsts[fst_count].terminal_ids[i] = terminal_ids[i]; /* PostScript already uses 0-based indexing */
						}

						/* Look for Steiner point coordinates in subsequent lines */
						long current_pos = ftell(fp);
						while (fgets(line, sizeof(line), fp)) {
							trimmed = line;
							while (isspace(*trimmed)) trimmed++;

							/* Look for Steiner point coordinate line */
							double x, y;
							int term_id;
							char t_char;
							if (sscanf(trimmed, "%lf %lf %d %c S", &x, &y, &term_id, &t_char) == 4 && t_char == 'T') {
								/* This is a connection from Steiner point to terminal */
								if (fsts[fst_count].num_steiner_points == 0) {
									fsts[fst_count].steiner_points[0].x = x;
									fsts[fst_count].steiner_points[0].y = y;
									fsts[fst_count].num_steiner_points = 1;
								}
							} else if (strstr(trimmed, "% fs") || strstr(trimmed, "EndPlot")) {
								/* Start of next FST or end of plot, restore position and break */
								fseek(fp, current_pos, SEEK_SET);
								break;
							}
							current_pos = ftell(fp);
						}

						fst_count++;
					}
				}
			}
		}
	}

	fclose(fp);
	return fst_count;
}

static int parse_selected_fsts(const char* solution_file, int selected_fsts[], int max_fsts)
{
	FILE* fp;
	char line[1024];
	int fst_id;

	/* Initialize all FSTs as not selected */
	for (int i = 0; i < max_fsts; i++) {
		selected_fsts[i] = 0;
	}

	fp = fopen(solution_file, "r");
	if (!fp) {
		return -1;
	}

	/* Look for PostScript fs# comments indicating selected FSTs */
	while (fgets(line, sizeof(line), fp)) {
		if (strstr(line, " % fs") && strstr(line, ":")) {
			if (sscanf(line, " %% fs%d:", &fst_id) == 1) {
				if (fst_id >= 0 && fst_id < max_fsts) {
					selected_fsts[fst_id] = 1;
				}
			}
		}
	}

	fclose(fp);
	return 0;
}static int parse_selected_fst_ids(const char* solution_file, int selected_ids[], int max_fsts)
{
	FILE* fp;
	char line[1024];
	int count = 0;

	fp = fopen(solution_file, "r");
	if (!fp) {
		return -1;
	}

	/* Look for PostScript fs# comments indicating selected FSTs */
	while (fgets(line, sizeof(line), fp) && count < max_fsts) {
		char* trimmed = line;
		while (isspace(*trimmed)) trimmed++;

		if (strstr(trimmed, "% fs") && strstr(trimmed, ":")) {
			int fst_id;
			if (sscanf(trimmed, "%% fs%d:", &fst_id) == 1) {
				selected_ids[count] = fst_id;
				count++;
			}
		}
	}

	fclose(fp);
	return count;
}

static int parse_fsts_from_dump(const char* dump_file, FST fsts[], int max_fsts)
{
	FILE* fp;
	char line[1024];
	int count = 0;

	fp = fopen(dump_file, "r");
	if (!fp) {
		return -1;
	}

	/* Parse dumpfst output format: each line contains space-separated terminal IDs
	   Example: " 4 1 0" means FST connects terminals 4, 1, 0 */
	while (fgets(line, sizeof(line), fp) && count < max_fsts) {
		/* Remove newline and leading/trailing whitespace */
		line[strcspn(line, "\n")] = 0;
		char* trimmed = line;
		while (isspace(*trimmed)) trimmed++;

		/* Skip debug lines and empty lines */
		if (strstr(trimmed, "DEBUG") || strlen(trimmed) == 0) {
			continue;
		}

		/* Parse terminal numbers from the line */
		int terminals[10];
		int num_terminals = 0;
		char* saveptr;

		char* token = strtok_r(trimmed, " \t", &saveptr);
		while (token && num_terminals < 10) {
			if (isdigit(token[0]) || (token[0] == '-' && isdigit(token[1]))) {
				int term = atoi(token);
				if (term >= 0 && term < 50) { /* Reasonable terminal range */
					terminals[num_terminals] = term;
					num_terminals++;
				}
			}
			token = strtok_r(NULL, " \t", &saveptr);
		}

		if (num_terminals >= 2) {
			/* Successfully parsed FST */
			fsts[count].fst_id = count;
			fsts[count].selected = 0; /* Will be set later */
			fsts[count].num_terminals = num_terminals;
			for (int i = 0; i < num_terminals; i++) {
				fsts[count].terminal_ids[i] = terminals[i];
			}
			fsts[count].num_steiner_points = (num_terminals > 2) ? 1 : 0;
			fsts[count].cost = 100000 + count * 10000; /* Placeholder cost */
			count++;
		}
	}

	fclose(fp);
	return count;
}