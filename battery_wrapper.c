#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>

/* Constants */
#define MAX_LINE_LENGTH 1024
#define DEFAULT_CHARGE_RATE 10.0
#define DEFAULT_DEMAND_RATE 5.0
#define MIN_BATTERY_LEVEL 0.0
#define MAX_BATTERY_LEVEL 100.0

/* Terminal structure */
typedef struct {
    double x, y;
    double battery;
    int terminal_id;
    int covered;       /* 1 if covered, 0 otherwise */
} Terminal;

/* Config structure */
typedef struct {
    char *input_file;
    char *solution_file;
    char *output_file;
    double charge_rate;
    double demand_rate;
    bool verbose;
    bool help;
} Config;

/* Function prototypes */
static void print_usage(const char *prog);
static int parse_arguments(int argc, char *argv[], Config *config);
static double clamp_battery(double val);
static int read_terminals(const char *filename, Terminal **terms, int *count);
static int check_for_selected_fsts(const char *filename);
static int parse_coverage_from_solution(const char *filename, int coverage[], int max);
static void apply_demand_only_update(Terminal *terms, int count, double demand_rate);
static void update_battery_levels(Terminal *terms, int count, double charge, double demand, bool verbose);
static int write_terminals(const char *filename, Terminal *terms, int count);
static void cleanup_terminals(Terminal *terms);

/* Print usage */
static void print_usage(const char *prog) {
    printf("Usage: %s -i <terminals> -s <solution> -o <output> [options]\n\n", prog);
    printf("Options:\n");
    printf("  -i, --input FILE     Input terminals file (x y battery)\n");
    printf("  -s, --solution FILE  Solution file from solver\n");
    printf("  -o, --output FILE    Output file (updated terminals)\n");
    printf("  -c, --charge RATE    Charge rate (default %.1f)\n", DEFAULT_CHARGE_RATE);
    printf("  -d, --demand RATE    Demand rate (default %.1f)\n", DEFAULT_DEMAND_RATE);
    printf("  -v, --verbose        Verbose logging\n");
    printf("  -h, --help           Show this help\n");
}

/* Parse arguments */
static int parse_arguments(int argc, char *argv[], Config *config) {
    memset(config, 0, sizeof(Config));
    config->charge_rate = DEFAULT_CHARGE_RATE;
    config->demand_rate = DEFAULT_DEMAND_RATE;

    static struct option long_opts[] = {
        {"input", required_argument, 0, 'i'},
        {"solution", required_argument, 0, 's'},
        {"output", required_argument, 0, 'o'},
        {"charge", required_argument, 0, 'c'},
        {"demand", required_argument, 0, 'd'},
        {"verbose", no_argument, 0, 'v'},
        {"help", no_argument, 0, 'h'},
        {0,0,0,0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "i:s:o:c:d:vh", long_opts, NULL)) != -1) {
        switch (c) {
            case 'i': config->input_file = optarg; break;
            case 's': config->solution_file = optarg; break;
            case 'o': config->output_file = optarg; break;
            case 'c': config->charge_rate = atof(optarg); break;
            case 'd': config->demand_rate = atof(optarg); break;
            case 'v': config->verbose = true; break;
            case 'h': config->help = true; return 0;
            default: return -1;
        }
    }

    if (!config->input_file || !config->solution_file || !config->output_file) {
        fprintf(stderr, "Error: -i, -s, and -o are required.\n");
        return -1;
    }

    return 0;
}

/* Clamp battery */
static double clamp_battery(double val) {
    if (val < MIN_BATTERY_LEVEL) return MIN_BATTERY_LEVEL;
    if (val > MAX_BATTERY_LEVEL) return MAX_BATTERY_LEVEL;
    return val;
}

/* Read terminals (x y battery) */
static int read_terminals(const char *filename, Terminal **terms, int *count) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "Error opening %s: %s\n", filename, strerror(errno));
        return -1;
    }
    int cap = 16;
    *terms = malloc(cap * sizeof(Terminal));
    if (!*terms) { fclose(f); return -1; }
    *count = 0;

    char line[MAX_LINE_LENGTH];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || strlen(line) < 2) continue;
        if (*count >= cap) {
            cap *= 2;
            Terminal *tmp = realloc(*terms, cap * sizeof(Terminal));
            if (!tmp) { free(*terms); fclose(f); return -1; }
            *terms = tmp;
        }
        Terminal *t = &(*terms)[*count];
        if (sscanf(line, "%lf %lf %lf", &t->x, &t->y, &t->battery) == 3) {
            t->battery = clamp_battery(t->battery);
            t->covered = 0;
            t->terminal_id = *count;
            (*count)++;
        }
    }
    fclose(f);
    return (*count > 0) ? 0 : -1;
}

/* Check for selected FSTs in solution */
static int check_for_selected_fsts(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[MAX_LINE_LENGTH];
    int selected = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "x[") && strstr(line, "] = 1.0")) selected++;
    }
    fclose(f);
    return selected;
}

/* Parse coverage (from old version) */
static int parse_coverage_from_solution(const char *filename, int coverage[], int max) {
    FILE *f = fopen(filename, "r");
    if (!f) return -1;
    char line[MAX_LINE_LENGTH];
    for (int i=0; i<max; i++) coverage[i] = 0;

    int found = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "% fs") && strstr(line, ":")) {
            char *colon = strchr(line, ':');
            if (colon) {
                char *tok = strtok(colon+1, " \t\n");
                while (tok) {
                    int id = atoi(tok);
                    if (id >= 0 && id < max) coverage[id] = 1;
                    tok = strtok(NULL, " \t\n");
                }
                found = 1;
            }
        }
    }
    fclose(f);
    return found ? 0 : -1;
}

/* Demand-only update if no FSTs */
static void apply_demand_only_update(Terminal *terms, int count, double demand_rate) {
    for (int i=0; i<count; i++) {
        if (i == 0) {
            terms[i].battery = 100.0;
            terms[i].covered = 1;
        } else {
            terms[i].battery -= demand_rate;
            terms[i].covered = 0;
        }
        terms[i].battery = clamp_battery(terms[i].battery);
    }
    printf("⚠️ No FSTs selected - demand-only update applied.\n");
}

/* Battery update */
static void update_battery_levels(Terminal *terms, int count, double charge, double demand, bool verbose) {
    printf("\n🔋 Updating batteries: charge=%.1f demand=%.1f\n", charge, demand);
    for (int i=0; i<count; i++) {
        double old = terms[i].battery;
        if (i == 0) {
            terms[i].battery = 100.0;
        } else {
            double add = terms[i].covered ? charge : 0;
            terms[i].battery = clamp_battery(old + add - demand);
        }
        if (verbose) {
            printf(" T%d: %.1f -> %.1f (covered=%d)\n",
                   i, old, terms[i].battery, terms[i].covered);
        }
    }
}

/* Write updated terminals */
static int write_terminals(const char *filename, Terminal *terms, int count) {
    FILE *f = fopen(filename, "w");
    if (!f) return -1;
    for (int i=0; i<count; i++) {
        fprintf(f, "%.6f %.6f %.2f\n", terms[i].x, terms[i].y, terms[i].battery);
    }
    fclose(f);
    return 0;
}

static void cleanup_terminals(Terminal *terms) {
    if (terms) free(terms);
}

/* Main */
int main(int argc, char *argv[]) {
    Config config;
    Terminal *terms = NULL;
    int count = 0;

    if (parse_arguments(argc, argv, &config) != 0) {
        print_usage(argv[0]);
        return 1;
    }
    if (config.help) { print_usage(argv[0]); return 0; }

    if (read_terminals(config.input_file, &terms, &count) != 0) {
        fprintf(stderr, "Error: could not read terminals.\n");
        return 1;
    }

    int selected = check_for_selected_fsts(config.solution_file);
    if (selected == 0) {
        apply_demand_only_update(terms, count, config.demand_rate);
        write_terminals(config.output_file, terms, count);
        cleanup_terminals(terms);
        return 0;
    }

    int *coverage = calloc(count, sizeof(int));
    if (!coverage) { cleanup_terminals(terms); return 1; }

    parse_coverage_from_solution(config.solution_file, coverage, count);

    for (int i=0; i<count; i++) {
        terms[i].covered = coverage[i];
    }

    free(coverage);

    update_battery_levels(terms, count, config.charge_rate, config.demand_rate, config.verbose);

    write_terminals(config.output_file, terms, count);

    cleanup_terminals(terms);
    return 0;
}