#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

def parse_solution(filename):
    """Parse the optimization solution output"""
    terminals = []
    selected_fsts = []

    try:
        with open(filename, 'r') as f:
            content = f.read()

        # Parse terminal locations and battery levels
        terminal_pattern = r'DEBUG P1READ: Terminal (\d+) battery=([\d.]+)'
        battery_matches = re.findall(terminal_pattern, content)

        # Parse terminal coordinates from setup section
        coord_pattern = r'\t([.\d]+)\t+([.\d]+)\t+DT'
        coord_matches = re.findall(coord_pattern, content)

        # Combine coordinates and batteries
        for i, (coord_match, battery_match) in enumerate(zip(coord_matches, battery_matches)):
            x_str, y_str = coord_match
            x = float(x_str)
            y = float(y_str)
            battery = float(battery_match[1])
            terminals.append({
                'id': i,
                'x': x,
                'y': y,
                'battery': battery
            })

        # Parse FST information
        fst_pattern = r'DEBUG OBJ: FST (\d+): tree_cost=([\d.]+), battery_cost=([\d.]+), combined=([\d.]+)'
        fst_matches = re.findall(fst_pattern, content)

        fsts = []
        for match in fst_matches:
            fst_id, tree_cost, battery_cost, combined = match
            fsts.append({
                'id': int(fst_id),
                'tree_cost': float(tree_cost),
                'battery_cost': float(battery_cost),
                'combined': float(combined)
            })

        # For now, select the FST with lowest combined cost that fits budget
        budget_pattern = r'DEBUG BUDGET: Using environment budget=([\d.]+)'
        budget_match = re.search(budget_pattern, content)
        budget = float(budget_match.group(1)) if budget_match else float('inf')

        # Select feasible FST with lowest combined cost
        feasible_fsts = [fst for fst in fsts if fst['tree_cost'] <= budget]
        if feasible_fsts:
            selected_fsts = [min(feasible_fsts, key=lambda x: x['combined'])]

        return terminals, selected_fsts, budget, fsts

    except Exception as e:
        print(f"Error parsing {filename}: {e}")
        return [], [], 0, []

def visualize_network(terminals, selected_fsts, budget, all_fsts, output_file='network_solution.pdf'):
    """Create a visualization of the network solution"""

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Plot 1: Network visualization
    ax1.set_title('Multi-Objective Steiner Network\nwith Battery-Aware Terminal Coverage', fontsize=14, fontweight='bold')

    if not terminals:
        ax1.text(0.5, 0.5, 'No terminal data available', ha='center', va='center', transform=ax1.transAxes)
        plt.savefig(output_file, bbox_inches='tight', dpi=300)
        return

    # Plot terminals with battery levels
    for terminal in terminals:
        x, y = terminal['x'], terminal['y']
        battery = terminal['battery']

        # Color coding: red for low battery, green for high battery
        color = plt.cm.RdYlGn(battery / 100.0)  # Assuming battery is 0-100 scale

        # Draw terminal
        circle = plt.Circle((x, y), 0.02, color=color, zorder=3)
        ax1.add_patch(circle)

        # Add terminal label with battery level
        ax1.annotate(f'T{terminal["id"]}\\n({battery:.1f})',
                    (x, y), xytext=(5, 5), textcoords='offset points',
                    fontsize=10, fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))

    # Draw connections for selected FSTs
    if selected_fsts:
        for fst in selected_fsts:
            # This is simplified - we'd need FST topology info to draw actual connections
            ax1.text(0.02, 0.98-len(selected_fsts)*0.05,
                    f"Selected FST {fst['id']}: Cost={fst['tree_cost']:.0f}, Battery={fst['battery_cost']:.1f}",
                    transform=ax1.transAxes, fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='lightblue', alpha=0.8))

    ax1.set_xlim(0, 1)
    ax1.set_ylim(0, 1)
    ax1.set_xlabel('X Coordinate')
    ax1.set_ylabel('Y Coordinate')
    ax1.grid(True, alpha=0.3)

    # Add legend
    legend_elements = [
        mpatches.Circle((0, 0), 0.1, facecolor='red', label='Low Battery'),
        mpatches.Circle((0, 0), 0.1, facecolor='yellow', label='Medium Battery'),
        mpatches.Circle((0, 0), 0.1, facecolor='green', label='High Battery')
    ]
    ax1.legend(handles=legend_elements, loc='upper right')

    # Plot 2: FST Analysis
    ax2.set_title('FST Cost Analysis', fontsize=14, fontweight='bold')

    if all_fsts:
        fst_ids = [f['id'] for f in all_fsts]
        tree_costs = [f['tree_cost'] for f in all_fsts]
        battery_costs = [f['battery_cost'] * 200 for f in all_fsts]  # Scale for visibility
        combined_costs = [f['combined'] for f in all_fsts]

        x_pos = np.arange(len(fst_ids))
        width = 0.25

        bars1 = ax2.bar(x_pos - width, tree_costs, width, label='Tree Cost', alpha=0.8)
        bars2 = ax2.bar(x_pos, battery_costs, width, label='Battery Cost (×200)', alpha=0.8)
        bars3 = ax2.bar(x_pos + width, combined_costs, width, label='Combined Cost', alpha=0.8)

        # Highlight budget constraint
        ax2.axhline(y=budget, color='red', linestyle='--', alpha=0.7, label=f'Budget: {budget:.0f}')

        # Mark selected FSTs
        if selected_fsts:
            for fst in selected_fsts:
                ax2.axvline(x=fst['id'], color='green', linestyle='-', alpha=0.5, linewidth=3, label='Selected')

        ax2.set_xlabel('FST ID')
        ax2.set_ylabel('Cost')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels([f'FST {i}' for i in fst_ids])
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Add cost details
        details_text = f"Budget: ${budget:.0f}\\n"
        if selected_fsts:
            for fst in selected_fsts:
                details_text += f"Selected FST {fst['id']}:\\n  Tree: ${fst['tree_cost']:.0f}\\n  Battery: {fst['battery_cost']:.1f}\\n  Total: ${fst['combined']:.0f}\\n"
        ax2.text(0.02, 0.98, details_text, transform=ax2.transAxes, fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    plt.savefig(output_file, bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {output_file}")

    # Print summary
    print("\\n=== Multi-Objective Steiner Network Solution ===")
    print(f"Terminals: {len(terminals)}")
    print(f"Budget: ${budget:.0f}")

    if terminals:
        avg_battery = sum(t['battery'] for t in terminals) / len(terminals)
        print(f"Average Battery Level: {avg_battery:.1f}")

        low_battery_terminals = [t for t in terminals if t['battery'] < 50]
        print(f"Low Battery Terminals (<50): {len(low_battery_terminals)}")

    if selected_fsts:
        print(f"\\nSelected FSTs: {len(selected_fsts)}")
        for fst in selected_fsts:
            print(f"  FST {fst['id']}: Tree Cost=${fst['tree_cost']:.0f}, Battery Impact={fst['battery_cost']:.1f}, Total=${fst['combined']:.0f}")
    else:
        print("\\nNo feasible FST found within budget constraints")

    print(f"\\nAll Available FSTs:")
    for fst in all_fsts:
        feasible = "✓" if fst['tree_cost'] <= budget else "✗"
        print(f"  {feasible} FST {fst['id']}: Tree=${fst['tree_cost']:.0f}, Battery={fst['battery_cost']:.1f}, Combined=${fst['combined']:.0f}")

if __name__ == "__main__":
    solution_file = sys.argv[1] if len(sys.argv) > 1 else 'solution_4points_fixed.txt'
    output_pdf = sys.argv[2] if len(sys.argv) > 2 else 'network_solution.pdf'

    terminals, selected_fsts, budget, all_fsts = parse_solution(solution_file)
    visualize_network(terminals, selected_fsts, budget, all_fsts, output_pdf)