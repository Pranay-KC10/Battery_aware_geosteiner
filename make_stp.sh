#!/bin/bash
# Usage: ./make_stp.sh N output.stp [seed]
# Example reproducible: ./make_stp.sh 70 points70.stp 42
# Example random:       ./make_stp.sh 70 points70.stp

N=$1
OUT=$2
SEED=$3

if [ -z "$N" ] || [ -z "$OUT" ]; then
  echo "Usage: $0 <num_points> <output_file> [seed]"
  exit 1
fi

# Choose rand_points command based on optional seed
if [ -z "$SEED" ]; then
  GEN="./rand_points $N"
else
  GEN="./rand_points $N $SEED"
fi

$GEN | awk -v n=$N 'BEGIN {
  print "NAME : " n "nodes";
  print "TYPE : STP";
  print "COMMENT : Randomly generated instance";
  print "DIMENSION : " n;
  print "TERMINALS : " n;
  print "EDGE_WEIGHT_TYPE : EUC_2D";
  print "NODE_COORD_SECTION";
}
{print NR, $1, $2}
END {print "EOF"}' > "$OUT"

echo "Wrote valid TSPLIB Steiner instance to $OUT"

