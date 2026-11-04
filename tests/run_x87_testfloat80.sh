#!/usr/bin/env bash
# Berkeley TestFloat against x87_executor in the x87's own formats
# (tb_x87_testfloat80.sv): every function, rounding mode and precision
# control, judged by testfloat_ver. Functions with extF80 operands run twice:
# on the operands whose significand fits 53 bits (what FLD m64 and the
# executor's own results give) and on all of them (TestFloat's full 64-bit
# operands, which the executor takes as 53 bits plus guard/round/sticky).
#
#   ./run_x87_testfloat80.sh TESTFLOAT_BUILD_DIR TESTBENCH OUT_DIR [LEVEL] [FUNCTION...]
#
# OUT_DIR/summary.tsv: function, precision, rounding, operands, cases, errors;
# OUT_DIR/<function>_<pc>_<rc>_<set>.ver: testfloat_ver's report with the
# failing cases (the first 100 each).
set -euo pipefail

if [[ $# -lt 3 ]]; then
    echo "usage: $0 TESTFLOAT_BUILD_DIR TESTBENCH OUT_DIR [LEVEL] [FUNCTION...]" >&2
    exit 2
fi
tf=$1
tb=$2
out=$3
level=${4:-1}
shift $(( $# < 4 ? $# : 4 ))

functions=("$@")
if [[ ${#functions[@]} -eq 0 ]]; then
    functions=(extF80_add extF80_sub extF80_mul extF80_div extF80_sqrt
               extF80_roundToInt
               extF80_eq extF80_le extF80_lt extF80_eq_signaling
               extF80_le_quiet extF80_lt_quiet
               extF80_to_f32 extF80_to_f64 extF80_to_i32 extF80_to_i64
               f32_to_extF80 f64_to_extF80 i32_to_extF80 i64_to_extF80)
fi
rounds=(rnear_even rmin rmax rminMag)      # x87 RC 0..3

mkdir -p "$out"
summary=$out/summary.tsv
printf "function\tprecision\trounding\toperands\tcases\terrors\n" > "$summary"

# The low 11 significand bits of a 20-digit extF80 field are zero.
fits53='[08]00$'

run_one() {    # function pc_bits pc rc set gen_options...
    local fn=$1 bits=$2 pc=$3 rc=$4 set=$5
    shift 5
    local base=$out/${fn}_${bits}_${rounds[$rc]}_${set}
    local opts=("$@")
    case $set in
        fits53) awk -v re="$fits53" \
                    '($1 ~ re) && (NF < 4 || $2 ~ re)' "$out/gen.txt" > "$base.in" ;;
        *)      cp "$out/gen.txt" "$base.in" ;;
    esac
    local cases
    cases=$(wc -l < "$base.in" | tr -d ' ')
    if [[ $cases -eq 0 ]]; then
        rm -f "$base.in"
        return
    fi
    "$tb" "+FUNC=$fn" "+PC=$pc" "+ROUND=$rc" "+IN=$base.in" "+OUT=$base.out" > /dev/null
    "$tf/testfloat_ver" -errors 1000000000 "${opts[@]}" "$fn" < "$base.out" 2>&1 \
        | tr '\r' '\n' | grep -v '^ *[0-9 ]*$' > "$base.ver" || true
    local errors
    errors=$(sed -n 's/.*tests performed; \([0-9]*\) errors* found.*/\1/p' "$base.ver" | tail -1)
    if [[ -z $errors ]]; then
        if grep -q "no errors found" "$base.ver"; then
            errors=0
        else
            errors="?"      # testfloat_ver gave no count: see the .ver file
        fi
    fi
    # Keep the report's first failing cases and its last line.
    { head -n 201 "$base.ver"; tail -n 1 "$base.ver"; } > "$base.ver.tmp"
    mv "$base.ver.tmp" "$base.ver"
    printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$fn" "$bits" "${rounds[$rc]}" "$set" \
        "$cases" "$errors" >> "$summary"
    rm -f "$base.in" "$base.out"
    [[ $errors == 0 ]] && rm -f "$base.ver"
    return 0
}

for fn in "${functions[@]}"; do
    case $fn in
        extF80_add|extF80_sub|extF80_mul|extF80_div|extF80_sqrt)
            precisions=("32 0" "64 2" "80 3") ;;
        *)  precisions=("80 3") ;;
    esac
    extra=()
    case $fn in
        extF80_roundToInt|extF80_to_i32|extF80_to_i64) extra=(-exact) ;;
    esac
    sets=(all)
    [[ $fn == extF80_* ]] && sets=(fits53 all)
    for p in "${precisions[@]}"; do
        read -r bits pc <<< "$p"
        for rc in 0 1 2 3; do
            opts=("-${rounds[$rc]}" "${extra[@]+"${extra[@]}"}")
            [[ $fn == extF80_* ]] && opts+=("-precision$bits")
            "$tf/testfloat_gen" -seed 1 -level "$level" "${opts[@]}" "$fn" > "$out/gen.txt"
            for set in "${sets[@]}"; do
                run_one "$fn" "$bits" "$pc" "$rc" "$set" "${opts[@]}"
            done
        done
    done
done
rm -f "$out/gen.txt"
column -t -s $'\t' "$summary"
