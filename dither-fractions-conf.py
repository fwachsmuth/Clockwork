#!/usr/bin/env python3
"""
Purely Rational AVR Timer1 DitherConfig generator using fractions.

Usage:
  ./compute_dither_config_fraction.py [options] freq1 [freq2 ...]

Where freq can be:
  - "NNN/DDD" for a rational fraction, or
  - a decimal string (like "287.712287712"), which Fraction will parse
    (note that "287.712..." is still turned into an exact rational
     with denominator = 10^digits).

Examples:
  # 1) The 'true' NTSC freq: (24000/1001)*12
     ./compute_dither_config_fraction.py  "(24000/1001)*12"

  # 2) Using a fraction directly:
     ./compute_dither_config_fraction.py  24000/1001

  # 3) Something else:
     ./compute_dither_config_fraction.py 216 300 --timer-factor=4

By default, prescaler=8 => f_timer=2,000,000
"""

import sys
import math
import re
from fractions import Fraction

def parse_fraction(arg: str) -> Fraction:
    """
    Parse a string as a Fraction.
    - If it contains '/', interpret it directly as 'num/den'.
    - If it contains an expression like "(24000/1001)*12",
      we'll do a small eval with Fraction-constructs.
    - Otherwise, fallback to Fraction(arg), which can parse decimal strings
      but that may result in a large denominator if it's not a neat fraction.
    """
    # 1) If user gave a simple expression like "24000/1001"
    if re.match(r'^\(?\d+/\d+\)?$', arg):
        # parse directly as fraction
        num, den = arg.strip("()").split('/')
        return Fraction(int(num), int(den))

    # 2) If there's an expression with parentheses, '*', '/', etc.
    #    we can do a minimal safe eval approach:
    #    e.g. "(24000/1001)*12"
    #    We'll replace all numeric segments with Fraction, then eval.
    #    (In a more robust approach, you'd parse it yourself.)
    if any(x in arg for x in '*/()'):
        # Replace every "A/B" with "Fraction(A,B)"
        # Replace bare integers with "Fraction(A,1)"
        # This is simplistic, but works for typical expressions.
        expr = arg

        # Step 1: replace "NNN/DDD" with "Fraction(NNN,DDD)"
        expr = re.sub(
            r'(\d+)\s*/\s*(\d+)',
            lambda m: f'Fraction({m.group(1)},{m.group(2)})',
            expr
        )
        # Step 2: replace bare integers with "Fraction(NNN,1)"
        expr = re.sub(
            r'\b(\d+)\b',
            lambda m: f'Fraction({m.group(1)},1)',
            expr
        )
        # Now we do a safe eval with a local dict that has only "Fraction"
        local_dict = {'Fraction': Fraction}
        return eval(expr, {"__builtins__": None}, local_dict)

    # 3) If no slash/parenthesis: parse as decimal or integer
    #    e.g. "287.7123" or "25".
    return Fraction(arg)

def fraction_floor(frac: Fraction) -> int:
    """Return floor(frac) as an int, i.e. the largest integer <= frac."""
    return frac.numerator // frac.denominator

def fraction_to_int_round_half_up(frac: Fraction) -> int:
    """
    Convert a fraction in [0..some big) to the nearest integer using
    half-up rounding. (Equivalent to Python's round(x + 0.0000000001) for x>0).
    e.g. 2.4 ->2, 2.5->3, 2.4999->2
    If frac < 0, we do the negative half-up approach.
    """
    # We'll do:
    #   truncated = frac.numerator // frac.denominator
    #   remainder = frac.numerator % frac.denominator
    # Then compare remainder*2 vs. frac.denominator
    neg = (frac < 0)
    if neg:
        frac = -frac  # handle negative by symmetry

    # integer part:
    intpart = frac.numerator // frac.denominator
    remainder = frac.numerator % frac.denominator
    doubled = remainder * 2
    denom = frac.denominator

    if doubled > denom:
        intpart += 1
    elif doubled == denom:
        # half => half-up => round away from zero
        intpart += 1

    return -intpart if neg else intpart


def compute_stats(base: int, frac32: int,
                  f_timer: Fraction, freq_target: Fraction,
                  timer_factor: int):
    """
    Return a dict with 'freq_actual' (Fraction), 'ppm' (float), etc.
    or None if invalid (like base <0 or zero divisor).
    """
    if base < 0:
        return None
    # sum = base + (frac32 / 2^32)
    # but do it as Fraction
    sum_frac = Fraction(base, 1) + Fraction(frac32, 1 << 32)
    if sum_frac <= 0:
        return None

    # freq_actual = (f_timer / sum_frac) / timer_factor
    freq_actual = f_timer / sum_frac / timer_factor

    # error = freq_actual - freq_target  (still fraction)
    error = freq_actual - freq_target

    # We'll produce "ppm" as float:
    freq_target_f = float(freq_target) if freq_target != 0 else 1.0
    freq_actual_f = float(freq_actual)
    error_f = freq_actual_f - freq_target_f
    ppm_f = (error_f / freq_target_f) * 1.0e6

    return {
        'base': base,
        'frac32': frac32,
        'freq_actual': freq_actual,  # fraction
        'error_frac': error,        # fraction
        'error_float': error_f,     # float
        'ppm': ppm_f,               # float
    }


def search_best_config(freq_target: Fraction, timer_factor: int,
                       f_timer: Fraction = Fraction(2_000_000,1),
                       search_range: int = 2):
    """
    1) ideal_div = f_timer / (freq_target * timer_factor)  (Fraction)
    2) base_floor = floor(ideal_div)
    3) check base in [base_floor-search_range .. base_floor+search_range]
       if fraction= ideal_div-base in [0..1),
          frac32= round_half_up(fraction*(1<<32)), keep best in ppm.
    """
    if freq_target <= 0:
        return None

    freq_isr = freq_target * timer_factor
    ideal_div = f_timer / freq_isr   # purely fraction
    print("DEBUG ideal_div fraction =", ideal_div, " float≈", float(ideal_div))
    base_floor = fraction_floor(ideal_div)

    candidates = set()
    # We'll also consider +1 around rounding
    # for partial coverage if ideal_div is near .9999...
    # or near .0something.
    possible_centers = [
        base_floor,
        base_floor - 1,
        base_floor + 1
    ]

    for c in possible_centers:
        for b in range(c - search_range, c + search_range + 1):
            candidates.add(b)

    best_result = None
    for base in sorted(candidates):
        fraction_part = ideal_div - Fraction(base, 1)
        # Must be in [0..1)
        if fraction_part < 0 or fraction_part >= 1:
            continue

        # convert fraction_part to frac32
        # fraction_part*(1<<32)
        frac_times_2_32 = fraction_part * (1 << 32)
        frac32 = fraction_to_int_round_half_up(frac_times_2_32)

        stats = compute_stats(base, frac32, f_timer, freq_target, timer_factor)
        if stats is None:
            continue
        if best_result is None or abs(stats['ppm']) < abs(best_result['ppm']):
            best_result = stats

    return best_result


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate Timer1 DitherConfig purely in Fraction arithmetic."
    )
    parser.add_argument(
        'frequencies',
        metavar='FREQ',
        nargs='+',
        help="List of frequency specs, e.g. '287.712' or '24000/1001' or '(24000/1001)*12'"
    )
    parser.add_argument(
        '--timer-factor',
        type=int,
        default=1,
        help='Software postscaler factor (default=1).'
    )
    parser.add_argument(
        '--search-range',
        type=int,
        default=2,
        help='Integer base search ±range around floor(ideal_div).'
    )
    args = parser.parse_args()

    f_timer = Fraction(2_000_000, 1)  # prescaler=8 @16MHz => 2MHz
    for freq_str in args.frequencies:
        freq_target = parse_fraction(freq_str)  # -> Fraction

        result = search_best_config(
            freq_target=freq_target,
            timer_factor=args.timer_factor,
            f_timer=f_timer,
            search_range=args.search_range
        )

        print("-----------------------------------------------------")
        print(f"Input freq_target = {freq_str}")
        print(f" => fraction = {freq_target} ~ {float(freq_target):.12f} Hz")
        print(f" timer_factor= {args.timer_factor}")

        if not result:
            print("No valid config found in given range.")
            continue

        base = result['base']
        frac32 = result['frac32']
        freq_act_frac = result['freq_actual']    # fraction
        freq_act_float = float(freq_act_frac)
        ppm = result['ppm']

        print(f" best base   = {base}")
        print(f" best frac32 = 0x{frac32:08X}  (decimal {frac32})")

        print(f" freqActual  = {freq_act_frac} ~ {freq_act_float:.12f} Hz")
        print(f" error Hz    = {result['error_float']:.12f}")
        print(f" ppm error   = {ppm:.6f}")

        print("DitherConfig entry:")
        print(f"  {{ {base}, 0x{frac32:08X}, {args.timer_factor} }}")
        print()


if __name__ == "__main__":
    main()