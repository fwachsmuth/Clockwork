#!/usr/bin/env python3
"""
Compute AVR Timer1 "DitherConfig" for a given target frequency
using prescaler=8 => Timer clock = 2 MHz.

Additionally, it does a local integer search around the
floor(idealDiv) to find the absolutely smallest ppm error.

Usage:
  compute_dither_config.py [--timer-factor=TF] [--search-range=SR] freq1 [freq2 ...]

Example:
  ./compute_dither_config.py 287.712287712
  ./compute_dither_config.py --timer-factor=4 216 108
  ./compute_dither_config.py --search-range=5 287.712287712
"""

import math
import sys

def compute_dither_config_exact(freq_target, timer_factor=1, f_timer=2_000_000):
    """
    Direct formula approach (no search):
      - idealDiv = f_timer / (freq_target * timer_factor)
      - base = floor(idealDiv)
      - frac = idealDiv - base
      - frac32 = round(frac * 2^32)
    Returns a dict with 'base', 'frac32', 'freq_actual', 'ppm', etc.
    """
    freq_isr = freq_target * timer_factor
    ideal_div = f_timer / freq_isr

    base = int(math.floor(ideal_div))
    frac = ideal_div - base
    frac32 = int(round(frac * (2**32)))

    result = calc_stats(base, frac32, timer_factor, freq_target, f_timer)
    return result

def calc_stats(base, frac32, timer_factor, freq_target, f_timer):
    """
    Given base, frac32, compute:
      - avgDiv = base + frac32/(2^32)
      - freq_actual = f_timer / avgDiv / timer_factor
      - error, ppm
    Return a dict of stats.
    """
    if base < 0:
        # invalid
        return None

    avg_div = base + (frac32 / (2**32))
    if avg_div <= 0:
        return None

    freq_actual = f_timer / avg_div / timer_factor
    error_hz = freq_actual - freq_target
    ppm = (error_hz / freq_target) * 1e6

    return {
        'base': base,
        'frac32': frac32,
        'avg_div': avg_div,
        'freq_actual': freq_actual,
        'error_hz': error_hz,
        'ppm': ppm,
        'timer_factor': timer_factor,
        'freq_target': freq_target
    }

def compute_dither_config_with_search(
    freq_target, 
    timer_factor=1, 
    f_timer=2_000_000, 
    search_range=2
):
    """
    1) Compute the 'exact' approach: idealDiv = f_timer/(freq_target*timer_factor).
    2) Let base_floor = floor(idealDiv).
    3) Search base in [base_floor - search_range ... base_floor + search_range].
       - fraction = (idealDiv - base) if that is >= 0
       - frac32 = round(fraction * 2^32)
       - Compute actual freq, check error
       - Keep the best in terms of absolute ppm error

    Return the best result dict.
    """
    freq_isr = freq_target * timer_factor
    ideal_div = f_timer / freq_isr

    base_floor = int(math.floor(ideal_div))

    best = None
    for candidate_base in range(base_floor - search_range, base_floor + search_range + 1):
        if candidate_base < 0:
            continue

        # fraction might be negative if candidate_base > ideal_div, so we can compute it.
        fraction = ideal_div - candidate_base
        # fraction in principle can be negative, but let's handle it by best approach:
        # We'll define frac = clamp( fraction, 0, 1 ), or skip negative fraction if we want strictly base+frac in [0..1).
        if fraction < 0:
            # We could theoretically dither between candidate_base-1 and candidate_base if fraction ~ 1.9,
            # but let's keep it simpler. If fraction < 0, skip.
            continue
        if fraction >= 1.0:
            # That means candidate_base is smaller than ideal_div-1. 
            # Possibly we skip it as well, or we handle it carefully.
            # We'll skip if fraction >=1 => 
            continue

        frac32 = int(round(fraction * (2**32)))
        stats = calc_stats(candidate_base, frac32, timer_factor, freq_target, f_timer)
        if stats is None:
            continue
        # Check error
        if (best is None) or (abs(stats['ppm']) < abs(best['ppm'])):
            best = stats

    return best


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Compute AVR Timer1 dither config for given frequency(ies) with local search for best ppm."
    )
    parser.add_argument(
        'frequencies',
        metavar='FREQ',
        nargs='+',
        help='Desired frequency in Hz (floating).'
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
        help='Search ±N around floor(idealDiv) to find best ppm (default=2).'
    )
    args = parser.parse_args()

    for freq_str in args.frequencies:
        freq = float(freq_str)
        best_result = compute_dither_config_with_search(
            freq,
            timer_factor=args.timer_factor,
            search_range=args.search_range
        )
        if not best_result:
            print("-"*60)
            print(f"Could not find a valid config for freq={freq}")
            continue

        # Format the output
        base = best_result['base']
        frac32 = best_result['frac32']
        frac32_hex = f"0x{frac32:08X}"
        freq_actual = best_result['freq_actual']
        ppm = best_result['ppm']
        error_hz = best_result['error_hz']

        # Print summary
        print("-"*60)
        print(f"Target freq:   {freq} Hz  (timerFactor={args.timer_factor})")
        print(f"Best base:     {base}")
        print(f"Best frac32:   {frac32_hex}  (decimal {frac32})")
        print(f"Actual freq:   {freq_actual:.12f} Hz")
        print(f"Error:         {error_hz:.6e} Hz")
        print(f"PPM:           {ppm:.6f}")

        # Snippet for the DitherConfig array
        print("DitherConfig entry:")
        print(f"  {{ {base}, {frac32_hex}, {args.timer_factor} }},")
        print()


if __name__ == "__main__":
    main()