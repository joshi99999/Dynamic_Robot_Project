"""Evaluation & Benchmark (AP 5) -- GERUEST.

Heute lauffaehig: Datensatz-Zusammenfassung (Episoden, Verwerf-Gruende,
Sync-Qualitaet). Die Benchmark-Szenarien (Stoergroessen, Ablationen
Wrist+Top vs. Wrist-only, mit/ohne Rauschen) folgen mit AP 5, sobald
Policies trainierbar sind.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/eval.py --data data_sim
"""

import argparse

import _bootstrap  # noqa: F401

from bc import metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True)
    args = parser.parse_args()
    print(metrics.format_summary(metrics.summarize(args.data)))


if __name__ == "__main__":
    main()
