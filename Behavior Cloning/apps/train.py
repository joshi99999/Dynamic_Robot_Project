"""Training der Diffusion Policy (AP 3) -- GERUEST.

Umfangsfestlegung AP 0.10: wird funktionsfaehig ausgebaut, sobald die
Abhaengigkeiten gepinnt und installiert sind (AP 0.9 Punkt 6/9):

    1. torch cu128 installieren (RTX 5070 Ti / sm_120 braucht CUDA >= 12.8,
       siehe AP 3.1) und mit tools/check_gpu.py verifizieren.
    2. LeRobot-Version pinnen, dataset.to_lerobot() implementieren.
    3. Diffusion Policy konfigurieren: ResNet18 je Kamera (ImageNet-
       vortrainiert), Action-Horizon 8-16, Bildgroesse 240x320 (Schema).

Geplanter Aufruf:
    python apps/train.py --data data_run1 --out checkpoints/run1
"""

import argparse

import _bootstrap  # noqa: F401

from bc import metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True)
    parser.add_argument("--out", default="checkpoints/run")
    args = parser.parse_args()

    # Was heute schon geht: den Datensatz pruefen, bevor Training ansteht.
    summary = metrics.summarize(args.data)
    print(metrics.format_summary(summary))
    if summary["episodes_usable"] == 0:
        raise SystemExit("Keine verwendbaren Episoden -- Training zwecklos.")

    raise SystemExit(
        "Training ist in dieser Ausbaustufe noch nicht implementiert "
        "(AP 0.10). Naechste Schritte siehe Docstring dieses Skripts."
    )


if __name__ == "__main__":
    main()
