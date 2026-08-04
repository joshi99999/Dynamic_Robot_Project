"""
Verifikation der CUDA/PyTorch-Toolchain fuer die RTX 5070 Ti (Blackwell, sm_120)
plus einfacher Durchsatztest (ResNet18, wie im Diffusion-Policy-Backbone).

Ausfuehren auf dem Trainings-/Inferenzrechner:
    python tools/check_gpu.py

Erwartete Voraussetzungen:
    - NVIDIA-Treiber, der CUDA 12.8+ meldet
    - PyTorch cu128 oder neuer:
      pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
    - Ein separates CUDA-Toolkit (nvcc) wird NICHT benoetigt, die Wheels
      bringen die CUDA-Runtime mit.
"""

import subprocess
import sys
import time


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def check_driver():
    section("1. NVIDIA-Treiber")
    try:
        out = subprocess.run(
            ["nvidia-smi"], capture_output=True, text=True, timeout=30
        )
        if out.returncode != 0:
            print("FEHLER: nvidia-smi meldet Fehlercode", out.returncode)
            return False
        for line in out.stdout.splitlines()[:12]:
            print(line)
        print("\nHinweis: Die im Kopf angezeigte 'CUDA Version' ist die vom "
              "Treiber maximal unterstuetzte Version, nicht die installierte.")
        return True
    except FileNotFoundError:
        print("FEHLER: nvidia-smi nicht gefunden -> kein NVIDIA-Treiber im PATH.")
        return False


def check_torch():
    section("2. PyTorch-Build")
    try:
        import torch
    except ImportError:
        print("FEHLER: PyTorch nicht installiert.")
        return None

    print("torch.__version__      :", torch.__version__)
    print("torch.version.cuda     :", torch.version.cuda)
    print("cuda.is_available()    :", torch.cuda.is_available())

    if not torch.cuda.is_available():
        print("\nFEHLER: Keine CUDA-faehige GPU sichtbar.")
        return None

    print("Device                 :", torch.cuda.get_device_name(0))
    cap = torch.cuda.get_device_capability(0)
    print("Compute Capability     : sm_%d%d" % cap)

    arch_list = torch.cuda.get_arch_list()
    print("Kompilierte Architekturen:", ", ".join(arch_list))

    # Der entscheidende Test: enthaelt der Build Kernels fuer diese GPU?
    needed = "sm_%d%d" % cap
    if needed not in arch_list:
        print(
            "\nFEHLER: Dieser PyTorch-Build enthaelt keine Kernel fuer %s.\n"
            "        Symptom im Betrieb: 'no kernel image is available for\n"
            "        execution on the device'.\n"
            "        Loesung: PyTorch cu128+ installieren." % needed
        )
        return None
    print("\nOK: Build enthaelt Kernel fuer", needed)
    return torch


def smoke_test(torch):
    """is_available() kann True sein, waehrend Kernel trotzdem fehlschlagen."""
    section("3. Smoke-Test (echte GPU-Rechnung)")
    try:
        a = torch.randn(2048, 2048, device="cuda")
        b = torch.randn(2048, 2048, device="cuda")
        c = a @ b
        torch.cuda.synchronize()
        print("Matmul auf GPU erfolgreich, Ergebnis-Norm:", float(c.norm()))
        return True
    except Exception as exc:
        print("FEHLER bei GPU-Rechnung:", exc)
        return False


def benchmark(torch, batch_size=64, res=224, steps=30):
    section("4. Durchsatztest (ResNet18, Forward+Backward)")
    try:
        import torchvision
    except ImportError:
        print("torchvision nicht installiert - Benchmark uebersprungen.")
        return

    model = torchvision.models.resnet18(weights=None).cuda()
    opt = torch.optim.AdamW(model.parameters(), lr=1e-4)
    scaler = torch.amp.GradScaler("cuda")
    x = torch.randn(batch_size, 3, res, res, device="cuda")
    target = torch.randint(0, 1000, (batch_size,), device="cuda")
    loss_fn = torch.nn.CrossEntropyLoss()

    def one_step():
        opt.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda", dtype=torch.bfloat16):
            loss = loss_fn(model(x), target)
        scaler.scale(loss).backward()
        scaler.step(opt)
        scaler.update()

    for _ in range(5):  # Warmup
        one_step()
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()

    t0 = time.perf_counter()
    for _ in range(steps):
        one_step()
    torch.cuda.synchronize()
    dt = time.perf_counter() - t0

    imgs = batch_size * steps
    peak_gb = torch.cuda.max_memory_allocated() / 1024**3
    print("Batch-Groesse          : %d @ %dx%d" % (batch_size, res, res))
    print("Zeit pro Schritt       : %.1f ms" % (dt / steps * 1000))
    print("Durchsatz              : %.0f Bilder/s" % (imgs / dt))
    print("Peak-VRAM (allokiert)  : %.2f GB" % peak_gb)
    print(
        "\nHinweis: Die Diffusion Policy nutzt ein ResNet18 PRO KAMERA "
        "(hier: 2 Streams)\n         plus den Denoising-Kopf. Rechne grob mit "
        "dem 2-3fachen VRAM\n         und der 2-3fachen Schrittzeit dieses "
        "Werts."
    )


def main():
    ok = check_driver()
    torch = check_torch()
    if torch is None:
        print("\nErgebnis: Toolchain NICHT einsatzbereit.")
        sys.exit(1)
    if not smoke_test(torch):
        print("\nErgebnis: Toolchain NICHT einsatzbereit.")
        sys.exit(1)
    benchmark(torch)
    section("Ergebnis")
    print("Toolchain einsatzbereit." if ok else
          "Toolchain nutzbar, aber Treiberpruefung war auffaellig.")


if __name__ == "__main__":
    main()
