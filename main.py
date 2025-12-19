import sys
import time

def fake_progress(target=65, width=40, seconds=2.5):
    steps = target
    delay = max(seconds / max(steps, 1), 0.01)

    for p in range(steps + 1):
        filled = int(width * p / 100)
        bar = "█" * filled + " " * (width - filled)
        sys.stdout.write(f"\r[{bar}] {p:3d}%")
        sys.stdout.flush()
        time.sleep(delay)

    sys.stdout.write("\n(Analisi immagine input.png.)\n")
    sys.stdout.flush()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nAnnullato.")

if __name__ == "__main__":
    fake_progress()