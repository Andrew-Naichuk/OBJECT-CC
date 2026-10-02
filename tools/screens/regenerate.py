#!/usr/bin/env python3
"""Regenerate the README screenshots in docs/screens from the firmware.

Builds tools/screens/emulator.cpp together with the unmodified
xiao_oled/xiao_oled.ino and the Adafruit GFX library, runs every scenario,
and turns the captured framebuffers into the images the README uses.

    python tools/screens/regenerate.py            # rebuild everything
    python tools/screens/regenerate.py --check    # exit 1 if docs/screens is stale
    python tools/screens/regenerate.py --raw-only # native 240x320 shots only

Needs Python 3.8+, Pillow (pip install pillow), a C++17 compiler (g++ or
clang++) and the Adafruit GFX library (the copy Arduino installed, or it is
cloned automatically).  See tools/screens/README.md.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit("Pillow is missing. Install it with:  python -m pip install pillow")

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SKETCH_DIR = REPO / "xiao_oled"
DOCS = REPO / "docs" / "screens"
BUILD = HERE / "build"
RAW = BUILD / "raw"
GFX_TAG = "1.12.6"  # Adafruit GFX version the screens were made with
GFX_URL = "https://github.com/adafruit/Adafruit-GFX-Library.git"
EXE = ".exe" if os.name == "nt" else ""

# ---------------------------------------------------------------------------
# What ends up in docs/screens.  Keys are README file names (without .png),
# values are emulator shot names (see shot(...) calls in emulator.cpp).

SCREENS = {  # full screen on a dark rounded mat
    "boot-checking-card": "01_boot_checking_card",
    "boot-card-ready": "03_boot_card_ready",
    "boot-resuming-ride": "04_boot_resuming_ride",
    "boot-no-card": "02_boot_no_card",
    "ride-stopped": "12_ride_stopped",
    "backlight-bright": "12_ride_stopped",
    "hold-2s": "20_hold_2s",
    "hold-1s": "21_hold_1s",
    "ride-saved": "22_flash_ride_saved",
    "new-ride": "23_new_ride_ready",
    "stats-reset": "26_flash_stats_reset",
    "save-failed": "24_flash_save_failed",
    "phone-allow-10s": "28_phone_allow_10s",
    "phone-allow-4s": "29_phone_allow_4s",
    "phone-connected": "30_phone_connected",
    "phone-sending": "31_phone_sending",
    "units-metric": "10_ride_recording",
    "units-imperial": "14_ride_imperial",
}

AS_SEEN = {  # framebuffer darkened by the backlight PWM duty of that shot
    "backlight-dim": "18_backlight_dim",
    "backlight-off": "19_backlight_off",
}

BANDS = {  # horizontal strip of a screen: (shot, top row, bottom row)
    "status-searching": ("05_idle_gps_searching", 0, 30),
    "status-satellites": ("07_idle_gps_fix", 0, 30),
    "status-gps": ("08_idle_gps_no_sat_count", 0, 30),
    "footer-recording": ("12_ride_stopped", 292, 320),
    "footer-phone": ("30_phone_connected", 292, 320),
    "footer-no-card": ("09_idle_no_card", 292, 320),
    "footer-idle": ("07_idle_gps_fix", 292, 320),
}

HERO = ["10_ride_recording", "20_hold_2s", "31_phone_sending"]

# Anatomy callouts: (number, top row, bottom row) in panel pixels, taken from
# the layout constants in xiao_oled.ino (STATUS slot, CAPTION_Y..HERO_Y+70,
# GAUGE_Y, ROWS_Y..+4*ROW_H, FOOTER_Y..+FOOTER_H).  Update if the layout moves.
ANATOMY_SHOT = "10_ride_recording"
ANATOMY = [(1, 4, 26), (2, 34, 130), (3, 138, 146), (4, 156, 292), (5, 296, 318)]

# Look of the images.
SCALE = 2          # nearest-neighbour; crisp on retina when shown at 240 px
PAD = 14
RADIUS = 18
MAT = (24, 25, 26, 255)
EDGE = (58, 60, 62, 255)
INK = (236, 238, 236, 255)
DIM = (143, 143, 143, 255)


# ---------------------------------------------------------------------------
# Build and run

def log(msg):
    print(msg, flush=True)


def find_gfx(explicit):
    """Adafruit GFX: --gfx, $OBJECT_GFX_DIR, Arduino's library folder, or a clone."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("OBJECT_GFX_DIR"):
        candidates.append(Path(os.environ["OBJECT_GFX_DIR"]))
    cli = shutil.which("arduino-cli")
    if cli:
        try:
            user = subprocess.run([cli, "config", "get", "directories.user"],
                                  capture_output=True, text=True, timeout=20).stdout.strip()
            if user:
                candidates.append(Path(user) / "libraries" / "Adafruit_GFX_Library")
        except (OSError, subprocess.SubprocessError):
            pass
    home = Path.home()
    for docs in [home / "Documents", Path(os.environ.get("OneDrive", home)) / "Documents", home]:
        candidates.append(docs / "Arduino" / "libraries" / "Adafruit_GFX_Library")
    for c in candidates:
        if (c / "Adafruit_GFX.cpp").is_file():
            return c, False
    clone = BUILD / f"Adafruit-GFX-Library-{GFX_TAG}"
    if not (clone / "Adafruit_GFX.cpp").is_file():
        if not shutil.which("git"):
            sys.exit("Adafruit GFX not found and git is missing. Install the library "
                     "from the Arduino Library Manager or pass --gfx PATH.")
        log(f"Cloning Adafruit GFX {GFX_TAG} into {clone.relative_to(REPO)}")
        BUILD.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "-q", "--depth", "1", "--branch", GFX_TAG,
                        GFX_URL, str(clone)], check=True)
    return clone, True


def library_version(gfx):
    props = gfx / "library.properties"
    if props.is_file():
        for line in props.read_text(errors="ignore").splitlines():
            if line.startswith("version="):
                return line.split("=", 1)[1].strip()
    return "unknown"


def find_cxx(explicit):
    for c in [explicit, os.environ.get("CXX"), "c++", "g++", "clang++"]:
        if c and shutil.which(c):
            return shutil.which(c)
    sys.exit("No C++ compiler found. Install one and retry, or pass --cxx PATH:\n"
             "  Windows: MSYS2 (pacman -S mingw-w64-ucrt-x86_64-gcc) or LLVM\n"
             "  macOS:   xcode-select --install\n"
             "  Linux:   your distro's g++ package")


def build(cxx, gfx):
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / f"emulator{EXE}"
    cmd = [cxx, "-std=c++17", "-O1", "-w", "-DARDUINO=10819",
           f"-I{HERE / 'stubs'}", f"-I{gfx}", f"-I{SKETCH_DIR}",
           "-o", str(exe), str(HERE / "emulator.cpp"), str(gfx / "Adafruit_GFX.cpp")]
    log(f"Compiling emulator with {Path(cxx).name}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        sys.exit("Emulator build failed. If the firmware now calls a new Arduino, SdFat or "
                 "Bluefruit function, add it to tools/screens/stubs/.")
    return exe


def run_scenarios(exe, only):
    names = subprocess.run([str(exe), "--list"], capture_output=True, text=True,
                           check=True).stdout.split()
    if only:
        unknown = set(only) - set(names)
        if unknown:
            sys.exit(f"Unknown scenario(s): {', '.join(sorted(unknown))}. "
                     f"Known: {', '.join(names)}")
        names = [n for n in names if n in only]
    if RAW.exists():
        shutil.rmtree(RAW)
    RAW.mkdir(parents=True)
    env = dict(os.environ)
    for n in names:
        log(f"[{n}]")
        r = subprocess.run([str(exe), str(RAW), n], env=env, stderr=subprocess.PIPE, text=True)
        sys.stdout.write(r.stderr)
        if r.returncode != 0:
            sys.exit(f"Scenario {n} failed with exit code {r.returncode}.")

    shots = {}
    for line in (RAW / "shots.tsv").read_text().splitlines():
        name, duty, ms, caption = line.split("\t", 3)
        ppm = RAW / f"{name}.ppm"
        Image.open(ppm).save(RAW / f"{name}.png")
        ppm.unlink()
        shots[name] = {"duty": int(duty), "ms": int(ms), "caption": caption}
    with open(RAW / "INDEX.md", "w", encoding="utf-8") as f:
        f.write("| Shot | Backlight PWM | Shows |\n| --- | --- | --- |\n")
        for name in sorted(shots):
            s = shots[name]
            f.write(f"| ![{name}]({name}.png) `{name}` | {s['duty']} | {s['caption']} |\n")
    return shots


# ---------------------------------------------------------------------------
# Compose

def native(shot):
    return Image.open(RAW / f"{shot}.png").convert("RGB")


def scaled(im):
    return im.resize((im.width * SCALE, im.height * SCALE), Image.NEAREST)


def mat(im, pad=PAD, extra_right=0):
    w, h = im.width + 2 * pad + extra_right, im.height + 2 * pad
    out = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ImageDraw.Draw(out).rounded_rectangle((0, 0, w - 1, h - 1), RADIUS,
                                          fill=MAT, outline=EDGE, width=2)
    out.paste(im, (pad, pad))
    return out


def number_font(size=22):
    for p in ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "C:/Windows/Fonts/arialbd.ttf",
              "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
              "/Library/Fonts/Arial Bold.ttf",
              "DejaVuSans-Bold.ttf", "arialbd.ttf"]:
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def compose(shots):
    out = {}
    for name, shot in SCREENS.items():
        out[name] = mat(scaled(native(shot)))
    for name, shot in AS_SEEN.items():
        k = shots[shot]["duty"] / 255.0
        out[name] = mat(scaled(native(shot).point(lambda v: int(v * k + 0.5))))
    for name, (shot, y0, y1) in BANDS.items():
        out[name] = mat(scaled(native(shot).crop((0, y0, 240, y1))), pad=10)

    tiles = [mat(scaled(native(s))) for s in HERO]
    gap = 28
    hero = Image.new("RGBA", (sum(t.width for t in tiles) + gap * (len(tiles) - 1),
                              tiles[0].height), (0, 0, 0, 0))
    x = 0
    for t in tiles:
        hero.alpha_composite(t, (x, 0))
        x += t.width + gap
    out["hero"] = hero

    base = scaled(native(ANATOMY_SHOT))
    im = mat(base, extra_right=96)
    d = ImageDraw.Draw(im)
    font = number_font()
    x0 = PAD + base.width + 14
    for n, y0, y1 in ANATOMY:
        a, b = PAD + y0 * SCALE, PAD + y1 * SCALE
        d.line((x0, a, x0 + 10, a), fill=DIM, width=2)
        d.line((x0, b, x0 + 10, b), fill=DIM, width=2)
        d.line((x0 + 10, a, x0 + 10, b), fill=DIM, width=2)
        cy = (a + b) // 2
        d.line((x0 + 10, cy, x0 + 34, cy), fill=DIM, width=2)
        cx, r = x0 + 56, 20
        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=INK)
        d.text((cx, cy + 1), str(n), font=font, fill=(16, 16, 16, 255), anchor="mm")
    out["anatomy"] = im
    return out


def same_pixels(path, im):
    if not path.is_file():
        return False
    old = Image.open(path).convert("RGBA")
    return old.size == im.size and old.tobytes() == im.convert("RGBA").tobytes()


def readme_refs():
    readme = REPO / "README.md"
    if not readme.is_file():
        return set()
    text = readme.read_text(encoding="utf-8")
    return {p.split("/")[-1][:-4] for p in re.findall(r"docs/screens/[A-Za-z0-9_-]+\.png", text)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--check", action="store_true",
                    help="do not write; exit 1 if any docs/screens image would change")
    ap.add_argument("--raw-only", action="store_true",
                    help="only write native shots to tools/screens/build/raw")
    ap.add_argument("--scenario", action="append",
                    help="run only this scenario (repeatable); implies --raw-only")
    ap.add_argument("--gfx", help="path to the Adafruit_GFX_Library folder")
    ap.add_argument("--cxx", help="C++ compiler to use")
    args = ap.parse_args()

    gfx, cloned = find_gfx(args.gfx)
    version = library_version(gfx)
    log(f"Adafruit GFX {version} from {gfx}")
    if version not in (GFX_TAG, "unknown"):
        log(f"  note: the committed screens were made with {GFX_TAG}; text may shift by a pixel")
    exe = build(find_cxx(args.cxx), gfx)
    shots = run_scenarios(exe, args.scenario)
    log(f"\n{len(shots)} native shots in {RAW.relative_to(REPO)} (see INDEX.md there)")
    if args.raw_only or args.scenario:
        return 0

    images = compose(shots)
    changed, added, same = [], [], []
    for name, im in sorted(images.items()):
        path = DOCS / f"{name}.png"
        if same_pixels(path, im):
            same.append(name)
            continue
        (changed if path.exists() else added).append(name)
        if not args.check:
            DOCS.mkdir(parents=True, exist_ok=True)
            im.save(path, optimize=True)

    verb = "would change" if args.check else "updated"
    log(f"docs/screens: {len(same)} unchanged, {len(changed)} {verb}, {len(added)} new")
    for n in changed:
        log(f"  {verb}: {n}.png")
    for n in added:
        log(f"  new: {n}.png")
    refs = readme_refs()
    for n in sorted(refs - set(images)):
        log(f"  warning: README uses docs/screens/{n}.png but nothing generates it")
    for n in sorted(set(images) - refs):
        log(f"  warning: {n}.png is generated but the README does not use it")
    if args.check and (changed or added):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
