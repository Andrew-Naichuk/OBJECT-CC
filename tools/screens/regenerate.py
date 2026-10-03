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
import io
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
    "settings-saved": "32_flash_settings_saved",
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

# Animated strips of the speed matrix and gauge: (frame sequence, top row,
# bottom row).  Sequences come from frames(...) calls in emulator.cpp.
MATRIX_ROWS = (54, 150)
ANIMS = {
    "anim-boot": ("anim_boot", *MATRIX_ROWS),
    "anim-heartbeat": ("anim_heartbeat", *MATRIX_ROWS),
    "anim-new-max": ("anim_new_max", *MATRIX_ROWS),
    "anim-milestone-live": ("anim_milestone_live", *MATRIX_ROWS),
    "anim-milestone": ("anim_milestone", *MATRIX_ROWS),
    "anim-face-wake": ("anim_face_wake", *MATRIX_ROWS),
    "anim-face": ("anim_face", *MATRIX_ROWS),
    "anim-face-sleep": ("anim_face_sleep", *MATRIX_ROWS),
    "anim-drain": ("anim_drain", *MATRIX_ROWS),
    "anim-firework": ("anim_firework", *MATRIX_ROWS),
    "anim-press": ("anim_press", *MATRIX_ROWS),
    "anim-rune": ("anim_rune", *MATRIX_ROWS),
}
ANIM_HOLD_MS = 700  # the last frame lingers before the strip loops

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

    seqs = {}
    meta = RAW / "frames.tsv"
    for line in (meta.read_text().splitlines() if meta.is_file() else []):
        name, count, step, ms, caption = line.split("\t", 4)
        folder = RAW / "frames" / name
        folder.mkdir(parents=True, exist_ok=True)
        for i in range(int(count)):
            ppm = RAW / f"{name}~{i:02d}.ppm"
            Image.open(ppm).save(folder / f"{i:02d}.png")
            ppm.unlink()
        seqs[name] = {"count": int(count), "step": int(step), "ms": int(ms), "caption": caption}

    with open(RAW / "INDEX.md", "w", encoding="utf-8") as f:
        f.write("| Shot | Backlight PWM | Shows |\n| --- | --- | --- |\n")
        for name in sorted(shots):
            s = shots[name]
            f.write(f"| ![{name}]({name}.png) `{name}` | {s['duty']} | {s['caption']} |\n")
        if seqs:
            f.write("\n| Frames | Count | Step | Shows |\n| --- | --- | --- | --- |\n")
            for name in sorted(seqs):
                s = seqs[name]
                f.write(f"| ![{name}](frames/{name}/00.png) `frames/{name}/` | {s['count']} "
                        f"| {s['step']} ms | {s['caption']} |\n")
    return shots, seqs


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


def compose_anims(seqs):
    """Animated GIF bytes per README name. Pillow writes the same bytes for the
    same frames, so comparing bytes tells whether a strip changed."""
    out = {}
    for name, (seq, y0, y1) in ANIMS.items():
        s = seqs.get(seq)
        if s is None:
            sys.exit(f"ANIMS[{name!r}] needs frame sequence {seq!r}; no scenario captured it.")
        tiles = []
        for i in range(s["count"]):
            im = Image.open(RAW / "frames" / seq / f"{i:02d}.png").convert("RGB")
            tiles.append(mat(scaled(im.crop((0, y0, 240, y1))), pad=10))
        frames, key = exact_palette(tiles)
        durations = [s["step"]] * (len(frames) - 1) + [max(s["step"], ANIM_HOLD_MS)]
        buf = io.BytesIO()
        frames[0].save(buf, format="GIF", save_all=True, append_images=frames[1:],
                       duration=durations, loop=0, disposal=2, transparency=key,
                       optimize=False)
        out[name] = buf.getvalue()
    return out


def exact_palette(tiles):
    """Map RGBA tiles onto one palette holding exactly their colours, with a
    key colour for the transparent corners (the mat's alpha is 0 or 255)."""
    key_rgb = (255, 0, 255)
    rgb = []
    colors = {key_rgb}
    for t in tiles:
        im = Image.new("RGB", t.size, key_rgb)
        im.paste(t, mask=t.getchannel("A"))
        found = im.getcolors(256)
        if found is None:
            sys.exit("An animation frame has more than 256 colours; it cannot be a GIF.")
        colors.update(c for _, c in found)
        rgb.append(im)
    if len(colors) > 256:
        sys.exit("An animated strip has more than 256 colours; it cannot be a GIF.")
    pal = sorted(colors)
    flat = [v for c in pal for v in c]
    pal_im = Image.new("P", (1, 1))
    pal_im.putpalette(flat + [0] * (768 - len(flat)))
    none = getattr(Image, "Dither", Image).NONE
    return [im.quantize(palette=pal_im, dither=none) for im in rgb], pal.index(key_rgb)


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
    return {p.split("/")[-1] for p in re.findall(r"docs/screens/[A-Za-z0-9_-]+\.(?:png|gif)", text)}


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
    shots, seqs = run_scenarios(exe, args.scenario)
    log(f"\n{len(shots)} native shots and {len(seqs)} frame sequences in "
        f"{RAW.relative_to(REPO)} (see INDEX.md there)")
    if args.raw_only or args.scenario:
        return 0

    files = {f"{name}.png": im for name, im in compose(shots).items()}
    files.update({f"{name}.gif": data for name, data in compose_anims(seqs).items()})
    changed, added, same = [], [], []
    for name, content in sorted(files.items()):
        path = DOCS / name
        if isinstance(content, bytes):
            unchanged = path.is_file() and path.read_bytes() == content
        else:
            unchanged = same_pixels(path, content)
        if unchanged:
            same.append(name)
            continue
        (changed if path.exists() else added).append(name)
        if not args.check:
            DOCS.mkdir(parents=True, exist_ok=True)
            if isinstance(content, bytes):
                path.write_bytes(content)
            else:
                content.save(path, optimize=True)

    verb = "would change" if args.check else "updated"
    log(f"docs/screens: {len(same)} unchanged, {len(changed)} {verb}, {len(added)} new")
    for n in changed:
        log(f"  {verb}: {n}")
    for n in added:
        log(f"  new: {n}")
    refs = readme_refs()
    for n in sorted(refs - set(files)):
        log(f"  warning: README uses docs/screens/{n} but nothing generates it")
    for n in sorted(set(files) - refs):
        log(f"  warning: {n} is generated but the README does not use it")
    if args.check and (changed or added):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
