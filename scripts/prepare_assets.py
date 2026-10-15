#!/usr/bin/env python3
"""Extract the homepage's approved media without changing research sources.

Run from any directory with Python 3 + Pillow and ffmpeg/ffprobe/pdftoppm on PATH.
The PPTX remains the authority for privacy-processed hardware footage. Gallery
clips are copied byte for byte; only the silent hero is cropped and encoded.
"""
from pathlib import Path
from zipfile import ZipFile
import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
HOME = ROOT / "homepage"
ASSETS = HOME / "assets"
CONTENT = HOME / "content"
DECK = ROOT / "demo_video/deliverables/TRACE_live_presentation_v33.pptx"
PAPER = ROOT / "build/en/main_en.pdf"
HERO_CROP = [64, 64, 1152, 648]
HERO_POSTER_SECONDS = 4

# package part, semantic id, human-readable title, category, slide, description
MEDIA = [
    ("presenter_b587bf5cd58e3a87.mp4", "motion-coordination", "Whole-body coordination", "motion", 15, "Coordinated arm, torso, and leg motion on the physical H2."),
    ("presenter_b139d78046960c61.mp4", "motion-squat", "Squat & recover", "motion", 15, "Large posture changes with a return to standing."),
    ("presenter_7c877d0ad864a6ab.mp4", "motion-kneel", "Kneel & recover", "motion", 15, "Single-knee kneeling followed by recovery."),
    ("presenter_eb019f06b53a8fa1.mp4", "motion-hop", "Single-leg hops", "motion", 15, "Dynamic hopping with single-leg support."),
    ("presenter_07ed18308acdf526.mp4", "motion-kick-left", "Left high kick", "motion", 16, "High-amplitude kicking on H2 hardware."),
    ("presenter_eeb600d9477b0b02.mp4", "motion-kick-right", "Right high kick", "motion", 16, "High-amplitude kicking on H2 hardware."),
    ("presenter_3dbf56da13aa9e13.mp4", "motion-sequence", "Whole-body sequence", "motion", 16, "An excerpt of continuous whole-body motion tracking."),
    ("presenter_0cc809545a700361.mp4", "motion-transition", "Whole-body transitions", "motion", 16, "Transitions between whole-body reference poses."),
    ("r32_p15_tl.mp4", "teleop-soccer", "Play soccer", "teleoperation", 17, "VR teleoperation coordinates approach and foot contact."),
    ("r32_p15_tr.mp4", "teleop-pillow", "Pick up a pillow", "teleoperation", 17, "Whole-body reaching and object pickup through teleoperation."),
    ("r32_p15_bl.mp4", "teleop-packing", "Pack a toy", "teleoperation", 17, "Teleoperated object placement into a bag."),
    ("r32_p15_br.mp4", "teleop-trash", "Open a trash can", "teleoperation", 17, "Foot contact and upper-body coordination through teleoperation."),
    ("presenter_5ad646acef5a5d4b.mp4", "teleop-door", "Open the door & walk through", "teleoperation", 18, "A continuous teleoperated door-opening and passage sequence."),
    ("r32_p17_tl.mp4", "vla-large-toy", "Place a large toy", "vla", 19, "An object-directed VLA demonstration with a large carrot toy."),
    ("r32_p17_tr.mp4", "vla-small-toy", "Place a small toy", "vla", 19, "An object-directed VLA demonstration with a small carrot toy."),
    ("r32_p17_bl.mp4", "vla-trash", "Open a trash can", "vla", 19, "VLA execution uses the adapted low-level motion interface."),
    ("r32_p17_br.mp4", "vla-chair", "Reposition a chair", "vla", 19, "VLA-directed whole-body object interaction."),
]


def run(*args):
    subprocess.run([str(x) for x in args], check=True)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ffmpeg(*args):
    run("ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *args)


def probe(path):
    data = json.loads(subprocess.check_output([
        "ffprobe", "-v", "error", "-show_entries",
        "stream=codec_type,codec_name,width,height,r_frame_rate,nb_frames:format=duration,size",
        "-of", "json", str(path),
    ]))
    return data


def prepare_hero():
    """Keep every frame of the privacy-reviewed door-opening and passage shot."""
    source = ASSETS / "teleop-door.mp4"
    source_info = probe(source)
    source_duration = float(source_info["format"]["duration"])
    source_frames = int(source_info["streams"][0]["nb_frames"])
    crop_x, crop_y, crop_width, crop_height = HERO_CROP
    # Remove only the baked-in label and retain the existing 16:9 framing.
    # No time trim: the final passage happens after the old 14-second cut.
    ffmpeg("-i", source,
           "-vf", f"crop={crop_width}:{crop_height}:{crop_x}:{crop_y},scale=1280:720:flags=lanczos,setsar=1",
           "-an", "-c:v", "libx264", "-preset", "medium", "-crf", 25,
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-map_metadata", -1,
           ASSETS / "hero-loop.mp4")
    ffmpeg("-ss", HERO_POSTER_SECONDS, "-i", ASSETS / "hero-loop.mp4", "-frames:v", 1,
           "-q:v", 2, ASSETS / "hero-poster.jpg")
    hero_info = probe(ASSETS / "hero-loop.mp4")
    assert int(hero_info["streams"][0]["nb_frames"]) == source_frames
    return {"src": "assets/hero-loop.mp4", "poster": "assets/hero-poster.jpg",
            "description": "The complete recorded H2 door-opening and passage sequence under VR teleoperation.",
            "source": "teleop-door", "sourceIntervalSeconds": [0, source_duration],
            "sourceIntervalFrames": [0, source_frames], "sourceSHA256": sha(source),
            "duration": float(hero_info["format"]["duration"]), "frames": source_frames,
            "crop": HERO_CROP, "speed": 1, "posterAtSeconds": HERO_POSTER_SECONDS}


def main():
    ASSETS.mkdir(exist_ok=True, parents=True)
    CONTENT.mkdir(exist_ok=True, parents=True)
    items = []
    with ZipFile(DECK) as deck:
        for package_name, item_id, title, category, slide, description in MEDIA:
            video = ASSETS / f"{item_id}.mp4"
            video.write_bytes(deck.read(f"ppt/media/{package_name}"))
            info = probe(video)
            duration = float(info["format"]["duration"])
            ffmpeg("-ss", min(2, duration * .4), "-i", video, "-frames:v", 1,
                   "-vf", "scale=640:-1", "-q:v", 3, ASSETS / f"{item_id}.jpg")
            items.append({
                "id": item_id, "title": title, "category": category,
                "description": description,
                "src": f"assets/{item_id}.mp4", "poster": f"assets/{item_id}.jpg",
                "duration": round(duration, 3), "speed": 1,
                "source": {"path": str(DECK.relative_to(ROOT)), "slide": slide,
                           "packagePart": f"ppt/media/{package_name}", "sha256": sha(video)},
                "width": info["streams"][0]["width"], "height": info["streams"][0]["height"],
            })

    hero = prepare_hero()

    # Use the updated compiled paper. The root TRACE.pdf is an earlier draft
    # that still reports six hours; it is intentionally not served by the site.
    shutil.copyfile(PAPER, ASSETS / "trace-paper.pdf")
    # Preserve the installed, versioned overview video and its source record.
    # Do not restore the historical 720p export or attach its old subtitle timeline.
    overview = json.loads((CONTENT / "media.json").read_text())["overview"]
    overview_video = HOME / overview["src"].split("?", 1)[0]
    overview_poster = HOME / overview["poster"].split("?", 1)[0]
    if not overview_video.is_file():
        raise FileNotFoundError(f"Installed project overview is missing: {overview_video}")
    ffmpeg("-ss", 2, "-i", overview_video, "-frames:v", 1, "-q:v", 2, overview_poster)

    # Render the formal, corrected Fig. 2 rather than its stale backing PNG.
    # This includes the LaTeX overlays (G1 1.32 m, command-space distribution).
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "page3"
        run("pdftoppm", "-f", 3, "-l", 3, "-scale-to", 2400,
            "-png", "-singlefile", PAPER, page)
        Image.open(page.with_suffix(".png")).crop((215, 165, 1647, 900)).save(
            ASSETS / "method-overview.webp", quality=92, method=6)
    for source, name, width in [
        ("imgs/Cover.jpg", "paper-teaser", 1800),
        ("04_Experiments/imgs/training_curves_en.png", "training-curves", 1600),
        ("04_Experiments/imgs/data_distribution_balancing.png", "data-curation", 1600),
        ("04_Experiments/imgs/motion_tracking_comparison_4col_no_dance.jpg", "tracking-comparison", 1800),
        ("04_Experiments/imgs/locomanipulation_realexp.jpg", "teleoperation-sequence", 2000),
        ("03_Method/imgs/sonic-h2-lora-details.png", "lora-details", 1600),
    ]:
        image = Image.open(ROOT / source).convert("RGB")
        if image.width > width:
            image.resize((width, round(image.height * width / image.width)), Image.Resampling.LANCZOS).save(
                ASSETS / f"{name}.webp", quality=90, method=6)
        else:
            image.save(ASSETS / f"{name}.webp", quality=90, method=6)

    manifest = {
        "version": 1,
        "hero": hero,
        "overview": overview,
        "items": items,
        "provenance": {
            "deck": str(DECK.relative_to(ROOT)), "deckSha256": sha(DECK),
            "privacy": "Privacy processing is inherited from the reviewed v33 embedded media. Non-VR faces are covered; VR wearers are exempt under the existing project policy.",
            "galleryTransformation": "Bit-exact extraction of embedded MP4s. Posters are sampled at two seconds or 40% of a shorter clip.",
            "speed": "All 17 gallery videos and the hero retain 1x playback.",
            "benchmarkBoundary": "Gallery demonstrations are qualitative examples and do not define the paper's VLA evaluation sample or success rates.",
            "paper": {"source": str(PAPER.relative_to(ROOT)), "sha256": sha(PAPER),
                      "reason": "The current compiled English paper reports approximately eight hours; the root TRACE.pdf is an older six-hour draft."},
        },
    }
    (CONTENT / "media.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Prepared {len(items)} gallery videos, the complete {hero['duration']:.3f}-second hero, paper figures, and source records.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hero-only", action="store_true", help="Rebuild the full hero from the already extracted door video, leaving other media unchanged.")
    args = parser.parse_args()
    if args.hero_only:
        manifest_path = CONTENT / "media.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["hero"] = prepare_hero()
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"Prepared complete {manifest['hero']['duration']:.3f}-second hero.")
    else:
        main()
