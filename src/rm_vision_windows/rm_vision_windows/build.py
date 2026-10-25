"""Build only the native algorithm library; record this checkout's runtime paths."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--opencv-dir", type=Path, default=ROOT / ".deps/native/opencv")
    parser.add_argument("--eigen-dir", type=Path, default=ROOT / ".deps/native/deps/eigen-3.4.0")
    parser.add_argument("--runtime-dir", type=Path, action="append", default=[])
    parser.add_argument("--generator", default="Ninja")
    parser.add_argument("--config", choices=("Release", "Debug"), default="Release")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    build = ROOT / "build/python-core"
    output = build / "bin"
    if not args.opencv_dir.is_dir() or not (args.eigen_dir / "Eigen/Core").is_file():
        parser.error("Provide an existing native OpenCV installation and Eigen headers; see standalone/README.md")
    subprocess.run(["cmake", "-S", str(ROOT / "standalone"), "-B", str(build), "-G", args.generator,
                    f"-DCMAKE_BUILD_TYPE={args.config}", f"-DOpenCV_DIR={args.opencv_dir.resolve()}",
                    f"-DEIGEN3_INCLUDE_DIR={args.eigen_dir.resolve()}",
                    f"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY={output}", f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={output}",
                    f"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_{args.config.upper()}={output}",
                    f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{args.config.upper()}={output}"], check=True)
    subprocess.run(["cmake", "--build", str(build), "--config", args.config, "--parallel", str(args.jobs)], check=True)
    directories = [path for path in args.opencv_dir.rglob("bin") if path.is_dir()]
    directories += args.runtime_dir
    if not args.runtime_dir:
        for compiler in ("c++", "g++", "clang++", "cl"):
            executable = shutil.which(compiler)
            if executable:
                directories.append(Path(executable).parent)
                break

    def stored(path: Path) -> str:
        path = path.resolve()
        return str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)

    library = output / ("dt46_aim.dll" if os.name == "nt" else "dt46_aim.so")
    if not library.is_file():
        raise FileNotFoundError(library)
    settings = {"library": stored(library), "dll_dirs": [stored(path) for path in directories],
                "opencv_dir": stored(args.opencv_dir), "eigen_dir": stored(args.eigen_dir),
                "generator": args.generator, "config": args.config}
    (ROOT / ".local").mkdir(exist_ok=True)
    (ROOT / ".local/native.json").write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
    print("Native core ready; local runtime configuration: .local/native.json")


if __name__ == "__main__":
    main()
