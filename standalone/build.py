"""Build the original detector/tracker as a local library, without ROS."""

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import urllib.request


SOURCES = {
    "opencv-4.12.0": (
        "https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.12.0",
        "44c106d5bb47efec04e531fd93008b3fcd1d27138985c5baf4eafac0e1ec9e9d"),
    "eigen-3.4.0": (
        "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz",
        "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72"),
}


def source_archive(name: str, cache: Path) -> Path:
    url, expected = SOURCES[name]
    archive = cache / f"{name}.tar.gz"
    if not archive.exists():
        print(f"Downloading {name}", flush=True)
        with urllib.request.urlopen(url, timeout=60) as response, archive.open("wb") as output:
            shutil.copyfileobj(response, output)
    actual = hashlib.sha256(archive.read_bytes()).hexdigest()
    if actual != expected:
        raise ValueError(f"{archive}: SHA256 mismatch")
    destination = cache / name
    if not destination.is_dir():
        with tarfile.open(archive) as source:
            source.extractall(cache, filter="data")
    return destination


def run(arguments: list[str]) -> None:
    subprocess.run(arguments, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path,
                        default=Path(__file__).resolve().parents[1]/".cache/native")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    work = args.work_dir.resolve()
    deps = work/"deps"
    deps.mkdir(parents=True, exist_ok=True)
    if os.name == "nt" and Path("D:/msys2/ucrt64/bin/g++.exe").exists():
        os.environ["PATH"] = "D:/msys2/ucrt64/bin;" + os.environ["PATH"]
    opencv = source_archive("opencv-4.12.0", deps)
    eigen = source_archive("eigen-3.4.0", deps)
    prefix = work/"opencv"
    if not (prefix/"OpenCVConfig.cmake").exists() and not (prefix/"lib/cmake/opencv4/OpenCVConfig.cmake").exists():
        run(["cmake", "-S", str(opencv), "-B", str(work/"opencv-build"), "-G", "Ninja",
             "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={prefix}",
             "-DBUILD_LIST=core,imgproc,calib3d,dnn", "-DBUILD_SHARED_LIBS=ON",
             "-DBUILD_TESTS=OFF", "-DBUILD_PERF_TESTS=OFF", "-DBUILD_EXAMPLES=OFF",
             "-DBUILD_opencv_apps=OFF", "-DBUILD_opencv_python3=OFF",
             "-DWITH_IPP=OFF", "-DWITH_OPENCL=OFF", "-DWITH_ITT=OFF",
             "-DWITH_FFMPEG=OFF", "-DWITH_MSMF=OFF", "-DWITH_OPENEXR=OFF",
             "-DBUILD_PROTOBUF=ON", "-DCPU_DISPATCH="])
        run(["cmake", "--build", str(work/"opencv-build"), "--target", "install",
             "--parallel", str(args.jobs)])
    run(["cmake", "-S", str(Path(__file__).resolve().parent), "-B", str(work/"build"),
         "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_PREFIX_PATH={prefix}",
         f"-DEIGEN3_INCLUDE_DIR={eigen}"])
    run(["cmake", "--build", str(work/"build"), "--parallel", str(args.jobs)])
    if os.name == "nt":
        folders = {str(p.parent) for p in prefix.rglob("*opencv_core*.dll")}
        os.environ["PATH"] = os.pathsep.join(sorted(folders)) + os.pathsep + os.environ["PATH"]
    run(["ctest", "--test-dir", str(work/"build"), "--output-on-failure"])
    print(f"Native library built in {work/'build'}", flush=True)


if __name__ == "__main__":
    main()
