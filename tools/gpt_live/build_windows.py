#!/usr/bin/env python3
"""Build a standalone Windows x64 EXE using Go and official embedded Python."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
from urllib.request import urlopen
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent
VERSION = "3.14.10"
RUNTIME_SHA256 = "90905e64a840557d8c1bc4f5b74db3d7ee85e3ffb217611cac1c7813acea4159"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--go", default=shutil.which("go"), help="Go compiler path")
    parser.add_argument("--runtime", type=Path, help="Previously downloaded Python embed ZIP")
    args = parser.parse_args()
    if not args.go:
        parser.error("Install Go or specify --go /path/to/go")
    build = ROOT / ".build/windows"
    build.mkdir(parents=True, exist_ok=True)
    runtime = args.runtime or build / f"python-{VERSION}-embed-amd64.zip"
    if not runtime.exists():
        url = f"https://www.python.org/ftp/python/{VERSION}/python-{VERSION}-embed-amd64.zip"
        print("Downloading official embedded Python...", flush=True)
        with urlopen(url, timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != RUNTIME_SHA256:
            raise ValueError("Python runtime checksum mismatch")
        runtime.write_bytes(data)
    if hashlib.sha256(runtime.read_bytes()).hexdigest() != RUNTIME_SHA256:
        raise ValueError("Python runtime checksum mismatch")
    goroot = Path(subprocess.check_output([args.go, "env", "GOROOT"], text=True).strip())
    # Explicit allowlist: never package the working tree or the user's config.
    with ZipFile(runtime) as source, ZipFile(build / "payload.zip", "w", ZIP_DEFLATED) as output:
        for item in source.infolist():
            output.writestr(item.filename, source.read(item))
        for name in ("server.py", "index.html", "app.js"):
            output.write(ROOT / name, name)
        output.write(goroot / "LICENSE", "GO-LICENSE.txt")
    for name in ("launcher.go", "launcher_test.go"):
        shutil.copyfile(ROOT / "windows" / name, build / name)
    env = {**os.environ, "GO111MODULE": "off", "CGO_ENABLED": "0"}
    subprocess.run([args.go, "test", "launcher.go", "launcher_test.go"], cwd=build, env=env, check=True)
    output = ROOT / "dist"
    output.mkdir(exist_ok=True)
    exe = output / "GPT-Live-Windows-x64.exe"
    env.update(GOOS="windows", GOARCH="amd64")
    subprocess.run([args.go, "build", "-trimpath", "-ldflags=-s -w", "-o", str(exe), "launcher.go"],
                   cwd=build, env=env, check=True)
    # Check the PE signature, rather than merely renaming a Linux executable.
    binary = exe.read_bytes()
    pe_offset = int.from_bytes(binary[60:64], "little")
    if binary[:2] != b"MZ" or binary[pe_offset:pe_offset + 6] != b"PE\0\0\x64\x86":
        raise ValueError("Output is not a Windows x64 PE executable")
    digest = hashlib.sha256(binary).hexdigest()
    (output / (exe.name + ".sha256")).write_text(f"{digest}  {exe.name}\n", encoding="ascii")
    print(f"Built {exe} ({len(binary) / 1024 / 1024:.1f} MiB)")


if __name__ == "__main__":
    main()
