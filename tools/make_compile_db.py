# 全スケッチのコンパイルデータベースを作り、.vscode/compile_commands.json にまとめる。
# clangd(.clangd の CompilationDatabase: .vscode)がスケッチのヘッダを解決するために使う。
# 使い方(リポジトリ直下で): python tools/make_compile_db.py
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sketch_dirs():
    for name in sorted(os.listdir(ROOT)):
        path = os.path.join(ROOT, name)
        if name.startswith("srdc_") and os.path.isfile(os.path.join(path, name + ".ino")):
            yield name, path


def build_db(name, path):
    build = os.path.join(tempfile.gettempdir(), "srdc_compile_db", name)
    cmd = ["arduino-cli", "compile", "--profile", "esp32", "--only-compilation-database",
           "--build-path", build, path]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(build, "compile_commands.json"), encoding="utf-8") as f:
        entries = json.load(f)

    sketch_build = os.path.normcase(os.path.join(build, "sketch"))
    for e in entries:
        src = os.path.normcase(os.path.normpath(e["file"]))
        # ビルド先にコピーされたスケッチのファイルを、元のスケッチのファイルに対応づける
        if src.startswith(sketch_build + os.sep):
            rel = os.path.relpath(e["file"], os.path.join(build, "sketch"))
            if rel.endswith(".ino.cpp"):
                rel = rel[: -len(".cpp")]
                # .ino はビルド時に Arduino.h の include が足されるので、同じものを読ませる
                e["arguments"][1:1] = ["-include", "Arduino.h"]
            e["file"] = os.path.join(path, rel)
            e["directory"] = path
    return entries


def main():
    db = []
    for name, path in sketch_dirs():
        print("compile db:", name)
        db.extend(build_db(name, path))
    out_dir = os.path.join(ROOT, ".vscode")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "compile_commands.json"), "w", encoding="utf-8") as f:
        json.dump(db, f, indent=2, ensure_ascii=False)
    print("wrote %d entries" % len(db))
    return 0


if __name__ == "__main__":
    sys.exit(main())
