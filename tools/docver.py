#!/usr/bin/env python3
"""문서 파일명 규칙 도우미: <이름>_v<주>[-<부>]_<MMDDHH>.<확장자>

  python3 tools/docver.py init  <파일...>   # 버전 없는 파일에 v1 + 현재 월일시를 붙인다
  python3 tools/docver.py touch <파일...>   # 작은 수정: 버전은 그대로, 월일시만 새로 (팀 규칙 9, 10/4부터 기본)
  python3 tools/docver.py minor <파일...>   # (예전 방식) v4 -> v4-1, v4-1 -> v4-2
  python3 tools/docver.py major <파일...>   # v4-2 -> v5             (이전 판은 archive/문서이력/ 로 복사)
  옵션: --stamp 100316 (월일시 직접 지정), --dry-run (바꾸지 않고 보여 주기만)

이름을 바꾸면 README.md·AGENTS.md·CLAUDE.md·GEMINI.md 와 docs/·src/ 아래 md 파일 안의 링크(파일 이름)도 새 이름으로 고친다.
minor/major 는 '내용을 고친 뒤' 실행한다: 고친 파일이 새 이름이 되고, 고치기 전 내용은 미리 만들어 둔
백업이 없으면 남길 수 없으므로, 고치기 전에 `snap` 으로 이전 판을 archive 에 먼저 떠 둔다.

  python3 tools/docver.py snap  <파일...>   # 고치기 전에 현재 판을 archive/문서이력/ 에 복사
"""
import argparse
import datetime as dt
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ARCHIVE = ROOT / "archive" / "문서이력"
PAT = re.compile(r"^(?P<base>.+?)_v(?P<major>\d+)(?:-(?P<minor>\d+))?_(?P<stamp>\d{6})$")
LINK_SCOPES = [ROOT / "README.md", ROOT / "AGENTS.md", ROOT / "CLAUDE.md", ROOT / "GEMINI.md", ROOT / "docs", ROOT / "src"]


def parse(path: Path):
    m = PAT.match(path.stem)
    if not m:
        return path.stem, None, None, None
    return m["base"], int(m["major"]), (int(m["minor"]) if m["minor"] else None), m["stamp"]


def build(base, major, minor, stamp, suffix):
    ver = f"v{major}" + (f"-{minor}" if minor else "")
    return f"{base}_{ver}_{stamp}{suffix}"


def md_files():
    for scope in LINK_SCOPES:
        if scope.is_file():
            yield scope
        elif scope.is_dir():
            yield from scope.rglob("*.md")


def rewrite_links(old_name: str, new_name: str, dry: bool):
    changed = []
    for f in md_files():
        text = f.read_text(encoding="utf-8")
        if old_name in text:
            changed.append(f)
            if not dry:
                f.write_text(text.replace(old_name, new_name), encoding="utf-8")
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["init", "touch", "minor", "major", "snap"])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--stamp", default=dt.datetime.now().strftime("%m%d%H"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if not re.fullmatch(r"\d{6}", a.stamp):
        sys.exit("--stamp 은 MMDDHH 6자리")
    for f in a.files:
        p = Path(f).resolve()
        if not p.is_file():
            print(f"없음: {f}")
            continue
        base, major, minor, _ = parse(p)
        if a.action == "snap":
            dst = ARCHIVE / p.name
            print(f"[snap] {p.relative_to(ROOT)} -> {dst.relative_to(ROOT)}")
            if not a.dry_run:
                ARCHIVE.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, dst)
            continue
        if a.action == "init":
            if major is not None:
                print(f"이미 버전 있음, 건너뜀: {p.name}")
                continue
            major, minor = 1, None
        elif major is None:
            sys.exit(f"버전이 없는 파일입니다. 먼저 init: {p.name}")
        elif a.action == "touch":
            pass  # 버전 그대로, 월일시만 바꾼다
        elif a.action == "minor":
            minor = (minor or 0) + 1
        else:  # major
            major, minor = major + 1, None
        new = p.with_name(build(base, major, minor, a.stamp, p.suffix))
        if new.exists() and new != p:
            sys.exit(f"이미 있는 이름: {new.name}")
        links = rewrite_links(p.name, new.name, a.dry_run)
        print(f"[{a.action}] {p.relative_to(ROOT)} -> {new.name}" + (f"  (링크 수정: {', '.join(str(x.relative_to(ROOT)) for x in links)})" if links else ""))
        if not a.dry_run:
            p.rename(new)


if __name__ == "__main__":
    main()
