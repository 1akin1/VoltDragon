"""Structural coverage with MC/DC for the host-tested firmware modules.

Builds the host unit tests with clang's source-based coverage and MC/DC
instrumentation (the "coverage" CMake preset), runs them, and writes
docs/mcdc-report.md. The module under the DO-178C level A objective, Node A's
safety logic (safety.c), must reach 100 % statement, decision (branch) and MC/DC
coverage; the other modules are listed for information.

    python3 tools/coverage/mcdc.py            # regenerate docs/mcdc-report.md
    python3 tools/coverage/mcdc.py --check    # CI: fail below 100 % or if the report is stale

Needs clang-19, llvm-19 and libclang-rt-19-dev (Debian bookworm), CMake and Ninja.
"""

from __future__ import annotations

import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build/coverage"
PROFILES = BUILD / "profiles"
OUT_DOC = ROOT / "docs/mcdc-report.md"
MODULE = "firmware/node_a/src/safety.c"
TOOL_SUFFIX = "-19"


def run(*args: str, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=ROOT, check=True, text=True, **kwargs)


def run_quietly(*args: str, **kwargs) -> None:
    """Runs a build step, showing its output only if it fails."""
    try:
        run(*args, capture_output=True, **kwargs)
    except subprocess.CalledProcessError as failure:
        sys.stderr.write(failure.stdout + failure.stderr)
        raise SystemExit(f"error: {' '.join(args)} failed") from None


def build_and_test() -> list[Path]:
    """Configures, builds and runs the instrumented tests; returns the test executables."""
    run_quietly("cmake", "--preset", "coverage")
    run_quietly("cmake", "--build", "--preset", "coverage")
    shutil.rmtree(PROFILES, ignore_errors=True)
    PROFILES.mkdir(parents=True)
    env = dict(os.environ, LLVM_PROFILE_FILE=str(PROFILES / "%p.profraw"))
    run_quietly("ctest", "--test-dir", str(BUILD), "--output-on-failure", env=env)
    tests = BUILD / "tests/unit"
    return sorted(p for p in tests.glob("test_*") if p.is_file() and os.access(p, os.X_OK))


def llvm(tool: str, *args: str) -> str:
    return run(f"llvm-{tool}{TOOL_SUFFIX}", *args, capture_output=True).stdout


def merge_profiles() -> Path:
    merged = BUILD / "tests.profdata"
    llvm("profdata", "merge", "-sparse", *map(str, sorted(PROFILES.glob("*.profraw"))),
         "-o", str(merged))
    return merged


def object_args(executables: list[Path]) -> list[str]:
    args = [str(executables[0])]
    for exe in executables[1:]:
        args += ["-object", str(exe)]
    return args


def rel(path: str) -> str:
    return Path(path).resolve().relative_to(ROOT).as_posix()


def percent(summary: dict) -> str:
    if summary["count"] == 0:
        return "-"
    return f"{summary['covered']}/{summary['count']} ({summary['percent']:.1f} %)"


# ---------------------------------------------------------------------------- MC/DC detail

def condition_text(source: list[str], line: int, col: int) -> str:
    """The source of one condition: from its start to the end of the operand at its depth
    (the next && or ||, or the end of the enclosing parenthesis or statement)."""
    text = "\n".join(source[line - 1:])[col - 1:]
    depth, out, i = 0, [], 0
    while i < len(text):
        two = text[i:i + 2]
        c = text[i]
        if depth == 0 and (two in ("&&", "||") or c in ";,?"):
            break
        if c == "(":
            depth += 1
        elif c == ")":
            if depth == 0:
                break
            depth -= 1
        out.append(c)
        i += 1
    return " ".join("".join(out).split())


def decisions(show: str, source: list[str]) -> list[dict]:
    """Parses the MC/DC blocks of `llvm-cov show --show-mcdc`."""
    found = []
    for block in show.split("MC/DC Decision Region ")[1:]:
        start = re.match(r"\((\d+):(\d+)\) to \((\d+):(\d+)\)", block)
        first, last = int(start.group(1)), int(start.group(3))
        conditions = [
            (name, condition_text(source, int(line), int(col)))
            for name, line, col in re.findall(r"Condition (C\d+) --> \((\d+):(\d+)\)", block)
        ]
        vectors = [
            ([v.strip() for v in values.split(",")], result)
            for values, result in re.findall(r"\d+ \{ ([TF, -]+?)\s*=\s*([TF])\s*\}", block)
        ]
        pairs = dict(re.findall(r"(C\d+)-Pair: (.+)", block))
        coverage = re.search(r"MC/DC Coverage for Decision: ([\d.]+)%", block)
        found.append({
            "lines": (first, last),
            "text": " ".join(" ".join(source[first - 1:last]).split()),
            "conditions": conditions,
            "vectors": vectors,
            "pairs": pairs,
            "percent": float(coverage.group(1)) if coverage else 0.0,
        })
    return found


def decision_section(d: dict) -> list[str]:
    first, last = d["lines"]
    where = f"line {first}" if first == last else f"lines {first}-{last}"
    names = [name for name, _ in d["conditions"]]
    out = [
        f"### {MODULE.rsplit('/', 1)[-1]}, {where}: {d['percent']:.0f} %",
        "",
        f"`{d['text']}`",
        "",
        "| Condition | Source | Independence pair (vectors) |",
        "|-----------|--------|-----------------------------|",
    ]
    for name, text in d["conditions"]:
        pair = d["pairs"].get(name, "not covered").strip()
        pair = re.sub(r"covered: \((\d+),(\d+)\)", r"\1 and \2", pair)
        out.append(f"| {name} | `{text}` | {pair} |")
    out += [
        "",
        "| Vector | " + " | ".join(names) + " | Decision |",
        "|-------:|" + "|".join(":-:" for _ in names) + "|:-:|",
    ]
    for i, (values, result) in enumerate(d["vectors"], 1):
        out.append(f"| {i} | " + " | ".join(values) + f" | {result} |")
    out.append("")
    return out


# ---------------------------------------------------------------------------- report

def render(executables: list[Path], profile: Path) -> tuple[str, list[str]]:
    objects = object_args(executables)
    export = json.loads(llvm("cov", "export", *objects, f"-instr-profile={profile}"))
    files = {
        rel(f["filename"]): f["summary"]
        for f in export["data"][0]["files"]
        if rel(f["filename"]).startswith("firmware/")
    }
    module = files[MODULE]
    show = llvm("cov", "show", *objects, f"-instr-profile={profile}", "--show-mcdc",
                str(ROOT / MODULE))
    source = (ROOT / MODULE).read_text(encoding="utf-8").splitlines()
    found = decisions(show, source)
    version = llvm("cov", "--version").splitlines()
    version = next((v.strip() for v in version if "LLVM version" in v), version[0])

    errors = [
        f"{MODULE}: {kind} coverage {module[kind]['covered']}/{module[kind]['count']}, "
        "objective 100 %"
        for kind in ("lines", "branches", "mcdc")
        if module[kind]["covered"] != module[kind]["count"]
    ]

    out = [
        "# MC/DC Coverage Report",
        "",
        "<!-- Generated by tools/coverage/mcdc.py. Do not edit by hand. -->",
        "",
        "Structural coverage of the host unit tests, measured with clang's source-based",
        "coverage. DO-178C asks for statement, decision and modified condition/decision",
        "coverage (MC/DC) of level A software: every condition in a decision must be shown",
        "to change the decision's outcome on its own. The objective is applied to Node A's",
        f"safety logic, [`{MODULE.rsplit('/', 1)[-1]}`](../{MODULE}), which commands",
        "return-to-home, landing and line avoidance (HLR-001 to HLR-006), and is tested by",
        "[`test_safety.c`](../tests/unit/test_safety.c). CI fails if it drops below 100 %.",
        "",
        "## Configuration",
        "",
        f"- Compiler: clang {version.split()[-1]} (`-fprofile-instr-generate -fcoverage-mapping"
        " -fcoverage-mcdc`), CMake preset `coverage`",
        f"- Tests: the {len(executables)} host unit test programs in `tests/unit`, run by ctest",
        "- MC/DC as clang measures it: unique-cause MC/DC with masking for short-circuit",
        "  operators, over decisions with two or more conditions (single-condition",
        "  decisions are covered by branch coverage)",
        "- Host build: the module runs with test doubles for its inputs and outputs, so the",
        "  timing of the real system is not part of this measurement",
        "",
        f"## {MODULE.rsplit('/', 1)[-1]}",
        "",
        "| Measure | Covered | Objective |",
        "|---------|---------|-----------|",
        f"| Functions | {percent(module['functions'])} | 100 % |",
        f"| Statements (lines) | {percent(module['lines'])} | 100 % |",
        f"| Decisions (branches) | {percent(module['branches'])} | 100 % |",
        f"| MC/DC (conditions) | {percent(module['mcdc'])} | 100 % |",
        "",
        "Each decision with two or more conditions follows: its conditions, the test",
        "vectors the tests executed (`-` marks a condition not evaluated, because of",
        "short-circuit evaluation), and for each condition the pair of vectors that differ",
        "in that condition and in the outcome while the other evaluated conditions are",
        "unchanged.",
        "",
    ]
    for d in found:
        out += decision_section(d)

    out += [
        "## Other host-tested modules",
        "",
        "For information: these modules have no MC/DC objective here.",
        "",
        "| File | Lines | Branches | MC/DC conditions |",
        "|------|-------|----------|------------------|",
    ]
    for path in sorted(files):
        if path == MODULE or files[path]["lines"]["count"] == 0:
            continue
        s = files[path]
        out.append(
            f"| `{path}` | {percent(s['lines'])} | {percent(s['branches'])} "
            f"| {percent(s['mcdc'])} |"
        )
    out.append("")
    return "\n".join(out), errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check", action="store_true", help="fail if docs/mcdc-report.md is out of date"
    )
    args = parser.parse_args()

    executables = build_and_test()
    text, errors = render(executables, merge_profiles())
    for error in errors:
        print(f"error: {error}", file=sys.stderr)

    current = OUT_DOC.read_text(encoding="utf-8") if OUT_DOC.exists() else ""
    if args.check:
        if current != text:
            print(
                "error: docs/mcdc-report.md is out of date; run python3 tools/coverage/mcdc.py",
                file=sys.stderr,
            )
            sys.stderr.writelines(difflib.unified_diff(
                current.splitlines(keepends=True), text.splitlines(keepends=True),
                "docs/mcdc-report.md (committed)", "docs/mcdc-report.md (this run)",
            ))
            return 1
    elif current != text:
        OUT_DOC.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {OUT_DOC.relative_to(ROOT).as_posix()}")
    print(f"{len(executables)} test programs, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
