"""Check the flight software against the project's MISRA C:2012 subset.

Runs cppcheck's MISRA addon over the C sources of the debug build, then sorts every
finding using docs/misra-deviations.md, which defines the subset: disapplied
advisory rules, documented deviations and known false positives of the tool. The
result is written to docs/misra-report.md. A finding that none of them accounts
for is reported as open, and makes the check fail.

    python3 tools/misra/misra.py            # regenerate docs/misra-report.md
    python3 tools/misra/misra.py --check    # CI: fail on open findings or a stale report

Needs cppcheck (the report records the version; CI uses Debian bookworm's 2.10, the
same as the development environment) and a configured debug build
(cmake --preset debug), whose compile_commands.json gives the include paths.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLAN_DOC = ROOT / "docs/misra-deviations.md"
OUT_DOC = ROOT / "docs/misra-report.md"
BUILD_DIR = ROOT / "build/debug"
WORK_DIR = ROOT / "build/misra"
THIRD_PARTY = "third_party/"

# MISRA C:2012 categories of the rules that appear in the plan or in findings.
# A rule that is reported but missing here fails the check until it is added.
CATEGORIES = {
    "1.2": "Advisory", "2.1": "Required", "2.2": "Required", "2.7": "Advisory",
    "8.4": "Required", "8.9": "Advisory", "8.11": "Advisory", "8.12": "Required",
    "9.3": "Required", "10.2": "Required", "10.4": "Required", "10.7": "Required",
    "10.8": "Required", "11.1": "Required", "11.4": "Advisory", "11.5": "Advisory",
    "11.6": "Required", "11.8": "Required", "11.9": "Required", "12.1": "Advisory",
    "12.2": "Required", "12.3": "Advisory", "13.4": "Advisory", "14.2": "Required",
    "14.4": "Required", "15.5": "Advisory", "15.7": "Required", "16.3": "Required",
    "17.1": "Required", "18.2": "Required", "18.3": "Required", "18.8": "Required",
    "19.2": "Advisory", "20.1": "Advisory", "20.2": "Required", "20.5": "Advisory",
    "20.7": "Required", "20.10": "Advisory", "21.1": "Required", "21.6": "Required",
    "21.15": "Required", "21.16": "Required",
}


@dataclass(frozen=True)
class Finding:
    rule: str  # "15.5" for MISRA, or cppcheck's own check id
    path: str  # relative to the repository root
    line: int
    text: str  # the source line

    @property
    def misra(self) -> bool:
        return self.rule[0].isdigit()


@dataclass
class Entry:
    """A row of the plan: a disapplied rule, a deviation or a false positive."""

    id: str
    rules: list[str]
    files: list[str] = field(default_factory=lambda: ["*"])
    pattern: re.Pattern[str] | None = None
    matches: list[Finding] = field(default_factory=list)

    def covers(self, f: Finding) -> bool:
        name = Path(f.path).name
        return (
            f.rule in self.rules
            and any(fnmatch.fnmatchcase(name, g) for g in self.files)
            and (self.pattern is None or bool(self.pattern.search(f.text)))
        )


def table_rows(path: Path, section: str) -> list[list[str]]:
    rows, current = [], ""
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            current = line[3:].strip()
        elif current == section and re.match(r"\|\s*[^\s\-]", line) and not re.match(
            r"\|\s*(ID|Rule)\s*\|", line
        ):
            rows.append([cell.strip() for cell in line.strip("|").split("|")])
    return rows


def load_plan() -> tuple[list[Entry], list[Entry], list[Entry]]:
    def backticked(cell: str) -> list[str]:
        return re.findall(r"`([^`]+)`", cell)

    def rules(cell: str) -> list[str]:
        return re.findall(r"\d+\.\d+", cell)

    disapplied = [Entry(r[0], [r[0]]) for r in table_rows(PLAN_DOC, "Disapplied advisory rules")]
    deviations = [
        Entry(r[0], rules(r[1]), backticked(r[2])) for r in table_rows(PLAN_DOC, "Deviations")
    ]
    false_positives = [
        Entry(r[0], rules(r[1]), backticked(r[2]), re.compile(backticked(r[3])[0]))
        for r in table_rows(PLAN_DOC, "Tool false positives")
    ]
    return disapplied, deviations, false_positives


def source_files() -> list[dict]:
    """The C translation units of the debug build, with -isystem turned into -I:
    cppcheck 2.10 ignores -isystem, and would then miss the FreeRTOS and lwIP headers."""
    commands = json.loads((BUILD_DIR / "compile_commands.json").read_text(encoding="utf-8"))
    units = []
    for entry in commands:
        rel = Path(entry["file"]).resolve().relative_to(ROOT).as_posix()
        if rel.startswith("firmware/") and rel.endswith(".c"):
            units.append(dict(entry, command=entry["command"].replace(" -isystem ", " -I")))
    return sorted(units, key=lambda e: e["file"])


def run_cppcheck(units: list[dict]) -> list[Finding]:
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    project = WORK_DIR / "compile_commands.json"
    project.write_text(json.dumps(units, indent=1), encoding="utf-8")
    result = WORK_DIR / "cppcheck.xml"
    subprocess.run(
        [
            "cppcheck", f"--project={project}", "--addon=misra", "--std=c11",
            "--platform=unix32", "--enable=missingInclude", "--inline-suppr",
            f"--cppcheck-build-dir={WORK_DIR}", "-j", "8", "--quiet", "--xml",
            f"--output-file={result}",
        ],
        cwd=ROOT,
        check=True,
    )
    findings = set()
    lines: dict[str, list[str]] = {}
    for error in ET.parse(result).getroot().iter("error"):
        check = error.get("id", "")
        location = error.find("location")
        if check == "missingIncludeSystem" or location is None:
            continue
        path = Path(location.get("file", ""))
        rel = (ROOT / path).resolve().relative_to(ROOT).as_posix()
        number = int(location.get("line", "0"))
        if rel not in lines:
            lines[rel] = (ROOT / rel).read_text(encoding="utf-8", errors="replace").splitlines()
        text = lines[rel][number - 1].strip() if 0 < number <= len(lines[rel]) else ""
        findings.add(Finding(check.removeprefix("misra-c2012-"), rel, number, text))
    return sorted(findings, key=lambda f: (f.path, f.line, f.rule))


def addon_rule_count() -> int:
    """How many rules cppcheck's MISRA addon implements, from its source."""
    exe = shutil.which("cppcheck")
    for base in (Path(exe).resolve().parent.parent if exe else None, Path("/usr/lib")):
        if base is None:
            continue
        for addon in base.rglob("addons/misra.py"):
            text = addon.read_text(encoding="utf-8")
            return len(set(re.findall(r"def misra_(\d+)_(\d+)\(", text)))
    return 0


def cppcheck_version() -> str:
    return subprocess.run(
        ["cppcheck", "--version"], capture_output=True, text=True, check=True
    ).stdout.strip()


def rule_key(rule: str) -> tuple[int, ...]:
    return tuple(int(part) for part in rule.split(".")) if rule[0].isdigit() else (999,)


def counts(findings: list[Finding]) -> str:
    by_file = Counter(Path(f.path).name for f in findings)
    return ", ".join(f"`{name}` {n}" for name, n in sorted(by_file.items()))


def render(units, findings, plan, rule_count, version) -> tuple[str, list[str]]:
    disapplied, deviations, false_positives = plan
    errors: list[str] = []
    ours = [f for f in findings if not f.path.startswith(THIRD_PARTY)]
    theirs = [f for f in findings if f.path.startswith(THIRD_PARTY)]

    status: dict[Finding, str] = {}
    for f in ours:
        # False positives first: a finding the tool got wrong is not a deviation.
        for entry in (*false_positives, *disapplied, *deviations):
            if entry.covers(f):
                entry.matches.append(f)
                status[f] = entry.id
                break
        else:
            status[f] = "Open"
    open_findings = [f for f in ours if status[f] == "Open"]
    for f in open_findings:
        errors.append(f"open finding: {f.path}:{f.line}: {f.rule}: {f.text}")
    for entry in false_positives:
        if not entry.matches:
            errors.append(f"{entry.id} matches no finding; remove it from the plan")
    named = {r for e in deviations for r in e.rules}
    for rule in sorted({f.rule for f in ours if f.misra} | named):
        if rule not in CATEGORIES:
            errors.append(f"rule {rule}: category unknown; add it to CATEGORIES")

    misra = [f for f in ours if f.misra]
    kinds = Counter(_kind(status[f]) for f in misra)
    other = [f for f in ours if not f.misra]
    errors += [f"cppcheck: {f.path}:{f.line}: {f.rule}: {f.text}" for f in other]
    out = [
        "# MISRA C:2012 Report",
        "",
        "<!-- Generated by tools/misra/misra.py from cppcheck's MISRA addon and"
        " docs/misra-deviations.md. Do not edit by hand. -->",
        "",
        "Static analysis of the flight software against the project's MISRA C:2012",
        "subset, as defined in the [compliance plan](misra-deviations.md). The",
        "checker runs on every CI build and fails if any finding is open.",
        "",
        "## Configuration",
        "",
        f"- Tool: {version}, MISRA addon ({rule_count} rules implemented); the other",
        "  rules and the directives are covered by code review",
        "- Options: `--std=c11 --platform=unix32` (32-bit `int`, `long` and pointers,",
        "  as on the Cortex-M4), include paths and defines from the debug build",
        f"- Translation units: {len(units)} C files in `firmware/` (the C++ TFLM wrapper and",
        "  third-party code are out of scope; see the plan)",
        "",
        "## Summary",
        "",
        "| Status | Findings |",
        "|--------|---------:|",
        f"| **Open** | **{kinds['Open']}** |",
        f"| Deviation | {kinds['Deviation']} |",
        f"| Disapplied advisory rule | {kinds['Disapplied']} |",
        f"| Tool false positive | {kinds['False positive']} |",
        f"| **Total in project code** | **{len(misra)}** |",
        f"| In third-party headers (not counted) | {sum(f.misra for f in theirs)} |",
        "",
        "## Findings by rule",
        "",
        "| Rule | Category | Findings | Status |",
        "|------|----------|---------:|--------|",
    ]
    by_rule: dict[str, list[Finding]] = {}
    for f in misra:
        by_rule.setdefault(f.rule, []).append(f)
    for rule in sorted(by_rule, key=rule_key):
        ids = sorted({status[f] for f in by_rule[rule]})
        labels = ", ".join(_label(i) for i in ids)
        out.append(
            f"| {rule} | {CATEGORIES.get(rule, '?')} | {len(by_rule[rule])} | {labels} |"
        )

    out += [
        "",
        "## Deviations",
        "",
        "| ID | Rules | Findings | Where |",
        "|----|-------|---------:|-------|",
    ]
    for e in deviations:
        where = counts(e.matches) or "none reported"
        out.append(f"| {e.id} | {', '.join(e.rules)} | {len(e.matches)} | {where} |")
    out += [
        "",
        "## Disapplied advisory rules",
        "",
        "| Rule | Findings | Where |",
        "|------|---------:|-------|",
    ]
    for e in disapplied:
        out.append(f"| {e.id} | {len(e.matches)} | {counts(e.matches)} |")
    out += [
        "",
        "## Tool false positives",
        "",
        "| ID | Rule | Findings | Where |",
        "|----|------|---------:|-------|",
    ]
    for e in false_positives:
        out.append(f"| {e.id} | {', '.join(e.rules)} | {len(e.matches)} | {counts(e.matches)} |")

    out += ["", "## Other cppcheck checks", ""]
    if other:
        out += [f"- **Open**: `{f.path}:{f.line}` {f.rule}" for f in other]
    else:
        out.append(
            "No error-level findings from cppcheck's own checks, and every project header was "
            "found. Two checks are suppressed inline, on code covered by D-003 and D-004."
        )
    out.append("")
    return "\n".join(out), errors


def _kind(status: str) -> str:
    """The class of a status: an entry id ("D-001", "FP-02", "15.5") or "Open"."""
    if status == "Open":
        return "Open"
    if status.startswith("D-"):
        return "Deviation"
    if status.startswith("FP-"):
        return "False positive"
    return "Disapplied"


def _label(status: str) -> str:
    kind = _kind(status)
    if kind == "Open":
        return "**Open**"
    return "Disapplied" if kind == "Disapplied" else status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check", action="store_true", help="fail if docs/misra-report.md is out of date"
    )
    args = parser.parse_args()

    if not (BUILD_DIR / "compile_commands.json").exists():
        print("error: configure the debug build first: cmake --preset debug", file=sys.stderr)
        return 1
    units = source_files()
    findings = run_cppcheck(units)
    text, errors = render(
        units, findings, load_plan(), addon_rule_count(), cppcheck_version()
    )
    for f in findings:
        if f.rule == "missingInclude" and f.path.startswith(THIRD_PARTY):
            errors.append(f"header not found: {f.path}:{f.line}: {f.text}")
    for error in errors:
        print(f"error: {error}", file=sys.stderr)

    current = OUT_DOC.read_text(encoding="utf-8") if OUT_DOC.exists() else ""
    if args.check:
        if current != text:
            print(
                "error: docs/misra-report.md is out of date; run python3 tools/misra/misra.py",
                file=sys.stderr,
            )
            return 1
    elif current != text:
        OUT_DOC.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {OUT_DOC.relative_to(ROOT).as_posix()}")
    ours = [f for f in findings if f.misra and not f.path.startswith(THIRD_PARTY)]
    print(f"{len(units)} files, {len(ours)} MISRA findings in project code, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
