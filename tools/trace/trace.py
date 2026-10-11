"""Build the HLR -> LLR -> code -> test traceability matrix.

The requirements documents are the single source of truth: docs/requirements/HLR.md
lists the high-level requirements, and each row of docs/requirements/LLR.md names
its parent HLRs, the code that implements it and the tests that verify it. This
script resolves every code file and test name in those tables against the
repository, so a renamed test or file shows up as an error instead of a stale
reference, and writes the result to docs/traceability.md.

    python tools/trace/trace.py            # regenerate docs/traceability.md
    python tools/trace/trace.py --check    # CI: fail if it is stale or a reference is broken
"""

from __future__ import annotations

import argparse
import fnmatch
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HLR_DOC = ROOT / "docs/requirements/HLR.md"
LLR_DOC = ROOT / "docs/requirements/LLR.md"
OUT_DOC = ROOT / "docs/traceability.md"

CODE_DIRS = ("firmware", "ground_station", "sim", "ml")
CODE_SUFFIXES = {".c", ".cc", ".h", ".ld", ".py"}

# Verification methods named in plain text in the LLR test column.
INSPECTION = "Inspection"
BUILD = "Build"
ALL_ROBOT = "All Robot tests"


@dataclass
class Hlr:
    id: str
    text: str
    section: str
    verification: str


@dataclass
class Test:
    kind: str  # "Robot", "Unit" or "pytest"
    name: str
    path: Path
    line: int

    @property
    def link(self) -> str:
        return f"[{self.name}](../{self.path.as_posix()}#L{self.line})"


@dataclass
class Llr:
    id: str
    text: str
    section: str
    parents: list[str]
    code: list[Path] = field(default_factory=list)
    tests: list[Test] = field(default_factory=list)
    other_methods: list[str] = field(default_factory=list)


class Repository:
    """Index of the code files and test cases the requirements may refer to."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.code: dict[str, list[Path]] = {}
        for top in CODE_DIRS:
            for path in sorted((root / top).rglob("*")):
                if path.suffix in CODE_SUFFIXES and "__pycache__" not in path.parts:
                    rel = path.relative_to(root)
                    self.code.setdefault(path.name, []).append(rel)
                    self.code.setdefault(rel.as_posix(), []).append(rel)
        self.tests: list[Test] = []
        self._index_robot(root / "tests/robot")
        self._index_unit(root / "tests/unit")
        for directory in ("tests/python", "tests/integration"):
            self._index_pytest(root / directory)

    def _rel(self, path: Path) -> Path:
        return path.relative_to(self.root)

    def _index_robot(self, directory: Path) -> None:
        for path in sorted(directory.glob("*.robot")):
            in_cases = False
            for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if line.startswith("***"):
                    in_cases = line.strip("* ").lower() == "test cases"
                elif in_cases and line and not line[0].isspace() and not line.startswith("#"):
                    self.tests.append(Test("Robot", line.strip(), self._rel(path), number))

    def _index_unit(self, directory: Path) -> None:
        for path in sorted([*directory.glob("test_*.c"), *directory.glob("test_*.cc")]):
            source = path.read_text(encoding="utf-8")
            lines = source.splitlines()
            for name in re.findall(r"^\s*RUN\((\w+)\);", source, re.MULTILINE):
                definition = re.compile(rf"^static void {name}\(")
                number = next(i for i, text in enumerate(lines, 1) if definition.match(text))
                self.tests.append(Test("Unit", name, self._rel(path), number))

    def _index_pytest(self, directory: Path) -> None:
        for path in sorted(directory.glob("test_*.py")):
            for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                match = re.match(r"def (test_\w+)\(", line)
                if match:
                    self.tests.append(Test("pytest", match.group(1), self._rel(path), number))

    def find_code(self, token: str, note: str) -> list[Path]:
        """Resolve a code reference; "(Node A)" or "(Node B)" after it picks one node."""
        paths = self.code.get(token, [])
        node = re.match(r"\s*\((Node [AB])\)", note)
        if node and len(paths) > 1:
            folder = "node_" + node.group(1)[-1].lower()
            paths = [p for p in paths if folder in p.parts]
        return paths

    def find_tests(self, token: str) -> list[Test]:
        """Resolve a test reference: a test file, a test name, or a name pattern."""
        stem = token.removesuffix(".robot").removesuffix(".py")
        stem = stem.removesuffix(".cc").removesuffix(".c")
        if stem != token or stem.startswith("test_"):
            by_file = [t for t in self.tests if t.path.stem == stem]
            if by_file:
                return by_file
        pattern = token.replace("...", "*")
        if "*" in pattern:
            return [t for t in self.tests if fnmatch.fnmatchcase(t.name, pattern)]
        return [t for t in self.tests if t.name == token]


def table_rows(path: Path) -> list[tuple[str, list[str]]]:
    """Return (section heading, cells) for every data row of the Markdown tables."""
    rows = []
    section = ""
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            section = line[3:].strip()
        elif line.startswith("|") and not re.match(r"\|\s*(ID|-)", line):
            rows.append((section, [cell.strip() for cell in line.strip("|").split("|")]))
    return rows


def expand_parents(text: str) -> list[str]:
    """Turn "HLR-001 to HLR-004, HLR-009" into the list of ids it names."""
    ids: list[str] = []
    for first, last in re.findall(r"HLR-(\d+)(?:\s+to\s+HLR-(\d+))?", text):
        stop = int(last or first)
        ids += [f"HLR-{n:03d}" for n in range(int(first), stop + 1)]
    return ids


def strip_markdown(text: str) -> str:
    return text.replace("**", "")


def load(repo: Repository) -> tuple[list[Hlr], list[Llr], list[str]]:
    errors: list[str] = []
    hlrs = [
        Hlr(cells[0], strip_markdown(cells[1]), section, cells[3])
        for section, cells in table_rows(HLR_DOC)
        if re.fullmatch(r"HLR-\d{3}", cells[0])
    ]
    known = {h.id for h in hlrs}
    if len(known) != len(hlrs):
        errors.append("HLR.md: duplicate HLR id")

    llrs: list[Llr] = []
    for section, cells in table_rows(LLR_DOC):
        if not re.fullmatch(r"LLR-\d{3}", cells[0]):
            continue
        ident, text, parent_cell, code_cell, test_cell = cells
        llr = Llr(ident, strip_markdown(text), section, expand_parents(parent_cell))
        if not llr.parents and "(derived)" not in parent_cell:
            errors.append(f"{ident}: no parent HLR and not marked (derived)")
        errors += [f"{ident}: unknown parent {p}" for p in llr.parents if p not in known]

        for match in re.finditer(r"`([^`]+)`([^`,]*)", code_cell):
            paths = repo.find_code(match.group(1), match.group(2))
            if not paths:
                errors.append(f"{ident}: code file `{match.group(1)}` not found")
            llr.code += [p for p in paths if p not in llr.code]

        for token in re.findall(r"`([^`]+)`", test_cell):
            tests = repo.find_tests(token)
            if not tests:
                errors.append(f"{ident}: test `{token}` not found")
            llr.tests += [t for t in tests if t not in llr.tests]
        plain = re.sub(r"`[^`]+`", "", test_cell)
        if ALL_ROBOT in plain:
            llr.other_methods.append("All Robot tests (every test boots the firmware)")
        if INSPECTION in plain:
            llr.other_methods.append(INSPECTION)
        if re.search(rf"\b{BUILD}\b", plain):
            llr.other_methods.append("Build (enforced at link time)")
        if not llr.tests and not llr.other_methods:
            errors.append(f"{ident}: no verification given")
        llrs.append(llr)

    ids = [llr.id for llr in llrs]
    errors += [f"LLR.md: duplicate id {i}" for i in sorted({i for i in ids if ids.count(i) > 1})]
    return hlrs, llrs, errors


def code_links(paths: list[Path]) -> str:
    return ", ".join(f"[`{p.name}`](../{p.as_posix()})" for p in paths) or "-"


def test_summary(tests: list[Test]) -> str:
    counts = {kind: sum(t.kind == kind for t in tests) for kind in ("Robot", "Unit", "pytest")}
    return ", ".join(f"{n} {kind}" for kind, n in counts.items() if n) or "-"


def render(hlrs: list[Hlr], llrs: list[Llr], repo: Repository) -> str:
    children = {h.id: [llr for llr in llrs if h.id in llr.parents] for h in hlrs}
    traced = {id(t) for llr in llrs for t in llr.tests}
    untraced = [t for t in repo.tests if id(t) not in traced]
    derived = [llr for llr in llrs if not llr.parents]
    only_other = [llr for llr in llrs if not llr.tests]

    out = [
        "# Traceability Matrix",
        "",
        "<!-- Generated by tools/trace/trace.py from docs/requirements/HLR.md and LLR.md."
        " Do not edit by hand: edit the requirements and run the script. -->",
        "",
        "This matrix links each [high-level requirement](requirements/HLR.md) to the",
        "[low-level requirements](requirements/LLR.md) that refine it, and each LLR to",
        "the code that implements it and the tests that verify it. It is generated from",
        "the requirements tables, and every code file and test named there is resolved",
        "against the repository, so the matrix cannot refer to a file or test that does",
        "not exist. CI runs `python tools/trace/trace.py --check` to keep it current.",
        "",
        "## Summary",
        "",
        "| Item | Count |",
        "|------|------:|",
        f"| High-level requirements | {len(hlrs)} |",
        f"| HLRs refined by at least one LLR | {sum(bool(c) for c in children.values())} |",
        f"| Low-level requirements | {len(llrs)} |",
        f"| Derived LLRs (no parent HLR) | {len(derived)} |",
        f"| LLRs verified by at least one named test | {len(llrs) - len(only_other)} |",
        f"| Test cases indexed (Robot / unit / pytest) | {len(repo.tests)} "
        f"({test_summary(repo.tests)}) |",
        f"| Test cases traced to an LLR | {len(repo.tests) - len(untraced)} |",
        "",
        "## HLR to LLR",
        "",
        "| HLR | Requirement | Verif. | LLRs | Tests |",
        "|-----|-------------|--------|------|-------|",
    ]
    for h in hlrs:
        kids = children[h.id]
        tests = {id(t): t for llr in kids for t in llr.tests}
        lrefs = ", ".join(f"[{c.id}](#{c.id.lower()})" for c in kids) or "**none**"
        out.append(
            f"| {h.id} | {h.text} | {h.verification} | {lrefs} "
            f"| {test_summary(list(tests.values()))} |"
        )

    gaps = [h for h in hlrs if not children[h.id]]
    if gaps:
        out += ["", "HLRs without an LLR:", ""]
        out += [f"- **{h.id}** ({h.section}): {h.text}" for h in gaps]

    out += ["", "## LLR to code and tests"]
    section = None
    for llr in llrs:
        if llr.section != section:
            section = llr.section
            out += ["", f"### {section}", ""]
        parents = ", ".join(llr.parents) or "derived"
        out += [
            f'<a id="{llr.id.lower()}"></a>**{llr.id}** ({parents}): {llr.text}',
            "",
            f"- Code: {code_links(llr.code)}",
        ]
        for kind in ("Robot", "Unit", "pytest"):
            tests = [t for t in llr.tests if t.kind == kind]
            if tests:
                out.append(f"- {kind}: " + ", ".join(t.link for t in tests))
        if llr.other_methods:
            out.append("- Other: " + ", ".join(llr.other_methods))
        out.append("")

    out += [
        "## Derived requirements",
        "",
        "These LLRs have no parent HLR: they come from design decisions (pin-out, start-up,",
        "debug tooling), not from system behaviour. Under DO-178C they are fed back to the",
        "system safety assessment; none of them affects a safety function here.",
        "",
    ]
    out += [f"- [{llr.id}](#{llr.id.lower()}) ({llr.section})" for llr in derived]
    out += [
        "",
        "## LLRs without a named test",
        "",
        "These LLRs are verified by inspection, by the build, or by every Robot test",
        "(start-up and console behaviour that all of them depend on).",
        "",
    ]
    out += [
        f"- [{llr.id}](#{llr.id.lower()}): {', '.join(llr.other_methods)}" for llr in only_other
    ]
    out += [
        "",
        "## Tests not traced to an LLR",
        "",
        "Test cases that no LLR names. They verify the test environment rather than the",
        "flight software: the plant model, the simulated autopilot, the ground station's",
        "command helper and the ML training pipeline. Under DO-178C these are tool",
        "qualification concerns, not requirements-based tests.",
        "",
        "| File | Tests |",
        "|------|-------|",
    ]
    by_file: dict[Path, list[Test]] = {}
    for t in untraced:
        by_file.setdefault(t.path, []).append(t)
    for path, tests in by_file.items():
        out.append(f"| `{path.as_posix()}` | " + ", ".join(t.link for t in tests) + " |")
    out.append("")
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check", action="store_true", help="fail if docs/traceability.md is out of date"
    )
    args = parser.parse_args()

    repo = Repository(ROOT)
    hlrs, llrs, errors = load(repo)
    text = render(hlrs, llrs, repo)
    for error in errors:
        print(f"error: {error}", file=sys.stderr)

    current = OUT_DOC.read_text(encoding="utf-8") if OUT_DOC.exists() else ""
    if args.check:
        if current != text:
            print(
                "error: docs/traceability.md is out of date; run python tools/trace/trace.py",
                file=sys.stderr,
            )
            return 1
    elif current != text:
        OUT_DOC.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {OUT_DOC.relative_to(ROOT).as_posix()}")
    print(f"{len(hlrs)} HLRs, {len(llrs)} LLRs, {len(repo.tests)} tests, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
