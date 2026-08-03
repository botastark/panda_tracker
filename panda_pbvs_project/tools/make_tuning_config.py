#!/usr/bin/env python3
"""Create a simulation tuning config without editing the checked-in baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_assignment(value: str) -> tuple[str, Any]:
    key, separator, raw_value = value.partition("=")
    if not separator or not key.strip():
        raise argparse.ArgumentTypeError(
            f"Invalid assignment {value!r}; expected KEY=JSON_VALUE."
        )

    try:
        parsed_value = json.loads(raw_value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid JSON value in {value!r}: {exc.msg}."
        ) from exc

    return key.strip(), parsed_value


def set_path(config: dict[str, Any], dotted_key: str, value: Any) -> None:
    parts = dotted_key.split(".")
    current: dict[str, Any] = config

    for part in parts[:-1]:
        child = current.get(part)
        if not isinstance(child, dict):
            raise KeyError(
                f"Cannot set {dotted_key!r}: {part!r} is not an object."
            )
        current = child

    final_key = parts[-1]
    if final_key not in current:
        raise KeyError(
            f"Refusing to create unknown config key {dotted_key!r}."
        )

    current[final_key] = value


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a derived PBVS simulation tuning config."
    )
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--set",
        dest="assignments",
        type=parse_assignment,
        action="append",
        required=True,
        metavar="KEY=JSON_VALUE",
    )
    parser.add_argument(
        "--allow-nonlocal-base",
        action="store_true",
        help="Allow deriving from a config whose panda_ip is not localhost.",
    )
    args = parser.parse_args()

    base = args.base.expanduser().resolve()
    output = args.output.expanduser().resolve()

    if base == output:
        parser.error("--output must differ from --base.")

    config = json.loads(base.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        parser.error("Base config must contain a JSON object.")

    panda_ip = config.get("panda_ip")
    if (
        not args.allow_nonlocal_base
        and panda_ip not in {"127.0.0.1", "localhost"}
    ):
        parser.error(
            "Refusing to derive a tuning config from a non-localhost base. "
            "Use pbvs_sim.json."
        )

    changes: list[tuple[str, Any, Any]] = []
    for key, value in args.assignments:
        current: Any = config
        for part in key.split("."):
            if not isinstance(current, dict) or part not in current:
                parser.error(f"Unknown config key: {key}")
            old_value = current[part]
            current = old_value

        try:
            set_path(config, key, value)
        except KeyError as exc:
            parser.error(str(exc))

        changes.append((key, old_value, value))

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(config, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"Created: {output}")
    for key, old_value, new_value in changes:
        print(
            f"  {key}: "
            f"{json.dumps(old_value)} -> {json.dumps(new_value)}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
