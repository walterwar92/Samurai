"""Entrypoint: python -m compute_node.mois_agent."""
from __future__ import annotations

import sys

from .agent import main

if __name__ == "__main__":
    sys.exit(main())
