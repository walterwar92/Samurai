"""
Entry point для compute_node.dashboard.

Запуск:
    python -m compute_node.dashboard

WIP (2026-04, #7): пока используется старый compute_node/dashboard_node.py.
Этот entry-point будет реализован в C13 после завершения incremental migration.
"""
import sys


def main():
    sys.stderr.write(
        '[dashboard] WIP: incremental migration in progress (#7).\n'
        '[dashboard] Use python compute_node/dashboard_node.py for now.\n'
    )
    sys.exit(2)


if __name__ == '__main__':
    main()
