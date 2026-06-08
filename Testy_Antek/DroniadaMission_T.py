import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Application.DroniadaMission import main

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Droniada mission test runner")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip append_waypoints and send_landing_sites",
    )
    args = parser.parse_args()
    main(dry_run=args.dry_run)
