"""Manage I2C addresses of HV switching boards on a DropBot.

Usage:
    # Reassign a board to a new address
    python -m dropbot.tools.switching_board_addresses assign 32 35
    python -m dropbot.tools.switching_board_addresses assign 32 35 --dry-run

    # Swap two boards' addresses
    python -m dropbot.tools.switching_board_addresses swap 32 34
    python -m dropbot.tools.switching_board_addresses swap 32 34 --temp 50
"""
import argparse
import sys
import time

import dropbot as db
from hv_switching_board.driver import HVSwitchingBoard


def assign_board_address(proxy, src: int, dst: int,
                         dry_run: bool = False) -> None:
    """Reassign a switching board from one I2C address to another.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
        Connected DropBot proxy.
    src : int
        Board's current I2C address.
    dst : int
        Desired new I2C address.
    dry_run : bool
        If True, only verify addresses without making changes.
    """
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"I2C devices on bus: {i2c_devices}")

    if src not in i2c_devices:
        print(f"ERROR: No device found at address {src}")
        sys.exit(1)
    if src == dst:
        print("ERROR: Source and destination addresses are the same.")
        sys.exit(1)
    if dst in i2c_devices:
        print(f"ERROR: Address {dst} is already in use.")
        sys.exit(1)

    if dry_run:
        print(f"DRY RUN: Would reassign board {src} -> {dst}")
        return

    print(f"\nReassigning board {src} -> {dst} ...")
    board = HVSwitchingBoard(proxy, address=src)
    board.set_i2c_address(dst)
    time.sleep(0.5)
    print(f"  Done.")

    # Verify
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"\nI2C devices after reassign: {i2c_devices}")

    if dst in i2c_devices and src not in i2c_devices:
        print(f"SUCCESS: Board reassigned ({src} -> {dst})")
    else:
        print("WARNING: Post-reassign verification failed. "
              "Check I2C scan above.")


def swap_board_addresses(proxy, addr_a: int, addr_b: int,
                         temp_addr: int = 50,
                         dry_run: bool = False) -> None:
    """Swap the I2C addresses of two HV switching boards.

    Uses a temporary address to avoid collisions during the swap.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
        Connected DropBot proxy.
    addr_a : int
        First board's current I2C address.
    addr_b : int
        Second board's current I2C address.
    temp_addr : int
        Temporary parking address (must not be in use).
    dry_run : bool
        If True, only verify addresses without making changes.
    """
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"I2C devices on bus: {i2c_devices}")

    if addr_a not in i2c_devices:
        print(f"ERROR: No device found at address {addr_a}")
        sys.exit(1)
    if addr_b not in i2c_devices:
        print(f"ERROR: No device found at address {addr_b}")
        sys.exit(1)
    if addr_a == addr_b:
        print("ERROR: Both addresses are the same, nothing to swap.")
        sys.exit(1)
    if temp_addr in i2c_devices:
        print(f"ERROR: Temporary address {temp_addr} is already in use. "
              f"Choose a different one with --temp.")
        sys.exit(1)

    if dry_run:
        print(f"DRY RUN: Would swap board {addr_a} <-> {addr_b} "
              f"(using temp address {temp_addr})")
        return

    # Step 1: addr_a -> temp
    print(f"\nStep 1/3: Moving board {addr_a} -> {temp_addr} ...")
    board_a = HVSwitchingBoard(proxy, address=addr_a)
    board_a.set_i2c_address(temp_addr)
    time.sleep(0.5)
    print(f"  Done. Board A now at {temp_addr}")

    # Step 2: addr_b -> addr_a
    print(f"Step 2/3: Moving board {addr_b} -> {addr_a} ...")
    board_b = HVSwitchingBoard(proxy, address=addr_b)
    board_b.set_i2c_address(addr_a)
    time.sleep(0.5)
    print(f"  Done. Board B now at {addr_a}")

    # Step 3: temp -> addr_b
    print(f"Step 3/3: Moving board {temp_addr} -> {addr_b} ...")
    board_a = HVSwitchingBoard(proxy, address=temp_addr)
    board_a.set_i2c_address(addr_b)
    time.sleep(0.5)
    print(f"  Done. Board A now at {addr_b}")

    # Verify
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"\nI2C devices after swap: {i2c_devices}")

    if addr_a in i2c_devices and addr_b in i2c_devices:
        print(f"SUCCESS: Boards swapped ({addr_a} <-> {addr_b})")
    else:
        print("WARNING: Post-swap verification failed. "
              "Check I2C scan above.")


def main():
    parser = argparse.ArgumentParser(
        description="Manage I2C addresses of HV switching boards.")
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Verify addresses without making changes")

    subparsers = parser.add_subparsers(dest="command", required=True)

    # assign subcommand
    assign_parser = subparsers.add_parser(
        "assign", help="Reassign a board to a new address")
    assign_parser.add_argument("src", type=int,
                               help="Board's current I2C address")
    assign_parser.add_argument("dst", type=int,
                               help="Desired new I2C address")

    # swap subcommand
    swap_parser = subparsers.add_parser(
        "swap", help="Swap two boards' addresses")
    swap_parser.add_argument("addr_a", type=int,
                             help="First board's I2C address")
    swap_parser.add_argument("addr_b", type=int,
                             help="Second board's I2C address")
    swap_parser.add_argument("--temp", type=int, default=50,
                             help="Temporary parking address (default: 50)")

    args = parser.parse_args()

    print("Connecting to DropBot...")
    kwargs = {"ignore": True}
    if args.port:
        kwargs["port"] = args.port
    proxy = db.SerialProxy(**kwargs)
    print(f"Connected on {proxy.port}\n")

    try:
        if args.command == "assign":
            assign_board_address(proxy, args.src, args.dst,
                                 dry_run=args.dry_run)
        elif args.command == "swap":
            swap_board_addresses(proxy, args.addr_a, args.addr_b,
                                 temp_addr=args.temp,
                                 dry_run=args.dry_run)
    finally:
        proxy.terminate()


if __name__ == "__main__":
    main()
