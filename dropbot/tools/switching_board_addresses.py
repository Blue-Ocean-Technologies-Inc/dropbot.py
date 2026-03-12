"""Manage I2C addresses and UUIDs of HV switching boards on a DropBot.

Usage:
    # Reassign a board to a new address
    python -m dropbot.tools.switching_board_addresses assign 32 35

    # Swap two boards' addresses
    python -m dropbot.tools.switching_board_addresses swap 32 34
    python -m dropbot.tools.switching_board_addresses swap 32 34 --temp 50

    # Use direct EEPROM write (bypasses bootloader)
    python -m dropbot.tools.switching_board_addresses assign 32 35 --direct
    python -m dropbot.tools.switching_board_addresses swap 32 34 --direct

    # Preview without making changes
    python -m dropbot.tools.switching_board_addresses assign 32 35 --dry-run

    # Force watchdog reboot instead of live config reload (not recommended
    # for boards near the control board -- may get stuck in reset loop)
    python -m dropbot.tools.switching_board_addresses assign 32 35 --direct --reboot

    # Read a board's UUID
    python -m dropbot.tools.switching_board_addresses uuid 34

    # Write a new UUID to a board
    python -m dropbot.tools.switching_board_addresses uuid 34 1c1540b5-1d95-4012-8b4d-0b323576a3d9

    # Generate and write a random UUID
    python -m dropbot.tools.switching_board_addresses uuid 34 --random
"""
import argparse
import struct
import sys
import time
import uuid as uuid_mod

import dropbot as db
from hv_switching_board.driver import HVSwitchingBoard

# BaseNode I2C command codes
_CMD_PERSISTENT_READ = 0x90
_CMD_PERSISTENT_WRITE = 0x91
_CMD_LOAD_CONFIG = 0x92
_CMD_REBOOT = 0xA2
_I2C_ADDR_EEPROM_OFFSET = 6
_UUID_EEPROM_OFFSET = 8
_UUID_LENGTH = 16


def _i2c_flush(proxy, board_addr: int) -> None:
    """Drain any stale I2C response data to re-sync the state machine.

    The BaseNode I2C protocol alternates between sending bytes_written_
    (2-byte length) and buffer data.  If a previous transaction was
    incomplete, the state machine may be stuck expecting a read.  This
    sends a dummy read to clear it.
    """
    try:
        raw = proxy.i2c_read(board_addr, 2).tobytes()
        length = struct.unpack('<H', raw)[0]
        if 0 < length <= 32:
            proxy.i2c_read(board_addr, length)
    except Exception:
        pass
    time.sleep(0.01)


def _eeprom_read(proxy, board_addr: int, eeprom_addr: int) -> int:
    """Read a byte from switching board EEPROM via persistent_read over I2C."""
    addr_lo, addr_hi = struct.unpack('BB', struct.pack('<H', eeprom_addr))
    proxy.i2c_write(board_addr, [_CMD_PERSISTENT_READ, addr_lo, addr_hi])
    time.sleep(0.02)
    length = struct.unpack('<H', proxy.i2c_read(board_addr, 2).tobytes())[0]
    if length > 32:
        # State machine out of sync — flush and retry once.
        print(f"  WARNING: got bad length {length} from board {board_addr}, "
              f"flushing and retrying...")
        _i2c_flush(proxy, board_addr)
        time.sleep(0.05)
        proxy.i2c_write(board_addr, [_CMD_PERSISTENT_READ, addr_lo, addr_hi])
        time.sleep(0.02)
        length = struct.unpack('<H', proxy.i2c_read(board_addr, 2).tobytes())[0]
        if length > 32:
            raise IOError(f"Board {board_addr} I2C state machine stuck "
                          f"(got length={length})")
    time.sleep(0.02)
    data = proxy.i2c_read(board_addr, length)
    return int(data[0])


def _eeprom_write(proxy, board_addr: int, eeprom_addr: int, value: int) -> None:
    """Write a byte to switching board EEPROM via persistent_write over I2C."""
    addr_lo, addr_hi = struct.unpack('BB', struct.pack('<H', eeprom_addr))
    proxy.i2c_write(board_addr, [_CMD_PERSISTENT_WRITE, addr_lo, addr_hi, value])
    time.sleep(0.02)
    # Drain response to keep I2C state machine in sync
    length = struct.unpack('<H', proxy.i2c_read(board_addr, 2).tobytes())[0]
    time.sleep(0.02)
    if length > 0:
        proxy.i2c_read(board_addr, length)
    time.sleep(0.01)


_REBOOT_NOTE = (
    "\nNOTE: The address change is applied live via config reload, but the\n"
    "board's internal state (shift registers, channel outputs) was also\n"
    "reloaded from EEPROM defaults.  To ensure a fully clean state,\n"
    "power-cycle the DropBot by unplugging BOTH USB and the 12V power\n"
    "supply, waiting a few seconds, then reconnecting.")


def read_uuid(proxy, board_addr: int) -> str:
    """Read the 16-byte UUID from a switching board's EEPROM.

    Returns the UUID as a standard formatted string
    (xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx), or a note if all zeros.
    """
    uuid_bytes = []
    for i in range(_UUID_LENGTH):
        b = _eeprom_read(proxy, board_addr, _UUID_EEPROM_OFFSET + i)
        uuid_bytes.append(b)
    raw = bytes(uuid_bytes)
    if raw == b'\x00' * _UUID_LENGTH:
        return "(all zeros — not set)"
    return str(uuid_mod.UUID(bytes=raw))


def write_uuid(proxy, board_addr: int, uuid_str: str,
               dry_run: bool = False) -> None:
    """Write a UUID to a switching board's EEPROM.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
    board_addr : int
        Board's current I2C address.
    uuid_str : str
        UUID string (with or without dashes).
    dry_run : bool
        If True, only read and display without writing.
    """
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"I2C devices on bus: {i2c_devices}")

    if board_addr not in i2c_devices:
        print(f"ERROR: No device found at address {board_addr}")
        sys.exit(1)

    # Read current UUID
    current = read_uuid(proxy, board_addr)
    print(f"Current UUID on board {board_addr}: {current}")

    if dry_run:
        print(f"DRY RUN: Would write UUID {uuid_str}")
        return

    # Parse and validate UUID
    try:
        new_uuid = uuid_mod.UUID(uuid_str)
    except ValueError:
        print(f"ERROR: Invalid UUID format: {uuid_str}")
        sys.exit(1)

    uuid_bytes = new_uuid.bytes
    print(f"\nWriting UUID {new_uuid} to board {board_addr} ...")

    for i in range(_UUID_LENGTH):
        _eeprom_write(proxy, board_addr, _UUID_EEPROM_OFFSET + i,
                      uuid_bytes[i])

    # Verify
    verify = read_uuid(proxy, board_addr)
    if verify == str(new_uuid):
        print(f"SUCCESS: UUID written and verified.")
    else:
        print(f"WARNING: Verification mismatch. Read back: {verify}")


def _set_address_via_eeprom(proxy, current_addr: int, new_addr: int,
                            use_reboot: bool = False) -> None:
    """Set I2C address by writing directly to EEPROM, bypassing bootloader.

    By default, uses CMD_LOAD_CONFIG to make the board reload its config
    and call Wire.begin(new_addr) — the address changes live without any
    reboot, completely avoiding the twiboot bootloader.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
    current_addr : int
        Board's current I2C address.
    new_addr : int
        Desired new I2C address.
    use_reboot : bool
        If True, use CMD_REBOOT (watchdog reset) instead of CMD_LOAD_CONFIG.
        Not recommended — boards near the control board may get stuck in a
        watchdog reset loop due to twiboot bootloader timing issues.
    """
    # Verify we can read the current stored address
    stored = _eeprom_read(proxy, current_addr, _I2C_ADDR_EEPROM_OFFSET)
    if stored != current_addr:
        print(f"  WARNING: EEPROM stored address ({stored}) != "
              f"current address ({current_addr})")

    # Write new address
    _eeprom_write(proxy, current_addr, _I2C_ADDR_EEPROM_OFFSET, new_addr)

    # Verify write before applying
    verify = _eeprom_read(proxy, current_addr, _I2C_ADDR_EEPROM_OFFSET)
    if verify != new_addr:
        raise IOError(f"EEPROM verify failed: wrote {new_addr}, read {verify}")

    if use_reboot:
        # Watchdog reboot — board disappears and re-enters bootloader.
        # WARNING: some boards (especially the one closest to the control
        # board) may get stuck in a watchdog reset loop because twiboot
        # doesn't disable the watchdog fast enough after the 15ms timeout.
        print(f"  Rebooting board at {current_addr} (watchdog reset)...")
        proxy.i2c_write(current_addr, _CMD_REBOOT)
        for _ in range(100):
            time.sleep(0.1)
            if new_addr in proxy.i2c_scan().tolist():
                return
        raise IOError(
            f"Board did not appear at address {new_addr} after reboot. "
            f"A power cycle should bring it up at the new address "
            f"(EEPROM was verified before reboot).")
    else:
        # Live config reload — send CMD_LOAD_CONFIG (0x92) with
        # use_defaults=0.  BaseNode::load_config() reads EEPROM and calls
        # Wire.begin(new_address), so the board switches instantly.
        # No reboot, no bootloader, no watchdog.
        print(f"  Reloading config (live address change)...")
        proxy.i2c_write(current_addr, [_CMD_LOAD_CONFIG, 0x00])
        time.sleep(0.2)

        # The board is now at new_addr.  The I2C response to the command
        # above will NACK (board already moved), which is expected.
        # Drain any stale state by scanning.
        for attempt in range(20):
            time.sleep(0.1)
            if new_addr in proxy.i2c_scan().tolist():
                return
        raise IOError(
            f"Board did not appear at address {new_addr} after config "
            f"reload.  A power cycle should bring it up at the new address "
            f"(EEPROM was verified before reload).")


def _move_board(proxy, src: int, dst: int, direct: bool = False,
                use_reboot: bool = False) -> None:
    """Move a board from src to dst address.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
    src : int
        Current I2C address.
    dst : int
        New I2C address.
    direct : bool
        If True, write directly to EEPROM (bypasses bootloader).
        If False, use bootloader method with EEPROM fallback.
    use_reboot : bool
        If True and direct mode, use watchdog reboot instead of live
        config reload.  Not recommended for boards near the control board.
    """
    if direct:
        _set_address_via_eeprom(proxy, src, dst, use_reboot=use_reboot)
    else:
        board = HVSwitchingBoard(proxy, address=src)
        try:
            board.set_i2c_address(dst)
        except IOError:
            print(f"  Bootloader unavailable, falling back to direct EEPROM write...")
            _set_address_via_eeprom(proxy, src, dst, use_reboot=use_reboot)


def assign_board_address(proxy, src: int, dst: int,
                         dry_run: bool = False,
                         direct: bool = False,
                         use_reboot: bool = False) -> None:
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
    direct : bool
        If True, write directly to EEPROM (bypasses bootloader).
    use_reboot : bool
        If True and direct mode, use watchdog reboot instead of live
        config reload.
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
    _move_board(proxy, src, dst, direct=direct, use_reboot=use_reboot)
    time.sleep(0.5)
    print(f"  Done.")

    # Verify
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"\nI2C devices after reassign: {i2c_devices}")

    if dst in i2c_devices and src not in i2c_devices:
        print(f"SUCCESS: Board reassigned ({src} -> {dst})")
        if not use_reboot:
            print(_REBOOT_NOTE)
    else:
        print("WARNING: Post-reassign verification failed. "
              "Check I2C scan above.")


def swap_board_addresses(proxy, addr_a: int, addr_b: int,
                         temp_addr: int = 50,
                         dry_run: bool = False,
                         direct: bool = False,
                         use_reboot: bool = False) -> None:
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
    direct : bool
        If True, write directly to EEPROM (bypasses bootloader).
    use_reboot : bool
        If True and direct mode, use watchdog reboot instead of live
        config reload.
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
    _move_board(proxy, addr_a, temp_addr, direct=direct,
                use_reboot=use_reboot)
    time.sleep(0.5)
    print(f"  Done. Board A now at {temp_addr}")

    # Step 2: addr_b -> addr_a
    print(f"Step 2/3: Moving board {addr_b} -> {addr_a} ...")
    _move_board(proxy, addr_b, addr_a, direct=direct,
                use_reboot=use_reboot)
    time.sleep(0.5)
    print(f"  Done. Board B now at {addr_a}")

    # Step 3: temp -> addr_b
    print(f"Step 3/3: Moving board {temp_addr} -> {addr_b} ...")
    _move_board(proxy, temp_addr, addr_b, direct=direct,
                use_reboot=use_reboot)
    time.sleep(0.5)
    print(f"  Done. Board A now at {addr_b}")

    # Verify
    i2c_devices = proxy.i2c_scan().tolist()
    print(f"\nI2C devices after swap: {i2c_devices}")

    if addr_a in i2c_devices and addr_b in i2c_devices:
        print(f"SUCCESS: Boards swapped ({addr_a} <-> {addr_b})")
        if not use_reboot:
            print(_REBOOT_NOTE)
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

    # shared arguments for both subcommands
    direct_help = ("Write directly to EEPROM instead of using "
                   "bootloader (use when bootloader is unavailable)")
    reboot_help = ("Force watchdog reboot instead of live config reload "
                   "when using --direct.  Not recommended for boards near "
                   "the control board (may get stuck in reset loop)")

    # assign subcommand
    assign_parser = subparsers.add_parser(
        "assign", help="Reassign a board to a new address")
    assign_parser.add_argument("src", type=int,
                               help="Board's current I2C address")
    assign_parser.add_argument("dst", type=int,
                               help="Desired new I2C address")
    assign_parser.add_argument("--direct", action="store_true",
                               help=direct_help)
    assign_parser.add_argument("--reboot", action="store_true",
                               help=reboot_help)

    # swap subcommand
    swap_parser = subparsers.add_parser(
        "swap", help="Swap two boards' addresses")
    swap_parser.add_argument("addr_a", type=int,
                             help="First board's I2C address")
    swap_parser.add_argument("addr_b", type=int,
                             help="Second board's I2C address")
    swap_parser.add_argument("--temp", type=int, default=50,
                             help="Temporary parking address (default: 50)")
    swap_parser.add_argument("--direct", action="store_true",
                             help=direct_help)
    swap_parser.add_argument("--reboot", action="store_true",
                             help=reboot_help)

    # uuid subcommand
    uuid_parser = subparsers.add_parser(
        "uuid", help="Read or write a board's UUID")
    uuid_parser.add_argument("addr", type=int,
                             help="Board's I2C address")
    uuid_parser.add_argument("new_uuid", type=str, nargs="?", default=None,
                             help="New UUID to write (omit to just read)")
    uuid_parser.add_argument("--random", action="store_true",
                             help="Generate and write a random UUID")

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
                                 dry_run=args.dry_run,
                                 direct=args.direct,
                                 use_reboot=args.reboot)
        elif args.command == "swap":
            swap_board_addresses(proxy, args.addr_a, args.addr_b,
                                 temp_addr=args.temp,
                                 dry_run=args.dry_run,
                                 direct=args.direct,
                                 use_reboot=args.reboot)
        elif args.command == "uuid":
            if args.random:
                new_uuid = str(uuid_mod.uuid4())
                print(f"Generated random UUID: {new_uuid}")
                write_uuid(proxy, args.addr, new_uuid,
                           dry_run=args.dry_run)
            elif args.new_uuid:
                write_uuid(proxy, args.addr, args.new_uuid,
                           dry_run=args.dry_run)
            else:
                current = read_uuid(proxy, args.addr)
                print(f"UUID on board {args.addr}: {current}")
    finally:
        proxy.terminate()


if __name__ == "__main__":
    main()
