#!/usr/bin/env python3
"""
strip_autolink_options.py

Remove (or filter) LC_LINKER_OPTION load commands from a Mach-O binary in
place. Used as a fallback when `-fno-autolink` did not fully prevent the
clang frontend from emitting `-framework UIUtilities` etc. into the static
archive(s) that we merged into PCLMobile.framework.

Supports:
  - Single Mach-O (32 / 64 bit).
  - Universal (fat) binaries; each slice is patched independently.

Usage:
    scripts/strip_autolink_options.py <binary> [--match SUBSTR ...]

Examples:
    # Strip every LC_LINKER_OPTION:
    scripts/strip_autolink_options.py build/.../PCLMobile.framework/PCLMobile

    # Strip only the ones that mention UIUtilities (default):
    scripts/strip_autolink_options.py PCLMobile

    # Strip a custom substring set:
    scripts/strip_autolink_options.py PCLMobile --match UIUtilities --match SwiftUI
"""

import argparse
import struct
import sys
from pathlib import Path

# Mach-O magics
MH_MAGIC      = 0xFEEDFACE
MH_CIGAM      = 0xCEFAEDFE
MH_MAGIC_64   = 0xFEEDFACF
MH_CIGAM_64   = 0xCFFAEDFE
FAT_MAGIC     = 0xCAFEBABE
FAT_CIGAM     = 0xBEBAFECA
FAT_MAGIC_64  = 0xCAFEBABF
FAT_CIGAM_64  = 0xBFBAFECA

LC_LINKER_OPTION = 0x2D
LC_REQ_DYLD      = 0x80000000


def _swap32(x): return struct.unpack(">I", struct.pack("<I", x))[0]


def _patch_slice(data, slice_off, slice_len, matchers, verbose):
    """Patch a single Mach-O slice. Returns (new_bytes, removed_count)."""
    hdr = data[slice_off:slice_off + 4]
    if len(hdr) < 4:
        return None, 0
    magic, = struct.unpack("<I", hdr)

    if magic in (MH_MAGIC, MH_CIGAM):
        is_64 = False
    elif magic in (MH_MAGIC_64, MH_CIGAM_64):
        is_64 = True
    else:
        return None, 0  # Not a Mach-O slice we recognise.

    swap = magic in (MH_CIGAM, MH_CIGAM_64)
    endian = ">" if swap else "<"

    header_size = 32 if is_64 else 28
    header_fmt = endian + ("IiiIIIII" if is_64 else "IiiIIII")
    fields = struct.unpack(header_fmt,
                           data[slice_off:slice_off + header_size])
    if is_64:
        cputype, cpusubtype, filetype, ncmds, sizeofcmds, flags, _reserved = fields[1:]
    else:
        cputype, cpusubtype, filetype, ncmds, sizeofcmds, flags = fields[1:]

    cmd_start = slice_off + header_size
    cmd_end   = cmd_start + sizeofcmds

    # Walk load commands.
    new_cmds = bytearray()
    removed = 0
    new_ncmds = 0
    p = cmd_start
    for _ in range(ncmds):
        if p + 8 > cmd_end:
            break
        cmd, cmdsize = struct.unpack(endian + "II", data[p:p + 8])
        cmd_kind = cmd & ~LC_REQ_DYLD
        body = data[p:p + cmdsize]

        keep = True
        if cmd_kind == LC_LINKER_OPTION:
            # Body layout: cmd (4) cmdsize (4) count (4) strings...
            count, = struct.unpack(endian + "I", body[8:12])
            strings = body[12:cmdsize]
            # Strings are NUL-terminated. Concatenate readable form for
            # matching.
            text = strings.split(b"\0")
            text_str = " ".join(s.decode("utf-8", "replace")
                                for s in text if s)
            if any(m in text_str for m in matchers):
                keep = False
                removed += 1
                if verbose:
                    print(f"    drop LC_LINKER_OPTION ({count}): {text_str}")

        if keep:
            new_cmds += body
            new_ncmds += 1
        p += cmdsize

    if removed == 0:
        return None, 0

    # Rewrite header: same total command region size (we pad with zeroes
    # rather than shrink, so file offsets / segment offsets stay valid).
    pad = sizeofcmds - len(new_cmds)
    if pad < 0:
        # Should be impossible -- we only ever drop commands.
        raise RuntimeError("internal error: new commands larger than old")
    new_cmds += b"\0" * pad

    if is_64:
        new_header = struct.pack(
            header_fmt,
            magic, cputype, cpusubtype, filetype,
            new_ncmds, sizeofcmds, flags, _reserved)
    else:
        new_header = struct.pack(
            header_fmt,
            magic, cputype, cpusubtype, filetype,
            new_ncmds, sizeofcmds, flags)

    out = bytearray(data)
    out[slice_off:slice_off + header_size] = new_header
    out[cmd_start:cmd_end] = new_cmds
    return bytes(out), removed


def _patch_file(path, matchers, verbose):
    raw = Path(path).read_bytes()
    if len(raw) < 4:
        print(f"  skip (too small): {path}")
        return 0

    magic, = struct.unpack(">I", raw[:4])

    # Not fat -> single slice at offset 0.
    if magic not in (FAT_MAGIC, FAT_CIGAM, FAT_MAGIC_64, FAT_CIGAM_64):
        new, removed = _patch_slice(raw, 0, len(raw), matchers, verbose)
        if new is not None:
            Path(path).write_bytes(new)
        return removed

    # Fat binary: walk fat_arch headers.
    nfat, = struct.unpack(">I", raw[4:8])
    is_64 = magic in (FAT_MAGIC_64, FAT_CIGAM_64)
    arch_size = 32 if is_64 else 20
    arch_fmt = ">IIQQII" if is_64 else ">IIIII"

    total_removed = 0
    out = bytearray(raw)
    for i in range(nfat):
        arch_off = 8 + i * arch_size
        fields = struct.unpack(arch_fmt, raw[arch_off:arch_off + arch_size])
        if is_64:
            _cpu, _sub, off, size, _align, _r = fields
        else:
            _cpu, _sub, off, size, _align = fields
        if verbose:
            print(f"  slice {i}: offset={off} size={size}")
        new, removed = _patch_slice(bytes(out), off, size, matchers, verbose)
        if new is not None:
            out = bytearray(new)
            total_removed += removed

    if total_removed:
        Path(path).write_bytes(bytes(out))
    return total_removed


def main():
    ap = argparse.ArgumentParser(
        description="Strip LC_LINKER_OPTION load commands from a Mach-O.")
    ap.add_argument("binary", type=Path, help="Path to a Mach-O binary.")
    ap.add_argument("--match", action="append", default=[],
                    help="Substring to match against the LC_LINKER_OPTION "
                         "string list. Repeatable. Defaults to "
                         "'UIUtilities'.")
    ap.add_argument("--all", action="store_true",
                    help="Drop every LC_LINKER_OPTION regardless of content.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not args.binary.exists():
        print(f"ERROR: {args.binary} does not exist", file=sys.stderr)
        sys.exit(1)

    if args.all:
        matchers = [""]  # "" matches every string.
    elif args.match:
        matchers = args.match
    else:
        matchers = ["UIUtilities"]

    print(f"==> patching {args.binary} (matchers={matchers!r})")
    removed = _patch_file(args.binary, matchers, args.verbose)
    print(f"   removed {removed} LC_LINKER_OPTION load command(s)")
    sys.exit(0)


if __name__ == "__main__":
    main()
