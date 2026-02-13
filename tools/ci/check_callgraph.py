#!/usr/bin/env python
#
# Based on cally.py (https://github.com/chaudron/cally/), Copyright 2018, Eelco Chaudron
# SPDX-FileCopyrightText: 2020-2023 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import re
from functools import partial
from typing import BinaryIO, Callable, Dict, Generator, List, Optional, Tuple, Set

import elftools
from elftools.elf import elffile

FUNCTION_REGEX = re.compile(
    r'^;; Function (?P<mangle>.*)\s+\((?P<function>\S+)(,.*)?\).*$'
)
CALL_REGEX = re.compile(r'^.*\(call.*"(?P<target>.*)".*$')
SYMBOL_REF_REGEX = re.compile(r'^.*\(symbol_ref[^()]*\("(?P<target>.*)"\).*$')


class RtlFunction(object):
    def __init__(self, name: str, rtl_filename: str, tu_filename: str) -> None:
        self.name = name
        self.rtl_filename = rtl_filename
        self.tu_filename = tu_filename
        self.calls: List[str] = list()
        self.refs: List[str] = list()
        self.sym = None


class SectionAddressRange(object):
    def __init__(self, name: str, addr: int, size: int) -> None:
        self.name = name
        self.low = addr
        self.high = addr + size

    def __str__(self) -> str:
        return '{}: 0x{:08x} - 0x{:08x}'.format(self.name, self.low, self.high)

    def contains_address(self, addr: int) -> bool:
        return self.low <= addr < self.high


TARGET_SECTIONS: Dict[str, List[SectionAddressRange]] = {
    'esp32': [
        SectionAddressRange('.rom.text', 0x40000000, 0x70000),
        SectionAddressRange('.rom.rodata', 0x3ff96000, 0x9018)
    ],
    'esp32s2': [
        SectionAddressRange('.rom.text', 0x40000000, 0x1bed0),
        SectionAddressRange('.rom.rodata', 0x3ffac600, 0x392c)
    ],
    'esp32s3': [
        SectionAddressRange('.rom.text', 0x40000000, 0x568d0),
        SectionAddressRange('.rom.rodata', 0x3ff071c0, 0x8e30)
    ]
}


class Symbol(object):
    def __init__(self, name: str, addr: int, local: bool, filename: Optional[str], section: Optional[str], st_type: Optional[str] = None, bind: Optional[str] = None, origin: Optional[str] = None) -> None:
        self.name = name
        self.addr = addr
        self.local = local
        self.filename = filename
        self.section = section
        self.st_type = st_type
        self.bind = bind
        self.origin = origin
        self.refers_to: List[Symbol] = list()
        self.referred_from: List[Symbol] = list()

    def is_func(self) -> bool:
        return self.st_type == 'STT_FUNC'

    def is_object(self) -> bool:
        return self.st_type in ('STT_OBJECT',)

    def __str__(self) -> str:
        return '{} @0x{:08x} [{}]{} {}{}{}'.format(
            self.name,
            self.addr,
            self.section or 'unknown',
            ' (local)' if self.local else '',
            self.filename or '',
            f' [{self.st_type}]' if self.st_type else '',
            f' <{self.origin}>' if self.origin else ''
        )


class Reference(object):
    def __init__(self, from_sym: Symbol, to_sym: Symbol) -> None:
        self.from_sym = from_sym
        self.to_sym = to_sym

    def __str__(self) -> str:
        return '{} @0x{:08x} ({}) -> {} @0x{:08x} ({})'.format(
            self.from_sym.name,
            self.from_sym.addr,
            self.from_sym.section,
            self.to_sym.name,
            self.to_sym.addr,
            self.to_sym.section
        )


class IgnorePair():
    def __init__(self, pair: str) -> None:
        self.symbol, self.function_call = pair.split('/')


class ElfInfo(object):
    def __init__(self, elf_file: BinaryIO) -> None:
        self.elf_file = elf_file
        self.elf_obj = elffile.ELFFile(self.elf_file)
        self.section_ranges = self._load_sections()
        self.symbols = self._load_symbols()

    def _load_symbols(self) -> List[Symbol]:
        symbols = []
        for s in self.elf_obj.iter_sections():
            if not isinstance(s, elftools.elf.sections.SymbolTableSection):
                continue
            filename = None
            for sym in s.iter_symbols():
                st_info = sym.entry['st_info']
                sym_type = st_info['type']
                bind = st_info['bind']
                if sym_type == 'STT_FILE':
                    filename = sym.name
                    continue
                if sym_type in ['STT_NOTYPE', 'STT_FUNC', 'STT_OBJECT']:
                    local = bind == 'STB_LOCAL'
                    addr = sym.entry['st_value']
                    symbols.append(
                        Symbol(
                            sym.name,
                            addr,
                            local,
                            filename if local else None,
                            self.section_for_addr(addr),
                            sym_type,
                            bind,
                        )
                    )
        return symbols

    def _load_sections(self) -> List[SectionAddressRange]:
        result = []
        for segment in self.elf_obj.iter_segments():
            if segment['p_type'] == 'PT_LOAD':
                for section in self.elf_obj.iter_sections():
                    if not segment.section_in_segment(section):
                        continue
                    result.append(
                        SectionAddressRange(
                            section.name, section['sh_addr'], section['sh_size']
                        )
                    )

        target = os.environ.get('IDF_TARGET')
        if target in TARGET_SECTIONS:
            result += TARGET_SECTIONS[target]

        return result

    def symbols_by_name(self, name: str) -> List['Symbol']:
        res = []
        for sym in self.symbols:
            if sym.name == name:
                res.append(sym)
        return res

    def section_for_addr(self, sym_addr: int) -> Optional[str]:
        for sar in self.section_ranges:
            if sar.contains_address(sym_addr):
                return sar.name
        return None


def load_rtl_file(rtl_filename: str, tu_filename: str, functions: List[RtlFunction], ignore_pairs: List[IgnorePair]) -> None:
    last_function: Optional[RtlFunction] = None
    for line in open(rtl_filename):
        # Find function definition
        match = re.match(FUNCTION_REGEX, line)
        if match:
            function_name = match.group('function')
            last_function = RtlFunction(function_name, rtl_filename, tu_filename)
            functions.append(last_function)
            continue

        if last_function:
            # Find direct function calls
            match = re.match(CALL_REGEX, line)
            if match:
                target = match.group('target')

                # if target matches on of the IgnorePair function_call attributes, remove
                # the last occurrence of the associated symbol from the last_function.refs list.
                call_matching_pairs = [pair for pair in ignore_pairs if pair.function_call == target]
                if call_matching_pairs and last_function and last_function.refs:
                    for pair in call_matching_pairs:
                        ignored_symbols = [ref for ref in last_function.refs if pair.symbol in ref]
                        if ignored_symbols:
                            last_ref = ignored_symbols.pop()
                            last_function.refs = [ref for ref in last_function.refs if last_ref != ref]

                if target not in last_function.calls:
                    last_function.calls.append(target)
                continue

            # Find symbol references
            match = re.match(SYMBOL_REF_REGEX, line)
            if match:
                target = match.group('target')
                if target not in last_function.refs:
                    last_function.refs.append(target)
                continue


def rtl_filename_matches_sym_filename(rtl_filename: str, symbol_filename: str) -> bool:
    # Symbol file names (from ELF debug info) are short source file names, without path: "cpu_start.c".
    # RTL file names are paths relative to the build directory, e.g.:
    # "build/esp-idf/esp_system/CMakeFiles/__idf_esp_system.dir/port/cpu_start.c.234r.expand"
    #
    # The check below may give a false positive if there are two files with the same name in
    # different directories. This doesn't seem to happen in IDF now, but if it does happen,
    # an assert in find_symbol_by_rtl_func should catch this.
    #
    # If this becomes and issue, consider also loading the .map file and using it to figure out
    # which object file was used as the source of each symbol. Names of the object files and RTL files
    # should be much easier to match.
    return os.path.basename(rtl_filename).startswith(symbol_filename)


class SymbolNotFound(RuntimeError):
    pass


def find_symbol_by_name(name: str, elfinfo: ElfInfo, local_func_matcher: Callable[[Symbol], bool]) -> Optional[Symbol]:
    """
    Find an ELF symbol for the given name.
    local_func_matcher is a callback function which checks is the candidate local symbol is suitable.
    """
    syms = elfinfo.symbols_by_name(name)
    if not syms:
        return None
    if len(syms) == 1:
        return syms[0]
    else:
        # There are multiple symbols with a given name. Find the best fit.
        local_candidate = None
        global_candidate = None
        for sym in syms:
            if not sym.local:
                assert not global_candidate  # can't have two global symbols with the same name
                global_candidate = sym
            elif local_func_matcher(sym):
                assert not local_candidate  # can't have two symbols with the same name in a single file
                local_candidate = sym

        # If two symbols with the same name are defined, a global and a local one,
        # prefer the local symbol as the reference target.
        return local_candidate or global_candidate


def match_local_source_func(rtl_filename: str, sym: Symbol) -> bool:
    """
    Helper for match_rtl_funcs_to_symbols, checks if local symbol sym is a good candidate for the
    reference source (caller), based on the RTL file name.
    """
    assert sym.filename  # should be set for local functions
    return rtl_filename_matches_sym_filename(rtl_filename, sym.filename)


def match_local_target_func(rtl_filename: str, sym_from: Symbol, sym: Symbol) -> bool:
    """
    Helper for match_rtl_funcs_to_symbols, checks if local symbol sym is a good candidate for the
    reference target (callee or referenced data), based on RTL filename of the source symbol
    and the source symbol itself.
    """
    assert sym.filename  # should be set for local functions
    if sym_from.local:
        # local symbol referencing another local symbol
        return sym_from.filename == sym.filename
    else:
        # global symbol referencing a local symbol;
        # source filename is not known, use RTL filename as a hint
        return rtl_filename_matches_sym_filename(rtl_filename, sym.filename)


def match_rtl_funcs_to_symbols(rtl_functions: List[RtlFunction], elfinfo: ElfInfo) -> Tuple[List[Symbol], List[Reference]]:
    symbols: List[Symbol] = []
    refs: List[Reference] = []

    # General idea:
    # - iterate over RTL functions.
    #   - for each RTL function, find the corresponding symbol
    #   - iterate over the functions and variables referenced from this RTL function
    #     - find symbols corresponding to the references
    #     - record every pair (sym_from, sym_to) as a Reference object

    for source_rtl_func in rtl_functions:
        maybe_sym_from = find_symbol_by_name(source_rtl_func.name, elfinfo, partial(match_local_source_func, source_rtl_func.rtl_filename))
        if maybe_sym_from is None:
            # RTL references a symbol, but the symbol is not defined in the generated object file.
            # This means that the symbol was likely removed (or not included) at link time.
            # There is nothing we can do to check section placement in this case.
            continue
        sym_from = maybe_sym_from

        if sym_from not in symbols:
            symbols.append(sym_from)

        for target_rtl_func_name in source_rtl_func.calls + source_rtl_func.refs:
            if '*.LC' in target_rtl_func_name:  # skip local labels
                continue

            maybe_sym_to = find_symbol_by_name(target_rtl_func_name, elfinfo, partial(match_local_target_func, source_rtl_func.rtl_filename, sym_from))
            if not maybe_sym_to:
                # This may happen for a extern reference in the RTL file, if the reference was later removed
                # by one of the optimization passes, and the external definition got garbage-collected.
                # TODO: consider adding some sanity check that we are here not because of some bug in
                # find_symbol_by_name?..
                continue
            sym_to = maybe_sym_to

            sym_from.refers_to.append(sym_to)
            sym_to.referred_from.append(sym_from)
            refs.append(Reference(sym_from, sym_to))
            if sym_to not in symbols:
                symbols.append(sym_to)

    return symbols, refs


def _classify_placement(sym: Symbol, iram_like: List[str], dram_like: List[str]) -> Optional[str]:
    sec = sym.section or ''
    # IRAM code placement: any function in text/iram sections
    if sym.is_func() and any(sec.startswith(s) for s in iram_like + ['.text', '.iram', '.iram.text', '.iram0.text']):
        return 'IRAM_LOADER'
    # DRAM data/rodata/bss placement
    if (sym.is_object() or sec.startswith('.rodata') or sec.startswith('.bss') or '.rodata' in sec or '.bss' in sec or '.data' in sec):
        if any(sec.startswith(s) for s in dram_like + ['.dram', '.dram0', '.dram0.bss', '.dram0.data', '.dram0.rodata']):
            return 'DRAM_LOADER'
    return None


def _in_loader_section(sec: Optional[str]) -> bool:
    return bool(sec) and ('loader' in sec)


def _origin_pattern(origin: str) -> str:
    import os as _os
    # Try archive(object) form
    if '(' in origin and ')' in origin:
        arch = _os.path.basename(origin.split('(')[0])
        inside = origin[origin.find('(')+1:origin.rfind(')')]
        obj = _os.path.basename(inside)
        stem = obj.split('.')[0]
        return f'*{arch}:{stem}.*'
    # Try plain object path
    base = _os.path.basename(origin)
    if base.endswith(('.o', '.obj')):
        stem = base.split('.')[0]
        return f'*{stem}.*'
    # Try plain source file name
    if base.endswith(('.c', '.cpp', '.cc', '.S', '.s')):
        stem = base.rsplit('.', 1)[0]
        return f'*{stem}.*'
    # Fallback
    return f'*{base}*'


def build_origin_map(map_path: str, symbol_names: Set[str]) -> Dict[str, str]:
    """
    Parse GNU ld map to associate symbols with their archive(object) origin.
    Only records origins for symbols in symbol_names.
    """
    origins: Dict[str, str] = {}
    origin_re = re.compile(r'(?P<origin>\S+\([^\)]+\)|\S+\.(?:o|obj))\s*$')
    sym_line_re = re.compile(r'^\s*0x[0-9A-Fa-f]+\s+(?P<symbol>\S+)\s*$')
    current_origin: Optional[str] = None
    try:
        with open(map_path, 'r', encoding='utf-8', errors='ignore') as mf:
            for raw in mf:
                line = raw.rstrip()
                m_origin = origin_re.search(line)
                if m_origin and (line.lstrip().startswith('.') or '0x' in line):
                    current_origin = m_origin.group('origin')
                    continue
                m_sym = sym_line_re.match(line)
                if m_sym and current_origin:
                    sym = m_sym.group('symbol')
                    if sym in symbol_names and sym not in origins:
                        origins[sym] = current_origin
    except OSError:
        pass
    return origins


def compute_minset(symbols: List[Symbol], refs: List[Reference], root_names: List[str], iram_like: List[str], dram_like: List[str]) -> List[Tuple[str, Symbol]]:
    # Build adjacency from existing references
    adj: Dict[Symbol, List[Symbol]] = {}
    by_name: Dict[str, List[Symbol]] = {}
    for s in symbols:
        by_name.setdefault(s.name, []).append(s)
    for r in refs:
        adj.setdefault(r.from_sym, []).append(r.to_sym)

    # Initialize frontier with all symbol objects that match roots
    sym_queue: List[Symbol] = []
    for rn in root_names:
        sym_queue.extend(by_name.get(rn, []))

    visited: Set[Tuple[str, int]] = set()
    required: List[Tuple[str, Symbol]] = []

    while sym_queue:
        cur = sym_queue.pop(0)
        key = (cur.name, cur.addr)
        if key in visited:
            continue
        visited.add(key)

        # Classify current symbol placement
        placement = _classify_placement(cur, iram_like, dram_like)
        if placement:
            required.append((placement, cur))

        # Only enqueue callees when current is a function
        if cur.is_func():
            for nxt in adj.get(cur, []):
                nkey = (nxt.name, nxt.addr)
                if nkey not in visited:
                    sym_queue.append(nxt)

    return required


def get_symbols_and_refs(rtl_list: List[str], elf_file: BinaryIO, ignore_pairs: List[IgnorePair], map_path: Optional[str] = None) -> Tuple[List[Symbol], List[Reference]]:
    elfinfo = ElfInfo(elf_file)

    rtl_functions: List[RtlFunction] = []
    for file_name in rtl_list:
        load_rtl_file(file_name, file_name, rtl_functions, ignore_pairs)

    symbols, refs = match_rtl_funcs_to_symbols(rtl_functions, elfinfo)

    # Optionally annotate symbols with origin from map
    if map_path:
        symbol_names: Set[str] = {s.name for s in symbols}
        origin_map = build_origin_map(map_path, symbol_names)
        for s in symbols:
            s.origin = origin_map.get(s.name)

    return symbols, refs


def list_refs_from_to_sections(refs: List[Reference], from_sections: List[str], to_sections: List[str]) -> int:
    found = 0
    for ref in refs:
        if (not from_sections or ref.from_sym.section in from_sections) and \
           (not to_sections or ref.to_sym.section in to_sections):
            print(str(ref))
            found += 1
    return found


def find_files_recursive(root_path: str, ext: str) -> Generator[str, None, None]:
    for root, _, files in os.walk(root_path):
        for basename in files:
            if basename.endswith(ext):
                filename = os.path.join(root, basename)
                yield filename


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--rtl-list',
        help='File with the list of RTL files',
        type=argparse.FileType('r'),
    )
    parser.add_argument(
        '--rtl-dirs', help='comma-separated list of directories where to look for RTL files, recursively'
    )
    parser.add_argument(
        '--elf-file',
        required=True,
        help='Program ELF file',
        type=argparse.FileType('rb'),
    )
    action_sub = parser.add_subparsers(dest='action')
    find_refs_parser = action_sub.add_parser(
        'find-refs',
        help='List the references coming from a given list of source sections'
             'to a given list of target sections.',
    )
    find_refs_parser.add_argument(
        '--from-sections', help='comma-separated list of source sections'
    )
    find_refs_parser.add_argument(
        '--to-sections', help='comma-separated list of target sections'
    )
    find_refs_parser.add_argument(
        '--ignore-symbols', help='comma-separated list of symbol/function_name pairs. \
                                  This will force the parser to ignore the symbol preceding the call to function_name'
    )
    find_refs_parser.add_argument(
        '--exit-code',
        action='store_true',
        help='If set, exits with non-zero code when any references found',
    )
    action_sub.add_parser(
        'all-refs',
        help='Print the list of all references',
    )

    # New: subcommand for evaluating the minimum set for iram_loader_seg and dram_loader_seg
    minset_parser = action_sub.add_parser(
        'find-loader-minset',
        help='Compute minimal IRAM/DRAM LOADER symbol set reachable from --roots.'
    )
    minset_parser.add_argument('--roots', required=True, help='Comma-separated loader root functions')
    minset_parser.add_argument('--iram-sections', default='', help='Comma-separated IRAM-like section prefixes')
    minset_parser.add_argument('--dram-sections', default='', help='Comma-separated DRAM-like section prefixes')
    minset_parser.add_argument('--map', required=True, help='Path to linker map file for origin grouping')
    minset_parser.add_argument('--summary', action='store_true', help='Emit grouped summary by origin')
    minset_parser.add_argument('--show-all', action='store_true', help='Print all required symbols, not only outplacers')

    args = parser.parse_args()
    if args.rtl_list:
        with open(args.rtl_list, 'r') as rtl_list_file:
            rtl_list = [line.strip() for line in rtl_list_file]
    else:
        if not args.rtl_dirs:
            raise RuntimeError('Either --rtl-list or --rtl-dirs must be specified')
        rtl_dirs = args.rtl_dirs.split(',')
        rtl_list = []
        for dir in rtl_dirs:
            rtl_list.extend(list(find_files_recursive(dir, '.expand')))

    if not rtl_list:
        raise RuntimeError('No RTL files specified')

    ignore_pairs = []
    ignore_symbols_arg = getattr(args, 'ignore_symbols', None)
    for pair in (ignore_symbols_arg.split(',') if ignore_symbols_arg else []):
        ignore_pairs.append(IgnorePair(pair))

    # Compute symbols and refs once and reuse across actions
    symbols: List[Symbol] = []
    refs: List[Reference] = []
    if args.action in ('find-refs', 'all-refs', 'find-loader-minset'):
        symbols, refs = get_symbols_and_refs(
            rtl_list, args.elf_file, ignore_pairs, getattr(args, 'map', None)
        )

    if args.action == 'find-refs':
        from_sections = args.from_sections.split(',') if args.from_sections else []
        to_sections = args.to_sections.split(',') if args.to_sections else []
        found = list_refs_from_to_sections(
            refs, from_sections, to_sections
        )
        if args.exit_code and found:
            raise SystemExit(1)
    elif args.action == 'all-refs':
        for r in refs:
            print(str(r))
    elif args.action == 'find-loader-minset':
        # Prepare inputs
        roots = [r for r in (p.strip() for p in args.roots.split(',')) if r]
        iram_like = [p.strip() for p in (args.iram_sections or '').split(',') if p.strip()]
        dram_like = [p.strip() for p in (args.dram_sections or '').split(',') if p.strip()]
        # Use preloaded symbols + refs to compute minimal set
        required = compute_minset(symbols, refs, roots, iram_like, dram_like)

        if not required:
            print('No required loader symbols found.')
            raise SystemExit(0)

        def origin_for(sym: Symbol) -> str:
            return sym.origin or sym.filename or '<unknown-origin>'

        # Select outplacers if not show-all
        def sort_key_item(item: Tuple[str, Symbol]):
            placement, s = item
            return (
                0 if placement == 'IRAM_LOADER' else 1,
                origin_for(s),
                s.section or '',
                s.name or '',
                s.addr,
            )

        def print_csv(items: List[Tuple[str, Symbol]]):
            print('placement,symbol,section,address,type,origin,filename')
            for placement, s in sorted(items, key=sort_key_item):
                stype = 'FUNC' if s.is_func() else 'OBJECT'
                print(f'{placement},{s.name},{s.section},0x{s.addr:08x},{stype},{origin_for(s)},{s.filename or ""}')

        outplacers: List[Tuple[str, Symbol]] = []
        for placement, s in required:
            if placement in ('IRAM_LOADER', 'DRAM_LOADER') and not _in_loader_section(s.section):
                outplacers.append((placement, s))

        objs: List[Tuple[str, Symbol]] = required if args.show_all else outplacers

        if args.summary:
            groups: Dict[Tuple[str, str], List[Symbol]] = {}
            for placement, s in objs:
                key = (placement, origin_for(s))
                groups.setdefault(key, []).append(s)
            if groups:
                print('placement,origin,count,symbols')
                ordered = sorted(groups.items(), key=lambda x: (0 if x[0][0]=='IRAM_LOADER' else 1, x[0][1]))
                for (placement, origin), items in ordered:
                    names = sorted({i.name for i in items})
                    print(f'{placement},{origin},{len(names)},{";".join(names)}')
                    patt = _origin_pattern(origin)
                    print(f'{patt}(.literal .text .literal.* .text.* .iram0 .iram0.* .iram1 .iram1.*)')
                    print(f'{patt}(.sbss .sbss.* .sbss2 .sbss2.* .bss .bss.*)')
                    print(f'{patt}(.data .data.* .rodata .rodata.* .sdata .sdata.* .sdata2 .sdata2.* .srodata .srodata.* .dram*)')
        else:
            print_csv(objs)

        # Exit code: non-zero if outplacers exist
        raise SystemExit(1 if outplacers and not args.show_all else 0)


if __name__ == '__main__':
    main()
