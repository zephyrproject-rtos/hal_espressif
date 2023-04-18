#!/usr/bin/env python.
# Copyright (c) 2022 Espressif Systems (Shanghai) PTE LTD.
# Distributed under the terms of Apache License v2.0 found in the top-level LICENSE file.

import argparse
import sys

def bin2c_array(data, var_name='esp32_net_fw_array'):
    out = []
    out.append('const unsigned char {var_name}[] = {{'.format(var_name=var_name))
    l = [ data[i:i+12] for i in range(0, len(data), 12) ]
    for i, x in enumerate(l):
        line = ', '.join([ '0x{val:02x}'.format(val=c) for c in x ])
        out.append('  {line}{end_comma}'.format(line=line, end_comma=',' if i<len(l)-1 else ''))
    out.append('};')
    out.append('const unsigned int {var_name}_size = {data_len};'.format(var_name=var_name, data_len=len(data)))
    return '\n'.join(out)

def main():
    parser = argparse.ArgumentParser(description='Binary to C array generator')
    parser.add_argument('-i', '--input', required=True , help='.bin input file')
    parser.add_argument('-o', '--out', required=True , help='c file name')
    parser.add_argument('-a', '--array', required=True , help='c array variable name')

    args = parser.parse_args()
    if not args:
        return 1

    with open(args.input, 'rb') as f:
        data = f.read()

    out = bin2c_array(data, args.array)
    with open(args.out, 'w') as f:
        f.write(out)

    return 0

if __name__ == '__main__':
    sys.exit(main())
