#!/usr/bin/env python
##
# @file onnx_rename_layer.py
# @author Constantin Blessing
# @brief This file implements the renaming of layers on the onnx file.
# @version 1.0
# @date 2023-08-17

import argparse
import onnx
import pathlib


def onnx_rename_layer(file: str, output: str, layer: str, name) -> None:
    print(f'opening \'{file}\'')
    model = onnx.load(file)

    for i in range(len(model.graph.node)):
        for j in range(len(model.graph.node[i].input)):
            if model.graph.node[i].input[j] == layer:
                print(f'\trenaming input \'{", ".join(model.graph.node[i].input)}\' of \'{model.graph.node[i].name}\' to \'{name}\'')
                model.graph.node[i].input[j] = name

        for j in range(len(model.graph.node[i].output)):
            if model.graph.node[i].output[j] == layer:
                print(f'\trenaming output \'{", ".join(model.graph.node[i].output)}\' of \'{model.graph.node[i].name}\' to \'{name}\'')
                model.graph.node[i].output[j] = name

    for i in range(len(model.graph.input)):
        if model.graph.input[i].name == layer:
            print(f'\trenaming \'{model.graph.input[i].name}\' to \'{name}\'')
            model.graph.input[i].name = name

    for i in range(len(model.graph.output)):
        if model.graph.output[i].name == layer:
            print(f'\trenaming \'{model.graph.output[i].name}\' to \'{name}\'')
            model.graph.output[i].name = name

    print(f'saving changed file to \'{output}\'')
    onnx.save(model, output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('onnx_rename_layer', description='renames a single layer inside the specified ONNX file')
    parser.add_argument('file', type=pathlib.Path, help='ONNX file to open for layer renaming')
    parser.add_argument('-o', '--output', type=pathlib.Path, help="output file name if overriding is not desired")
    parser.add_argument('--layer', required=True, help='name of the layer to rename')
    parser.add_argument('--name', required=True, help='new name for the layer')
    args = parser.parse_args()

    # Implementation based on https://gist.github.com/Norod/610ee5e70791c2cecee8980c31711764.
    onnx_rename_layer(args.file, args.file if args.output is None else args.output, args.layer, args.name)
