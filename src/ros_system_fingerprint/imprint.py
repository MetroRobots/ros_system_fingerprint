import argparse
import pathlib
import yaml

from ros_system_fingerprint import modules


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--print', action='store_true')
    parser.add_argument('output_path', type=pathlib.Path,
                        nargs='?', default='fingerprint.yaml')
    args = parser.parse_args()

    D = {}
    for module in modules:
        try:
            D[module.__name__] = module()
        except ConnectionRefusedError:
            pass

    contents = yaml.safe_dump(D)

    if args.print:
        print(contents)

    print(f'saving fingerprint to {args.output_path}')
    with open(args.output_path, 'w') as f:
        f.write(contents)
