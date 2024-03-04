import argparse
import graphviz
import pathlib
import tempfile
import yaml


def get_graph(contents, ignore_hidden=True, ignore_debug=True):
    c = 0
    dot = graphviz.Digraph()

    topics = {}

    for name, node in contents.get('nodes', {}).items():
        if ignore_hidden and len(name) > 1 and name[1] == '_':
            continue
        dot_name = f'n{c}'
        c += 1
        dot.node(dot_name, name)

        for key, is_forward in [('pubs', True), ('subs', False)]:
            for topic in node.get(key, []):
                if ignore_debug and topic in ['/rosout', '/parameter_events']:
                    continue
                if topic not in topics:
                    topics[topic] = f't{c}'
                    c += 1
                    text = topic
                    if topic in contents.get('topics', {}):
                        text += '\n'
                        text += contents['topics'][topic]
                    dot.node(topics[topic], text, shape='box')
                if is_forward:
                    dot.edge(dot_name, topics[topic])
                else:
                    dot.edge(topics[topic], dot_name)

    return dot


def visualize(contents):
    dot = get_graph(contents)
    with tempfile.NamedTemporaryFile(suffix='.png') as f:
        try:
            dot.render(outfile=f.name)
            dot.view(f.name)
        except KeyboardInterrupt:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('fingerprint_path', type=pathlib.Path, default='fingerprint.yaml', nargs='?')
    args = parser.parse_args(argv)

    visualize(yaml.safe_load(open(args.fingerprint_path)))
