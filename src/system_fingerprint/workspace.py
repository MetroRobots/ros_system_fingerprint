import os
import pathlib
import git


def _get_parent_dirs(cur_dir):
    folder = cur_dir.resolve()
    while folder:
        yield folder
        if folder.parent == folder:
            return
        else:
            folder = folder.parent


def get_workspace_root(cur_dir=pathlib.Path('.')):
    prefix_path = os.environ.get('COLCON_PREFIX_PATH')
    if prefix_path:
        first = prefix_path.split(':')[0]
        install_dir = pathlib.Path(first)
        return 'colcon', install_dir.parent

    return 'colcon', None


def get_git_repos(root):
    queue = [root]
    repos = []
    while queue:
        folder = queue.pop(0)
        if (folder / '.git').exists():
            repos.append(folder)
            continue

        for new_path in folder.iterdir():
            if new_path.is_dir():
                queue.append(new_path)
    return repos


def workspace():
    d = {}
    build_tool, workspace_root = get_workspace_root()
    d['build_tool'] = build_tool
    d['folder'] = str(workspace_root)
    d['repos'] = {}
    for repo_folder in get_git_repos(workspace_root / 'src'):
        rd = {}
        repo = git.Repo(repo_folder)
        rd['remotes'] = {rem.name: rem.url for rem in repo.remotes}
        rd['hash'] = repo.head.object.hexsha
        rd['branch'] = repo.active_branch.name
        rd['folder'] = str(repo_folder.relative_to(workspace_root))

        d['repos'][repo_folder.stem] = rd
    return d
