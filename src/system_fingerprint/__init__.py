import datetime
import os
import platform
import rclpy
from rcl_interfaces.srv import ListParameters, GetParameters
from ros2action.api import get_action_names_and_types
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import get_action_client_info, get_action_server_info
from ros2node.api import get_service_client_info, get_service_server_info
from ros2node.api import get_publisher_info, get_subscriber_info
from ros2param.api import get_value
from ros2service.api import get_service_names_and_types
from ros2topic.api import get_topic_names_and_types

from .workspace import workspace

main_node = NodeStrategy({}).__enter__()

specific_platform_methods = {
    'Linux': platform.libc_ver,
    'Windows': platform.win32_ver,
    'Darwin': platform.mac_ver,
}


def _succeed(args):
    code, msg, val = args
    if code != 1:
        return None
    else:
        return val


def _get_nodes():
    for name in get_node_names(node=main_node, include_hidden_nodes=True):
        yield name.full_name


def system():
    d = {}
    d['architecture'] = '/'.join(platform.architecture())
    for key in ['machine', 'node', 'platform', 'processor', 'python_version', 'release', 'system', 'version']:
        d[key] = getattr(platform, key)()

    if d['system'] in specific_platform_methods:
        d['info'] = '/'.join(specific_platform_methods[d['system']]())

    if d['system'] == 'Linux':
        try:
            linux = {}
            for line in open('/etc/os-release').readlines():
                a, _, b = line.strip().partition('=')
                linux[a] = b
            d['linux_codename'] = linux['PRETTY_NAME']
        except OSError:
            pass

    now = datetime.datetime.now()
    d['timestamp'] = now.timestamp()
    d['time'] = now

    return d


def environmental_variables():
    d = {}
    for k, v in os.environ.items():
        for prefix in ['ROS_', 'RCUTILS_', 'COLCON_', 'AMENT_']:
            if k.startswith(prefix):
                d[k] = v
    return d


def easy_client_call(type_, name, timeout_sec=1.0, **kwargs):
    client = main_node.create_client(type_, name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return
    request = type_.Request()
    for k, v in kwargs.items():
        setattr(request, k, v)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(main_node, future)
    return future.result()


def parameters():
    d = {}
    for node_name in _get_nodes():
        node_d = {}
        param_list_resp = easy_client_call(ListParameters, f'{node_name}/list_parameters')
        if not param_list_resp:
            continue
        parameter_names = param_list_resp.result.names
        get_param_resp = easy_client_call(GetParameters, f'{node_name}/get_parameters', names=parameter_names)

        for name, pvalue in zip(parameter_names, get_param_resp.values):
            node_d[name] = get_value(parameter_value=pvalue)

        d[node_name] = node_d
    return d


def nodes():
    d = {}
    for node_name in _get_nodes():
        node_d = {}
        for name, method in [('pubs', get_publisher_info),
                             ('subs', get_subscriber_info),
                             ('srvs', get_service_server_info),
                             ('srv_clients', get_service_client_info),
                             ('actions', get_action_server_info),
                             ('action_clients', get_action_client_info),
                             ]:
            results = method(node=main_node, remote_node_name=node_name, include_hidden=True)
            if not results:
                continue
            node_d[name] = [topic.name for topic in results]
        d[node_name] = node_d
    return d


def _get_type_dict(method, **kwargs):
    d = {}
    for name, type_ in method(node=main_node, **kwargs):
        if len(type_) > 1:
            d[name] = type_
        elif type_:
            d[name] = type_[0]

    return d


def topics():
    return _get_type_dict(get_topic_names_and_types, include_hidden_topics=True)


def services():
    return _get_type_dict(get_service_names_and_types, include_hidden_services=True)


def actions():
    return _get_type_dict(get_action_names_and_types)


modules = [system, environmental_variables, parameters, nodes, topics, services, actions, workspace]
