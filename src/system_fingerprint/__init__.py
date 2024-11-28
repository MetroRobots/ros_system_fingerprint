import datetime
import os
import platform
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.time import Time
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
from tf2_msgs.msg import TFMessage
import time

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


def tf_tree(listen_duration=5.0):
    frame_names = set()
    latest = {}
    start = {}
    counts = {}
    static_keys = set()

    def tf_callback(msg, static=False):
        for transform in msg.transforms:
            frame_names.add(transform.header.frame_id)
            frame_names.add(transform.child_frame_id)

            key = transform.header.frame_id, transform.child_frame_id
            latest[key] = transform
            if key in counts:
                counts[key] += 1
            else:
                counts[key] = 1
                start[key] = transform.header.stamp

            if static:
                static_keys.add(key)

    tf_sub = main_node.create_subscription(TFMessage, '/tf', tf_callback, 10)
    latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    static_sub = main_node.create_subscription(TFMessage, '/tf_static',
                                               lambda msg: tf_callback(msg, True),
                                               latching_qos)

    t0 = time.time()
    while time.time() < t0 + listen_duration:
        rclpy.spin_once(main_node, timeout_sec=listen_duration)

    tf_sub.destroy()
    static_sub.destroy()

    parentless = set(frame_names)
    for key in latest:
        parentless.remove(key[1])

    info = {}
    queue = []

    for frame in parentless:
        queue.append((frame, info, None))

    while queue:
        frame, d, parent_frame = queue.pop(0)
        f_info = {}

        for parent, child in latest:
            if parent == frame:
                queue.append((child, f_info, frame))

        key = parent_frame, frame
        if key in static_keys:
            f_info['static'] = True
        elif key in counts:
            stats = {}
            f_info['stats'] = stats
            stats['count'] = counts[key]

            start_t = Time.from_msg(start[key])
            end_t = Time.from_msg(latest[key].header.stamp)
            stats['window'] = round((end_t - start_t).nanoseconds / 1e9, 3)
            stats['freq'] = round(stats['count'] / stats['window'], 3)

        if key in latest:
            for field, subfields in [('translation', 'xyz'), ('rotation', 'xyzw')]:
                value = getattr(latest[key].transform, field)

                f_info[field] = {}
                for subfield in subfields:
                    f_info[field][subfield] = getattr(value, subfield)

        d[frame] = f_info

    return info


modules = [system, environmental_variables, parameters, nodes, topics, services, actions, workspace, tf_tree]
