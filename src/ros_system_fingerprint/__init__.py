import datetime
import os
import rospy
import platform
from rosnode import ID
import roslib.scriptutil

master = roslib.scriptutil.get_master()

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
        if k.startswith('ROS_'):
            d[k] = v
    return d


def parameters():
    return rospy.get_param('/')


def nodes():
    d = {}
    state = _succeed(master.getSystemState(ID))
    if not state:
        return d
    for name, info in zip(['pubs', 'subs', 'srvs'], state):
        for topic, nodes in sorted(info):
            for node in nodes:
                if node not in d:
                    d[node] = {}
                if name not in d[node]:
                    d[node][name] = []
                if topic not in d[node][name]:
                    d[node][name].append(topic)
    return d


def topics():
    d = {}
    pub_topics = _succeed(master.getPublishedTopics(ID, '/'))
    if not pub_topics:
        return d

    return {a: b for (a, b) in pub_topics}


modules = [system, environmental_variables, parameters, nodes, topics]
