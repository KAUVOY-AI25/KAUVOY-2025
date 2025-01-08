## *********************************************************
##
## File autogenerated for the velodyne_pointcloud package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'min_range', 'type': 'double', 'default': 0.9, 'level': 0, 'description': 'min range to publish', 'min': 0.1, 'max': 10.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'max_range', 'type': 'double', 'default': 130.0, 'level': 0, 'description': 'max range to publish', 'min': 0.1, 'max': 200.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'view_direction', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'angle defining the center of view', 'min': -3.141592653589793, 'max': 3.141592653589793, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'view_width', 'type': 'double', 'default': 6.283185307179586, 'level': 0, 'description': 'angle defining the view width', 'min': 0.0, 'max': 6.283185307179586, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'fixed_frame', 'type': 'str', 'default': '', 'level': 0, 'description': 'world fixed frame used for ego motion compensation (leave empty if no ego motion compensation is required)', 'min': '', 'max': '', 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'target_frame', 'type': 'str', 'default': '', 'level': 0, 'description': 'output frame of the final PointCloud2 msg (leave empty to use the frame specified in the original scan msg)', 'min': '', 'max': '', 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'organize_cloud', 'type': 'bool', 'default': False, 'level': 0, 'description': 'organize cloud', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

