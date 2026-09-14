#!/usr/bin/env python3
"""Check isolation, active launch parameters, plugins and Python/XML syntax."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'scripts'))
from session import validate_environment


def main():
    validate_environment()
    findings={'python_files':0,'xml_files':0,'binaries':[]}
    for package in ('go2_exploration','go2_explore_lite','go2_exploration_safety'):
        root=WS/'src'/package
        for file in root.rglob('*.py'):
            ast.parse(file.read_text(),filename=str(file));findings['python_files']+=1
        for pattern in ('*.launch','*.launch.xml','package.xml','*plugins.xml'):
            for file in root.rglob(pattern): ET.parse(file);findings['xml_files']+=1
        for file in root.rglob('*'):
            if file.is_symlink() and WS not in file.resolve().parents:
                raise RuntimeError('External package symlink: '+str(file))
    for path in sorted((WS/'devel/lib').rglob('*')):
        if not path.is_file():continue
        with path.open('rb') as stream:
            if stream.read(4)!=b'\x7fELF':continue
        name=str(path.relative_to(WS/'devel/lib'))
        output=subprocess.check_output(['ldd',str(path)],text=True)
        if 'not found' in output or '/go2_nav_ws/' in output:
            raise RuntimeError('Library isolation failed: '+name+'\n'+output)
        findings['binaries'].append(name)
    for real in ('false','true'):
        command=['roslaunch','--dump-params','go2_exploration','go2_exploration.launch','use_real_sdk:='+real]
        dumped=subprocess.check_output(command,text=True,stderr=subprocess.PIPE)
        params=yaml.safe_load(dumped)
        assert params['/go2_map_builder/output_path'].startswith(str(WS/'maps')+'/')
        assert params['/move_base/global_costmap/static_layer/map_topic']=='/nav_static_map'
        assert params['/move_base/global_costmap/footprint']==[[.35,.155],[.35,-.155],[-.35,-.155],[-.35,.155]]
        assert params['/mid360_mount/base_link_to_lidar_link/pitch_deg']==39.
        assert params['/go2_velocity_shaper/max_vx']==.30
        assert params['/go2_velocity_shaper/max_wz']==.50
        assert params['/go2_velocity_shaper/allow_reverse'] is False
        assert params['/go2_terrain_guard/output/publish_debug_clouds'] is True
        assert params['/go2_terrain_guard/health/require_ground_geometry'] is False
        assert params['/exploration/terrain_geometry_checks'] is False
        assert params['/go2_velocity_shaper/require_terrain_health'] is True
        assert params['/move_base/local_costmap/obstacle_layer/terrain_clearing/topic']=='/exploration/self_filter/clearing_points'
        assert '/go2_exploration_sensor_guard/self_filter/enabled' in params
        assert params['/move_base/local_costmap/obstacle_layer/terrain_marking/topic']=='/terrain/obstacle_points'
        assert params['/move_base/local_costmap/obstacle_layer/terrain_marking/observation_persistence']==0.
        assert params['/point_filter_num']==1
        assert params['/go2_exploration_odom_health/require_sensor_health'] is True
        assert params['/exploration_camera/realsense2_camera/enable_gyro'] is False
        assert not any('ndt_localizer' in key or 'map_server' in key for key in params)
        output=WS/'artifacts'/('launch_'+('real' if real=='true' else 'mock')+'_params.yaml')
        output.write_text(dumped)
    findings['passed']=True
    (WS/'artifacts/workspace_validation.json').write_text(json.dumps(findings,indent=2))
    print(json.dumps(findings,indent=2))


if __name__=='__main__': main()
