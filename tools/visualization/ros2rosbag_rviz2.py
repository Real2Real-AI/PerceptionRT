import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import argparse
from general.config.config import cfg, cfg_from_yaml_file
from general.utilities import common_utils
from tools.ros2.centerpoint_node import ros_python_main

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/waymo_models/centerpoint_pillar_inference.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default='../ckpts/waymo_iou_branch.pth',
                        help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.npy', help='specify the extension of your point cloud data file')
    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

if __name__ == '__main__':
    args, cfg = parse_config()
    logger = common_utils.create_logger()
    ros_python_main(cfg=cfg, args=args, logger=logger)
