import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import argparse
import datetime
import glob
import re
import time
from pathlib import Path
from general.utilities.file_utils import read_files
from general.utilities.eval_utils import box_to_dict

import numpy as np
import torch
from tensorboardX import SummaryWriter

from general.config.config import cfg, cfg_from_list, cfg_from_yaml_file, log_config_to_file
from general.utilities import common_utils
from general.utilities import eval_utils
from object_detection.datasets import build_dataloader
from object_detection.detectors3d import build_network

try:
    import object_detection.pycenterpoint as cp
    # import pycenterpoint as cp

    print("Pybind imported!!")
except:
    print("No Pybind!!")

import open3d
from tools.visualization.visual_utils import open3d_vis_utils as V


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--onnx_dir', type=str, default=None, required=False,
                        help='a directory with onnx, config, trt files ')
    parser.add_argument('--npy_dir', type=str,
                        default='./data/waymo/waymo_processed_data_v0_5_0/segment-10017090168044687777_6380_000_6400_000_with_camera_labels',
                        help='a directory with npy files for inference')

    args = parser.parse_args()
    return args


def main():
    args = parse_config()

    npy_dir = Path(args.npy_dir).absolute()
    print(npy_dir)

    npy_file_list = read_files(path=npy_dir, extension='*.npy')
    npy_file_list.sort()
    print(len(npy_file_list))
    print(npy_file_list)

    if len(npy_file_list) == 0:
        print("No npy files")
        exit()

    if args.onnx_dir is None:
        onnx_dir = Path(os.path.abspath(__file__)).parent.parent / 'onnx'
    else:
        onnx_dir = Path(args.onnx_dir)

    print('onnx_dir: ', onnx_dir)

    model_path = "{}/model.trt".format(onnx_dir)
    config_path = "{}/config.yaml".format(onnx_dir)

    print('config_path: ', config_path)
    print('model_path: ', model_path)

    model = cp.PyCenterPoint(config_path, model_path)
    print("**********************************************************************")
    print("************************** load tensorRT *****************************")
    print("**********************************************************************")

    for pc_path in npy_file_list:
        pc = np.load(pc_path)[:, :4]
        pc[:, 3] = np.tanh(pc[:, 3])
        print(pc.shape)
        print(pc_path)
        trt_boxes = model.forward(pc)
        pred_dicts = box_to_dict(trt_boxes)

        print(pred_dicts)
        V.draw_scenes(
            points=pc,
            ref_boxes=pred_dicts[0]['pred_boxes'],
            ref_scores=pred_dicts[0]['pred_scores'],
            ref_labels=pred_dicts[0]['pred_labels']
        )

    pass


if __name__ == '__main__':
    main()
