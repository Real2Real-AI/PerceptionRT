import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import datetime
import glob
import re
import time
from pathlib import Path
from general.utilities.file_utils import read_files

import numpy as np
import torch
from tensorboardX import SummaryWriter

from general.config.config import cfg, cfg_from_list, cfg_from_yaml_file, log_config_to_file
from general.utilities import common_utils
from general.utilities import eval_utils
from object_detection.datasets import build_dataloader
from object_detection.detectors3d import build_network

try:
    import pycenterpoint as cp

    print("Pybind imported!!")
except:
    print("No Pybind!!")

#################33


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    # parser.add_argument('--cfg_file', type=str, default=None, help='specify the config for inference')
    # parser.add_argument('--batch_size', type=int, default=None, required=False, help='batch size for training')
    # parser.add_argument('--workers', type=int, default=4, help='number of workers for dataloader')
    # parser.add_argument('--extra_tag', type=str, default='default', help='extra tag for this experiment')
    parser.add_argument('--onnx_dir', type=str, default=None, required=False, help='a directory with onnx, config, trt files ')
    parser.add_argument('--npy_dir', type=str, default='./data/waymo/waymo_processed_data_v0_5_0/segment-10017090168044687777_6380_000_6400_000_with_camera_labels', help='a directory with npy files for inference')
    # parser.add_argument('--pretrained_model', type=str, default=None, help='pretrained_model')
    # parser.add_argument('--launcher', choices=['none', 'pytorch', 'slurm'], default='none')
    # parser.add_argument('--tcp_port', type=int, default=18888, help='tcp port for distrbuted training')
    # parser.add_argument('--local_rank', type=int, default=None, help='local rank for distributed training')
    # parser.add_argument('--set', dest='set_cfgs', default=None, nargs=argparse.REMAINDER,
    #                     help='set extra config keys if needed')
    # parser.add_argument('--max_waiting_mins', type=int, default=30, help='max waiting minutes')
    # parser.add_argument('--start_epoch', type=int, default=0, help='')
    # parser.add_argument('--eval_tag', type=str, default='default', help='eval tag for this experiment')
    # parser.add_argument('--eval_all', action='store_true', default=False, help='whether to evaluate all checkpoints')
    # parser.add_argument('--ckpt_dir', type=str, default=None, help='specify a ckpt directory to be evaluated if needed')
    # parser.add_argument('--save_to_file', action='store_true', default=False, help='')
    # parser.add_argument('--infer_time', action='store_true', default=False, help='calculate inference latency')
    # parser.add_argument('--TensorRT', action='store_true', default=False, help='Evaluation with TensorRT model')

    args = parser.parse_args()

    # cfg_from_yaml_file(args.cfg_file, cfg)
    # cfg.TAG = Path(args.cfg_file).stem
    # cfg.EXP_GROUP_PATH = '/'.join(args.cfg_file.split('/')[1:-1])  # remove 'cfgs' and 'xxxx.yaml'
    #
    # np.random.seed(1024)

    # if args.set_cfgs is not None:
    #     cfg_from_list(args.set_cfgs, cfg)

    # return args, cfg
    return args

# model_path = "{}/OpenPCDet/centerpoint/model/model.trt".format(os.path.expanduser('~'))
# config_path = "{}/OpenPCDet/centerpoint/config/config.yaml".format(os.path.expanduser('~'))
# centerpoint = cp.CenterPoint(config_path, model_path)

def getBox(box):
    x, y, z = box.x(), box.y(), box.z()
    l, w, h = box.l(), box.w(), box.h()
    yaw, score, cls = box.yaw(), box.score(), box.cls()

    return x, y, z, l, w, h, yaw, score, cls


# pc_dir = sorted(glob("{}/OpenPCDet/data/waymo/waymo_processed_data_v0_5_0/segment-10335539493577748957_1372_870_1392_870_with_camera_labels/*.npy".format(os.path.expanduser('~'))))
# for pc_path in pc_dir:
#     pc = np.load(pc_path)[:,:4]
#     pc[:,3] = np.tanh(pc[:,3])
#     print(pc.shape)
#     print(pc_path)
#     boxes = centerpoint.forward(pc)
#     for box in boxes:
#         x, y, z, l, w, h, yaw, score, cls = getBox(box)




def main():
    args = parse_config()

    npy_dir = Path(args.npy_dir).absolute()
    print(npy_dir)

    npy_file_list = read_files(path=npy_dir, extension='*.npy')
    print(npy_file_list)

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

    pass


if __name__ == '__main__':
    main()
        