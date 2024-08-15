import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import onnx
from pathlib import Path
from general.config.config import cfg, cfg_from_yaml_file
from object_detection.detectors3d.centerpoint_pillar.onnx_utils import convert_onnx, pillarscatter_surgeon, \
    export_config


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/waymo_models/centerpoint_pillar_inference.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default='../ckpts/waymo_iou_branch.pth',
                        help='specify the pretrained model')
    parser.add_argument('--max_voxels', type=int, default=25000, help='specify max number of voxels')
    parser.add_argument('--max_points', type=int, default=200000, help='specify max number of points')
    parser.add_argument('--score_threshold', type=float, default=0.5, help='specify score threshold')
    parser.add_argument('--num_voxel_feature', type=int, default=10, help='specify # of voxel features')
    parser.add_argument('--save_dir', type=str, default=None, help='specify directory for saving onnx model')
    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg


if __name__ == '__main__':
    args, cfg = parse_config()

    if args.save_dir is None:
        save_dir = Path(os.path.abspath(__file__)).parent.parent / 'onnx'
    else:
        save_dir = Path(args.save_dir)

    if not save_dir.exists():
        save_dir.mkdir(parents=True)

    onnx_raw_path = save_dir / "model_raw.onnx"
    onnx_sim_path = save_dir / "model_sim.onnx"
    onnx_path = save_dir / "model.onnx"

    if not onnx_sim_path.exists():
        print("Exporting model to ONNX...")
        convert_onnx(args, onnx_raw_path, onnx_sim_path)

    if not onnx_path.exists():
        print("Model optimization...")
        onnx_model = onnx.load(onnx_sim_path)
        modified_model = pillarscatter_surgeon(onnx_model)
        onnx.save(modified_model, onnx_path)
        print("Model exported to model.onnx")

    export_config(save_dir=save_dir, max_voxels=args.max_voxels, max_points=args.max_points,
                  score_threshold=args.score_threshold, num_voxel_feature=args.num_voxel_feature)
