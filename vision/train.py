import argparse
import os
import logging
import sys
import itertools

import torch
from torch.utils.data import DataLoader, ConcatDataset
from torch.optim.lr_scheduler import CosineAnnealingLR, MultiStepLR

from mobilenetv2_ssd_lite import create_mobilenetv2_ssd_lite
from data_preprocessing import TrainAugmentation, TestTransform
import mobilenet_ssd_confg

parser = argparse.ArgumentParser(
	description = 'Object detector training with Pytorch')

parser.add_argument('--datasets', nargs='+', help='Dataset directory path.')
parser.add_argument('--validation_dataset', help='Dataset directory path.')

parser.add_argument('--freeze_base_net', actions='store_true', 
	help='Freeze base net layers.')
parser.add_argument('--freeze_net', action='store_true', 
	help='Freeze all the layers except the prediction head.')

parser.add_argument('--mb2_width_multi', default=1.0, type=float, 
	help='Width Multiplier for MobileNetV2')

# Params for SGD
parser.add_argument('--lr', '--learning-rate', default=1e-3, type=float, 
	help='initial learning rate')
parser.add_argument('--momentum', default=0.9, type=float,
	help='Momentum value for optim')
parser.add_argument('--weight_decay', default=5e-4, type=float,
	help='Weight decay for SGD')
parser.add_argument('--gamma', default=0.1, type=float,
	help='Gamma update for SGD')
parser.add_argument('--base_net_lr', default=None, type=float,
	help='initial learning rate for base net.')
parser.add_argument('--extra_layers_lr', default=None, type=float,
	help='initial learning rate for the layers not in base net and prediction heads.')

# Params for loading pretrained basenet or checkpoints.
parser.add_argument('--base_net',
	help='Pretrained base model')
parser.add_argument('--pretrained_ssd', 
	help='Pre-trained base model')
parser.add_argument('--resume', default=None, type=str,
	help='Checkpoint state_dict file to resume training from')

# Scheduler
parser.add_argument('--scheduler', default="multi-step", type=str,
	help="Scheduler for SGD. It can one of multi-step and cosine")

# Params for Cosine Annealing
parser.add_argument('--t_max', default=120, type=float,
	help='T_max value for Cosine Annealing Scheduler.')

# Train params
parser.add_argument('--batch_size', default=32, type=int,
	help='Batch size for training')
parser.add_argument('--num_epochs', default=120, type=int,
	help='the number epochs')
parser.add_argument('--num_workers', default=4, type=int,
	help='Number of workers used in dataloading')
parser.add_argument('--validation_epochs', default=5, type=int,
	help='the number epochs')
parser.add_argument('--debug_steps', default=100, type=int,
	help='Set the debug log output frequency.')
parser.add_argument('--use_cuda', default=True, type=str2bool,
	help='Use CUDA to train model')

parser.add_argument('--checkpoint_folder', default='models/',
	help='Directory for saving checkpoint models')


logging.basicConfig(stream=sys.stdout, level=logging.INFO,
	format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

args = parser.parse_args()
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() and args.use_cuda else "cpu")

if args.use_cuda and torch.cuda.is_available():
    torch.backends.cudnn.benchmark = True
    logging.info("Use Cuda.")

def main(){
	#TODO: Implement main
}

if __name__ == '__main__':
	main()