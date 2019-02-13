#!/usr/bin/env python
__author__ = "Rishav Rajendra"

import argparse
import os
import logging
import sys
import itertools

import torch
from utils.misc import str2bool, Timer, freeze_net_layers, store_labels
from torch.utils.data import DataLoader, ConcatDataset
from torch.optim.lr_scheduler import CosineAnnealingLR, MultiStepLR

from mobilenetv2_ssd_lite import create_mobilenetv2_ssd_lite
from data_preprocessing import TrainAugmentation, TestTransform
import mobilenet_ssd_config

parser = argparse.ArgumentParser(
	description = 'Object detector training with Pytorch')

parser.add_argument('--train_dataset', nargs='+', help='Train dataset directory path.')
parser.add_argument('--test_dataset', help='Test dataset directory path.')

parser.add_argument('--net', default="mb2-ssd-lite",
	help="The network architecture, it can be only be mb2-ssd-lite")
parser.add_argument('--freeze_base_net', actions='store_true', 
	help='Freeze base net layers.')
parser.add_argument('--freeze_net', action='store_true', 
	help='Freeze all the layers except the prediction head.')

parser.add_argument('--mb2_width_mult', default=1.0, type=float, 
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
def train(loader, net, criterion, optimizer, device, debug_steps=100, epoch=-1):
	net.train(True)
	running_loss = 0.0
	running_regression_loss = 0.0
	running_classification_loss = 0.0
	for i, data in enumerate(loader):
		images, boxes, labels = data
		images = images.to(device)
		boxes = boxes.to(device)
		labels = labels.to(device)

		optimizer.zero_grad()
		confidence, locations = net(images)
		regression_loss, classification_loss = criterion(confidence, locations, labels, boxes)  # TODO CHANGE BOXES
		loss = regression_loss + classification_loss
		loss.backward()
		optimizer.step()

		running_loss += loss.item()
		running_regression_loss += regression_loss.item()
		running_classification_loss += classification_loss.item()
		if i and i % debug_steps == 0:
			avg_loss = running_loss / debug_steps
			avg_reg_loss = running_regression_loss / debug_steps
			avg_clf_loss = running_classification_loss / debug_steps
			logging.info(
				f"Epoch: {epoch}, Step: {i}, " +
				f"Average Loss: {avg_loss:.4f}, " +
				f"Average Regression Loss {avg_reg_loss:.4f}, " +
				f"Average Classification Loss: {avg_clf_loss:.4f}"
				)
			running_loss = 0.0
			running_regression_loss = 0.0
			running_classification_loss = 0.0

def test(loader, net, criterion, device):
	net.eval()
	running_loss = 0.0
	running_regression_loss = 0.0
	running_classification_loss = 0.0
	num = 0
	for _, data in enumerate(loader):
		images, boxes, labels = data
		images = images.to(device)
		boxes = boxes.to(device)
		labels = labels.to(device)
		num += 1

		with torch.no_grad():
			confidence, locations = net(images)
			regression_loss, classification_loss = criterion(confidence, locations, labels, boxes)
			loss = regression_loss + classification_loss

		running_loss += loss.item()
		running_regression_loss += regression_loss.item()
		running_classification_loss += classification_loss.item()
	return running_loss / num, running_regression_loss / num, running_classification_loss / num

def main():
	timer = Timer()

	if args.net == "mb2-ssd-lite":
		create_net = lambda num: create_mobilenetv2_ssd_lite(num, width_mult=args.mb2_width_mult)
		config = mobilenet_ssd_config
	else:
		logging.fatal("The net type is wrong.")
		parser.print_help(sys.stderr)
		sys.exit(1)

	train_transform = TestTransform(config.image_size, config.image_mean, config.image_std)
	target_transform = MatchPrior(config.priors, config.center_variance, config.size_variance, 0.5)

	test_transform = TestTransform(config.image_size, config.image_mean, config.image_std)

	"""
    Dataset in Pascal VOC format
    Preparing the training and testing dataset
    """

	logging.info("Preparing training dataset")

	train_dataset = VOCDataset(args.train_dataset, transform=train_transform,
    	target_transform=target_transform)

    #TODO: check os path
	label_file = os.path.join(args.checkpoint_folder, "voc-model-labels.txt")

	store_labels(label_file, train_dataset.class_names)
	num_classes = len(train_dataset.class_names)

	logging.info(f"Stored labels into file {label_file}.")

	logging.info(f"Train dataset size:{len(train_dataset)}")
	logging.info("Training dataset created")

	logging.info("Preparing test dataset")
	test_dataset = VOCDataset(args.test_dataset, transform=test_transform, 
    	target_transform=target_transform, is_test=True)

	logging.info(f"Test dataset size: {len(test_dataset)}")

	test_loader = DataLoader(test_dataset, args.batch_size, 
    	num_workers=args.num_workers, shuffle=False)
	logging.info("Test dataset ready")

	logging.info("Build network")
	net = create_net(num_classes)
	min_loss = -10000.0
	last_epoch = -1

	timer.start("Load Model")
	if args.resume:
		logging.info(f"Resume from the model {args.resume}")
		net.load(args.resume)
	elif args.base_net:
		logging.info(f"Init from base net {args.base_net}")
		net.init_from_base_net(args.base_net)
	elif args.pretrained_ssd:
		logging.info(f"Init from pretrained ssd {args.pretrained_ssd}")
		net.init_from_pretrained_ssd(args.pretrained_ssd)
	logging.info(f'Took {timer.end("Load Model"):.2f} seconds to load the model.')

	net.to(DEVICE)

	criterion = MultiboxLoss(config.priors, iou_threshold=0.5, neg_pos_ratio=3,
		center_variance=0.1, size_variance=0.2, device=DEVICE)
	optimizer = torch.optim.SGD(params, lr=args.lr, momentum=args.momentum,
		weight_decay=args.weight_decay)
	logging.info(f"Learning rate: {args.lr}, Base net learning rate: {base_net_lr}, "
		+ f"Extra Layers learning rate: {extra_layers_lr}.")

	if args.scheduler == 'multi-step':
		logging.info("Uses MultiStepLR scheduler.")
		milestones = [int(v.strip()) for v in args.milestones.split(",")]
		scheduler = MultiStepLR(optimizer, milestones=milestones,
			gamma=0.1, last_epoch=last_epoch)
	elif args.scheduler == 'cosine':
		logging.info("Uses CosineAnnealingLR scheduler.")
		scheduler = CosineAnnealingLR(optimizer, args.t_max, last_epoch=last_epoch)
	else:
		logging.fatal(f"Unsupported Scheduler: {args.scheduler}.")
		parser.print_help(sys.stderr)
		sys.exit(1)

	logging.info(f"Start training from epoch {last_epoch + 1}.")
	for epoch in range(last_epoch + 1, args.num_epochs):
		scheduler.step()
		train(train_loader, net, criterion, optimizer,
			device=DEVICE, debug_steps=args.debug_steps, epoch=epoch)

		if epoch % args.validation_epochs == 0 or epoch == args.num_epochs - 1:
			val_loss, val_regression_loss, val_classification_loss = test(val_loader, net, criterion, DEVICE)
			logging.info(
				f"Epoch: {epoch}, " +
				f"Validation Loss: {val_loss:.4f}, " +
				f"Validation Regression Loss {val_regression_loss:.4f}, " +
				f"Validation Classification Loss: {val_classification_loss:.4f}"
				)
			model_path = os.path.join(args.checkpoint_folder, f"{args.net}-Epoch-{epoch}-Loss-{val_loss}.pth")
			net.save(model_path)
			logging.info(f"Saved model {model_path}")

if __name__ == '__main__':
	main()