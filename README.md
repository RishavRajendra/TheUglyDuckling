# The Ugly Duckling

Autonomous robot for 2019 IEEE Region 5 Autonomous Robotics Competition.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Tested on Python 3.7.2

What things you need to install the software and how to install them

```
pip install pySerial
pip install keyboard
pip install pytorch
pip install torchvision
pip install numpy
pip install pandas
```

### Training
```
python train.py --train_dataset datasets/images/train/ --test_dataset datasets/images/test/ --net mb2-ssd-lite --pretrained_ssd models/mb2-ssd-lite-mp-0_686.pth --batch_size 24 --num_epochs 200 --scheduler cosine --lr 0.01 --t_max 200
```
You can use multiple datasets to train. Run the command inside the vision directory.

### Run live demo
```
python ssd_lite_demo.py mb2-ssd-lite vision/models/mb2-ssd-lite-Epoch-199-Loss-4.174241065979004.pth vision/models/voc-model-labels.txt <Video file> 
```
<Video file>: Enter relative path to video file you want to process.
  
## Authors

* **Rishav Rajendra** - [Website](https://rishavrajendra.github.io)
* **Benji Lee**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
