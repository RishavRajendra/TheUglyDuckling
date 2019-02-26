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
python ssd_lite_demo.py mb2-ssd-lite <trained model path> vision/models/voc-model-labels.txt <Video file path> 
python run_ssd_demo.py mb2-ssd-lite <trained model path> vision/models/voc-model-labels.txt <Image file path>
```
### Evaluate Training
```
Average Precision Per-class:
start: 0.3444457841009565
blockA: 0.08031108961960026
blockB: 0.18432601880877744
blockC: 0.18694415340756804
blockD: 0.3126812163320044
blockE: 0.25913768033331286
blockF: 0.18424260178516613
obstacle: 0.6296255317719224
side: 0.4578452604821188
corner: 0.5763307895753582

Average Precision Across All Classes:0.32158901262167844
```
Code to reproduce model:
```
python eval_ssd.py --net mb2-ssd-lite --dataset vision/datasets/images/test/ --trained_model <trained model path> --label_file vision/models/voc-model-labels.txt
```
  
## Authors

* **Rishav Rajendra** - [Website](https://rishavrajendra.github.io)
* **Benji Lee**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
