# The Ugly Duckling

Autonomous robot for 2019 IEEE Region 5 Autonomous Robotics Competition.

![alt text](https://github.com/RishavRajendra/TheUglyDuckling/blob/master/pictures/playingField.png)

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
start: 0.2950170396978908
blockA: 0.1090909090909091
blockB: 0.13636363636363635
blockC: 0.17349640650611525
blockD: 0.2740371733360128
blockE: 0.377404371048673
blockF: 0.31433934059094526
obstacle: 0.6017093755498263
side: 0.3591425086781124
corner: 0.6141706643674154

Average Precision Across All Classes:0.3254771425229537
```
Code to reproduce model:
```
python eval_ssd.py --net mb2-ssd-lite --dataset vision/datasets/images/test/ --trained_model <trained model path> --label_file vision/models/voc-model-labels.txt
```

```
Average Precision Per-class:
start: 0.6644306719561556
blockA: 0.32447063535564824
blockB: 0.23398957377105215
blockC: 0.4250320574160577
blockD: 0.7446259793558014
blockE: 0.6657902435290295
blockF: 0.5846939265384001
obstacle: 0.8851199111546342
side: 0.6430556619779979
corner: 0.9090909090909093

Average Precision Across All Classes:0.6080299570145686
```
Code to reproduce model:
```
python eval_ssd.py --net vgg16-ssd --dataset vision/datasets/images/test/ --trained_model vision/models/vgg16-ssd-VGG16-Epoch-20-Loss-2.440702438354492.pth --label_file vision/models/voc-model-labels.txt
```
## Authors

* **Rishav Rajendra** - [Website](https://rishavrajendra.github.io)
* **Benji Lee**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
