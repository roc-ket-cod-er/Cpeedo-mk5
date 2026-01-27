# Cpeedo-mk5

>NOTE: This project was funded by Hack Club's Blueprint program. Without their support, this project would not have been possible.

A _REALLY_ fancy bike speedo.

![0077](https://github.com/user-attachments/assets/feea3d7d-3d2a-4b13-b158-af5bbd43f456)

> ⚠️ Looking for mk1 through 5? They are currently unavailible as they ended up not meeting the design requirements and ended up evolving into other boards.

## Key Features

As the main goal is to measure speed, it obviously does so.

In addition, it also supports:
- a cell connection;
- anti-theft features, including using that cell connection to send the bike's position to the internet;
- a double LCD + bonus E-INK display, for live speed (etc) monitoring;
- two independent ways of measuring speed: a wheel rotation counter + GPS;
- an SD card;
- features for live navigation.

## PCB

<img width="1920" height="1080" alt="0495" src="https://github.com/user-attachments/assets/91539cd3-8342-47a7-b1e2-508bc382deba" />


The PCB has a 6-layer stackup to keep 2 solid ground planes, that is essential to keep the ground planes completely unbroken otherwise the antenna will be unhappy.

The stackup is:

1. Signals & Ground
2. Ground
3. Signals & Ground
4. Signals & Ground
5. Ground
6. Signals & Ground

### PCB Layers

Here's the PCB's Top layer (ground fill hidden)

<img width="434" height="756" alt="image" src="https://github.com/user-attachments/assets/59aa148a-cdff-43f8-9b5a-a33b71d53a71" />

Layer 3:

<img width="497" height="762" alt="image" src="https://github.com/user-attachments/assets/9f4da303-ac96-4594-8121-5f9935cbb603" />

Layer 4:

<img width="392" height="737" alt="image" src="https://github.com/user-attachments/assets/4b62ab2c-da47-4872-96de-41967e7d5842" />

And the bottom Layer (ground fill hidden)

<img width="528" height="784" alt="image" src="https://github.com/user-attachments/assets/03a32652-bcb3-4246-bba4-8dccad48fa47" />

> Remember, layers 2 & 5 are solid ground.

### PCB 3D Views:

Top: 

<img width="474" height="729" alt="image" src="https://github.com/user-attachments/assets/1859c031-05c5-42d7-b54e-3c52a0892319" />

Bottom:

<img width="459" height="814" alt="image" src="https://github.com/user-attachments/assets/d1f65325-85fe-4531-8a38-7845fd75a97a" />

Bonus:

<img width="887" height="395" alt="image" src="https://github.com/user-attachments/assets/32d7ae3c-621d-44ab-8db2-c571e09348ed" />

Bonus 2:

<img width="1450" height="862" alt="image" src="https://github.com/user-attachments/assets/801a340e-8071-4448-9ce8-955ec4eb68b1" />

## Inspiration

Always, whenever I have been in motion, I have always wanted to know how fast I'm going. Like for some reason, you just expect to be able to know. And once you know, you can start racing yourself, start getting better, start pacing yourself, and just have more fun in general. I have made several previous editions, but in motion it has always been very hard to read the screens, and thus this time I have implemented an E-INK along with bright LCDs.

In addition, I have become increasingly paranoid about my bike being stolen, thus it would make ulitimate sense to achieve both things in one project.

## How to use

In this repository, you will find the files you need to print the case yourself, along side to get the PCB manufactured. The code too, shall come with time. Thus, it is as simple as loading the code onto the board, and you should be good. The LCDs have ribbon cables, making the connection really easy, whilst the E-INK required the normal jumper wires to be hooked up.

## BOM

|Item            |Cost (native)   |Cost (USD)|Link                                                                                                                                                        |NOTE                                                             |TOTAL: |
|----------------|----------------|----------|------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------|-------|
|PCB             |US$199.53+13%GST|$225.47   |JLCPCB.com                                                                                                                                                  |GST may not be collected.                                        |$340.89|
|SIM card        |CA$25           |$20.76    |simbase.com                                                                                                                                                 |Includes Adding Funds to the SIM. Taxes may reduce 13%           |       |
|Waveshare       |US$62.47        |$62.47    |https://www.waveshare.com/catalog/product/view/id/6414/s/2.8inch-capacitive-touch-lcd/category/398/ and https://www.waveshare.com/2.9inch-e-paper-module.htm|Includes 2x LCD, + 1x E-INK + Shipping. Taxes may increase by 13%|       |
|3D print        |--              |$10.00    |Library                                                                                                                                                     |Estimate                                                         |       |
|Screws          |US$7.83+13% GST |$8.85     |https://www.aliexpress.com/item/1005005523283652.html                                                                                                       |M2.5x8mm, 100pcs (minimum selected)                              |       |
|Screw Head      |US$2.94+13% GST |$3.32     |https://www.aliexpress.com/item/1005003319507557.html                                                                                                       |T8                                                               |       |
|Compas Pre-Order|US$8.87 +13% GST|$10.02    |jlcpcb.com                                                                                                                                                  |I may change the compas as it went out of stock :(               |       |
