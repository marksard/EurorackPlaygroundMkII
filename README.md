# Eurorack Playground MkII
10HP Eurorack modular compatible application playground

## Specification

### Input

|Name|Description|
|:--|:--|
|V/OCT|ADC in|
|GATE|Digital in|
|CV1|ADC in with pot<br>With bipolar and unipolar switching function|
|CV2|ADC in with pot<br>With bipolar and unipolar switching function|

### Output

|Name|Description|
|:--|:--|
|OUT 1~6|PWM out|

With PWM output confirmation LED.  
2nd order CR lowpass filter cutoff frequency: 8.7kHz  
Opamp buffered out with bipolar and unipolar switching function.  

### Controller

For application control

|Name|Description|
|:--|:--|
|Pot||
|Encoder||
|Button A||
|Button B||
|Encoder Button(Button C)||
|Buttion A LED||
|Buttion B LED||

## Image

![img](https://marksard.github.io/assets/photos/20240718_20240718-IMGP8787.jpg)

## Schematic

![img](_data/mkII_sch_01.png)  
![img](_data/mkII_sch_02.png)  
![img](_data/mkII_sch_03.png)  
![img](_data/mkII_sch_04.png)  

### Note
The common bias circuitry affects channel separation. I knew this and made it common, but if you are going to copy the circuit, it is better to modify it.  

## Demonstration
[![img](https://github.com/user-attachments/assets/0b5ccc6b-40a2-4344-9bb2-f58af75253c2)](https://youtu.be/szY7E-fJ0OA)  

[![img](https://github.com/user-attachments/assets/1eeb8981-f021-4f7d-a006-8a0cc3b4000d)](https://youtu.be/rw5O4wigSXM?si=Y_idfcUrhKUAo_wJ)  
