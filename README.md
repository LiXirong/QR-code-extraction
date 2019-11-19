# QR-code-extraction

[![996.icu](https://img.shields.io/badge/link-996.icu-red.svg)](https://996.icu)
[![LICENSE](https://img.shields.io/badge/license-Anti%20996-blue.svg)](https://github.com/996icu/996.ICU/blob/master/LICENSE)

This work uses some simple image processing algorithm to extract QR code in complex background.

And convert the QR code to binary matrix, do not decode.

## How to use
1. Open `gui.m`
2. Select an image
![1](https://github.com/LiXirong/QR-code-extraction/blob/master/image/1rbg2gray.png)
3. Do some processes in order
![2](https://github.com/LiXirong/QR-code-extraction/blob/master/image/2edge.png)
![3](https://github.com/LiXirong/QR-code-extraction/blob/master/image/3binarization.png)
![4](https://github.com/LiXirong/QR-code-extraction/blob/master/image/4location.png)
![5](https://github.com/LiXirong/QR-code-extraction/blob/master/image/5segment.png)
![6-1](https://github.com/LiXirong/QR-code-extraction/blob/master/image/6-1correction.png)
![6-2](https://github.com/LiXirong/QR-code-extraction/blob/master/image/6-2correction.png)
![7](https://github.com/LiXirong/QR-code-extraction/blob/master/image/7binarization.png)
![8](https://github.com/LiXirong/QR-code-extraction/blob/master/image/8segmentation.png)

4. Finally save the result
![results1](https://github.com/LiXirong/QR-code-extraction/blob/master/image/9getresult.png)
![results2](https://github.com/LiXirong/QR-code-extraction/blob/master/image/10result.png)
