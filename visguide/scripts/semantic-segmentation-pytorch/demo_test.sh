#!/bin/bash

# Image and model names
TEST_IMG=l_image-12.jpg
MODEL_PATH=baseline-resnet18dilated-c1_deepsup
RESULT_PATH=./

ENCODER=$MODEL_PATH/encoder_epoch_20.pth
DECODER=$MODEL_PATH/decoder_epoch_20.pth

# Download model weights and image
if [ ! -e $MODEL_PATH ]; then
  mkdir $MODEL_PATH
fi
if [ ! -e $ENCODER ]; then
  wget -P $MODEL_PATH http://sceneparsing.csail.mit.edu/model/pytorch/$ENCODER
fi
if [ ! -e $DECODER ]; then
  wget -P $MODEL_PATH http://sceneparsing.csail.mit.edu/model/pytorch/$DECODER
fi
if [ ! -e $TEST_IMG ]; then
  wget -P $RESULT_PATH http://sceneparsing.csail.mit.edu/data/ADEChallengeData2016/images/validation/$TEST_IMG
fi

# Inference
python3 -u test.py \
  --model_path $MODEL_PATH \
  --test_imgs $TEST_IMG \
  --arch_encoder resnet18dilated \
  --arch_decoder c1_deepsup \
  --fc_dim 512 \
  --result $RESULT_PATH
