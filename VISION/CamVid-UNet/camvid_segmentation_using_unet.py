# -*- coding: utf-8 -*-
"""camvid-segmentation-using-unet.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1tZM9bPJ1BGSEKpQTrl2JpN4sONGjNaGh
"""

import numpy as np # linear algebra
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
from matplotlib import pyplot as plt
import cv2
from keras.preprocessing.image import load_img, img_to_array
from keras.utils import to_categorical ,Sequence
from keras import backend as K
from keras.layers import Input, Conv2D, MaxPooling2D, concatenate, Conv2DTranspose, BatchNormalization, Activation, Dropout
from keras.optimizers import Adadelta, Nadam ,Adam
from keras.models import Model, load_model
import tensorflow as tf
from os import listdir



img_size = 256
class_map_df = pd.read_csv("class_dict.csv")

class_map = []
for index,item in class_map_df.iterrows():
    class_map.append(np.array([item['r'], item['g'], item['b']]))
    
def dice_coef(y_true, y_pred, smooth=1):
  intersection = K.sum(y_true * y_pred, axis=[1,2,3])
  union = K.sum(y_true, axis=[1,2,3]) + K.sum(y_pred, axis=[1,2,3])
  dice = K.mean((2. * intersection + smooth)/(union + smooth), axis=0)
  return dice


def conv_block(tensor, nfilters, size=3, padding='same', initializer="he_normal"):
    x = Conv2D(filters=nfilters, kernel_size=(size, size), padding=padding, kernel_initializer=initializer)(tensor)
    x = BatchNormalization()(x)
    x = Activation("relu")(x)
    x = Conv2D(filters=nfilters, kernel_size=(size, size), padding=padding, kernel_initializer=initializer)(x)
    x = BatchNormalization()(x)
    x = Activation("relu")(x)
    return x


def deconv_block(tensor, residual, nfilters, size=3, padding='same', strides=(2, 2)):
    y = Conv2DTranspose(nfilters, kernel_size=(size, size), strides=strides, padding=padding)(tensor)
    y = concatenate([y, residual], axis=3)
    y = conv_block(y, nfilters)
    return y


def Unet(h, w, filters, pretrained_weights = None):
# down
    input_layer = Input(shape=(h, w, 3), name='image_input')
    conv1 = conv_block(input_layer, nfilters=filters)
    conv1_out = MaxPooling2D(pool_size=(2, 2))(conv1)
    conv2 = conv_block(conv1_out, nfilters=filters*2)
    conv2_out = MaxPooling2D(pool_size=(2, 2))(conv2)
    conv3 = conv_block(conv2_out, nfilters=filters*4)
    conv3_out = MaxPooling2D(pool_size=(2, 2))(conv3)
    conv4 = conv_block(conv3_out, nfilters=filters*8)
    conv4_out = MaxPooling2D(pool_size=(2, 2))(conv4)
    conv4_out = Dropout(0.5)(conv4_out)
    conv5 = conv_block(conv4_out, nfilters=filters*16)
    conv5 = Dropout(0.5)(conv5)
# up
    deconv6 = deconv_block(conv5, residual=conv4, nfilters=filters*8)
    deconv6 = Dropout(0.5)(deconv6)
    deconv7 = deconv_block(deconv6, residual=conv3, nfilters=filters*4)
    deconv7 = Dropout(0.5)(deconv7) 
    deconv8 = deconv_block(deconv7, residual=conv2, nfilters=filters*2)
    deconv9 = deconv_block(deconv8, residual=conv1, nfilters=filters)
    output_layer = Conv2D(filters=32, kernel_size=(1, 1), activation='softmax')(deconv9)

    model = Model(inputs=input_layer, outputs=output_layer, name='Unet')

    if (pretrained_weights):
        model.load_weights(pretrained_weights)
    return model

model = Unet(img_size , img_size , 64, 'top-weights.h5')

model.compile(optimizer='adam', loss='categorical_crossentropy' ,metrics=['accuracy'])


def make_prediction(model, img_path, shape):
    img= img_to_array(load_img(img_path , target_size= shape))/255.0
    img = np.expand_dims(img,axis=0)
    labels = model.predict(img)
    labels = np.argmax(labels[0],axis=2)
    return labels

# pred_label = make_prediction(model, '0.jpg', (img_size,img_size,3))

def form_colormap(prediction, mapping):
    h,w = prediction.shape
    color_label = np.zeros((h,w,3),dtype=np.uint8)    
    color_label = mapping[prediction]
    color_label = color_label.astype(np.uint8)
    return color_label

# pred_colored = form_colormap(pred_label,np.array(class_map))

# plt.imshow(pred_colored/255.)
# plt.title('predicted labels')
# plt.show()

cap = cv2.VideoCapture('sample_vid/tsr_ramsis_sample_for _vehcile_only.mkv')
ret, image_src = cap.read()
out = cv2.VideoWriter('tsr_ramsis_sample_for _vehcile_only.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (img_size,img_size))

# while cap.isOpened():
#     ret, image_src = cap.read()
#     if ret:
#         pred_label = make_prediction(model, image_src, (img_size, img_size, 3))
#         pred_colored = form_colormap(pred_label,np.array(class_map))
#         out.write(pred_colored)
#         print("...")
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     else:
#         break
# cap.release()
# out.release()
# cv2.destroyAllWindows()

for filename in listdir('CamVid/test/'):
    if filename.endswith(".png"): 
        image_path = 'CamVid/test/' + filename
        pred_label = make_prediction(model, image_path, (img_size, img_size, 3))
        pred_colored = form_colormap(pred_label,np.array(class_map))
        cv2.imwrite('CamVid_Test_Output/' + filename + '.png', pred_colored)
        print(filename + " Done")

