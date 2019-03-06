import os
import numpy as np
import pickle
import matplotlib.pyplot as plt
import random

TRAIN_DATASETS = ['data_batch_1', 'data_batch_2', 'data_batch_3', 'data_batch_4', 'data_batch_5' ]
TEST_DATASET = 'test_batch'
FOLDER_NAME = 'cifar-10-batches-py/'
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32
IMAGE_DEPTH = 3
# 10 classes: (airplane, auto, bird, cat, deer, dog, frog, horse, ship, truck)


def get_label_name(label):
	"""
	Load the label names from file
	"""
	index = np.argmax(label)
	labels = ['airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']
	return labels[int(index)]

def one_hot_encode(np_array, num_label):
	temp = (np.arange(num_label) == np_array[:,None]).astype(np.float32)
	return temp


def reformat_data(dataset, label):
	np_dataset_ = dataset.reshape((len(dataset), IMAGE_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT)).transpose(0, 2, 3, 1)
	num_label = len(np.unique(label))
	np_label_ = one_hot_encode(np.array(label, dtype=np.float32), num_label)
	np_dataset, np_label = randomize(np_dataset_, np_label_)
	return np_dataset, np_label

def randomize(dataset, label):
	permutation = np.random.permutation(label.shape[0])
	shuffled_dataset = dataset[permutation, :, :]
	shuffled_label = label[permutation]
	return shuffled_dataset, shuffled_label

def data_process():
	current_path = os.path.dirname(os.path.abspath(__file__))
	file_path = current_path + '/' + FOLDER_NAME
	# load train and test dictionary
	train_dataset, train_label = [], []
	for dataset in TRAIN_DATASETS:
		with open(file_path + dataset, 'rb') as f0:
			train_dict = pickle.load(f0, encoding = 'bytes')
			train_dataset_temp, train_label_temp = train_dict[b'data'], train_dict[b'labels']
			train_dataset.append(train_dataset_temp)
			train_label += train_label_temp

	with open(file_path + TEST_DATASET, 'rb') as f1:
		test_dict = pickle.load(f1, encoding = 'bytes')
		test_dataset, test_label = test_dict[b'data'], test_dict[b'labels']

	train_dataset = np.concatenate(train_dataset, axis = 0)
	train_dataset, train_label = reformat_data(train_dataset, train_label)
	test_dataset, test_label = reformat_data(test_dataset, test_label)
	print("training dataset contains {} images, each of size {}".format(train_dataset[:,:,:,:].shape[0], train_dataset[:,:,:,:].shape[1:]))
	print("test dataset contains {} images, each of size {}".format(test_dataset[:,:,:,:].shape[0], test_dataset[:,:,:,:].shape[1:]))
	return train_dataset, train_label, test_dataset, test_label



