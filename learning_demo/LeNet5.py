# dataset: CIFAR10
# 10 classes: (airplane, auto, bird, cat, deer, dog, frog, horse, ship, truck)
import tensorflow as tf
import data_process as dp
import numpy as np
# implement LeNet5: a five layer CNN
'''
layer 1: convolutional layer(followed by average pooling)
layer 2: convolutional layer(followed by average pooling)
layer 3: fully connected network
layer 4: fully connected network
layer 5: output layer
'''
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32
LABELS = 10
IMAGE_DEPTH = 3

BATCH_SIZE = 32
FILTER_SIZE = 5
FILTER_DEPTH_1 = 6
FILTER_DEPTH_2 = 16
HIDDEN_CELLS_1 = 120
HIDDEN_CELLS_2 = 84

STEPS = 10001
learning_rates = [0.001, 0.0015, 0.002, 0.0025]

train_dataset, train_label, test_dataset, test_label = dp.data_process() 

def flatten_tf_array(array):
    shape = array.get_shape().as_list()
    return tf.reshape(array, [shape[0], shape[1] * shape[2] * shape[3]])

def accuracy(pred, label):
	return (100.0 * np.sum(np.argmax(pred, 1) == np.argmax(label, 1)) / pred.shape[0])

def init_variables(filter_size = FILTER_SIZE, filter_depth1 = FILTER_DEPTH_1, 
					filter_depth2 = FILTER_DEPTH_2, 
					num_hidden1 = HIDDEN_CELLS_1, num_hidden2 = HIDDEN_CELLS_2,
					image_width = IMAGE_WIDTH, image_height = IMAGE_HEIGHT, image_depth = IMAGE_DEPTH, num_labels = LABELS):

	w1 = tf.Variable(tf.truncated_normal([filter_size, filter_size, image_depth, filter_depth1], stddev=0.1))
	b1 = tf.Variable(tf.zeros([filter_depth1]))

	w2 = tf.Variable(tf.truncated_normal([filter_size, filter_size, filter_depth1, filter_depth2], stddev=0.1))
	b2 = tf.Variable(tf.constant(1.0, shape=[filter_depth2]))

	w3 = tf.Variable(tf.truncated_normal([(image_width // filter_size)*(image_height // filter_size)*filter_depth2, num_hidden1], stddev=0.1))
	b3 = tf.Variable(tf.constant(1.0, shape = [num_hidden1]))

	w4 = tf.Variable(tf.truncated_normal([num_hidden1, num_hidden2], stddev=0.1))
	b4 = tf.Variable(tf.constant(1.0, shape = [num_hidden2]))

	w5 = tf.Variable(tf.truncated_normal([num_hidden2, num_labels], stddev=0.1))
	b5 = tf.Variable(tf.constant(1.0, shape = [num_labels]))
	variables = {
		'w1': w1, 'w2': w2, 'w3': w3, 'w4': w4, 'w5': w5,
		'b1': b1, 'b2': b2, 'b3': b3, 'b4': b4, 'b5': b5
	}
	return variables

def model_lenet5(data, variables):
	layer1_conv = tf.nn.conv2d(data, variables['w1'], [1, 1, 1, 1], padding='SAME')
	layer1_actv = tf.sigmoid(layer1_conv + variables['b1'])
	layer1_pool = tf.nn.avg_pool(layer1_actv, [1, 2, 2, 1], [1, 2, 2, 1], padding='SAME')

	layer2_conv = tf.nn.conv2d(layer1_pool, variables['w2'], [1, 1, 1, 1], padding='VALID')
	layer2_actv = tf.sigmoid(layer2_conv + variables['b2'])
	layer2_pool = tf.nn.avg_pool(layer2_actv, [1, 2, 2, 1], [1, 2, 2, 1], padding='SAME')

	# layer3_conv = tf.nn.conv2d(layer1_pool, variables['w2'], [1, 1, 1, 1], padding='VALID')
	# layer3_actv = tf.sigmoid(layer2_conv + variables['b2'])
	# layer_pool = tf.nn.avg_pool(layer2_actv, [1, 2, 2, 1], [1, 2, 2, 1], padding='SAME')

	flat_layer = flatten_tf_array(layer2_pool)
	layer3_fccd = tf.matmul(flat_layer, variables['w3']) + variables['b3']
	layer3_actv = tf.nn.sigmoid(layer3_fccd)

	layer4_fccd = tf.matmul(layer3_actv, variables['w4']) + variables['b4']
	layer4_actv = tf.nn.sigmoid(layer4_fccd)
	
	logits = tf.matmul(layer4_actv, variables['w5']) + variables['b5']
	return logits


for LR in learning_rates:
	print('learning_rate:', LR)
	graph = tf.Graph()
	with graph.as_default():
		tf_train_dataset = tf.placeholder(tf.float32, shape = (BATCH_SIZE, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_DEPTH))
		tf_train_label = tf.placeholder(tf.float32, shape = (BATCH_SIZE, LABELS))
		tf_test_dataset = tf.constant(test_dataset, tf.float32)

		# initilization of weight and bias
		variables_ = init_variables
		variables = variables_(image_width = IMAGE_WIDTH, image_height = IMAGE_HEIGHT, image_depth = IMAGE_DEPTH, num_labels = LABELS)

		# initialize model
		model = model_lenet5
		logits = model(tf_train_dataset, variables)

		# loss
		loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=tf_train_label))

		# optimizer
		optimizer = tf.train.AdamOptimizer(learning_rate = LR).minimize(loss)

		# prediction for training and test data
		train_pred = tf.nn.softmax(logits)

		logits_test = model_lenet5(tf_test_dataset, variables)
		test_pred = tf.nn.softmax(logits_test)

	with tf.Session(graph = graph) as session:
		tf.global_variables_initializer().run()
		for step in range(STEPS):
			offset = (step * BATCH_SIZE) % (train_label.shape[0] - BATCH_SIZE)
			data = train_dataset[offset:(offset + BATCH_SIZE), :, :, :]
			label = train_label[offset:(offset + BATCH_SIZE), :]

			feed_dict = {tf_train_dataset: data, tf_train_label:label}
			_, cost, pred = session.run([optimizer, loss, train_pred], feed_dict = feed_dict)
			train_accuracy = accuracy(pred, label)

			if step % 1000 == 0:
				test_accuracy = accuracy(test_pred.eval(), test_label)
				summary = "step {:04d} : loss is {:06.2f}, accuracy on training set {:02.2f} %, accuracy on test set {:02.2f} %".format(step, cost, train_accuracy, test_accuracy)
				print(summary)


