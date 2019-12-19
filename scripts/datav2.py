import numpy as np
import csv
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.metrics import classification_report


dataset_frontier=[]

def main(file):

	f = open("/home/dhagash/auto_navws/src/agv_nav/scripts/dataset_custom.txt","r")
	f_1 = f.readlines()
	for lines in f_1:
	
		dataset_frontier.append(int(lines)-1)

	
	#dataset_frontier_1 = dataset_frontier[:16200]
	#dataset_frontier_2 = dataset_frontier[16201:21201]

	test_dataset = []
	
	with open(file, 'r') as csvfile: 
		csvreader = csv.reader(csvfile) 
		for row in csvreader:
			test_dataset.append(row)

	for i in range(np.shape(test_dataset)[0]):

		for j in range(np.shape(test_dataset)[1]):

			test_dataset[i][j]=int(test_dataset[i][j])


	#test_dataset_1 = test_dataset[:16200]
	#test_dataset_2 = test_dataset[16201:21201]

	print 'Confusion matrix: '
	print confusion_matrix(test_dataset, dataset_frontier)
	print 'Accuracy score: Dataset  ', accuracy_score(test_dataset, dataset_frontier)
	print classification_report(test_dataset,dataset_frontier)

    #print np.size(test_dataset), np.size(test_dataset_1),np.size(test_dataset_2)

if __name__ == '__main__':

	file = "/home/dhagash/frontiers_data/frontier_custom.csv"
	main(file)

