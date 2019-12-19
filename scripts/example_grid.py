import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import Queue as queue
import sys
from os import listdir
from os.path import isfile, join



def visualize(grid):

	copy = np.copy(grid)
	
	plt.imshow(copy)
	#plt.plot(copy.shape[0]/2, copy.shape[1]/2, 'ro', markersize=15)
	plt.xlim(150,300)
	plt.ylim(200,350)
	plt.show(block=False)
	plt.pause(1)
	plt.close()


def readfile(file):
	grid = []
	with open(file, 'r') as csvfile: 
		csvreader = csv.reader(csvfile) 
		for row in csvreader:
			grid.append(row)


	for i in range(np.shape(grid)[0]):

		for j in range(np.shape(grid)[1]):

			grid[i][j]=int(grid[i][j])

	frontier(grid)

def readFiles(dir):
	onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
	lst = []
	for file in onlyfiles:
		num = ""
		for ele in file:
			if ele.isdigit():
				num += ele
		lst.append(int(num))

	lst.sort()
	files = [dir+'/'+"file"+str(num)+".npy" for num in lst]

	return files



def TotalFrontiers(grid):

	copy = np.zeros(np.shape(grid))

	open_list = queue.Queue()

	frontier_number = 1

	for i in range(np.shape(grid)[0]):

		for j in range(np.shape(grid)[1]):

			if grid[i][j] != 2 or copy[i][j]>0:

				continue
			
			start_cell = [i,j]
			open_list.put(start_cell)
			#pixels = 0
			while not open_list.empty():

				cell = open_list.get()

				copy[cell[0]][cell[1]] = frontier_number

				#pixels = pixels +1
				
				for k in range(cell[0]-2,cell[0]+2):

					for l in range(cell[1]-2,cell[1]+2):

						if grid[k][l] != 2 or copy[k][l]>0:
							continue

						next_cell = [k,l]
						copy[next_cell[0]][next_cell[1]] = frontier_number
						open_list.put(next_cell)
#Different logic counting number of pixels beforehand only and then assigning the number, the number will be updated after if in one region pixels are more than 40			
			# if pixels < 40:
			# 	continue
			frontier_number = frontier_number + 1

#Here the number of frontier regions are more and then we will be sorting and thersholding the clusters 

 	clusters = []
	for n in range(1,frontier_number+1):

		cluster = 0
	 	for l in range(np.shape(grid)[0]):

	 		for m in range(np.shape(grid)[1]):

	 			if copy[l][m] == n:

	 				cluster += 1

	 	clusters.append(cluster)

	clusters.sort(reverse=True)

	# if np.size(clusters)<4:
		
	# 	total_frontiers = 1
	# 	f.write("%d \n" % (total_frontiers))

	# else:

	clusters = clusters[:3]
	eliminate= []

	if clusters[0] - clusters[1] < 70 and clusters[0] < 200 and clusters[0]>110:
		eliminate.append(clusters[1])

		# if clusters[2] > clusters[1]/2:

		# 	eliminate.append(1)

	for x in range(2):

		if clusters[0]>200 and clusters[x+1]+10<(clusters[0]/2):

			if clusters[x+1]>44:
				continue
			eliminate.append(clusters[x+1])


			
		else:

			if clusters[x+1]<clusters[0]/2 and clusters[0]<200:

				eliminate.append(clusters[x+1])

	


	#print clusters
	#print eliminate

	total_frontiers = np.size(clusters)-np.size(eliminate)	
	f.write("%d \n" % (total_frontiers))		
	print total_frontiers

	#print frontier_number
	print clusters


def frontier(file):
	grid = np.load(file)
	print file
	frontier = np.copy(grid)

	for i in range(np.shape(grid)[0]):

		for j in range(np.shape(grid)[1]):

			if grid[i][j] == -1:

				if j+1 <= np.shape(grid)[1]-1 and grid[i][j+1] == 0:

					frontier[i][j+1] = 2
				
				if grid[i][j-1] == 0 and j-1 > 0:

					frontier[i][j-1] = 2

				if i+1<=np.shape(grid)[1]-1 and grid[i+1][j] == 0:
				
					frontier[i+1][j] = 2

				if grid[i-1][j] == 0 and i-1 > 0:
					
					frontier[i-1][j] = 2

				
			j += 1
		
		i += 1
	
	#print frontier

	
	crop = np.copy(frontier)

	# frontier[np.where(crop== -1)] = 0 
	# frontier[np.where(crop ==0)] = 50 
	# frontier[np.where(crop==1)] = 100 
	# frontier[np.where(crop ==2)] = 200 
	
	#visualize(grid)
	TotalFrontiers(crop)
	#visualize(frontier)
if __name__ == '__main__':
	
	#file = "/home/dhagash/grid.csv"
	#grid = [[-1 for i in range(50)] for i in range(50)]
	#visualize(grid)
	#readfile(file)
	if(len(sys.argv) != 2):
		print("Usage: %s directory" % sys.argv[0])
		sys.exit(1)
	f= open("dataset_custom.txt","a+")
	files = readFiles(sys.argv[1])
	# frontier(files[1946])
	for i in range(176	,len(files)):
		frontier(files[i])
		i += 1
	 	