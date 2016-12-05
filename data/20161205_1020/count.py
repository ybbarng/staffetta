import os

def average(lst):
	return sum(lst) / len(lst)

def variance(lst):
	avg = average(lst)
	var = 0
	for elem in lst:
		var = var + (elem - avg) ** 2
	return var / len(lst)

def main():
	file_lst = []
	for root, dirs, files in os.walk("./"):
		for f in files:
			if ".txt" in f:
				file_lst.append(f)

	file_lst = sorted(file_lst)

	for f in file_lst:
		fi = open(f, "r")

		recv = []
		power = {}

		start = False
		for line in fi:
			if not start:
				if "Sink" in line:
					start = True
				continue
			else:
				lst = line.split("\t")

				sec = float(lst[0].split(":")[1])

				if sec >= 10.0:
					break

				node_id = int(lst[1].split(":")[1].strip())

				if node_id == 1:
					if lst[2] not in recv:
						recv.append(lst[2])
				else:
					val = lst[2].strip().split(" ")

					if not val[0].isdigit():
						continue
		
					if int(val[0]) == 6:
						power[node_id] = (float(val[1]) / 1000)

		node = f.split(".")[0]

		if node[-2].isdigit():
			nodes = int(node[-2:])
		else:
			nodes = int(node[-1])

		print ("file: ", f)
		print ("count: ", len(recv))
		print ("success: ", len(recv) / (nodes * 10.0))
		print ("avg power: ", average(power.values()))
		print ("var power: ", variance(power.values()))
		print ("power: ", power)
		print ("\n")

if __name__ == "__main__":
	main()
