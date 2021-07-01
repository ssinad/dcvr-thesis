from sklearn.metrics import pairwise_distances
import numpy as np


filenames = ["pr76", "gr96", "gr202"]
for filename in filenames:
    p = []
    with open(filename + ".tsp") as f:
        for line in f:
            line = line.strip()
            if line != "NODE_COORD_SECTION":
                continue
            else:
                break
        for line in f:
            line = line.strip()
            if line == "EOF":
                break
            # print(line)
            _, x, y = [float(x) for x in line.split()]
            p.append((x, y))
    arr = np.array(p)
    n = len(arr)
    # print(arr)
    D = pairwise_distances(arr)
    tour_length = 0
    with open(filename + ".opt.tour") as f:
        for line in f:
            line = line.strip()
            if line != "TOUR_SECTION":
                continue
            else:
                break
        previous_node = 0
        for line in f:
            line = line.strip()
            if line == "-1":
                break
            current_node = int(line) - 1
            tour_length += D[previous_node][current_node]
            previous_node = current_node
    with open(filename + ".opt.tour.len", 'w') as f:
        f.write(str(tour_length) + "\n")
    np.savetxt(fname="{}.txt".format(filename), X=D, fmt="%.5f", header="{} 0".format(n), comments='')
    with open(filename + ".txt", 'a') as f:
        f.write(str(tour_length))

    