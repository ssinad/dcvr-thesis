# Input format
The first line contains two space-separated integers. The first one (`N`) represents the number of nodes in the graph (including the root), and the second one is the index of the root node. (0-indexed)

The next `N` lines represent the shortest path distance matrix between the nodes.
The `j`-th number on the `i`-th line represents the length of the path going from node `i` to node `j`.

There may be one last line in the file, in which case it represents the reward associated with each node - including the root.