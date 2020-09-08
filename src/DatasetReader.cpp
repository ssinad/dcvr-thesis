#include <fstream>
#include <iostream>
#include "DatasetReader.h"

// using namespace std;

void DatasetReader::read_file(std::string filename, bool read_rewards)
{
  std::istream* file = &std::cin;
  std::ifstream file_stream;
  if (filename != ""){
    file_stream.open(filename);
    file = &file_stream;
    // std::ifstream file file(filename);
  }
  // std::string line;
  // while (file.good())
  // {
  *file >> num_nodes >> root_node;

  distances = Matrix(num_nodes, std::vector<distance_t>(num_nodes));
  rewards = Rewards(num_nodes);
  for (int row = 0; row < num_nodes; row++)
  {
    for (int col = 0; col < num_nodes; col++)
    {
      *file >> distances[row][col];
    }
  }
  if (read_rewards)
  {
    for (int col = 0; col < num_nodes; col++)
    {
      *file >> rewards[col];
    }
  }
  if (filename != ""){
    static_cast<std::ifstream*>(file) -> close();
    // std::ifstream file file(filename);
  }
  
  // }
}

Matrix DatasetReader::get_matrix()
{
  return distances;
}

Penalties DatasetReader::get_penalties()
{
  return rewards;
}
Vertices DatasetReader::get_vertices()
{
  vertices = Vertices();
  for (int i = 0; i < num_nodes; i++)
  {
    vertices.insert(i);
  }
  return vertices;
}

Node DatasetReader::get_root_node()
{
  return root_node;
}

// int main()
// {
//   DatasetReader dr = DatasetReader();
//   dr.read_file("datasets/fisher-1994-set-f/F-n045-k4.txt");
//   for (penalty_t p : dr.get_penalties())
//   {
//     std::cout << p << " ";
//   }
//   cout << endl;
//   return 0;
// }