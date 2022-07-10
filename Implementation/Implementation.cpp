#include <bits/stdc++.h>
#include <math.h>
#include <C:\Users\Dell\Documents\20CS10040\.vscode\DSA\rapidxml-1.13\rapidxml.hpp>

#define R 6373.0
using namespace std;
using namespace rapidxml;

xml_document<> doc;
xml_node<> *root_node = NULL;
class node
{
public:
  string name;
  string id;
  string longitude;
  string latitude;
  long double distance;
  void shortest_path(int src);
} * node_list;
class way
{
public:
  string id;
  string name;
  vector<string> node_id;

} * ways_list;

struct adj_list_node
{
  int dest;
  int weight;
  struct adj_list_node *next;
};

struct adj_list
{
  struct adj_list_node *head;
};

struct Graph
{
  int V;
  struct adj_list *array;
};

struct adj_list_node *newadj_list_node(int dest, int weight)
{
  struct adj_list_node *newNode = (struct adj_list_node *)malloc(sizeof(struct adj_list_node));
  newNode->dest = dest;
  newNode->weight = weight;
  newNode->next = NULL;
  return newNode;
}

struct Graph *create_graph(int V)
{
  struct Graph *graph = (struct Graph *)malloc(sizeof(struct Graph));
  graph->V = V;
  graph->array = (struct adj_list *)malloc(V * sizeof(struct adj_list));

  for (int i = 0; i < V; ++i)
    graph->array[i].head = NULL;

  return graph;
}

void addEdge(struct Graph *graph, int src, int dest, int weight)
{
  struct adj_list_node *newNode =
      newadj_list_node(dest, weight);
  newNode->next = graph->array[src].head;
  graph->array[src].head = newNode;
  newNode = newadj_list_node(src, weight);
  newNode->next = graph->array[dest].head;
  graph->array[dest].head = newNode;
}

struct min_heap_node
{
  int v;
  int dist;
};

struct min_heap
{
  int size;
  int capacity;
  int *pos;
  struct min_heap_node **array;
};

struct min_heap_node *newmin_heap_node(int v, int dist)
{
  struct min_heap_node *min_heap_node = (struct min_heap_node *)malloc(sizeof(struct min_heap_node));
  min_heap_node->v = v;
  min_heap_node->dist = dist;
  return min_heap_node;
}

struct min_heap *createmin_heap(int capacity)
{
  struct min_heap *min_heap = (struct min_heap *)malloc(sizeof(struct min_heap));
  min_heap->pos = (int *)malloc(
      capacity * sizeof(int));
  min_heap->size = 0;
  min_heap->capacity = capacity;
  min_heap->array = (struct min_heap_node **)malloc(capacity * sizeof(struct min_heap_node *));
  return min_heap;
}

int Find_node_name(string s1, string s2)
{
  int M = s1.length();
  int N = s2.length();

  /* A loop to slide pat[] one by one */
  for (int i = 0; i <= N - M; i++)
  {
    int j;
    for (j = 0; j < M; j++)
      if (s2[i + j] != s1[j])
        break;

    if (j == M)
      return i;
  }

  return -1;
}

long double radian(const long double &degree)
{
  long double one_deg = (M_PI) / 180;
  return one_deg * degree;
}

long double distance(string id1, string id2, int node_count)
{
  long double lat1, lat2;
  long double lon1, lon2;
  for (int i = 0; i < node_count; i++)
  {

    if (id1 == node_list[i].id)
    {
      lat1 = stold(node_list[i].latitude);
      lon1 = stold(node_list[i].longitude);
    }
    if (id2 == node_list[i].id)
    {
      lat2 = stold(node_list[i].latitude);
      lon2 = stold(node_list[i].longitude);
    }
  }
  lat1 = radian(lat1);
  lat2 = radian(lat2);
  lon1 = radian(lon1);
  lon2 = radian(lon2);
  long double dlon = lon2 - lon1;
  long double dlat = lat2 - lat1;

  long double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  long double c = 2 * asin(sqrt(a));

  long double dist = R * c;

  return dist;
}



void swapmin_heap_node(struct min_heap_node **a, struct min_heap_node **b)
{
  struct min_heap_node *t = *a;
  *a = *b;
  *b = t;
}

void min_heapify(struct min_heap *min_heap, int idx)
{
  int smallest, left, right;
  smallest = idx;
  left = 2 * idx + 1;
  right = 2 * idx + 2;

  if (left < min_heap->size && min_heap->array[left]->dist < min_heap->array[smallest]->dist)
    smallest = left;

  if (right < min_heap->size && min_heap->array[right]->dist < min_heap->array[smallest]->dist)
    smallest = right;

  if (smallest != idx)
  {
    min_heap_node *smallestNode = min_heap->array[smallest];
    min_heap_node *idxNode = min_heap->array[idx];
    min_heap->pos[smallestNode->v] = idx;
    min_heap->pos[idxNode->v] = smallest;
    swapmin_heap_node(&min_heap->array[smallest], &min_heap->array[idx]);

    min_heapify(min_heap, smallest);
  }
}

int isEmpty(struct min_heap *min_heap)
{
  return min_heap->size == 0;
}

struct min_heap_node *extract_min(struct min_heap *min_heap)
{
  if (isEmpty(min_heap))
    return NULL;
  struct min_heap_node *root = min_heap->array[0];
  struct min_heap_node *last_node =  min_heap->array[min_heap->size - 1];
  min_heap->array[0] = last_node;
  min_heap->pos[root->v] = min_heap->size - 1;
  min_heap->pos[last_node->v] = 0;
  --min_heap->size;
  min_heapify(min_heap, 0);

  return root;
}

void decrease_key(struct min_heap *min_heap, int v, int dist)
{
  int i = min_heap->pos[v];

  min_heap->array[i]->dist = dist;
  while (i && min_heap->array[i]->dist <  min_heap->array[(i - 1) / 2]->dist)
  {
    min_heap->pos[min_heap->array[i]->v] =  (i - 1) / 2;
    min_heap->pos[min_heap->array[(i - 1) / 2]->v] = i;
    swapmin_heap_node(&min_heap->array[i],  &min_heap->array[(i - 1) / 2]);
    i = (i - 1) / 2;
  }
}

bool isInmin_heap(struct min_heap *min_heap, int v)
{
  if (min_heap->pos[v] < min_heap->size)
    return true;
  return false;
}

void printArr(int dist[], int n)
{
  printf("Vertex   Distance from Source\n");
  for (int i = 0; i < n; ++i)
  printf("%d \t\t %d\n", i, dist[i]);
}



class comp_distance
{
public:
  bool operator()(node const &p1, node const &p2)
  {
    // return "true" if "p1" is ordered
    // before "p2", for example:
    return p1.distance > p2.distance;
  }
};

void dijkstra(struct Graph *graph, int src, int des)
{
  int V = graph->V;
  int dist[V];
  struct min_heap *min_heap = createmin_heap(V);
  for (int v = 0; v < V; ++v)
  {
    dist[v] = INT_MAX;
    min_heap->array[v] = newmin_heap_node(v, dist[v]);
    min_heap->pos[v] = v;
  }
  min_heap->array[src] =  newmin_heap_node(src, dist[src]);
  min_heap->pos[src] = src;
  dist[src] = 0;
  decrease_key(min_heap, src, dist[src]);
  min_heap->size = V;

  while (!isEmpty(min_heap))
  {
    struct min_heap_node *min_heap_node =
        extract_min(min_heap);
    int u = min_heap_node->v;
    struct adj_list_node *pCrawl =
        graph->array[u].head;
    while (pCrawl != NULL)
    {
      int v = pCrawl->dest;
      if (isInmin_heap(min_heap, v) &&
          dist[u] != INT_MAX &&
          pCrawl->weight + dist[u] < dist[v])
      {
        dist[v] = dist[u] + pCrawl->weight;
        decrease_key(min_heap, v, dist[v]);
      }
      pCrawl = pCrawl->next;
    }
    //if(u==des)break;
  }
  if(dist[des]==INT_MAX){cout<< "2 nodes are disconnected.";return;};
   cout<< "shortest distance between source and destinatiom is: ";
   cout<< dist[des];
  //printArr(dist, V);
}
int main()
{

  node_list = new node[50000];
  ways_list = new way[50000];
  int count_node = 0,src,des;
  int count_ways = 0;
  string st,s,d;
  ifstream map_file("map.osm");
  vector<char> buffer((istreambuf_iterator<char>(map_file)), istreambuf_iterator<char>());
  buffer.push_back('\0');
  // Parse the buffer
  doc.parse<0>(&buffer[0]);

  // Find out the root node
  root_node = doc.first_node("osm");
  string name_data;
  for (xml_node<> *node = root_node->first_node("node"); node; node = node->next_sibling("node"))
  {
    node_list[count_node].id = node->first_attribute("id")->value();
    node_list[count_node].longitude = node->first_attribute("lon")->value();
    node_list[count_node].latitude = node->first_attribute("lat")->value();
    cout << "node_id " << node_list[count_node].id << "  ";
    cout << "node_lat " << node_list[count_node].longitude << "  ";
    cout << "node_lon " << node_list[count_node].latitude << "  ";
    cout << "\n";
    for (xml_node<> *node_name = node->first_node("tag"); node_name; node_name = node_name->next_sibling("tag"))
    {
      name_data = node_name->first_attribute("k")->value();
      if (name_data == "name")
      {
        node_list[count_node].name = node_name->first_attribute("v")->value();
        //cout << "node_name " << node_list[count_node].name << endl;
      }
    }
    count_node++;
  }

  for (xml_node<> *way = root_node->first_node("way"); way; way = way->next_sibling("way"))
  {
    ways_list[count_ways].id = way->first_attribute("id")->value();
    // cout << "ways_id " << ways_list[count_ways].id << endl;
    int i = 0;
    for (xml_node<> *node_Id = way->first_node("nd"); node_Id; node_Id = node_Id->next_sibling("nd"))
    {
      ways_list[count_ways].node_id.push_back(node_Id->first_attribute("ref")->value());
      // cout << "node_id " << ways_list[count_ways].node_id[i] << endl;
      i++;
    }

    for (xml_node<> *way_name = way->first_node("tag"); way_name; way_name = way_name->next_sibling("tag"))
    {
      name_data = way_name->first_attribute("k")->value();
      if (name_data == "name")
      {
        ways_list[count_ways].name = way_name->first_attribute("v")->value();
        // cout << "node_name " << ways_list[count_ways].name << endl;
      }
    }
    count_ways++;
  }

  cout << "The no. of nodes is:" << count_node << endl;
  cout << "The no. of ways is:" << count_ways << endl;

  cout << "press 1 to search an element" << endl;
  cout << "press 2 to print id, longitude, latitude" << endl;
  cout << "press 3 to find shortest path between 2 nodes" << endl;
  int key;
  cin >> key;
  string id1;
  switch (key)
  {
  case 1:
  {
    string str_name;
    cout << "Enter the value name :" << endl;
    cin >> str_name;
    int name_found;
    int flag = 0;
    for (int i = 0; i < count_node; i++)
    {
      name_found = Find_node_name(str_name, node_list[i].name);
      if (name_found > -1)
      {
        cout << node_list[i].name << endl;
        flag = 1;
      }
    }
    if (flag == 0)
      cout << "Name not found" << endl;
    //break;
  }
  case 2:
  {
    // finding distance between two points
    cout << "Enter the distance id of node for getting k closest node point" << endl;
    cin >> id1;
    int k;
    cout << "Enter the value of 'k'" << endl;
    cin >> k;
    for (int i = 0; i < count_node; i++)
    {
      node_list[i].distance = distance(id1, node_list[i].id, count_node);
    }
    priority_queue<node, vector<node>, comp_distance> node_priority;
    for (int i = 0; i < count_node; i++)
      node_priority.push(node_list[i]);
    for (int i = 0; i < k; i++)
    {
      node p = node_priority.top();
      cout << setprecision(8);
      cout << p.id << "   " << p.distance << "   " << p.longitude << "   " << p.latitude << endl;
      node_priority.pop();
    }
  }
  case 3:
  {
    cout << "enter source id:"<< endl;
    cin>> s;
    for(int i=0; i<count_node; i++){
        if(node_list[i].id == s){
          src=i;break;
        }
    }
    cout<< "enter destination id:"<< endl;
    cin>> d;
    for(int i=0;i<count_node;i++){
      if(node_list[i].id==d){
        des=i;break;
      }
    }
    struct Graph *graph = create_graph(count_node);
    unordered_map<string, int> node_order;
    for (int i = 0; i < count_node; i++)
    {
      node_order[node_list[i].id] = i;
    }
    int pos1;
    int pos2;
    int i = 0;
    for (int i = 0; i < count_ways; i++)
    {
      pos1 = node_order[ways_list[i].node_id[0]];
      for (int j = 1; j < ways_list[i].node_id.size(); j++)
      {
        pos2 = node_order[ways_list[i].node_id[j]];
        addEdge(graph, pos1, pos2, distance(ways_list[i].node_id[j], ways_list[i].node_id[j - 1], count_node));
      }
    }
    dijkstra(graph, src,des);
  }
  
  }
  delete node_list;
  delete ways_list;
  return 0;
}

