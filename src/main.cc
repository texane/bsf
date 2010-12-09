#include <list>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/types.h>
#ifndef __USE_BSD
# define __USE_BSD
#endif
#include <time.h>
#include <sys/time.h>


using std::list;
using std::vector;


#define CONFIG_NODE_COUNT 30000
#define CONFIG_NODE_DEGREE 10
#define CONFIG_MAX_THREAD 32
#define CONFIG_PATH_DEPTH 100
#define CONFIG_DEBUG 1
#define CONFIG_ITER 10


typedef struct node
{
  list<struct node*> adjlist;
  bool state;

#if CONFIG_DEBUG
  unsigned int id;
#endif

  bool has_adj(const struct node* adj) const
  {
    list<struct node*>::const_iterator pos = adjlist.begin();
    list<struct node*>::const_iterator end = adjlist.end();

    for (; pos != end; ++pos)
      if (*pos == adj)
	return true;

    return false;
  }

  void add_adj(struct node* adj)
  {
    adjlist.push_back(adj);
  }

  bool is_marked() const
  {
    return state;
  }

  void mark()
  {
    state = true;
  }

  void unmark()
  {
    state = false;
  }

} node_t;


typedef struct graph
{
  vector<node_t> nodes;

  void unmark_nodes()
  {
    vector<node_t>::iterator pos = nodes.begin();
    vector<node_t>::iterator end = nodes.end();
    for (; pos != end; ++pos)
      pos->unmark();
  }

  void initialize(unsigned int node_count)
  {
    nodes.resize(node_count);

    vector<node_t>::iterator pos = nodes.begin();
    vector<node_t>::iterator end = nodes.end();

#if CONFIG_DEBUG
    unsigned int id = 0;
#endif

    for (; pos != end; ++pos)
    {
      pos->adjlist.clear();
      pos->state = false;
#if CONFIG_DEBUG
      pos->id = id++;
#endif
    }
  }

  unsigned int node_count() const
  {
    return nodes.size();
  }

  const node_t* at_const(unsigned int i) const
  {
    return &nodes[i];
  }

  node_t* at(unsigned int i)
  {
    return &nodes[i];
  }

#if CONFIG_DEBUG
  void print() const
  {
    vector<node_t>::const_iterator pos = nodes.begin();
    vector<node_t>::const_iterator end = nodes.end();
    for (; pos != end; ++pos)
    {
      printf("%u:", (*pos).id);

      list<node_t*>::const_iterator adjpos = (*pos).adjlist.begin();
      list<node_t*>::const_iterator adjend = (*pos).adjlist.end();

      for (; adjpos != adjend; ++adjpos)
	printf(" %u", (*adjpos)->id);
      printf("\n");
    }
    printf("\n");
  }
#endif

} graph_t;


static void make_a_path
(graph_t& graph, node_t* a, node_t* b, unsigned int depth)
{
  // depth the number of edges in between

  node_t* prev = a;

  if (depth > 1)
  {
    for (; depth; --depth)
    {
      node_t* const node = graph.at(rand() % graph.node_count());

      prev->add_adj(node);
      node->add_adj(prev);

      prev = node;
    }
  }

  if (prev->has_adj(b) == false)
  {
    prev->add_adj(b);
    b->add_adj(prev);
  }
}


static void generate_random_graph
(graph_t& graph, unsigned int node_count)
{
  graph.initialize(node_count);

  unsigned int edge_count = node_count * CONFIG_NODE_DEGREE / 2;
  for (; edge_count; --edge_count)
  {
    node_t* const a = graph.at(rand() % node_count);
    node_t* const b = graph.at(rand() % node_count);
   
    if ((a == b) || (a->has_adj(b))) continue ;

    a->add_adj(b);  b->add_adj(a);
  }
}


#if 1 // sequential version

typedef struct path_node
{
  struct path_node* prev;
  node_t* node;

  path_node(path_node* _prev, node_t* _node)
    : prev(_prev), node(_node) {}

  path_node(node_t* _node)
    : prev(NULL), node(_node) {}

} path_node_t;


static unsigned int count_path_depth
(const path_node_t* pos)
{
  unsigned int depth = 0;
  for (; pos != NULL; pos = pos->prev)
    ++depth;
  return depth;
}


static void free_path_node_list
(list<path_node_t*>& l)
{
  list<path_node_t*>::const_iterator pos = l.begin();
  list<path_node_t*>::const_iterator end = l.end();
  for (; pos != end; ++pos)
    delete *pos;
}


static void add_path_node_adjlist
(
 path_node_t* path_node,
 list<path_node_t*>& to_visit
)
{
  list<node_t*>::const_iterator pos = path_node->node->adjlist.begin();
  list<node_t*>::const_iterator end = path_node->node->adjlist.end();

  for (; pos != end; ++pos)
    to_visit.push_back(new path_node_t(path_node, *pos));
}

static unsigned int find_shortest_path_seq
(graph_t& g, node_t* from, node_t* to)
{
  // father child relation
  list<path_node_t*> to_visit;
  list<path_node_t*> visited;
  unsigned int path_depth = 0;

  to_visit.push_front(new path_node_t(from));

  while (to_visit.empty() == false)
  {
    path_node_t* const path_node = to_visit.front();

    to_visit.pop_front();
    visited.push_back(path_node);

    if (path_node->node == to)
    {
      path_depth = count_path_depth(path_node);
      break ;
    }

    if (path_node->node->is_marked())
      continue ;

    path_node->node->mark();

    add_path_node_adjlist(path_node, to_visit);
  }

  free_path_node_list(to_visit);
  free_path_node_list(visited);

  return path_depth;
}

#endif // sequential


#if 0 // parallel version

typedef struct thief_work
{
  list<node_t*> nodes;
  node_t* to_find;
} thief_work_t;

typedef struct thief_res
{
  list<node_t*> to_visit;
  node_t* found;
} thief_res_t;

static void thief_entry(void* p)
{
  thief_res_t* const r = static_cast<thief_res_t*>
    (kaapi_get_thief_result(sc));
  thief_work_t* const w = static_cast<thief_work_t*>(p);

  list<node_t*>::const_iterator pos = w->nodes.begin();
  list<node_t*>::const_iterator end = w->nodes.end();

  // find work->to_find in work->nodes
  for (; pos != end; pos = pos->next)
  {
    const node_t* const node = *pos;
    if (node == w->to_find)
    {
      r->to_find = node;
      return ;
    }

    // mark the node
    node->mark;

    // add adj list to visit
    r->to_visit += node->adj_nodes;
  }
}

static void reducer()
{
  // node found
  if (tres->to)
  {
    vres->is_found = 1;
    return ;
  }

  merge(vres->to_visit, tres->to_visit);
}



static path_t* find_shortest_path_par
(graph_t* graph, node_t* a, node_t* b)
{
  // allocate metas
  // enqueue start node
}

#endif // parallel version


static void peek_random_pair
(graph_t& g, node_t*& a, node_t*& b)
{
  a = g.at(rand() % g.node_count());
  b = g.at(rand() % g.node_count());
}


static void initialize_stuff()
{
  srand(getpid() * time(0));
}


int main(int ac, char** av)
{
  graph_t g;
  node_t* from;
  node_t* to;
  struct timeval tms[3];

  initialize_stuff();

  generate_random_graph(g, CONFIG_NODE_COUNT);
  peek_random_pair(g, from, to);
  make_a_path(g, from, to, CONFIG_PATH_DEPTH);

  for (unsigned int iter = 0; iter < CONFIG_ITER; ++iter)
  {
    g.unmark_nodes();

    gettimeofday(&tms[0], NULL);
    const unsigned int depth = find_shortest_path_seq(g, from, to);
    gettimeofday(&tms[1], NULL);
    timersub(&tms[1], &tms[0], &tms[2]);

    printf("found %u in %lf usecs\n", depth, (double)tms[2].tv_sec * 1E6 + (double)tms[2].tv_usec);
  }

  return 0;
}
