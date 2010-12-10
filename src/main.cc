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


#define CONFIG_SEQUENTIAL 1
#define CONFIG_PARALLEL 0
#define CONFIG_SEQ_GRAIN 128
#define CONFIG_NODE_COUNT 30000
#define CONFIG_NODE_DEGREE 10
#define CONFIG_MAX_THREAD 32
#define CONFIG_PATH_DEPTH 10
#define CONFIG_DEBUG 1
#define CONFIG_ITER 10


typedef struct node
{
  list<struct node*> adjlist;

#if CONFIG_PARALLEL
  volatile bool state __attribute__((aligned));
#else
  bool state;
#endif

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
  { adjlist.push_back(adj); }

  void unmark()
  { state = false; }

#if CONFIG_PARALLEL
  bool mark_ifnot()
  { return __bool_compare_and_swap(&state, 0, 1); }
#else
  bool mark_ifnot()
  { 
    if (state == false)
    {
      state = true;
      return true;
    }
    return false;
  }
#endif


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

  for (; depth > 1; --depth)
  {
    node_t* const node = graph.at(rand() % graph.node_count());

    prev->add_adj(node);
    node->add_adj(prev);

    prev = node;
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


#if CONFIG_SEQUENTIAL // sequential version

static void append_node_adjlist_seq
(node_t* node, list<node_t*>& to_visit)
{
  list<node_t*>::const_iterator pos = node->adjlist.begin();
  list<node_t*>::const_iterator end = node->adjlist.end();

  for (; pos != end; ++pos)
  {
    if ((*pos)->mark_ifnot() == true)
      to_visit.push_back(*pos);
  }
}


static unsigned int find_shortest_path_seq
(graph_t& g, node_t* from, node_t* to)
{
  // current and levels
  list<node_t*> to_visit[2];
  unsigned int depth = 0;

  // bootstrap algorithm
  to_visit[1].push_front(from);

  // while next level not empty
  while (to_visit[1].empty() == false)
  {
    // process nodes at level
    to_visit[0].swap(to_visit[1]);

    while (to_visit[0].empty() == false)
    {
      node_t* const node = to_visit[0].front();
      to_visit[0].pop_front();

      if (node == to) return depth;

      append_node_adjlist_seq(node, to_visit[1]);
    }

    ++depth;
  }

  return 0;
}

#endif // CONFIG_SEQUENTIAL


#if CONFIG_PARALLEL // parallel version

// parallel, stealable work
typedef struct par_work
{
  volatile list<path_node_t*>* to_visit __attiribute__((aligned));
} par_work_t;

typedef struct thief_work
{
  vector<path_node_t*> to_visit;
  node_t* to_find;
} thief_work_t, seq_work_t;

typedef struct thief_result
{
  list<path_node_t*> adj_nodes;
  bool is_found;
} thief_result_t;

static void initialize_par_work(par_work_t* work)
{
  work->to_visit = NULL;
}

static void set_par_work
(par_work_t* work, list<path_node_t*>* to_visit)
{
  work->to_visit = to_visit;
}

static path_node_t* pop_par_work(par_work_t* work)
{
}

static int split_par_work
(kaapi_stealcontext_t* sc, int nreq, kaapi_request_t* req, void* args)
{
  /* victim work */
  par_work_t* const vw = (work_t*)args;
  
  /* stolen range */
  kaapi_workqueue_index_t i, j;
  kaapi_workqueue_index_t range_size;
  
  /* reply count */
  int nrep = 0;
  
  /* size per request */
  kaapi_workqueue_index_t unit_size;
  
redo_steal:
  /* do not steal if range size <= PAR_GRAIN */
#define CONFIG_PAR_GRAIN 128
  range_size = kaapi_workqueue_size(&vw->cr);
  if (range_size <= CONFIG_PAR_GRAIN)
    return 0;
  
  /* how much per req */
  unit_size = range_size / (nreq + 1);
  if (unit_size == 0)
  {
    nreq = (range_size / CONFIG_PAR_GRAIN) - 1;
    unit_size = CONFIG_PAR_GRAIN;
  }
  
  /* perform the actual steal. if the range
   changed size in between, redo the steal
   */
  if (kaapi_workqueue_steal(&vw->cr, &i, &j, nreq * unit_size))
    goto redo_steal;
  
  for (; nreq; --nreq, ++req, ++nrep, j -= unit_size)
  {
    /* for reduction, a result is needed. take care of initializing it */
    kaapi_taskadaptive_result_t* const ktr =
    kaapi_allocate_thief_result(req, sizeof(thief_work_t), NULL);
    
    /* thief work: not adaptive result because no preemption is used here  */
    thief_work_t* const tw = kaapi_reply_init_adaptive_task
    ( sc, req, (kaapi_task_body_t)thief_entrypoint, sizeof(thief_work_t), ktr );
    tw->key = vw->key;
    tw->beg = vw->array+j-unit_size;
    tw->end = vw->array+j;
    tw->res = 0;
    
    /* initialize ktr task may be preempted before entrypoint */
    ((thief_work_t*)ktr->data)->beg = tw->beg;
    ((thief_work_t*)ktr->data)->end = tw->end;
    ((thief_work_t*)ktr->data)->res = 0;
    
    /* reply head, preempt head */
    kaapi_reply_pushhead_adaptive_task(sc, req);
  }
  
  return nrep;
}

static bool common_entry
(list<path_node_t*>& to_visit, list<path_node_t*>& adj_nodes)
{
  // to_visit, input. the list of path node to visit.
  // adj_nodes, output. filled with a merge of all adj list.

  vector<path_node_t*> pos = to_visit.begin();
  vector<path_node_t*> end = to_visit.end();

  for (; pos != end; ++pos)
  {
    if (pos->node == to_find) return true;
    add_path_node_adjlist(*pos, adj_nodes);
  }

  return false;
}

static void thief_entry(thief_work_t* work)
{
  list<path_node_t*> to_visit;
  list<path_node_t*> visited;
  unsigned int path_depth = 0;

  to_visit.push_front(new path_node_t(from));

  while (twork->to_visit.empty() == false)
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

static void reduce_thief
(kaapi_stealcontext_t* sc, void* targ, void* tdata, size_t tsize, void* varg)
{
  /* victim work */
  work_t* const vw = (work_t*)varg;
  
  /* thief work */
  thief_work_t* const tw = (thief_work_t*)tdata;
  
  /* thief range continuation */
  kaapi_workqueue_index_t beg, end;
  
  /* if the master already has a result, the
   reducer purpose is only to abort thieves
   */
  if (vw->res != (kaapi_workqueue_index_t)-1)
    return 0;
  
  /* check if the thief found a result */
  if (tw->res != 0)
  {
    /* do not continue the work */
    vw->res = tw->res - vw->array;
    return 0;
  }
  
  /* otherwise, continue preempted thief work */
  beg = (kaapi_workqueue_index_t)(tw->beg - vw->array);
  end = (kaapi_workqueue_index_t)(tw->end - vw->array);
  kaapi_workqueue_set(&vw->cr, beg, end);
  
  return 0;
}

static void abort_thief
(kaapi_stealcontext_t* sc, void* targ, void* tdata, size_t tsize, void* varg)
{ return ; }


static void abort_thieves
(kaapi_stealcontext_t* ksc)
{
}


// reduction

static void reduce_thief
(kaapi_stealcontext_t* sc, void* targ, void* tdata, size_t tsize, void* varg)
{
  // victim results
  work_t* const vres = (bsf_result_t*)varg;
  
  // thief results
  thief_work_t* const tres = (bsf_result_t*)tdata;

  if (vres->is_found == true)
  {
    tres->is_found = true;
    return true;
  }

  vres->to_visit.splice(0, tres->to_visit);
}

static bool reduce_thieves
(kaapi_stealcontext_t* ksc, list<node_t*>& to_visit)
{
  bsf_result_t res;

  res.is_found = false;
  res.to_visit = to_visit;

  while (preempt_thief(&res, reducer))
  {
    if (res.is_found == true)
    {
      // preempt all thieves, runtime constraint
      abort_thieves(ksc);
      return true;
    }
  }

  return false;
}


// core routines

static void append_node_adjlist_par
(node_t* node, list<node_t*>& to_visit)
{
  list<node_t*>::const_iterator pos = node->adjlist.begin();
  list<node_t*>::const_iterator end = node->adjlist.end();

  for (; pos != end; ++pos)
  {
    if ((*pos)->mark_ifnot() == true)
      to_visit.push_back(*pos);
  }
}

static unsigned int find_shortest_path_par
(graph_t* graph, node_t* a, node_t* b)
{
  // current and levels
  list<node_t*> to_visit[2];
  unsigned int depth = 0;

  // bootstrap algorithm
  to_visit[1].push_front(from);

  // while next level not empty
  while (to_visit[1].empty() == false)
  {
    // process nodes at level
    to_visit[0].swap(to_visit[1]);

    while (to_visit[0].empty() == false)
    {
      node_t* const node = to_visit[0].front();
      to_visit[0].pop_front();

      if (node == to)
      {
	abort_thieves(sc);
	return depth;
      }

      append_node_adjlist_par(node, to_visit[1]);
    }

    if (reduce_thieves(sc, to_visit[0]) == true)
      return depth;

    ++depth;
  }

  return 0;
}

#endif // parallel version


static void peek_random_pair
(graph_t& g, node_t*& a, node_t*& b)
{
  a = g.at(rand() % g.node_count());
  b = g.at(rand() % g.node_count());
}


static void initialize_stuff(void)
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

#if CONFIG_SEQUENTIAL
    gettimeofday(&tms[0], NULL);
    const unsigned int seq_depth = find_shortest_path_seq(g, from, to);
    gettimeofday(&tms[1], NULL);
    timersub(&tms[1], &tms[0], &tms[2]);
    double seq_usecs = (double)tms[2].tv_sec * 1E6 + (double)tms[2].tv_usec;
    printf("%u %lf ", seq_depth, seq_usecs);
#endif

#if CONFIG_PARALLEL
    gettimeofday(&tms[0], NULL);
    const unsigned int par_depth = find_shortest_path_par(g, from, to);
    gettimeofday(&tms[1], NULL);
    timersub(&tms[1], &tms[0], &tms[2]);
    double par_usecs = (double)tms[2].tv_sec * 1E6 + (double)tms[2].tv_usec;
    printf("%u %lf ", par_depth, par_usecs);
#endif

    printf("\n");
  }

  return 0;
}
