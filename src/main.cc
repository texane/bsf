#include <list>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>
#ifndef __USE_BSD
# define __USE_BSD
#endif
#include <time.h>
#include <sys/time.h>


using std::list;
using std::vector;


#define CONFIG_SEQ_GRAIN 128
#define CONFIG_PATH_DEPTH 30
#define CONFIG_NODEID 1
#define CONFIG_ITER 50


typedef struct node
{
  list<struct node*> adjlist;

#if CONFIG_PARALLEL
  volatile bool state __attribute__((aligned));
#else
  bool state;
#endif

#if CONFIG_NODEID
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
  { return __sync_bool_compare_and_swap(&state, 0, 1); }
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

#if CONFIG_NODEID
    unsigned int id = 0;
#endif

    for (; pos != end; ++pos)
    {
      pos->adjlist.clear();
      pos->state = false;
#if CONFIG_NODEID
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

#if 0
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


static void __attribute__((unused)) make_a_path
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


static void __attribute__((unused)) generate_random_graph
(graph_t& graph, unsigned int node_count, unsigned int node_degree)
{
  graph.initialize(node_count);

  unsigned int edge_count = node_count * node_degree / 2;
  for (; edge_count; --edge_count)
  {
    node_t* const a = graph.at(rand() % node_count);
    node_t* const b = graph.at(rand() % node_count);
   
    if ((a == b) || (a->has_adj(b))) continue ;

    a->add_adj(b);  b->add_adj(a);
  }
}


// graph loading

typedef struct mapped_file
{
  unsigned char* base;
  size_t off;
  size_t len;
} mapped_file_t;

static int map_file(mapped_file_t* mf, const char* path)
{
  int error = -1;
  struct stat st;

  const int fd = open(path, O_RDONLY);
  if (fd == -1)
    return -1;

  if (fstat(fd, &st) == -1)
    goto on_error;

  mf->base = (unsigned char*)mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if (mf->base == MAP_FAILED)
    goto on_error;

  mf->off = 0;
  mf->len = st.st_size;

  /* success */
  error = 0;

 on_error:
  close(fd);

  return error;
}

static void unmap_file(mapped_file_t* mf)
{
  munmap((void*)mf->base, mf->len);
  mf->base = (unsigned char*)MAP_FAILED;
  mf->len = 0;
}

static int next_line(mapped_file_t* mf, char* line)
{
  const unsigned char* end = mf->base + mf->len;
  const unsigned char* const base = mf->base + mf->off;
  const unsigned char* p;
  size_t skipnl = 0;
  size_t len = 0;
  char* s;

  for (p = base, s = line; p != end; ++p, ++s, ++len)
  {
    if (*p == '\n')
    {
      skipnl = 1;
      break;
    }

    *s = (char)*p;
  }

  *s = 0;

  if (p == base) return -1;

  mf->off += (p - base) + skipnl;

  return 0;
}

static bool next_edge
(mapped_file_t& mf, unsigned int& from, unsigned int& to)
{
  char line[64];
  if (next_line(&mf, line) == -1) return false;
  sscanf(line, "%u %u", &from, &to);
  return true;
}

static bool next_uint
(mapped_file_t& mf, unsigned int& ui)
{
  char line[64];
  if (next_line(&mf, line) == -1) return false;
  sscanf(line, "%u", &ui);
  return true;
}

static void load_graph(graph_t& graph, const char* path)
{
  unsigned int node_count, from, to;

  mapped_file_t mapped = {NULL, 0, 0};

  if (map_file(&mapped, path) == -1)
  {
    graph.initialize(0);
    return ;
  }

  // node count
  next_uint(mapped, node_count);
  graph.initialize(node_count);

  // edges
  while (next_edge(mapped, from, to))
    graph.at(from)->add_adj(graph.at(to));

  unmap_file(&mapped);
}


static void __attribute__((unused)) store_graph
(const graph_t& graph, const char* path)
{
  char line[256];
  int len;

  const int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC);
  if (fd == -1) return ;

  // node count
  len = sprintf(line, "%u\n", graph.node_count());
  write(fd, line, strlen(line));

  // edges
  vector<node_t>::const_iterator pos = graph.nodes.begin();
  vector<node_t>::const_iterator end = graph.nodes.end();

  for (; pos != end; ++pos)
  {
    list<node_t*>::const_iterator adjpos = (*pos).adjlist.begin();
    list<node_t*>::const_iterator adjend = (*pos).adjlist.end();

    for (; adjpos != adjend; ++adjpos)
    {
      len = sprintf(line, "%u %u\n", pos->id, (*adjpos)->id);
      write(fd, line, len);
    }
  }

  close(fd);
}


static void __attribute__((unused)) append_node_adjlist
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


#if CONFIG_SEQUENTIAL // sequential version

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

      append_node_adjlist(node, to_visit[1]);
    }

    ++depth;
  }

  return 0;
}

#endif // CONFIG_SEQUENTIAL


#if CONFIG_PARALLEL // parallel version

#include "kaapi.h"

// reduction

typedef struct victim_result
{
  bool is_found;
  list<node_t*>& to_visit;

  victim_result(list<node_t*>& _to_visit)
    : is_found(false), to_visit(_to_visit) {}

} victim_result_t;

typedef struct thief_result
{
  bool is_found;
  list<node_t*> to_visit;

  thief_result() : is_found(false) {}

} thief_result_t;

static int abort_thief
(kaapi_stealcontext_t* sc, void* targ, void* tdata, size_t tsize, void* varg)
{ return 0; }

static void abort_thieves(kaapi_stealcontext_t* ksc)
{
  kaapi_taskadaptive_result_t* ktr;
  while ((ktr = kaapi_get_thief_head(ksc)) != NULL)
    kaapi_preempt_thief(ksc, ktr, NULL, abort_thief, NULL);
}

static int reduce_thief
(kaapi_stealcontext_t* sc, void* targ, void* tdata, size_t tsize, void* varg)
{
  // victim result
  victim_result_t* const vres = (victim_result_t*)varg;
  
  // thief result
  thief_result_t* const tres = (thief_result_t*)tdata;

  if (tres->is_found == true)
    vres->is_found = true;
  else
    vres->to_visit.splice(vres->to_visit.end(), tres->to_visit);

  return 0;
}

static bool reduce_thieves
(kaapi_stealcontext_t* ksc, list<node_t*>& to_visit)
{
  kaapi_taskadaptive_result_t* ktr;

  victim_result_t res(to_visit);
  
  while ((ktr = kaapi_get_thief_head(ksc)) != NULL)
  {
    kaapi_preempt_thief(ksc, ktr, NULL, reduce_thief, (void*)&res);
    // return true on found
    if (res.is_found == true) return true;
  }

  return false;
}

// parallel work

typedef struct parallel_work
{
  volatile unsigned long lok __attribute__((aligned(64)));
  list<node_t*> nodes;
  node_t* to_find;

  parallel_work() : lok(0) {}

  void lock()
  {
    while (!__sync_bool_compare_and_swap(&lok, 0, 1))
      __asm__ __volatile__ ("pause \n\t");
  }

  void unlock() { lok = 0; }

  node_t* pop()
  {
    node_t* node = NULL;

    lock();
    if (nodes.empty() == false)
    {
      node = nodes.front();
      nodes.pop_front();
    }
    unlock();

    return node;
  }

  void set(list<node_t*>& to_visit)
  {
    lock();
    nodes.swap(to_visit);
    unlock();
  }

} parallel_work_t;

// splitter

typedef struct thief_work
{
  node_t* from;
  node_t* to;

  thief_work(node_t* _from, node_t* _to)
    : from(_from), to(_to) {}

} thief_work_t;

static void thief_entrypoint
(void* args, kaapi_thread_t* thread, kaapi_stealcontext_t* ksc)
{
  thief_work_t* const work = (thief_work_t*)args;
  thief_result_t* const res = (thief_result_t*)kaapi_adaptive_result_data(ksc);

  if (work->from == work->to)
  {
    res->is_found = true;
    return ;
  }

  append_node_adjlist(work->from, res->to_visit);
}

static int splitter
(kaapi_stealcontext_t* ksc, int nreq, kaapi_request_t* req, void* args)
{
  parallel_work_t* const par_work = (parallel_work_t*)args;

  int nrep = 0;
  for (; nreq; --nreq, ++nrep, ++req)
  {
    node_t* const node = par_work->pop();
    if (node == NULL) break ;

    kaapi_taskadaptive_result_t* const ktr =
      kaapi_allocate_thief_result(req, sizeof(thief_result_t), NULL);
    new (ktr->data) thief_result_t();
    
    thief_work_t* const tw = (thief_work_t*)kaapi_reply_init_adaptive_task
      (ksc, req, (kaapi_task_body_t)thief_entrypoint, sizeof(thief_work_t), ktr);
    new (tw) thief_work_t(node, par_work->to_find);

    kaapi_reply_pushhead_adaptive_task(ksc, req);
  }

  return nrep;
}

static unsigned int find_shortest_path_par
(graph_t& graph, node_t* from, node_t* to)
{
  // kaapi related
  kaapi_thread_t* const thread = kaapi_self_thread();
  kaapi_stealcontext_t* ksc;

  // bootstrap algorithm
  parallel_work_t par_work;
  ksc = kaapi_task_begin_adaptive
    (thread, KAAPI_SC_CONCURRENT | KAAPI_SC_PREEMPTION, splitter, &par_work);
  par_work.to_find = to;

  list<node_t*> to_visit;
  unsigned int depth = 0;

  to_visit.push_back(from);

  // while next level not empty
  while (to_visit.empty() == false)
  {
    par_work.set(to_visit);

    node_t* node;
    while ((node = par_work.pop()) != NULL)
    {
      // found, abort thieves
      if (node == to) goto on_abort;

      append_node_adjlist(node, to_visit);
    }

    // found, abort remaining thieves
    if (reduce_thieves(ksc, to_visit) == true)
      goto on_abort;

    ++depth;
  }

  // not found
  depth = 0;

 on_done:
  kaapi_task_end_adaptive(ksc);
  return depth;

  // abort the remaining thieves
 on_abort:
  abort_thieves(ksc);
  goto on_done;
}

#endif // CONFIG_PARALLEL


static void __attribute__((unused)) peek_random_pair
(graph_t& g, node_t*& a, node_t*& b)
{
  a = g.at(rand() % g.node_count());
  b = g.at(rand() % g.node_count());
}


static void initialize_stuff(void)
{
  srand(getpid() * time(0));

#if CONFIG_PARALLEL
  kaapi_init();
#endif
}


static void finalize_stuff()
{
#if CONFIG_PARALLEL
  kaapi_finalize();
#endif
}


int main(int ac, char** av)
{
  graph_t g;
  node_t* from;
  node_t* to;
  struct timeval tms[3];

  initialize_stuff();

#if !(CONFIG_PARALLEL || CONFIG_SEQUENTIAL)
  // generate and store a random graph
  const unsigned int node_count = atoi(av[1]);
  const unsigned int node_degree = atoi(av[2]);
  generate_random_graph(g, node_count, node_degree);
  store_graph(g, av[3]);
  return 0;
#endif

  load_graph(g, av[1]);
  from = g.at(atoi(av[2]));
  to = g.at(atoi(av[3]));

  unsigned int depth;
  double usecs = 0.f;

  for (unsigned int iter = 0; iter < CONFIG_ITER; ++iter)
  {
    g.unmark_nodes();
    gettimeofday(&tms[0], NULL);
#if CONFIG_SEQUENTIAL
    depth = find_shortest_path_seq(g, from, to);
#elif CONFIG_PARALLEL
    depth = find_shortest_path_par(g, from, to);
#endif
    gettimeofday(&tms[1], NULL);
    timersub(&tms[1], &tms[0], &tms[2]);
    usecs += (double)tms[2].tv_sec * 1E6 + (double)tms[2].tv_usec;
  }

  printf("%u %lf\n", depth, usecs / CONFIG_ITER);

  finalize_stuff();

  return 0;
}
