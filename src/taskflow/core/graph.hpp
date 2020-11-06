#pragma once

#include "../error/error.hpp"
#include "../utility/traits.hpp"
#include "../utility/passive_vector.hpp"
#include "declarations.hpp"

namespace tf {

// Class: Graph
class Graph {

  friend class Node;
  
  public:

    Graph() = default;
    Graph(const Graph&) = delete;
    Graph(Graph&&);

    Graph& operator = (const Graph&) = delete;
    Graph& operator = (Graph&&);
    
    void clear();

    bool empty() const;

    size_t size() const;
    
    template <typename ...Args>
    Node& emplace_back(Args&& ...); 

    Node& emplace_back();

    std::vector<std::unique_ptr<Node>>& nodes();

    const std::vector<std::unique_ptr<Node>>& nodes() const;

  private:
    
    std::vector<std::unique_ptr<Node>> _nodes;
};

// ----------------------------------------------------------------------------

// Class: Node
class Node {

  friend class Task;
  friend class TaskView;
  friend class Topology;
  friend class Taskflow;
  friend class Executor;
  friend class FlowBuilder;
  friend class Subflow;

  public:
  
  using StaticWork  = std::function<void()>;
  using DynamicWork = std::function<void(Subflow&)>;
  using ConditionWork = std::function<int()>;
  
  // state bit flag
  constexpr static int SPAWNED = 0x1;
  constexpr static int BRANCH  = 0x2;

  // variant index
  constexpr static int STATIC_WORK    = 1;
  constexpr static int DYNAMIC_WORK   = 2;
  constexpr static int CONDITION_WORK = 3; 

    //Node() = default;

    // Constructor 
    template <typename ...Args>
    Node(Args&&... args);

    ~Node();

    //void dump(std::ostream&) const;

    size_t num_successors() const;
    size_t num_dependents() const;
    size_t num_strong_dependents() const;
    size_t num_weak_dependents() const;
    
    const std::string& name() const;

    //std::string dump() const;

  private:
    
    std::string _name;
    std::variant<std::monostate, StaticWork, DynamicWork, ConditionWork> _work;

    tf::PassiveVector<Node*> _successors;
    tf::PassiveVector<Node*> _dependents;

    std::optional<Graph> _subgraph;

    Topology* _topology {nullptr};
    Taskflow* _module {nullptr};
    
    Node* _parent {nullptr};

    int _state {0};

    std::atomic<int> _join_counter {0};
    
    void _precede(Node*);
    void _set_state(int);
    void _unset_state(int);
    void _clear_state();
    void _set_up_join_counter();

    bool _has_state(int) const;
};

//// Constructor
//template <typename ...Args>
//Node::Node(Args&&... args) : _work{std::forward<Args>(args)...} {
//}
    
template <typename ...Args>
Node::Node(Args&&... args): _work{std::forward<Args>(args)...} {
} 

// Destructor
inline Node::~Node() {
  // this is to avoid stack overflow
  if(_subgraph.has_value()) {
    std::vector<std::unique_ptr<Node>> nodes;
    std::move(
     _subgraph->_nodes.begin(), _subgraph->_nodes.end(), std::back_inserter(nodes)
    );
    _subgraph->_nodes.clear();
    _subgraph.reset();
    size_t i = 0;
    while(i < nodes.size()) {
      if(auto& sbg = nodes[i]->_subgraph; sbg) {
        std::move(
          sbg->_nodes.begin(), sbg->_nodes.end(), std::back_inserter(nodes)
        );
        sbg->_nodes.clear();
        sbg.reset();
      }
      ++i;
    }
  }
}

// Procedure: _precede
inline void Node::_precede(Node* v) {
  _successors.push_back(v);
  v->_dependents.push_back(this);
}

// Function: num_successors
inline size_t Node::num_successors() const {
  return _successors.size();
}

// Function: dependents
inline size_t Node::num_dependents() const {
  return _dependents.size();
}

// Function: num_weak_dependents
inline size_t Node::num_weak_dependents() const {
  return std::count_if(
    _dependents.begin(), 
    _dependents.end(), 
    [](Node* node){ return node->_work.index() == Node::CONDITION_WORK; } 
  );
}

// Function: num_strong_dependents
inline size_t Node::num_strong_dependents() const {
  return std::count_if(
    _dependents.begin(), 
    _dependents.end(), 
    [](Node* node){ return node->_work.index() != Node::CONDITION_WORK; } 
  );
}

// Function: name
inline const std::string& Node::name() const {
  return _name;
}
//
//// Function: dump
//inline std::string Node::dump() const {
//  std::ostringstream os;  
//  dump(os);
//  return os.str();
//}
//
//// Function: dump
//inline void Node::dump(std::ostream& os) const {
//
//  os << 'p' << this << "[label=\"";
//  if(_name.empty()) os << 'p' << this;
//  else os << _name;
//  os << "\" ";
//
//  // condition node is colored green
//  if(_work.index() == CONDITION_WORK) {
//    os << " shape=diamond color=black fillcolor=aquamarine style=filled";
//  }
//
//  os << "];\n";
//  
//  for(size_t s=0; s<_successors.size(); ++s) {
//    if(_work.index() == CONDITION_WORK) {
//      // case edge is dashed
//      os << 'p' << this << " -> p" << _successors[s] 
//         << " [style=dashed label=\"" << s << "\"];\n";
//    }
//    else {
//      os << 'p' << this << " -> p" << _successors[s] << ";\n";
//    }
//  }
//  
//  // subflow join node
//  if(_parent && _successors.size() == 0) {
//    os << 'p' << this << " -> p" << _parent << ";\n";
//  }
//  
//  if(_subgraph && !_subgraph->empty()) {
//
//    os << "subgraph cluster_p" << this << " {\nlabel=\"Subflow: ";
//    if(_name.empty()) os << 'p' << this;
//    else os << _name;
//
//    os << "\";\n" << "color=blue\n";
//
//    for(const auto& n : _subgraph->nodes()) {
//      n->dump(os);
//    }
//    os << "}\n";
//  }
//}
    
// Procedure: _set_state
inline void Node::_set_state(int flag) { 
  _state |= flag; 
}

// Procedure: _unset_state
inline void Node::_unset_state(int flag) { 
  _state &= ~flag; 
}

// Procedure: _clear_state
inline void Node::_clear_state() { 
  _state = 0; 
}

// Procedure: _set_up_join_counter
inline void Node::_set_up_join_counter() {

  int c = 0;

  for(auto p : _dependents) {
    if(p->_work.index() == Node::CONDITION_WORK) {
      _set_state(Node::BRANCH);
    }
    else {
      c++;
    }
  }

  _join_counter.store(c, std::memory_order_relaxed);
}

// Function: _has_state
inline bool Node::_has_state(int flag) const {
  return _state & flag;
}

// ----------------------------------------------------------------------------

/*// Class: NodePool
class NodePool {

  public:

    template <typename C>
    std::unique_ptr<Node> acquire(C&&);

    std::unique_ptr<Node> acquire();

    void release(std::unique_ptr<Node>);
  
  private:
    
    //std::mutex _mutex;

    std::vector<std::unique_ptr<Node>> _nodes;

    void _recycle(Node&);
};

// Function: acquire
template <typename C>
inline std::unique_ptr<Node> NodePool::acquire(C&& c) {
  if(_nodes.empty()) {
    return std::make_unique<Node>(std::forward<C>(c));
  }
  else {
    auto node = std::move(_nodes.back());
    node->_work = std::forward<C>(c);
    _nodes.pop_back();
    return node;
  }
}

// Function: acquire
inline std::unique_ptr<Node> NodePool::acquire() {
  if(_nodes.empty()) {
    return std::make_unique<Node>();
  }
  else {
    auto node = std::move(_nodes.back());
    _nodes.pop_back();
    return node;
  }
}

// Procedure: release
inline void NodePool::release(std::unique_ptr<Node> node) {

  return;

  //assert(node);
  if(_nodes.size() >= 65536) {
    return;
  }
  
  auto children = node->_extract_children();

  for(auto& child : children) {
    _recycle(*child);
  }
  _recycle(*node);

  std::move(children.begin(), children.end(), std::back_inserter(_nodes));  
  _nodes.push_back(std::move(node));
}

// Procedure: _recycle
inline void NodePool::_recycle(Node& node) {
  node._name.clear();
  node._work = {};
  node._successors.clear();
  node._dependents.clear();
  node._topology = nullptr;
  node._module = nullptr;
  node._state = 0;
  node._join_counter.store(0, std::memory_order_relaxed);
  //assert(!node._subgraph);
}

// ----------------------------------------------------------------------------

namespace this_thread {
  inline thread_local NodePool nodepool;
}
*/

// ----------------------------------------------------------------------------

// Move constructor
inline Graph::Graph(Graph&& other) : 
  _nodes {std::move(other._nodes)} {
}

// Move assignment
inline Graph& Graph::operator = (Graph&& other) {
  _nodes = std::move(other._nodes);
  return *this;
}

// Procedure: clear
// clear and recycle the nodes
inline void Graph::clear() {
  _nodes.clear();
}

// Function: size
// query the size
inline size_t Graph::size() const {
  return _nodes.size();
}

// Function: empty
// query the emptiness
inline bool Graph::empty() const {
  return _nodes.empty();
}
    
// Function: nodes
// return a mutable reference to the node data structure
//inline std::vector<std::unique_ptr<Node>>& Graph::nodes() {
inline std::vector<std::unique_ptr<Node>>& Graph::nodes() {
  return _nodes;
}

// Function: nodes
// returns a constant reference to the node data structure
//inline const std::vector<std::unique_ptr<Node>>& Graph::nodes() const {
inline const std::vector<std::unique_ptr<Node>>& Graph::nodes() const {
  return _nodes;
}

// Function: emplace_back
// create a node from a give argument; constructor is called if necessary
template <typename ...Args>
Node& Graph::emplace_back(Args&&... args) {
  _nodes.push_back(std::make_unique<Node>(std::forward<Args>(args)...));
  return *(_nodes.back());
}

// Function: emplace_back
// create a node from a give argument; constructor is called if necessary
inline Node& Graph::emplace_back() {
  _nodes.push_back(std::make_unique<Node>());
  return *(_nodes.back());
}


}  // end of namespace tf. ---------------------------------------------------





