#pragma once
//------------------------------------------------------------------------------
// A simple scene graph and virtual scene graph to support reactive rendering.
//
// Attempts to use C++ value semantics as defined by
// https://github.com/arximboldi/lager
//
// Uses immutable data structures from:
// https://github.com/arximboldi/immer
//
// Depends on the idea that a forest of const shared_ptrs have value semantics.
// (there are some caveats to the above, but generally it works)
//
// TODO: Needs to actually be a scene graph (include model matrices)
// TODO: Ensure that we can also have 2D (orthographic views)
//------------------------------------------------------------------------------
#include <any>
#include <memory>
#include <type_traits>
#include <utility>

#include <immer/algorithm.hpp>
#include <immer/vector.hpp>

namespace vsg {

// Not part of our public interface.
template<typename T>
using vector = immer::vector<T>;

// Not part of our public interface.
template<typename T>
using box = std::shared_ptr<T>;

// Temporary function used to make the compiler happy.
// TODO: make this not usable somehow
template<typename ReturnT, typename ArgT>
auto update_gpu(ArgT const & /*unused*/) -> box<ReturnT> {
  return box<ReturnT>{};
}

struct placeholder {};
template<typename ContextT>
void draw(
  box<ContextT> const &ctx,
  placeholder const &place_holder1,
  placeholder const &place_holder2
);

template<typename ContextT>
struct node_i {
  node_i()                                                           = default;
  node_i(const node_i<ContextT> &)                                   = default;
  node_i(node_i<ContextT> &&) noexcept                               = default;
  auto operator=(const node_i<ContextT> &) -> node_i<ContextT>     & = default;
  auto operator=(node_i<ContextT> &&) noexcept -> node_i<ContextT> & = default;
  virtual ~node_i()                                                  = default;

  virtual void draw(box<ContextT> const &ctx) const                               = 0;
  [[nodiscard]] virtual auto marker() const -> std::any                           = 0;
  [[nodiscard]] virtual auto draw_args() const -> std::any                        = 0;
  [[nodiscard]] virtual auto children() const -> vector<box<node_i<ContextT>>>    = 0;
  virtual void update_children(vector<box<node_i<ContextT>>> const &new_children) = 0;
  virtual void update_draw_args(std::any draw_args)                               = 0;
};

//------------------------------------------------------------------------------
// Nodes are the concrete elements of the scene graph that will be rendered.
//
// They hold the original args used to create them as well as the gpu data created
// when they were uploaded to the GPU and the change marker that was specified
// when they were created. They also hold a set of children nodes that forms the tree.
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct node : public node_i<ContextT> {
  DrawArgsT m_draw_args;
  using gpu_data_t = decltype(update_gpu(m_draw_args));
  gpu_data_t m_gpu_data;
  ChangeMarkerT m_change_marker;

  // TODO: This makes the children completely generic, which means ONLY the
  // Context
  //       can be passed along, none of the draw_args_t. :(
  //       Can we fix that?
  vector<box<node_i<ContextT>>> m_children;

  node(DrawArgsT args, ChangeMarkerT marker, vector<box<node_i<ContextT>>> children)
    : m_draw_args{args}
    , m_gpu_data{update_gpu(args)}
    , m_change_marker{marker}
    , m_children{std::move(children)} {}

  // Rule of 5.
  ~node() override                           = default;
  node(const node &)                         = default;
  node(node &&) noexcept                     = default;
  auto operator=(const node &) -> node     & = default;
  auto operator=(node &&) noexcept -> node & = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto marker() const -> std::any override { return std::any(m_change_marker); }
  [[nodiscard]] auto draw_args() const -> std::any override { return std::any(m_draw_args); }

  void draw(box<ContextT> const &ctx) const override {
    using vsg::draw;
    draw(ctx, m_gpu_data, m_draw_args);
    immer::for_each(m_children, [&](const box<node_i<ContextT>> &cur_node) {
      cur_node->draw(ctx);
    });
  }

  [[nodiscard]] auto children() const -> vector<box<node_i<ContextT>>> override {
    return m_children;
  }

  void update_children(vector<box<node_i<ContextT>>> const &new_children) override {
    m_children = new_children;
  }

  void update_draw_args(std::any new_args) override {
    if (new_args.type() != typeid(m_draw_args)) { return; }
    m_draw_args = std::any_cast<DrawArgsT>(new_args);
  }
};

//------------------------------------------------------------------------------
// Virtual nodes represent a potential graph element.
//
// They should be cheap to create and copy, and can be compared to a node in
// order to determine if they will create a different node.
// They only hold the args and change markerk
template<typename ContextT>
struct v_node_i {
  v_node_i()                                                             = default;
  v_node_i(const v_node_i<ContextT> &)                                   = default;
  v_node_i(v_node_i<ContextT> &&) noexcept                               = default;
  auto operator=(const v_node_i<ContextT> &) -> v_node_i<ContextT>     & = default;
  auto operator=(v_node_i<ContextT> &&) noexcept -> v_node_i<ContextT> & = default;
  virtual ~v_node_i()                                                    = default;

  [[nodiscard]] virtual auto marker() const -> std::any                                        = 0;
  [[nodiscard]] virtual auto draw_args() const -> std::any                                     = 0;
  [[nodiscard]] virtual auto children() const -> vector<box<v_node_i<ContextT>>>               = 0;
  [[nodiscard]] virtual auto change_marker_equals(node_i<ContextT> const &other) const -> bool = 0;
  [[nodiscard]] virtual auto node() const -> box<node_i<ContextT>>                             = 0;
};

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
struct v_node : public v_node_i<ContextT> {
  DrawArgsT m_draw_args;
  ChangeMarkerT m_change_marker;
  vector<box<v_node_i<ContextT>>> m_children;

  v_node(DrawArgsT args, ChangeMarkerT marker, vector<box<v_node_i<ContextT>>> children)
    : m_draw_args{args}
    , m_change_marker{marker}
    , m_children{std::move(children)} {}

  // Rule of 5.
  ~v_node() override                             = default;
  v_node(const v_node &)                         = default;
  v_node(v_node &&) noexcept                     = default;
  auto operator=(const v_node &) -> v_node     & = default;
  auto operator=(v_node &&) noexcept -> v_node & = default;

  // Provide the base interface elements for our types.
  [[nodiscard]] auto marker() const -> std::any override { return std::any(m_change_marker); }

  [[nodiscard]] auto draw_args() const -> std::any override { return std::any(m_draw_args); }

  [[nodiscard]] auto children() const -> vector<box<v_node_i<ContextT>>> override {
    return m_children;
  }
  [[nodiscard]] auto change_marker_equals(node_i<ContextT> const &other) const -> bool override {
    auto other_change_marker = other.marker();
    if (other_change_marker.type() != typeid(m_change_marker)) { return false; }
    return std::any_cast<ChangeMarkerT>(other_change_marker) == m_change_marker;
  }

  [[nodiscard]] auto node() const -> box<node_i<ContextT>> override {
    vector<box<node_i<ContextT>>> children_nodes;
    immer::for_each(m_children, [&](const box<v_node_i<ContextT>> &child) {
      children_nodes = children_nodes.push_back(child->node());
    });
    return std::make_shared<vsg::node<ContextT, DrawArgsT, ChangeMarkerT>>(
      m_draw_args, m_change_marker, children_nodes
    );
  }
};

template<typename ContextT>
auto update_graph(box<node_i<ContextT>> graph, box<v_node_i<ContextT>> const &v_graph)
  -> box<node_i<ContextT>> {
  if (!graph) {
    // If the current graph is empty, create it from scratch.
    return v_graph->node();
  }

  // Else try and update things.
  if (
    // if the changeMarker did not change
    v_graph->change_marker_equals(*graph) &&
    // and the type of the node did not change
    v_graph->draw_args().type() == graph->draw_args().type()

  ) {
    // This node will remain the same, but update its children
    std::size_t index = 0;

    auto old_children = graph->children();
    auto v_children   = v_graph->children();
    immer::vector<box<node_i<ContextT>>> new_children;

    while (index < v_children.size()) {
      if (index < old_children.size()) {
        // Patch the children together
        new_children = new_children.push_back(update_graph(old_children[index], v_children[index]));
      } else {
        // Completely new child.
        new_children = new_children.push_back(v_children[index]->node());
      }
      ++index;
    }
    graph->update_draw_args(v_graph->draw_args());
    graph->update_children(new_children);
    return graph;
  }
  // Create the entire thing from this point onwards.
  return v_graph->node();
}

//--------------------------------------------------------------------------
// In HTML we'd have loads of nodes to use as "containers" here, we don't
// So we'll make a simple one.
struct fragment {};
struct fragment_gpu_data {};

inline auto update_gpu([[maybe_unused]] fragment frag) -> std::shared_ptr<fragment_gpu_data> {
  return std::make_shared<fragment_gpu_data>();
}

template<typename ContextT>
void draw(
  [[maybe_unused]] std::shared_ptr<ContextT> const &ctx,
  [[maybe_unused]] std::shared_ptr<fragment_gpu_data> const &data,
  [[maybe_unused]] fragment const &node
) {}

//--------------------------------------------------------------------------
// Helper functions that allow various options
template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker, vector<box<v_node_i<ContextT>>> children)
  -> box<v_node_i<ContextT>> {
  return std::make_shared<v_node<ContextT, DrawArgsT, ChangeMarkerT>>(val, marker, children);
}

// Specialization to allow for use without a marker.
struct no_change_marker {};
inline auto operator==(
  [[maybe_unused]] no_change_marker const &lhs,
  [[maybe_unused]] no_change_marker const &rhs
) -> bool {
  return true;
}

template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val, vector<box<v_node_i<ContextT>>> children) -> box<v_node_i<ContextT>> {
  return n<ContextT>(val, no_change_marker{}, children);
}

template<typename ContextT, typename DrawArgsT, typename ChangeMarkerT>
auto n(DrawArgsT val, ChangeMarkerT marker) -> box<v_node_i<ContextT>> {
  return n<ContextT>(val, marker, {});
}

template<typename ContextT, typename DrawArgsT>
auto n(DrawArgsT val) -> box<v_node_i<ContextT>> {
  return n<ContextT>(val, no_change_marker{}, {});
}

template<typename ContextT, typename ChangeMarkerT>
auto f(ChangeMarkerT marker, vector<box<v_node_i<ContextT>>> children) -> box<v_node_i<ContextT>> {
  return n<ContextT>(fragment{}, marker, children);
}

template<typename ContextT>
auto f(vector<box<v_node_i<ContextT>>> children) -> box<v_node_i<ContextT>> {
  return n<ContextT>(fragment{}, no_change_marker{}, std::move(children));
}

template<typename ContextT>
auto empty() -> box<v_node_i<ContextT>> {
  return n<ContextT, fragment, no_change_marker>(fragment{}, no_change_marker{}, {});
}

// A helper to define values that aren't used until needed.
template<typename T>
class lazy_val {
protected:
  using gen_t = std::function<T()>;
  gen_t m_generator;  // NOLINT

public:
  explicit lazy_val(T val)
    : m_generator{[val] { return val; }} {}

  explicit lazy_val(gen_t gen)
    : m_generator{[gen]() { return gen(); }} {}

  explicit operator T() { return m_generator(); }

  auto operator()() -> T { return m_generator(); }
};
}  // namespace vsg
